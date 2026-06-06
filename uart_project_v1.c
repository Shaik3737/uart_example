/**
 * =============================================================================
 * EC200U AT Command Driver for ESP32
 * =============================================================================
 *
 * WHAT THIS FILE DOES:
 *   This file implements a complete, production-ready AT command driver for the
 *   Quectel EC200U 4G LTE module running on an ESP32 using ESP-IDF + FreeRTOS.
 *
 * WHY IT IS STRUCTURED IN THREE LAYERS:
 *   Splitting the driver into layers means each layer has one job.
 *   When something breaks, you know exactly which layer to look at.
 *   You can also reuse lower layers for different modules in future projects.
 *
 *   Layer 1 — UART Physical Driver
 *       Job:  Move raw bytes between ESP32 and EC200U over UART1.
 *       How:  uart_init() configures the hardware.
 *             uart_event_task() runs forever, waking only when bytes arrive.
 *             It assembles bytes into complete lines (\n terminated) and
 *             routes each line to the correct FreeRTOS queue.
 *
 *   Layer 2 — AT Command Manager
 *       Job:  Send a command and reliably wait for the expected response.
 *       How:  at_command_send_raw() writes the command string over UART.
 *             at_command_send_and_wait() sends the command, then reads lines
 *             from the response queue until it finds the expected reply,
 *             an ERROR, or the timeout expires.
 *             command_in_progress flag tells the UART task where to route
 *             incoming lines — to the response queue or the URC queue.
 *
 *   Layer 3 — EC200U Application Functions
 *       Job:  Provide clean, human-readable functions for your application.
 *       How:  Each function calls at_command_send_and_wait() with the correct
 *             AT command, timeout, and expected response for that operation.
 *             Parsing of the response (sscanf, strstr) happens here.
 *
 * HARDWARE CONNECTION:
 *
 *   ESP32 GPIO17 (TX) ──────────────► EC200U RX
 *   ESP32 GPIO16 (RX) ◄────────────── EC200U TX
 *   ESP32 GND         ────────────────── EC200U GND
 *
 *   Note: EC200U UART is 3.3V compatible. No level shifter needed with ESP32.
 *
 * WHAT IS A URC (Unsolicited Result Code)?
 *   The EC200U can send messages at any time WITHOUT being asked.
 *   Examples:
 *       +QMTSTAT: 0,1   → MQTT connection was dropped by broker
 *       +QMTRECV: ...   → An MQTT message arrived for you
 *       RDY             → Module just powered up or rebooted
 *   These arrive even while you are waiting for a command response.
 *   The command_in_progress flag separates them from command responses.
 *
 * HOW LINES ARE ROUTED:
 *
 *   EC200U sends bytes
 *         │
 *         ▼
 *   uart_event_task() assembles bytes into lines
 *         │
 *         ├─── command_in_progress == true  ──► response_queue ──► at_command_send_and_wait()
 *         │
 *         └─── command_in_progress == false ──► urc_queue ──► urc_handler_task()
 *
 * AUTHOR:  Javid
 * TARGET:  ESP32 Wrover + Quectel EC200U
 * SDK:     ESP-IDF v5.4.x
 * =============================================================================
 */

#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "driver/gpio.h"
#include "driver/uart.h"

/* =============================================================================
 * HARDWARE CONFIGURATION
 * =============================================================================
 * Change these defines if you wire the module to different GPIO pins.
 * UART_NUM_1 is used so UART_NUM_0 remains free for the USB debug monitor.
 */
#define EC200U_UART_PORT        UART_NUM_1
#define EC200U_UART_TX_PIN      GPIO_NUM_17     /* ESP32 TX → EC200U RX        */
#define EC200U_UART_RX_PIN      GPIO_NUM_16     /* ESP32 RX ← EC200U TX        */
#define EC200U_UART_BAUD_RATE   115200          /* EC200U default baud rate     */

/* =============================================================================
 * BUFFER AND QUEUE SIZES
 * =============================================================================
 * UART_RX_RING_BUFFER_SIZE:
 *   Internal ring buffer the UART driver uses to store incoming bytes before
 *   our task reads them. 512 bytes is enough for AT command responses.
 *   Increase if you expect long HTTP or MQTT payloads.
 *
 * UART_EVENT_QUEUE_DEPTH:
 *   How many UART driver events (UART_DATA, UART_FIFO_OVF, etc.) can queue up
 *   before older ones are dropped. 10 is more than enough.
 *
 * AT_LINE_MAX_LEN:
 *   Maximum length of one line from the EC200U (one \n-terminated string).
 *   256 bytes covers all standard AT responses including long MQTT payloads.
 *
 * AT_RESPONSE_QUEUE_DEPTH / AT_URC_QUEUE_DEPTH:
 *   How many complete lines each queue can hold at once.
 */
#define UART_RX_RING_BUFFER_SIZE    512
#define UART_EVENT_QUEUE_DEPTH      10
#define AT_LINE_MAX_LEN             256
#define AT_RESPONSE_QUEUE_DEPTH     10
#define AT_URC_QUEUE_DEPTH          10

/* =============================================================================
 * QUEUE HANDLES
 * =============================================================================
 * uart_event_queue:
 *   Created by uart_driver_install(). The UART driver posts events here
 *   whenever something happens on the line (data arrived, overflow, etc.).
 *   uart_event_task() blocks on this queue.
 *
 * response_queue:
 *   Complete lines from the EC200U that are responses to commands we sent.
 *   at_command_send_and_wait() reads from this queue.
 *
 * urc_queue:
 *   Complete lines that arrived when no command was active — URCs.
 *   urc_handler_task() reads from this queue.
 */
static QueueHandle_t uart_event_queue  = NULL;
static QueueHandle_t response_queue    = NULL;
static QueueHandle_t urc_queue         = NULL;

/* =============================================================================
 * AT COMMAND MUTEX
 * =============================================================================
 * WHY THIS EXISTS:
 *   at_command_send_and_wait() must never be called by two tasks at the same
 *   time. If Task A and Task B both try to send an AT command simultaneously:
 *       - Both commands get sent back-to-back on the UART line.
 *       - The EC200U receives garbled input.
 *       - Both tasks read from the same response_queue — responses get mixed up.
 *
 * HOW IT WORKS:
 *   At the start of at_command_send_and_wait(), we take this mutex.
 *   No other task can enter the function until we release the mutex at the end.
 *   This guarantees only one AT command is in flight at any time.
 */
static SemaphoreHandle_t at_command_mutex = NULL;

/* =============================================================================
 * COMMAND ACTIVE FLAG
 * =============================================================================
 * WHY THIS EXISTS:
 *   The EC200U sends two types of messages:
 *       Type 1 — Responses to commands we sent:  "OK", "+CSQ: 18,0", etc.
 *       Type 2 — Unsolicited messages (URCs):    "+QMTSTAT: 0,1", "RDY", etc.
 *
 *   uart_event_task() needs to route each line to the right queue.
 *   This flag tells it which queue to use:
 *       true  → a command is waiting for a response → send to response_queue
 *       false → no command active → send to urc_queue
 *
 * WHY volatile:
 *   Written by at_command_send_and_wait() and read by uart_event_task(),
 *   which run in different task contexts (potentially different cores).
 *   volatile prevents the compiler from caching the value in a register —
 *   every read goes to actual RAM, so both tasks always see the current value.
 *
 * WHY NO MUTEX ON THIS FLAG:
 *   bool is one byte. On ESP32, single-byte reads and writes are atomic at
 *   the hardware level — the CPU cannot be interrupted mid-operation.
 *   volatile + single-byte = safe without a mutex for this specific use case.
 *   The at_command_mutex above protects the bigger critical section.
 */
static volatile bool command_in_progress = false;

/* =============================================================================
 * AT LINE STRUCTURE
 * =============================================================================
 * WHY A STRUCT AND NOT JUST char*?
 *   FreeRTOS queues copy items BY VALUE — they copy sizeof(item) bytes into
 *   internal queue storage when you call xQueueSend().
 *
 *   If you put a char* pointer in a queue:
 *       - The queue stores the pointer address (4 bytes), not the string.
 *       - The actual string lives on the stack of uart_event_task().
 *       - The stack gets reused when the task loops — the pointer becomes stale.
 *       - The receiving task reads garbage.
 *
 *   By putting the string INSIDE the struct, the queue copies all
 *   AT_LINE_MAX_LEN bytes of actual string data. No pointer issues.
 *   Think of the struct as an envelope — fixed size, data inside.
 *
 * FIELDS:
 *   line[] — the actual null-terminated string from the EC200U.
 *   len    — number of valid bytes in line[] (excluding null terminator).
 *            Useful for fast length checks without calling strlen().
 */
typedef struct {
    char line[AT_LINE_MAX_LEN];
    int  len;
} at_line_t;

/* =============================================================================
 * AT RESULT ENUM
 * =============================================================================
 * Return type for at_command_send_and_wait().
 * Using an enum instead of int/bool makes calling code self-documenting.
 *
 *   AT_OK        → EC200U responded with the expected string (e.g. "OK")
 *   AT_ERROR     → EC200U responded with "ERROR" or "+CME ERROR"
 *   AT_TIMEOUT   → No matching response arrived within the timeout period
 */
typedef enum {
    AT_OK,
    AT_ERROR,
    AT_TIMEOUT,
} at_result_t;


/* =============================================================================
 * LAYER 1 — UART PHYSICAL DRIVER
 * =============================================================================
 */

/**
 * uart_driver_init()
 * ------------------
 * WHAT:  Initialises UART1 hardware for communication with the EC200U.
 *
 * WHY:   UART0 is reserved for the USB debug monitor (printf, idf monitor).
 *        We use UART1 with custom GPIO pins so the debug port stays free.
 *
 * HOW:   Three steps in the correct order per ESP-IDF documentation:
 *        1. uart_driver_install() — allocates the driver, ring buffers,
 *           and the uart_event_queue. Must come FIRST.
 *        2. uart_param_config()   — writes baud rate, data bits, parity,
 *           stop bits into UART hardware registers.
 *        3. uart_set_pin()        — maps UART1 TX/RX signals to our GPIO pins.
 *
 * UART SETTINGS (8N1 = industry standard for AT command modems):
 *   - 115200 baud  — EC200U default, fast enough for all AT commands
 *   - 8 data bits  — one byte per frame
 *   - No parity    — no error detection bit (not needed at short distances)
 *   - 1 stop bit   — one idle bit between frames
 *   - No flow ctrl — we don't use RTS/CTS hardware flow control pins
 */
static void uart_driver_init(void)
{
    const uart_config_t uart_config = {
        .baud_rate  = EC200U_UART_BAUD_RATE,
        .data_bits  = UART_DATA_8_BITS,
        .parity     = UART_PARITY_DISABLE,
        .stop_bits  = UART_STOP_BITS_1,
        .flow_ctrl  = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_APB,
    };

    /* Step 1 — Install driver and allocate ring buffers + event queue.
     * rx_buffer_size must be > hardware FIFO length (128 bytes on ESP32).
     * tx_buffer_size > 0 means uart_write_bytes() returns immediately after
     * copying to the TX ring buffer — the driver ISR sends bytes in background. */
    uart_driver_install(EC200U_UART_PORT,
                        UART_RX_RING_BUFFER_SIZE,
                        UART_RX_RING_BUFFER_SIZE,
                        UART_EVENT_QUEUE_DEPTH,
                        &uart_event_queue,
                        0);

    /* Step 2 — Apply communication parameters to hardware registers. */
    uart_param_config(EC200U_UART_PORT, &uart_config);

    /* Step 3 — Route UART1 TX/RX signals to our chosen GPIO pins.
     * UART_PIN_NO_CHANGE for RTS/CTS because hardware flow control is off. */
    uart_set_pin(EC200U_UART_PORT,
                 EC200U_UART_TX_PIN,
                 EC200U_UART_RX_PIN,
                 UART_PIN_NO_CHANGE,
                 UART_PIN_NO_CHANGE);
}

/**
 * uart_event_task()
 * -----------------
 * WHAT:  Receives raw bytes from the EC200U and assembles them into complete
 *        lines, then routes each line to the correct FreeRTOS queue.
 *
 * WHY:   The EC200U sends responses as text lines terminated with \r\n.
 *        A single UART_DATA event may contain part of a line, a full line,
 *        or multiple lines. We must reassemble them correctly before any
 *        higher layer can parse the content.
 *
 * HOW:   The task blocks on uart_event_queue (zero CPU usage while idle).
 *        When UART_DATA arrives, it reads exactly event.size bytes.
 *        It processes bytes one at a time:
 *            - Non-newline bytes → append to line_assembly_buf[]
 *            - '\n' detected     → line is complete → push to queue → reset
 *
 *        ROUTING DECISION (command_in_progress flag):
 *            true  → line is a response to our command → response_queue
 *            false → line arrived unsolicited          → urc_queue
 *
 * WHY pdMS_TO_TICKS(100) IN uart_read_bytes():
 *   uart_read_bytes() internally takes a semaphore.
 *   Using portMAX_DELAY there can deadlock with the UART driver ISR.
 *   A 100ms timeout avoids this while still being fast enough for any response.
 *
 * OVERFLOW GUARD:
 *   If a line is longer than AT_LINE_MAX_LEN-1 bytes (malformed response),
 *   line_pos resets to 0 to prevent writing outside the buffer.
 *   The malformed line is discarded — better than corrupting memory.
 */
static void uart_event_task(void *arg)
{
    uart_event_t event;
    uint8_t      raw_byte_buf[128];          /* temporary buffer for raw bytes from driver */
    char         line_assembly_buf[AT_LINE_MAX_LEN]; /* assembles one complete line        */
    int          line_pos = 0;               /* current write position in line_assembly_buf */

    while (1)
    {
        /* xQueueReceive(uart_event_queue, &event, portMAX_DELAY)
         * 
         * Check the queue.
         * See that it is empty.
         * Put the current task into the Blocked state.
         * Remove the task from the scheduler's ready list.
         * Switch to another runnable task.
         * So the task is sleeping, consuming essentially no CPU.
         * When some other task or ISR does xQueueSend(uart_event_queue,...) to this queue.
         * Then FreeRTOS:
           * Copies the item into the queue.
           * Notices that a task is waiting on that queue.
           * Moves the blocked task back to the Ready state.
           * Scheduler eventually runs it.
           * xQueueReceive() returns pdTRUE.
    
         * NOTE: while using portMAX_DELAY in xQUEUE_RECEIVE() will always return pdTRUE only, never pdFALSE.
         */
        if (!xQueueReceive(uart_event_queue, &event, portMAX_DELAY))
            continue;

        if (event.type != UART_DATA)
            continue;   /* ignore FIFO overflow, frame errors, etc. for now */

        /* Read exactly the number of bytes the driver says arrived. */
        int bytes_read = uart_read_bytes(EC200U_UART_PORT,
                                         raw_byte_buf,
                                         event.size,
                                         pdMS_TO_TICKS(100));
        if (bytes_read <= 0)
            continue;

        /* Process one byte at a time to find complete lines. */
        for (int i = 0; i < bytes_read; i++)
        {
            char c = (char)raw_byte_buf[i];
            if (c == '\r') continue;
            if (c == '\n')
            {
                /* '\n' marks end of line. Null-terminate and push to queue. */
                line_assembly_buf[line_pos] = '\0';

                /* Only push non-empty lines — skip bare \r\n separators. */
                if (line_pos > 0)
                {
                    at_line_t queued_line;
                    memcpy(queued_line.line, line_assembly_buf, line_pos + 1);
                    queued_line.len = line_pos;

                    /* Route to correct queue based on whether a command is active. */
                    if (command_in_progress)
                        xQueueSend(response_queue, &queued_line, 0);
                    else
                        xQueueSend(urc_queue, &queued_line, 0);
                }

                line_pos = 0;   /* reset for next line */
            }
            else
            {
                /* Accumulate byte. Guard against buffer overflow. */
                if (line_pos < AT_LINE_MAX_LEN - 1)
                    line_assembly_buf[line_pos++] = c;
                else
                    line_pos = 0;   /* discard malformed oversized line */
            }
        }
    }
}


/* =============================================================================
 * LAYER 2 — AT COMMAND MANAGER
 * =============================================================================
 */

/**
 * at_command_send_raw()
 * ---------------------
 * WHAT:  Sends a raw AT command string over UART to the EC200U.
 *
 * WHY const char*:
 *   The caller's string must not be modified. Using const char* enforces this
 *   at compile time. We copy to a local buffer before appending \r\n so the
 *   original string is never touched.
 *
 * WHY \r\n:
 *   Every AT command must end with CR+LF (\r\n). The EC200U ignores commands
 *   without this terminator. This function adds \r\n automatically if missing,
 *   so callers can write clean strings like "AT+CSQ" without remembering to add it.
 */
static void at_command_send_raw(const char *command)
{
    char tx_buffer[AT_LINE_MAX_LEN] = {0};
    int  cmd_len = strlen(command);

    /* Check if caller already appended \r\n — don't double-add it. */
    if (cmd_len >= 2 &&
        command[cmd_len - 2] == '\r' &&
        command[cmd_len - 1] == '\n')
    {
        snprintf(tx_buffer, sizeof(tx_buffer), "%s", command);
    }
    else
    {
        snprintf(tx_buffer, sizeof(tx_buffer), "%s\r\n", command);
    }

    uart_write_bytes(EC200U_UART_PORT, tx_buffer, strlen(tx_buffer));
}

/**
 * at_command_send_and_wait()
 * --------------------------
 * WHAT:  Sends an AT command and waits until the expected response arrives,
 *        an ERROR is received, or the timeout expires.
 *        Optionally captures all response lines into a caller-provided buffer.
 *
 * WHY THIS IS THE CORE FUNCTION:
 *   Every EC200U operation (check network, open MQTT, publish) is just an AT
 *   command + expected response. This one function handles all of them cleanly.
 *   Higher-level functions (Layer 3) call this without worrying about UART,
 *   queues, or timeouts — they just check the return value.
 *
 * PARAMETERS:
 *   cmd              — AT command string (e.g. "AT+CSQ")
 *   expected_reply   — string to look for in response (e.g. "OK")
 *   timeout_ms       — max milliseconds to wait before giving up
 *   reply_buffer     — if not NULL, all received lines are appended here
 *   reply_buffer_len — size of reply_buffer (to prevent overflow)
 *
 * WHY strstr() AND NOT strcmp():
 *   strcmp() requires an exact full-string match.
 *   EC200U responses contain extra characters: "+CSQ: 18,0\r" not just "+CSQ".
 *   strstr() finds the expected string ANYWHERE inside the response line.
 *   Example: strstr("+CSQ: 18,0\r", "CSQ") → match ✓
 *            strcmp("+CSQ: 18,0\r", "CSQ") → no match ✗
 *
 * HOW THE TIMEOUT WORKS:
 *   We record the tick count before sending the command.
 *   Each loop iteration checks if (now - start) exceeds the timeout.
 *   xQueueReceive() waits up to 10ms per iteration — this is the granularity.
 *   Total wait = up to timeout_ms, checked every 10ms.
 *
 * HOW REPLY BUFFER ACCUMULATION WORKS:
 *   If reply_buffer is not NULL, every received line is appended with strncat().
 *   strncat(buf, line, space_remaining - 1) appends at most space_remaining-1
 *   bytes, always leaving room for the null terminator. This prevents overflow.
 *   After calling this function, reply_buffer contains all lines separated by \n,
 *   ready for sscanf() or strstr() parsing in Layer 3.
 *
 * MUTEX PROTECTION:
 *   at_command_mutex prevents two tasks from entering this function at the same
 *   time. Only one AT command can be in flight at any moment.
 *
 * RETURNS:
 *   AT_OK      — expected_reply found in a response line
 *   AT_ERROR   — "ERROR" found in a response line
 *   AT_TIMEOUT — timeout expired with no matching response
 */
at_result_t at_command_send_and_wait(const char *cmd,
                                      const char *expected_reply,
                                      uint32_t    timeout_ms,
                                      char       *reply_buffer,
                                      int         reply_buffer_len)
{
    /* Lock — only one command allowed at a time. */
    xSemaphoreTake(at_command_mutex, portMAX_DELAY);

    /* Flush stale lines from previous command before starting. */
    at_line_t discard;
    while (xQueueReceive(response_queue, &discard, 0) == pdTRUE);

    /* Signal uart_event_task to route incoming lines to response_queue. */
    command_in_progress = true;

    at_command_send_raw(cmd);

    uint32_t start_tick = xTaskGetTickCount();
    at_result_t result  = AT_TIMEOUT;

    while ((xTaskGetTickCount() - start_tick) < pdMS_TO_TICKS(timeout_ms))
    {
        at_line_t received_line;

        if (!xQueueReceive(response_queue, &received_line, pdMS_TO_TICKS(10)))
            continue;   /* nothing yet — keep waiting */

        /* Optionally accumulate this line into the caller's buffer. */
        if (reply_buffer != NULL)
        {
            int space_left = reply_buffer_len - (int)strlen(reply_buffer) - 1;
            if (space_left > 0)
            {
                strncat(reply_buffer, received_line.line, space_left);
                strncat(reply_buffer, "\n", reply_buffer_len - (int)strlen(reply_buffer) - 1);
            }
        }

        /* Check for expected response. */
        if (strstr(received_line.line, expected_reply) != NULL)
        {
            result = AT_OK;
            break;
        }

        /* Check for error response. */
        if (strstr(received_line.line, "ERROR") != NULL)
        {
            result = AT_ERROR;
            break;
        }
    }

    /* Release — allow uart_event_task to route future lines to urc_queue. */
    command_in_progress = false;

    /* Unlock — allow next command to proceed. */
    xSemaphoreGive(at_command_mutex);

    return result;
}

/**
 * urc_handler_task()
 * ------------------
 * WHAT:  Handles Unsolicited Result Codes (URCs) from the EC200U.
 *
 * WHY:   The EC200U sends URCs without being asked — at any time.
 *        If we ignored them, we would miss critical events like:
 *            - MQTT broker disconnecting us
 *            - Incoming MQTT messages
 *            - Module reboot (needs full re-initialization)
 *
 * HOW:   Blocks on urc_queue with portMAX_DELAY — zero CPU when idle.
 *        When a URC arrives, checks which type it is using strstr().
 *        Each type triggers the appropriate handler action.
 *
 * EXTENDING THIS FUNCTION:
 *   Add more else-if blocks for additional URCs your project needs.
 *   EC200U URCs relevant to MQTT projects:
 *       +QMTSTAT: 0,x  → MQTT connection state changed (x=1 means dropped)
 *       +QMTRECV: ...  → Incoming MQTT message with topic and payload
 *       +CEREG: x      → Network registration status changed
 *       RDY            → Module rebooted — must re-run full initialization
 */
static void urc_handler_task(void *arg)
{
    at_line_t urc_line;

    while (1)
    {
        /* Block until a URC line arrives — zero CPU usage while waiting. */
        if (!xQueueReceive(urc_queue, &urc_line, portMAX_DELAY))
            continue;

        /* Identify and handle each URC type. */
        if (strstr(urc_line.line, "+QMTSTAT") != NULL)
        {
            /* MQTT connection state change.
             * Format: +QMTSTAT: <client_idx>,<err_code>
             * err_code 1 = connection closed by network/broker.
             * TODO: trigger MQTT reconnect logic here. */
            printf("[URC] MQTT status changed: %s\n", urc_line.line);
        }
        else if (strstr(urc_line.line, "+QMTRECV") != NULL)
        {
            /* Incoming MQTT message.
             * Format: +QMTRECV: <client_idx>,<msgid>,<topic>,<payload>
             * TODO: parse topic and payload, dispatch to application. */
            printf("[URC] MQTT message received: %s\n", urc_line.line);
        }
        else if (strstr(urc_line.line, "RDY") != NULL)
        {
            /* Module powered up or rebooted spontaneously.
             * All previous state (PDP context, MQTT connection) is lost.
             * TODO: trigger full re-initialization sequence. */
            printf("[URC] Module rebooted — re-initialization required\n");
        }
        else if (strstr(urc_line.line, "+CEREG") != NULL)
        {
            /* Network registration status changed.
             * TODO: parse registration state, update application state machine. */
            printf("[URC] Network registration changed: %s\n", urc_line.line);
        }
        else
        {
            /* Unknown URC — log it for debugging. */
            printf("[URC] Unknown: %s\n", urc_line.line);
        }
    }
}


/* =============================================================================
 * LAYER 3 — EC200U APPLICATION FUNCTIONS
 * =============================================================================
 * These functions are what your application calls.
 * Each one wraps exactly one AT command exchange.
 * They hide all AT syntax, timeout values, and response parsing from the caller.
 *
 * NAMING CONVENTION:
 *   ec200u_<action>()
 *   Returns bool (true = success) or int (value, -1 = failure).
 */

/**
 * ec200u_is_alive()
 * -----------------
 * WHAT:  Checks if the EC200U is powered on and responding to AT commands.
 *
 * HOW:   Sends "AT" — the most basic AT command.
 *        EC200U always responds with "OK" if alive.
 *        Timeout 1000ms is generous for this simple check.
 *
 * USE CASE:
 *   Call this first on startup before any other command.
 *   If it returns false, check power supply and UART wiring.
 */
bool ec200u_is_alive(void)
{
    at_result_t result = at_command_send_and_wait("AT", "OK", 1000, NULL, 0);
    return (result == AT_OK);
}

/**
 * ec200u_is_network_registered()
 * --------------------------------
 * WHAT:  Checks if the SIM card is registered on a cellular network.
 *
 * HOW:   Sends "AT+CREG?" — network registration status query.
 *        EC200U responds with: +CREG: <n>,<stat>
 *        We parse <stat>:
 *            0 = not registered, not searching
 *            1 = registered, home network   ← we want this
 *            2 = not registered, searching
 *            3 = registration denied
 *            5 = registered, roaming        ← or this
 *
 * WHY strstr() BEFORE sscanf():
 *   reply_buffer may contain multiple lines: "+CREG: 0,1\nOK\n"
 *   sscanf() on the whole buffer only works if +CREG: is at position 0.
 *   strstr() finds +CREG: wherever it is in the buffer, then we
 *   run sscanf() starting from that exact position. Always reliable.
 *
 * TIMEOUT: 3000ms — network queries can take a moment to complete.
 */
bool ec200u_is_network_registered(void)
{
    char reply_buf[128] = {0};

    at_result_t result = at_command_send_and_wait("AT+CREG?",
                                                   "OK",
                                                   3000,
                                                   reply_buf,
                                                   sizeof(reply_buf));
    if (result != AT_OK)
        return false;

    /* Find the +CREG: line inside the accumulated reply buffer. */
    char *creg_line = strstr(reply_buf, "+CREG:");
    if (creg_line == NULL)
        return false;

    int n = 0, stat = 0;
    sscanf(creg_line, "+CREG: %d,%d", &n, &stat);

    /* stat 1 = home network, stat 5 = roaming — both mean registered. */
    if (stat == 1 || stat == 5)
    {
        printf("[EC200U] Network registered (stat=%d)\n", stat);
        return true;
    }

    printf("[EC200U] Network not registered (stat=%d)\n", stat);
    return false;
}

/**
 * ec200u_get_signal_strength()
 * ----------------------------
 * WHAT:  Returns the current cellular signal strength (RSSI).
 *
 * HOW:   Sends "AT+CSQ" — signal quality query.
 *        EC200U responds with: +CSQ: <rssi>,<ber>
 *        <rssi> range: 0-31 (higher = stronger), 99 = unknown/no signal
 *        <ber>  = bit error rate (we ignore this for now)
 *
 * RETURNS:
 *   RSSI value (0-31) on success.
 *   -1 if command failed or response could not be parsed.
 *
 * SIGNAL STRENGTH GUIDE:
 *   0-9   = Marginal (may drop connection)
 *   10-14 = OK
 *   15-19 = Good
 *   20-31 = Excellent
 *   99    = No signal
 */
int ec200u_get_signal_strength(void)
{
    char reply_buf[128] = {0};

    at_result_t result = at_command_send_and_wait("AT+CSQ",
                                                   "OK",
                                                   1000,
                                                   reply_buf,
                                                   sizeof(reply_buf));
    if (result != AT_OK)
        return -1;

    /* Find the +CSQ: line — same pattern as +CREG: above. */
    char *csq_line = strstr(reply_buf, "+CSQ:");
    if (csq_line == NULL)
        return -1;

    int rssi = 0, ber = 0;
    sscanf(csq_line, "+CSQ: %d,%d", &rssi, &ber);

    printf("[EC200U] Signal strength RSSI=%d\n", rssi);
    return rssi;
}


/* =============================================================================
 * APPLICATION ENTRY POINT
 * =============================================================================
 */

/**
 * app_main()
 * ----------
 * WHAT:  ESP-IDF entry point. Runs once at startup.
 *
 * INITIALIZATION ORDER — ORDER MATTERS:
 *   1. uart_driver_init()     — hardware must be ready before anything else
 *   2. xQueueCreate() calls   — queues must exist before tasks that use them
 *   3. xSemaphoreCreateMutex()— mutex must exist before at_command_send_and_wait()
 *   4. xTaskCreate() calls    — tasks start immediately; all resources must be ready
 *   5. vTaskDelay(1000ms)     — give EC200U time to finish booting and send "RDY"
 *   6. Application logic      — only after hardware and tasks are all running
 *
 * NOTE ON app_main() RETURNING:
 *   In ESP-IDF, app_main() is allowed to return. The FreeRTOS scheduler
 *   continues running all created tasks. "Returned from app_main()" in the
 *   monitor output is expected and normal.
 */
void app_main(void)
{
    /* Step 1 — Initialise UART1 hardware. */
    uart_driver_init();

    /* Step 2 — Create FreeRTOS queues.
     * sizeof(at_line_t) tells the queue how many bytes to copy per item.
     * All queues must exist before the tasks that use them start. */
    response_queue = xQueueCreate(AT_RESPONSE_QUEUE_DEPTH, sizeof(at_line_t));
    urc_queue      = xQueueCreate(AT_URC_QUEUE_DEPTH,      sizeof(at_line_t));

    /* Step 3 — Create the AT command mutex.
     * Must exist before any task calls at_command_send_and_wait(). */
    at_command_mutex = xSemaphoreCreateMutex();

    /* Step 4 — Start FreeRTOS tasks.
     * uart_event_task: highest priority — never miss an incoming byte.
     * urc_handler_task: lower priority — URCs can wait a few milliseconds. */
    xTaskCreate(uart_event_task,   "uart_event",   (1024 * 4), NULL, 10, NULL);
    xTaskCreate(urc_handler_task,  "urc_handler",  (1024 * 2), NULL,  5, NULL);

    /* Step 5 — Wait for EC200U to finish booting.
     * On power-up the EC200U sends "RDY\r\n" after ~1-3 seconds.
     * We wait 2000ms to let that URC arrive and be processed before
     * we start sending commands. */
    vTaskDelay(pdMS_TO_TICKS(2000));

    /* Step 6 — Application startup sequence. */

    /* Check module is alive — must pass before any other command. */
    if (ec200u_is_alive())
        printf("[APP] EC200U is alive and responding.\n");
    else
    {
        printf("[APP] EC200U not responding. Check power and wiring.\n");
        return;
    }

    /* Check SIM registration — must be registered before MQTT is possible. */
    if (ec200u_is_network_registered())
        printf("[APP] SIM registered on network. Ready for data.\n");
    else
        printf("[APP] SIM not registered. Check SIM card and antenna.\n");

    /* Read signal strength — useful for diagnosing connection problems. */
    int rssi = ec200u_get_signal_strength();
    if (rssi >= 0)
        printf("[APP] Signal strength: %d/31\n", rssi);
    else
        printf("[APP] Could not read signal strength.\n");

    /* TODO: next steps
     *   ec200u_pdp_activate()    — activate data session
     *   ec200u_mqtt_open()       — open TCP connection to broker
     *   ec200u_mqtt_connect()    — authenticate with broker
     *   ec200u_mqtt_publish()    — send sensor data
     *   ec200u_mqtt_subscribe()  — receive commands from cloud
     */
}