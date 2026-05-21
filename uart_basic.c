#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/gpio.h"
#include "driver/uart.h"

/*
 * =============================================================================
 * PIN & UART CONFIGURATION
 * =============================================================================
 *
 * UART_NUM  : We use UART_NUM_1 (the second hardware UART on ESP32).
 *             UART_NUM_0 is reserved for the USB debug/flash port (Serial Monitor),
 *             so we must NOT use it for GSM or any external peripheral.
 *
 * BUF_SIZE  : Size of the UART driver's internal ring buffer (in bytes).
 *             1024 bytes is enough for typical AT command responses.
 *             If you expect large data (e.g. HTTP responses), increase this.
 *
 * TXD_PIN   : GPIO pin connected to the GSM module's RX pin.
 *             ESP32 TX → GSM RX
 *
 * RXD_PIN   : GPIO pin connected to the GSM module's TX pin.
 *             GSM TX → ESP32 RX
 */
#define UART_NUM    UART_NUM_1
#define BUF_SIZE    (1024)
#define TXD_PIN     17
#define RXD_PIN     16

/*
 * uart_queue
 * ----------
 * A FreeRTOS queue handle created by uart_driver_install().
 * The UART driver posts events (UART_DATA, UART_FIFO_OVF, etc.) into this
 * queue whenever something happens on the UART line.
 * Our receive task blocks on this queue and wakes up only when an event arrives,
 * so it consumes zero CPU while idle — this is the correct ESP-IDF pattern.
 */
QueueHandle_t uart_queue;

/*
 * =============================================================================
 * uart_init()
 * =============================================================================
 * Initialises UART_NUM_1 for communication with the GSM module.
 *
 * IMPORTANT — Order of calls matters:
 *   1. uart_driver_install()  — allocates the driver, ring buffers, and the
 *                               event queue. Must come FIRST.
 *   2. uart_param_config()    — applies baud rate, data bits, parity, etc.
 *   3. uart_set_pin()         — maps UART signals to physical GPIO pins.
 *
 * Swapping steps 2 and 3 with step 1 can cause a crash because the driver
 * needs to be installed before the hardware registers are reconfigured.
 */
void uart_init(void)
{
    /*
     * uart_config_t — hardware settings for the UART peripheral.
     *
     * baud_rate  : Must match the GSM module's baud rate exactly.
     *              SIM800L / SIM7600 default to 115200.
     *
     * data_bits  : 8-bit data frames — standard for AT command protocols.
     *
     * parity     : No parity bit. GSM modules typically use 8N1
     *              (8 data bits, No parity, 1 stop bit).
     *
     * stop_bits  : 1 stop bit — part of the 8N1 standard.
     *
     * flow_ctrl  : Hardware flow control disabled. If your GSM module
     *              supports RTS/CTS and you have those pins wired, you can
     *              enable UART_HW_FLOWCTRL_CTS_RTS for more robust comms.
     *
     * source_clk : APB clock (80 MHz on ESP32). This is the default and
     *              gives accurate baud rates for standard speeds.
     */
    const uart_config_t uartConfig = {
        .baud_rate  = 115200,
        .data_bits  = UART_DATA_8_BITS,
        .parity     = UART_PARITY_DISABLE,
        .stop_bits  = UART_STOP_BITS_1,
        .flow_ctrl  = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_APB,
    };

    /*
     * uart_driver_install()
     * ----------------------
     * Installs the UART driver and allocates all internal resources.
     *
     * Arguments:
     *   UART_NUM        — which UART port (UART_NUM_1 here)
     *   BUF_SIZE        — RX ring buffer size (bytes). Data arriving from GSM
     *                     is stored here by the driver ISR until we read it.
     *   BUF_SIZE        — TX ring buffer size (bytes). Data we send is queued
     *                     here before the driver clocks it out.
     *   10              — event queue depth. Max 10 unread events can be
     *                     queued before older ones are dropped.
     *   &uart_queue     — pointer to our QueueHandle_t. The driver fills this
     *                     with the newly created queue handle.
     *   0               — interrupt flags (0 = default, no special flags).
     */
    uart_driver_install(UART_NUM, BUF_SIZE, BUF_SIZE, 10, &uart_queue, 0);

    /*
     * uart_param_config()
     * --------------------
     * Applies the uart_config_t settings to the UART hardware registers.
     * Called after driver install so the driver context already exists.
     */
    uart_param_config(UART_NUM, &uartConfig);

    /*
     * uart_set_pin()
     * ---------------
     * Maps UART_NUM_1's TX/RX signals to physical GPIO pins.
     *
     * Arguments:
     *   UART_NUM            — the UART port
     *   TXD_PIN (17)        — ESP32 transmit → connect to GSM module RX
     *   RXD_PIN (16)        — ESP32 receive  ← connect to GSM module TX
     *   UART_PIN_NO_CHANGE  — RTS pin: not used (no hardware flow control)
     *   UART_PIN_NO_CHANGE  — CTS pin: not used (no hardware flow control)
     */
    uart_set_pin(UART_NUM, TXD_PIN, RXD_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
}

/*
 * =============================================================================
 * send_AT_command()
 * =============================================================================
 * Sends an AT command string to the GSM module over UART.
 *
 * GSM modules require every AT command to be terminated with \r\n (CR LF).
 * This function handles two cases:
 *   - Command already ends with \r\n → send as-is.
 *   - Command does NOT end with \r\n → append \r\n before sending.
 *
 * Parameters:
 *   command — null-terminated AT command string, e.g. "AT+GMR" or "AT\r\n"
 */
void send_AT_command(const char *command)
{
    char    buffer[128] = {0};
    uint8_t len = strlen(command);

    /*
     * Case 1: Command already has \r\n at the end (positions len-2 and len-1).
     * Copy it into buffer unchanged and send it directly.
     */
    if (len > 0 && command[len-2] == '\r' && command[len-1] == '\n')
    {
        snprintf(buffer, sizeof(buffer), "%s", command);
        uart_write_bytes(UART_NUM, buffer, strlen(buffer));
    }
    /*
     * Case 2: Command does NOT end with \r\n.
     * Append \r\n via snprintf so the GSM module can recognise the end of command.
     * Note: we send 'buffer' (with \r\n) NOT the original 'command' (without).
     */
    else if (len > 0 && command[len-2] != '\r' && command[len-1] != '\n')
    {
        snprintf(buffer, sizeof(buffer), "%s\r\n", command);
        uart_write_bytes(UART_NUM, buffer, strlen(buffer));
    }
    /*
     * Case 3: Malformed command (e.g. empty string or only one character).
     * Print a warning — nothing is sent to the GSM module.
     */
    else
    {
        printf("Invalid AT command format. Command should end with \\r\\n.\n");
    }
}

/*
 * =============================================================================
 * uart_receive_task()
 * =============================================================================
 * FreeRTOS task that listens for incoming data from the GSM module.
 *
 * How it works:
 *   1. Blocks on xQueueReceive() — sleeps with zero CPU usage until the UART
 *      driver posts an event (e.g. new bytes arrived).
 *   2. Checks the event type — we only care about UART_DATA events here.
 *      Other types (UART_FIFO_OVF, UART_BUFFER_FULL, etc.) can be added later.
 *   3. Calls uart_read_bytes() to pull the raw bytes out of the driver's
 *      internal ring buffer into our local buf[].
 *   4. Null-terminates the buffer and prints it.
 *
 * Why a separate task?
 *   UART data can arrive at any time. A dedicated task lets it be processed
 *   immediately without blocking the rest of the application (app_main, other tasks).
 *
 * Stack size: 4096 bytes — enough for printf + UART driver internals.
 *             2048 is too small and causes stack overflow crashes.
 */
void uart_receive_task(void *pvParameter)
{
    uart_event_t event;
    uint8_t      buf[1024];  // local buffer to hold received bytes

    while (1)
    {
        /*
         * xQueueReceive()
         * ----------------
         * Blocks this task indefinitely (portMAX_DELAY) until the UART driver
         * posts an event. The task uses NO CPU while waiting — FreeRTOS switches
         * to other tasks or goes idle.
         *
         * When an event arrives, it is copied into 'event' and this call returns pdTRUE.
         */
        if (xQueueReceive(uart_queue, (void *)&event, portMAX_DELAY))
        {
            /*
             * event.type == UART_DATA
             * ------------------------
             * Means the driver received one or more bytes from the GSM module.
             * event.size tells us exactly how many bytes are waiting in the
             * ring buffer right now.
             *
             * Other possible event types (not handled here):
             *   UART_FIFO_OVF    — hardware FIFO overflowed (data lost)
             *   UART_BUFFER_FULL — ring buffer full (data lost)
             *   UART_BREAK       — break condition detected on the line
             *   UART_PARITY_ERR  — parity error in received data
             *   UART_FRAME_ERR   — framing error (baud rate mismatch?)
             */
            if (event.type == UART_DATA)
            {
                /*
                 * uart_read_bytes()
                 * -----------------
                 * Copies bytes from the driver's internal ring buffer into buf[].
                 *
                 * Arguments:
                 *   UART_NUM           — which UART port to read from
                 *   buf                — destination buffer
                 *   event.size         — number of bytes to read (exactly what arrived)
                 *   pdMS_TO_TICKS(100) — timeout: wait up to 100ms for the bytes
                 *                        to be available in the ring buffer.
                 *
                 * CRITICAL: Do NOT use portMAX_DELAY here.
                 *   uart_read_bytes() internally takes a semaphore. Using portMAX_DELAY
                 *   on that semaphore can deadlock with the UART driver's ISR and causes:
                 *   "assert failed: xQueueSemaphoreTake queue.c:1713 (uxItemSize == 0)"
                 *   A fixed timeout like pdMS_TO_TICKS(100) avoids this entirely.
                 *
                 * Returns: actual number of bytes read, or -1 on error.
                 */
                int len = uart_read_bytes(UART_NUM, buf, event.size, pdMS_TO_TICKS(100));

                if (len > 0)
                {
                    buf[len] = '\0';  // null-terminate so we can use printf %s safely
                    printf("Received data len: %d\n", len);
                    printf("Received data: %s\n", buf);
                }
            }
        }
    }
}

/*
 * =============================================================================
 * app_main()
 * =============================================================================
 * Entry point — runs once on startup (equivalent to main() in standard C).
 *
 * Sequence:
 *   1. Initialise UART hardware.
 *   2. Create the receive task so it is ready before we send anything.
 *   3. Send AT commands to the GSM module.
 *
 * Note: app_main() returning is normal in ESP-IDF. FreeRTOS continues running
 * all created tasks. The "Returned from app_main()" log message is expected.
 */
void app_main(void)
{
    /* Step 1 — Set up UART1 hardware, pins, and driver */
    uart_init();

    /* Step 2 — Launch the receive task BEFORE sending commands,
     *           so no incoming response is missed */
    xTaskCreate(
        uart_receive_task,      // task function
        "uart_receive_task",    // task name (shown in stack traces / FreeRTOS stats)
        4096,                   // stack size in bytes (must be >= 4096 for this task)
        NULL,                   // parameter passed to task (none needed)
        10,                     // priority (higher number = higher priority)
        NULL                    // task handle (NULL = we don't need to reference it later)
    );

    /* Step 3 — Send AT commands to the GSM module.
     *           Responses will be printed by uart_receive_task. */
    const char *at_command = "AT+GMR";  // query firmware version
    send_AT_command(at_command);

    const char *at_command2 = "HELLO, JAAVID! HOW CAN YOU SEE ME?";
    send_AT_command(at_command2);
}
