/**
 * @file    button_led.c
 * @brief   Multi-press button detection with immediate LED pattern switching on ESP32
 *
 * HARDWARE:
 *   - Button: GPIO 26, active low (pulled high internally, pressed = GND)
 *   - LED:    GPIO 27, active high
 *
 * BEHAVIOR:
 *   - Single press → LED blinks continuously (500ms on, 500ms off)
 *   - Double press → LED double-blinks continuously (two fast blinks, long pause)
 *   - Triple press → LED triple-blinks continuously (three fast blinks, long pause)
 *   - Pattern switches IMMEDIATELY when a new press is detected, without
 *     waiting for the current blink cycle to finish.
 *
 * ARCHITECTURE — WHY TWO TASKS?
 *   Splitting button detection and LED control into separate tasks keeps each
 *   unit of work focused and independent. The ISR stays minimal (hardware rule),
 *   the button task owns all press-counting logic, and the LED task owns all
 *   blink logic. Neither task blocks the other.
 *
 *   ISR  ──(vTaskNotifyGiveFromISR)──▶  button_task  ──(xTaskNotifyGive)──▶  led_task
 *
 * AUTHOR: (your name)
 * DATE:   (date)
 */

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "driver/gpio.h"

/* ─── Pin Definitions ──────────────────────────────────────────────────────── */
#define BUTTON_GPIO     GPIO_NUM_26
#define LED_GPIO        GPIO_NUM_27

/* ─── Timing Constants ─────────────────────────────────────────────────────── */
/**
 * DEBOUNCE_MS — Why 200ms?
 * Mechanical buttons produce multiple electrical edges (bounces) within the
 * first ~50ms of a single physical press. 200ms is safely above that noise
 * window while still feeling instantaneous to a human finger.
 */
#define DEBOUNCE_MS     200

/**
 * PRESS_WINDOW_MS — Why 500ms?
 * After each detected edge, we wait up to 500ms for another press before
 * deciding the press count. This is a SLIDING window — it restarts after
 * every press, so the decision only happens after 500ms of silence.
 *
 * Example: triple press at 0ms, 250ms, 400ms → decision at 900ms.
 *
 * Why 500ms and not shorter?
 * A human double-tap typically lands 150–300ms apart. 500ms gives comfortable
 * room without making the system feel sluggish.
 */
#define PRESS_WINDOW_MS 500

/* ─── Shared State ──────────────────────────────────────────────────────────── */

/**
 * last_time_button_pressed — written ONLY inside the ISR.
 * No task reads this variable, so no mutex is needed here.
 * It exists solely to implement debounce inside the ISR.
 */
static uint32_t last_time_button_pressed = 0;

/**
 * led_mode — shared between button_task (writer) and led_task (reader).
 * Protected by semaphore_handle to prevent a race condition where
 * button_task writes a new mode at the exact moment led_task is reading it.
 */
typedef enum {
    IDLE   = 0,  // startup state, LED is off
    SINGLE,      // single press detected
    DOUBLE,      // double press detected
    TRIPLE       // triple press detected
} led_mode_t;

static volatile led_mode_t led_mode = IDLE;

/* ─── RTOS Handles ──────────────────────────────────────────────────────────── */
static TaskHandle_t     buttonTaskHandle = NULL;
static TaskHandle_t     led_taskHandle   = NULL;
static SemaphoreHandle_t semaphore_handle = NULL;

/* ─── ISR ───────────────────────────────────────────────────────────────────── */

/**
 * gpio_handler_isr — fires on every falling edge (finger down).
 *
 * WHY so short?
 * ISRs run with interrupts disabled. Any delay here blocks the entire system.
 * Rule: ISR only captures the event and signals a task. All logic lives in tasks.
 *
 * WHY debounce here and not in the task?
 * Debounce must reject bounces before they ever reach the task. If we let
 * every bounce through, the task would count 5–10 "presses" per physical press.
 * Comparing timestamps inside the ISR is the earliest possible rejection point.
 *
 * WHY portYIELD_FROM_ISR?
 * vTaskNotifyGiveFromISR may unblock button_task. If button_task has higher
 * priority than the currently running task, portYIELD_FROM_ISR forces an
 * immediate context switch so button_task runs right away instead of waiting
 * for the next scheduler tick.
 */
static void IRAM_ATTR gpio_handler_isr(void *args)
{
    uint32_t now = xTaskGetTickCountFromISR();

    // Reject bounces — ignore edges that arrive within DEBOUNCE_MS of the last valid edge
    if (now - last_time_button_pressed < pdMS_TO_TICKS(DEBOUNCE_MS)) {
        return;
    }
    last_time_button_pressed = now;

    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    vTaskNotifyGiveFromISR(buttonTaskHandle, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

/* ─── Button Task ───────────────────────────────────────────────────────────── */

/**
 * button_task — counts press edges and classifies single / double / triple.
 *
 * DETECTION STRATEGY — sliding window timeout:
 *   ulTaskNotifyTake() blocks until the ISR sends a notification OR the
 *   timeout expires. This creates a self-resetting window:
 *
 *     press → 500ms window starts fresh
 *     press → 500ms window starts fresh again
 *     ...silence for 500ms → timeout fires → classify and act
 *
 *   The system only decides AFTER the user stops pressing for 500ms.
 *   This naturally handles any number of presses without a fixed time slot.
 *
 * WHY pdFALSE (not pdTRUE)?
 *   pdFALSE decrements the notification counter by 1 each call instead of
 *   clearing it to zero. This matters when two presses arrive faster than the
 *   task can process them — with pdFALSE, the second press is preserved in the
 *   counter and caught on the next ulTaskNotifyTake() call. pdTRUE would
 *   silently discard it.
 */
void button_task(void *pvParameter)
{
    uint32_t buttonPressed = 0;

    while (1) {
        if (ulTaskNotifyTake(pdFALSE, pdMS_TO_TICKS(PRESS_WINDOW_MS))) {
            // A falling edge arrived — another press in the current window
            buttonPressed++;
            printf("Button pressed %ld times\n", buttonPressed);

        } else {
            // 500ms of silence — user is done pressing, classify the count
            if (buttonPressed == 1) {
                printf("Single press detected\n");
                if (xSemaphoreTake(semaphore_handle, portMAX_DELAY)) {
                    led_mode = SINGLE;
                    xSemaphoreGive(semaphore_handle);
                }
                // Wake led_task so it reads the new mode immediately
                xTaskNotifyGive(led_taskHandle);

            } else if (buttonPressed == 2) {
                printf("Double press detected\n");
                if (xSemaphoreTake(semaphore_handle, portMAX_DELAY)) {
                    led_mode = DOUBLE;
                    xSemaphoreGive(semaphore_handle);
                }
                xTaskNotifyGive(led_taskHandle);

            } else if (buttonPressed >= 3) {
                printf("Triple press detected\n");
                if (xSemaphoreTake(semaphore_handle, portMAX_DELAY)) {
                    led_mode = TRIPLE;
                    xSemaphoreGive(semaphore_handle);
                }
                xTaskNotifyGive(led_taskHandle);
            }
            // Reset for the next press sequence regardless of which branch ran
            buttonPressed = 0;
        }
    }
}

/* ─── LED Helpers ───────────────────────────────────────────────────────────── */
static inline void led_on(void)  { gpio_set_level(LED_GPIO, 1); }
static inline void led_off(void) { gpio_set_level(LED_GPIO, 0); }

/* ─── LED Task ──────────────────────────────────────────────────────────────── */

/**
 * led_task — runs the correct blink pattern and switches immediately on demand.
 *
 * WHY xTaskNotifyWait() instead of vTaskDelay()?
 *   vTaskDelay() puts the task to sleep for a fixed time with no way to
 *   wake it early. xTaskNotifyWait() does the same sleep BUT wakes up
 *   immediately if button_task sends a notification. This is what gives
 *   us instant pattern switching — the current blink step is abandoned
 *   the moment a new press is classified.
 *
 * HOW pattern switching works:
 *   1. button_task writes new led_mode under mutex protection.
 *   2. button_task calls xTaskNotifyGive(led_taskHandle).
 *   3. led_task wakes from xTaskNotifyWait() mid-blink.
 *   4. It takes the mutex, reads the new led_mode into current_led_mode.
 *   5. break exits the switch case → while(1) re-enters with new mode.
 *
 * WHY a local copy (current_led_mode)?
 *   led_task reads led_mode only when a notification arrives, not every loop.
 *   Between notifications it uses its local copy so it never needs to hold
 *   the mutex during a blink delay (holding a mutex during a delay is bad
 *   practice — it blocks button_task from writing).
 *
 * DEFAULT case — WHY portMAX_DELAY?
 *   At startup no button has been pressed. The LED must stay off and the task
 *   must sleep without wasting CPU. portMAX_DELAY means "wait forever" — the
 *   task only wakes when the very first button press arrives.
 */
void led_task(void *pvParameter)
{
    led_mode_t current_led_mode = IDLE;

    while (1) {
        switch (current_led_mode) {

            /* ── Single: slow continuous blink ─────────────────────────────── */
            case SINGLE:
                led_on();
                if (xTaskNotifyWait(0, 0, NULL, pdMS_TO_TICKS(500))) {
                    // New press arrived mid-blink — switch pattern immediately
                    if (xSemaphoreTake(semaphore_handle, portMAX_DELAY)) {
                        current_led_mode = led_mode;
                        xSemaphoreGive(semaphore_handle);
                    }
                    led_off(); // always leave LED off when abandoning a pattern
                    break;
                }
                led_off();
                if (xTaskNotifyWait(0, 0, NULL, pdMS_TO_TICKS(500))) {
                    if (xSemaphoreTake(semaphore_handle, portMAX_DELAY)) {
                        current_led_mode = led_mode;
                        xSemaphoreGive(semaphore_handle);
                    }
                    break;
                }
                break; // no new press — loop and repeat same pattern

            /* ── Double: two fast blinks then long pause ────────────────────── */
            case DOUBLE:
                led_on();
                if (xTaskNotifyWait(0, 0, NULL, pdMS_TO_TICKS(100))) {
                    if (xSemaphoreTake(semaphore_handle, portMAX_DELAY)) {
                        current_led_mode = led_mode;
                        xSemaphoreGive(semaphore_handle);
                    }
                    led_off();
                    break;
                }
                led_off();
                if (xTaskNotifyWait(0, 0, NULL, pdMS_TO_TICKS(100))) {
                    if (xSemaphoreTake(semaphore_handle, portMAX_DELAY)) {
                        current_led_mode = led_mode;
                        xSemaphoreGive(semaphore_handle);
                    }
                    break;
                }
                led_on();
                if (xTaskNotifyWait(0, 0, NULL, pdMS_TO_TICKS(500))) {
                    if (xSemaphoreTake(semaphore_handle, portMAX_DELAY)) {
                        current_led_mode = led_mode;
                        xSemaphoreGive(semaphore_handle);
                    }
                    led_off();
                    break;
                }
                led_off();
                if (xTaskNotifyWait(0, 0, NULL, pdMS_TO_TICKS(500))) {
                    if (xSemaphoreTake(semaphore_handle, portMAX_DELAY)) {
                        current_led_mode = led_mode;
                        xSemaphoreGive(semaphore_handle);
                    }
                    break;
                }
                break;

            /* ── Triple: three fast blinks then long pause ──────────────────── */
            case TRIPLE:
                led_on();
                if (xTaskNotifyWait(0, 0, NULL, pdMS_TO_TICKS(100))) {
                    if (xSemaphoreTake(semaphore_handle, portMAX_DELAY)) {
                        current_led_mode = led_mode;
                        xSemaphoreGive(semaphore_handle);
                    }
                    led_off();
                    break;
                }
                led_off();
                if (xTaskNotifyWait(0, 0, NULL, pdMS_TO_TICKS(100))) {
                    if (xSemaphoreTake(semaphore_handle, portMAX_DELAY)) {
                        current_led_mode = led_mode;
                        xSemaphoreGive(semaphore_handle);
                    }
                    break;
                }
                led_on();
                if (xTaskNotifyWait(0, 0, NULL, pdMS_TO_TICKS(100))) {
                    if (xSemaphoreTake(semaphore_handle, portMAX_DELAY)) {
                        current_led_mode = led_mode;
                        xSemaphoreGive(semaphore_handle);
                    }
                    led_off();
                    break;
                }
                led_off();
                if (xTaskNotifyWait(0, 0, NULL, pdMS_TO_TICKS(100))) {
                    if (xSemaphoreTake(semaphore_handle, portMAX_DELAY)) {
                        current_led_mode = led_mode;
                        xSemaphoreGive(semaphore_handle);
                    }
                    break;
                }
                led_on();
                if (xTaskNotifyWait(0, 0, NULL, pdMS_TO_TICKS(100))) {
                    if (xSemaphoreTake(semaphore_handle, portMAX_DELAY)) {
                        current_led_mode = led_mode;
                        xSemaphoreGive(semaphore_handle);
                    }
                    led_off();
                    break;
                }
                led_off();
                if (xTaskNotifyWait(0, 0, NULL, pdMS_TO_TICKS(100))) {
                    if (xSemaphoreTake(semaphore_handle, portMAX_DELAY)) {
                        current_led_mode = led_mode;
                        xSemaphoreGive(semaphore_handle);
                    }
                    break;
                }
                led_on();
                if (xTaskNotifyWait(0, 0, NULL, pdMS_TO_TICKS(500))) {
                    if (xSemaphoreTake(semaphore_handle, portMAX_DELAY)) {
                        current_led_mode = led_mode;
                        xSemaphoreGive(semaphore_handle);
                    }
                    led_off();
                    break;
                }
                led_off();
                if (xTaskNotifyWait(0, 0, NULL, pdMS_TO_TICKS(500))) {
                    if (xSemaphoreTake(semaphore_handle, portMAX_DELAY)) {
                        current_led_mode = led_mode;
                        xSemaphoreGive(semaphore_handle);
                    }
                    break;
                }
                break;

            /* ── Default: startup state, LED off, wait for first press ──────── */
            default:
                led_off();
                // portMAX_DELAY — sleep forever, burn zero CPU
                // Only wakes when button_task calls xTaskNotifyGive()
                if (xTaskNotifyWait(0, 0, NULL, portMAX_DELAY)) {
                    if (xSemaphoreTake(semaphore_handle, portMAX_DELAY)) {
                        current_led_mode = led_mode;
                        xSemaphoreGive(semaphore_handle);
                    }
                    break;
                }
                break;
        }
    }
}

/* ─── Entry Point ───────────────────────────────────────────────────────────── */
void app_main(void)
{
    // Configure button GPIO — input with internal pull-up
    // Pull-up means idle state = HIGH, pressed state = LOW (active low)
    gpio_config_t io_conf = {
        .intr_type    = GPIO_INTR_NEGEDGE,         // falling edge = finger down
        .mode         = GPIO_MODE_INPUT,
        .pin_bit_mask = (1ULL << BUTTON_GPIO),
        .pull_down_en = 0,
        .pull_up_en   = 1
    };
    gpio_config(&io_conf);

    // Configure LED GPIO — output, no pull resistors needed
    gpio_config_t led_io_conf = {
        .intr_type    = GPIO_INTR_DISABLE,
        .mode         = GPIO_MODE_OUTPUT,
        .pin_bit_mask = (1ULL << LED_GPIO),
        .pull_down_en = 0,
        .pull_up_en   = 0
    };
    gpio_config(&led_io_conf);

    // Create mutex BEFORE creating tasks — tasks access it immediately on start
    semaphore_handle = xSemaphoreCreateMutex();

    xTaskCreate(button_task, "button_task", 2048, NULL, 10, &buttonTaskHandle);
    xTaskCreate(led_task,    "led_task",    2048, NULL, 10, &led_taskHandle);

    // Install ISR service AFTER task handles are valid —
    // the ISR uses buttonTaskHandle, so it must exist before any interrupt fires
    gpio_install_isr_service(0);
    gpio_isr_handler_add(BUTTON_GPIO, gpio_handler_isr, NULL);
}