/*
 * =====================================================================
 * ESP32 Button Interrupt Example using FreeRTOS Software Timer
 * =====================================================================
 *
 * This example demonstrates:
 *
 * 1. GPIO input configuration
 * 2. Internal pull-up resistor usage
 * 3. GPIO interrupt handling
 * 4. ISR (Interrupt Service Routine)
 * 5. FreeRTOS Software Timer usage
 * 6. ISR -> Timer -> Callback communication
 * 7. Software debounce
 *
 * =====================================================================
 * IMPORTANT CONCEPT
 * =====================================================================
 *
 * Timer is NOT directly replacing Queue/Semaphore/EventGroup
 * as a signaling mechanism.
 *
 * Instead:
 *
 * ISR starts/restarts a software timer.
 *
 * When timer expires:
 *      Timer callback function executes.
 *
 * This is commonly used for:
 *
 *      - debounce handling
 *      - delayed processing
 *      - retry mechanisms
 *      - timeout systems
 *
 * =====================================================================
 * Hardware Connection
 * =====================================================================
 *
 * GPIO26 -------- BUTTON -------- GND
 *
 * Internal pull-up resistor is enabled.
 *
 * Therefore:
 *
 * Button NOT pressed:
 *      GPIO = HIGH
 *
 * Button PRESSED:
 *      GPIO = LOW
 *
 * Interrupt Type:
 *      HIGH -> LOW
 *      Falling Edge
 *      GPIO_INTR_NEGEDGE
 *
 * =====================================================================
 * Why Use Timer for Debounce?
 * =====================================================================
 *
 * Mechanical buttons bounce electrically.
 *
 * One physical press may generate multiple interrupts.
 *
 * Instead of processing immediately:
 *
 * ISR starts a timer.
 *
 * Every bounce restarts the timer again.
 *
 * Only after signal becomes stable for 100ms:
 *      timer callback executes.
 *
 * This is one of the BEST professional debounce methods.
 *
 * =====================================================================
 * Why ISR Should Be Short?
 * =====================================================================
 *
 * ISR should execute very quickly.
 *
 * ISR should NOT:
 *      - printf()
 *      - malloc()
 *      - vTaskDelay()
 *      - heavy processing
 *
 * Therefore ISR only restarts software timer.
 *
 * The timer callback performs actual processing.
 *
 */

#include <stdio.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/timers.h"

#include "driver/gpio.h"

// =====================================================================
// GPIO Number
// =====================================================================

#define BUTTON_GPIO GPIO_NUM_26

// =====================================================================
// Software Timer Handle
// =====================================================================

/*
 * Handle for debounce timer.
 *
 * ISR restarts this timer whenever interrupt occurs.
 */
static TimerHandle_t debounce_timer;

// =====================================================================
// Timer Callback Function
// =====================================================================

/*
 * This function executes when timer expires.
 *
 * Timer expires only if:
 *
 * no new interrupt occurs within 100ms.
 *
 * Therefore signal is now stable.
 */
void debounce_timer_callback(TimerHandle_t xTimer)
{
    /*
     * Validate button state.
     *
     * Since pull-up is enabled:
     *
     * Pressed = LOW = 0
     */
    if (gpio_get_level(BUTTON_GPIO) == 0)
    {
        printf("VALID BUTTON PRESS\n");
    }
}

// =====================================================================
// GPIO Interrupt Service Routine (ISR)
// =====================================================================

/*
 * IRAM_ATTR:
 *
 * Places ISR inside internal RAM.
 *
 * Important because interrupts must work even when
 * external flash is busy/unavailable.
 */
static void IRAM_ATTR gpio_isr_handler(void *arg)
{
    /*
     * Required by FreeRTOS.
     *
     * If ISR wakes a higher priority task,
     * scheduler may immediately switch to it.
     */
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    /*
     * Restart timer from ISR.
     *
     * IMPORTANT:
     *
     * Use:
     *      xTimerResetFromISR()
     *
     * NOT:
     *      xTimerReset()
     *
     * because ISR-safe APIs are required.
     *
     * Every bounce restarts timer again.
     */
    xTimerResetFromISR(debounce_timer,
                       &xHigherPriorityTaskWoken);

    /*
     * Request context switch if needed.
     */
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

// =====================================================================
// Main Application
// =====================================================================

void app_main(void)
{
    // =================================================================
    // GPIO Configuration Structure
    // =================================================================

    gpio_config_t io_conf = {

        /*
         * Select GPIO pin using bit mask.
         *
         * 1ULL:
         *      unsigned long long (64-bit)
         *
         * Shift bit to GPIO26 position.
         */
        .pin_bit_mask = (1ULL << BUTTON_GPIO),

        /*
         * Configure GPIO as INPUT.
         */
        .mode = GPIO_MODE_INPUT,

        /*
         * Enable internal pull-up resistor.
         *
         * Default GPIO state becomes HIGH.
         */
        .pull_up_en = GPIO_PULLUP_ENABLE,

        /*
         * Disable pull-down resistor.
         */
        .pull_down_en = GPIO_PULLDOWN_DISABLE,

        /*
         * Interrupt Type:
         *
         * Trigger interrupt on:
         *
         * HIGH -> LOW
         *
         * Called:
         *      Falling Edge
         *      Negative Edge
         */
        .intr_type = GPIO_INTR_NEGEDGE
    };

    // Apply GPIO configuration
    gpio_config(&io_conf);

    // =================================================================
    // Create Software Timer
    // =================================================================

    /*
     * Timer Parameters:
     *
     * "debounce_timer"
     *      Timer name
     *
     * pdMS_TO_TICKS(100)
     *      Timer period = 100ms
     *
     * pdFALSE
     *      One-shot timer
     *
     * NULL
     *      Timer ID
     *
     * debounce_timer_callback
     *      Function called when timer expires
     */
    debounce_timer = xTimerCreate("debounce_timer",
                                  pdMS_TO_TICKS(100),
                                  pdFALSE,
                                  NULL,
                                  debounce_timer_callback);

    // =================================================================
    // Install GPIO ISR Service
    // =================================================================

    /*
     * Installs global GPIO interrupt handler service.
     */
    gpio_install_isr_service(0);

    // =================================================================
    // Attach ISR Handler to GPIO
    // =================================================================

    gpio_isr_handler_add(BUTTON_GPIO,      // GPIO number
                         gpio_isr_handler, // ISR function
                         NULL);            // ISR argument

    printf("Waiting for button press...\n");
}