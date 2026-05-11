/*
 * =====================================================================
 * ESP32 Button Interrupt Example using Binary Semaphore
 * =====================================================================
 *
 * This example demonstrates:
 *
 * 1. GPIO input configuration
 * 2. Internal pull-up resistor usage
 * 3. GPIO interrupt handling
 * 4. ISR (Interrupt Service Routine)
 * 5. Binary semaphore usage
 * 6. ISR -> Task communication
 * 7. Software debounce
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
 * Why Use Semaphore?
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
 * Therefore ISR only wakes the task using semaphore.
 *
 * The task performs actual processing.
 *
 * =====================================================================
 * Debounce
 * =====================================================================
 *
 * Mechanical buttons bounce electrically.
 *
 * One physical press may generate multiple interrupts.
 *
 * We solve this using timestamp-based software debounce.
 *
 */

#include <stdio.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

#include "driver/gpio.h"

// =====================================================================
// GPIO Number
// =====================================================================

#define BUTTON_GPIO GPIO_NUM_26

// =====================================================================
// Semaphore Handle
// =====================================================================

/*
 * Binary semaphore used for:
 *
 * ISR  --->  Task signaling
 *
 * ISR gives semaphore.
 * Task takes semaphore.
 */
static SemaphoreHandle_t button_semaphore = NULL;

// =====================================================================
// Debounce Variable
// =====================================================================

/*
 * Stores last valid button press time.
 *
 * Used to ignore bouncing interrupts.
 */
static uint32_t last_press_time = 0;

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
     * Give semaphore from ISR.
     *
     * IMPORTANT:
     *
     * Use:
     *      xSemaphoreGiveFromISR()
     *
     * NOT:
     *      xSemaphoreGive()
     *
     * because normal APIs are not ISR-safe.
     */
    xSemaphoreGiveFromISR(button_semaphore,
                          &xHigherPriorityTaskWoken);

    /*
     * Request context switch if needed.
     *
     * If higher priority task woke up,
     * scheduler immediately switches to it.
     */
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

// =====================================================================
// Button Processing Task
// =====================================================================

void button_task(void *arg)
{
    while (1)
    {
        /*
         * Wait forever until semaphore becomes available.
         *
         * Initially semaphore is EMPTY.
         *
         * Task enters BLOCKED state here and consumes
         * almost zero CPU.
         */
        if (xSemaphoreTake(button_semaphore, portMAX_DELAY))
        {
            /*
             * Get current RTOS tick count.
             */
            uint32_t now = xTaskGetTickCount();

            /*
             * Software Debounce
             *
             * Ignore interrupts occurring within 50ms.
             */
            if ((now - last_press_time) > pdMS_TO_TICKS(100))
            {
                /*
                 * Optional extra validation.
                 *
                 * Check if button is STILL pressed.
                 *
                 * Since we use pull-up:
                 *
                 * Pressed = LOW = 0
                 */
                if (gpio_get_level(BUTTON_GPIO) == 0)
                {
                    printf("VALID BUTTON PRESS\n");

                    /*
                     * Store current time as last valid press.
                     */
                    last_press_time = now;
                }
            }
            else
            {
                printf("BOUNCE IGNORED\n");
            }
        }
    }
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
    // Create Binary Semaphore
    // =================================================================

    /*
     * Initially semaphore is EMPTY.
     *
     * Task will block until ISR gives semaphore.
     */
    button_semaphore = xSemaphoreCreateBinary();

    // =================================================================
    // Create Button Task
    // =================================================================

    xTaskCreate(button_task,     // Task function
                "button_task",   // Task name
                2048,            // Stack size
                NULL,            // Parameters
                10,              // Priority
                NULL);           // Task handle

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

    gpio_isr_handler_add(BUTTON_GPIO,   // GPIO number
                         gpio_isr_handler, // ISR function
                         NULL);            // ISR argument

    printf("Waiting for button press...\n");
}