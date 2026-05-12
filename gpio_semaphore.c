/*
 * =====================================================================
 * ESP32 Button Interrupt Example
 * Binary Semaphore + ISR Debounce
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
 * 7. Software debounce INSIDE ISR
 * 8. Using xTaskGetTickCountFromISR()
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
 * Why Use Binary Semaphore?
 * =====================================================================
 *
 * ISR should execute very quickly.
 *
 * ISR should NOT perform heavy processing.
 *
 * Therefore:
 *
 * ISR only:
 *      - validates debounce
 *      - wakes button task
 *
 * Button task performs actual processing.
 *
 * =====================================================================
 * Important ISR Rule
 * =====================================================================
 *
 * Many beginners misunderstand the phrase:
 *
 *      "Keep ISR short"
 *
 * "Short" does NOT mean:
 *      - one line only
 *
 * "Short" ACTUALLY means:
 *      - no blocking
 *      - no waiting
 *      - no heavy computation
 *
 * A simple timestamp comparison is extremely fast.
 *
 * Example:
 *
 *      now - last_button_press_time
 *
 * This is just a few CPU instructions.
 *
 * Completely safe inside ISR.
 *
 * =====================================================================
 * The Rule More Precisely
 * =====================================================================
 *
 * ❌ NOT allowed in ISR:
 *
 *      - vTaskDelay()
 *      - printf()
 *      - malloc()
 *      - xSemaphoreTake() with delay
 *      - any blocking/waiting function
 *      - heavy computation
 *
 * ✅ Allowed in ISR:
 *
 *      - simple math
 *      - variable read/write
 *      - comparisons
 *      - timestamp checks
 *      - xSemaphoreGiveFromISR()
 *      - xQueueSendFromISR()
 *      - return early
 *
 * Your instinct to keep ISR short was correct.
 *
 * But avoiding even lightweight logic is unnecessary.
 *
 * Timestamp debounce inside ISR is a very common and
 * professional embedded systems technique.
 *
 * =====================================================================
 * Debounce
 * =====================================================================
 *
 * Mechanical buttons bounce electrically.
 *
 * One physical press may generate:
 *
 *      LOW HIGH LOW HIGH LOW
 *
 * within a few milliseconds.
 *
 * Without debounce:
 *
 * ISR may trigger multiple times.
 *
 * We solve this using:
 *
 *      timestamp-based software debounce
 *
 * Logic:
 *
 * If interrupts occur too quickly,
 * ignore them.
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
// Binary Semaphore Handle
// =====================================================================

/*
 * Binary semaphore used for:
 *
 * ISR  --->  Task signaling
 *
 * ISR gives semaphore.
 * Task takes semaphore.
 */
SemaphoreHandle_t buttonSemaphore = NULL;

// =====================================================================
// Debounce Variable
// =====================================================================

/*
 * Stores last accepted button interrupt time.
 *
 * Used to ignore bouncing interrupts.
 */
uint32_t last_button_press_time = 0;

// =====================================================================
// GPIO Interrupt Service Routine (ISR)
// =====================================================================

/*
 * IRAM_ATTR:
 *
 * Places ISR inside internal RAM.
 *
 * Important because interrupts must work even
 * when flash memory is temporarily unavailable.
 */
static void IRAM_ATTR button_isr(void *arg)
{
    // ================================================================
    // Get Current RTOS Tick Count
    // ================================================================

    /*
     * IMPORTANT:
     *
     * Inside ISR use:
     *
     *      xTaskGetTickCountFromISR()
     *
     * NOT:
     *
     *      xTaskGetTickCount()
     *
     * because ISR-safe version is required.
     */
    uint32_t now = xTaskGetTickCountFromISR();

    // ================================================================
    // Software Debounce
    // ================================================================

    /*
     * If current interrupt occurs too soon
     * after previous interrupt:
     *
     * Ignore it.
     *
     * Example:
     *
     * Previous interrupt:
     *      1000ms
     *
     * Current interrupt:
     *      1050ms
     *
     * Difference:
     *      50ms
     *
     * Since debounce threshold is 200ms:
     *
     * Ignore interrupt.
     */
    if (now - last_button_press_time < pdMS_TO_TICKS(200))
    {
        /*
         * Return immediately.
         *
         * This interrupt is considered bounce.
         */
        return;
    }

    // ================================================================
    // Store Current Valid Interrupt Time
    // ================================================================

    /*
     * Interrupt passed debounce validation.
     *
     * Store current timestamp.
     */
    last_button_press_time = now;

    // ================================================================
    // FreeRTOS Context Switch Helper Variable
    // ================================================================

    /*
     * Used by FreeRTOS scheduler.
     *
     * If ISR wakes a higher-priority task,
     * FreeRTOS may immediately switch to it.
     *
     * Initially:
     *
     *      pdFALSE
     *
     * Meaning:
     *
     *      "No higher-priority task woke yet."
     */
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    // ================================================================
    // Give Semaphore from ISR
    // ================================================================

    /*
     * Wake button task using semaphore.
     *
     * IMPORTANT:
     *
     * Use:
     *
     *      xSemaphoreGiveFromISR()
     *
     * NOT:
     *
     *      xSemaphoreGive()
     *
     * because normal APIs are not ISR-safe.
     */
    xSemaphoreGiveFromISR(buttonSemaphore,
                          &xHigherPriorityTaskWoken);

    // ================================================================
    // Request Context Switch if Needed
    // ================================================================

    /*
     * If ISR woke a higher-priority task,
     * scheduler may immediately switch to it.
     *
     * This improves responsiveness.
     */
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

// =====================================================================
// Button Processing Task
// =====================================================================

void button_task(void *pvParameter)
{
    while (1)
    {
        // ============================================================
        // Wait Forever for Semaphore
        // ============================================================

        /*
         * Task enters BLOCKED state here.
         *
         * CPU usage becomes almost zero.
         *
         * Task wakes only when ISR gives semaphore.
         */
        if (xSemaphoreTake(buttonSemaphore,
                           portMAX_DELAY) == pdTRUE)
        {
            /*
             * Valid button press received.
             */
            printf("Button pressed!\n");
        }
    }
}

// =====================================================================
// Main Application
// =====================================================================

void app_main()
{
    // =================================================================
    // GPIO Configuration Structure
    // =================================================================

    gpio_config_t io_conf =
    {
        /*
         * Interrupt Type:
         *
         * GPIO_INTR_NEGEDGE
         *
         * Trigger interrupt on:
         *
         * HIGH -> LOW transition
         *
         * Since pull-up is enabled:
         *
         * Button press causes:
         *
         * HIGH -> LOW
         */
        .intr_type = GPIO_INTR_NEGEDGE,

        /*
         * Configure GPIO as INPUT.
         */
        .mode = GPIO_MODE_INPUT,

        /*
         * Select GPIO pin using bit mask.
         *
         * Shift bit to GPIO26 position.
         */
        .pin_bit_mask = (1ULL << BUTTON_GPIO),

        /*
         * Disable internal pull-down resistor.
         */
        .pull_down_en = 0,

        /*
         * Enable internal pull-up resistor.
         *
         * Default GPIO state becomes HIGH.
         */
        .pull_up_en = 1
    };

    // Apply GPIO configuration
    gpio_config(&io_conf);

    // =================================================================
    // Create Binary Semaphore
    // =================================================================

    /*
     * Initially semaphore is EMPTY.
     *
     * Task blocks until ISR gives semaphore.
     */
    buttonSemaphore = xSemaphoreCreateBinary();

    // =================================================================
    // Create Button Task
    // =================================================================

    xTaskCreate(button_task,     // Task function
                "button_task",   // Task name
                2048,            // Stack size
                NULL,            // Parameters
                5,               // Priority
                NULL);           // Task handle

    // =================================================================
    // Install GPIO ISR Service
    // =================================================================

    /*
     * Installs global GPIO interrupt service.
     */
    gpio_install_isr_service(0);

    // =================================================================
    // Attach ISR Handler to GPIO
    // =================================================================

    gpio_isr_handler_add(BUTTON_GPIO, // GPIO number
                         button_isr,  // ISR function
                         NULL);       // ISR argument

    printf("Waiting for button press...\n");
}