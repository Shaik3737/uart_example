/*
 * ================================================================
 * ESP32 Button Interrupt Example using Queue + FreeRTOS Task
 * ================================================================
 *
 * This program demonstrates:
 *
 * 1. Configuring a GPIO pin as button input
 * 2. Using GPIO interrupts
 * 3. Using ISR (Interrupt Service Routine)
 * 4. Sending data from ISR to a FreeRTOS task using Queue
 * 5. Software debounce handling
 *
 * ---------------------------------------------------------------
 * Hardware Connection
 * ---------------------------------------------------------------
 *
 * GPIO26 -------- BUTTON -------- GND
 *  *
 * Internal Pull-up is enabled.
 *
 * Therefore:
 *
 * Button NOT pressed:
 *      GPIO reads HIGH (1)
 *
 * Button pressed:
 *      GPIO reads LOW (0)
 *
 * Therefore interrupt type:
 *      HIGH -> LOW
 *      Falling Edge
 *      GPIO_INTR_NEGEDGE
 *
 * ---------------------------------------------------------------
 * Why Queue?
 * ---------------------------------------------------------------
 *
 * ISR should be extremely short.
 *
 * ISR should NOT:
 *      - print
 *      - allocate memory
 *      - delay
 *      - use mutex
 *      - do heavy processing
 *
 * Therefore ISR only sends an event to queue.
 *
 * The task performs the actual processing.
 *
 * ---------------------------------------------------------------
 * Debounce
 * ---------------------------------------------------------------
 *
 * Mechanical buttons bounce electrically.
 *
 * One physical press may generate:
 *      multiple interrupts.
 *
 * We solve this using timestamp-based software debounce.
 *
 */

#include <stdio.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "driver/gpio.h"

// ================================================================
// GPIO Number
// ================================================================

#define BUTTON_GPIO GPIO_NUM_26

// ================================================================
// Debounce Variable
// ================================================================

/*
 * Stores last valid button press time.
 *
 * Used to ignore bouncing interrupts.
 */
static uint32_t last_press_time = 0;

// ================================================================
// Queue Handle
// ================================================================

/*
 * Queue is used for communication between:
 *
 * ISR  --->  FreeRTOS Task
 *
 * ISR sends GPIO number.
 * Task receives GPIO number.
 */
static QueueHandle_t gpio_evt_queue = NULL;

// ================================================================
// GPIO Interrupt Service Routine (ISR)
// ================================================================

/*
 * IRAM_ATTR:
 *
 * Places ISR in internal RAM.
 *
 * Why?
 *
 * During flash operations, external flash may become unavailable.
 * ISRs must still work.
 *
 * Therefore critical ISR code is stored in IRAM.
 */
static void IRAM_ATTR gpio_isr_handler(void *arg)
{
    /*
     * arg contains the GPIO number.
     *
     * We passed this argument when registering ISR.
     */
    uint32_t gpio_num = (uint32_t)arg;

    /*
     * Send GPIO number to queue.
     *
     * IMPORTANT:
     *
     * We must use:
     *      xQueueSendFromISR()
     *
     * NOT:
     *      xQueueSend()
     *
     * Because ISR context is special.
     */
    xQueueSendFromISR(gpio_evt_queue, &gpio_num, NULL);
}
// ================================================================
// Button Processing Task
// ================================================================
void button_task(void *arg)
{
    uint32_t io_num;
    while (1)
    {
    /*
        * Wait forever for queue event.
        *
        * portMAX_DELAY means:
        *      block indefinitely.
        *  *
        * Task consumes almost zero CPU while blocked.
        */
        if (xQueueReceive(gpio_evt_queue, &io_num, portMAX_DELAY))
        {
            /*
                        * Current RTOS tick count.
                        */
            uint32_t now = xTaskGetTickCount();
            /*
                        * Debounce Check
                        *
                        * Ignore interrupts occurring within 50ms.
                        */
            if ((now-last_press_time) > pdMS_TO_TICKS(50))
            {
                printf("VALID BUTTON PRESS\n");
                /*
                                * Update last valid press time.
                                */
                last_press_time = now;
            }
            else
            {
                printf("BOUNCE IGNORED\n");
            }
        }
        
    }
}
// ================================================================
// Main Application
// ================================================================

void app_main(void)
{
    // ============================================================
    // GPIO Configuration Structure
    // ============================================================
    gpio_config_t io_conf = {
    /*
            * Bit mask for GPIO selection.
                *
            * 1ULL means:
            *      unsigned long long (64-bit)
            *
            * Shift 1 to GPIO26 position.
            */
            .pin_bit_mask = (1ULL << BUTTON_GPIO),
            /*
                    * GPIO configured as INPUT.
                    */
            .mode = GPIO_MODE_INPUT,
            /*
                    * Enable internal pull-up resistor.
                    *
                    * Default state becomes HIGH.
                    */
            .pull_up_en = GPIO_PULLUP_ENABLE,
            /*
            * Pull-down disabled.
            */
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            /*
                    * Interrupt type.
                    *
                    * Trigger interrupt when signal transitions:
                    *      HIGH -> LOW
                    *
                    * Called Falling Edge / Negative Edge.
                    */
            .intr_type = GPIO_INTR_NEGEDGE
    };
    // Apply GPIO configuration
    gpio_config(&io_conf);

    // ============================================================
    // Create Queue
    // ============================================================
    /*
        * Queue length:
        *      10 elements
        *
        * Each element size:
        *      
    */
    gpio_evt_queue = xQueueCreate(10, sizeof(uint32_t));
    // ============================================================
    // Create Task
    // ============================================================

    xTaskCreate(button_task, "button_task", 2048, NULL, 10, NULL);

    // ============================================================
    // Install GPIO ISR Service
    // ============================================================
    gpio_install_isr_service(0);

    // ============================================================
    // Attach ISR Handler
    // ============================================================

    gpio_isr_handler_add(BUTTON_GPIO, gpio_isr_handler, (void *)BUTTON_GPIO);
    printf("Waiting for button press...\n");
}