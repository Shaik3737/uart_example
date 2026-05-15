    #include <stdio.h>
    #include "freertos/FreeRTOS.h"
    #include "freertos/task.h"
    #include "driver/gpio.h"

    #define BUTTON GPIO_NUM_26
    uint32_t last_btn_pressed_time = 0;

    TaskHandle_t button_task_handle = NULL;

    static void IRAM_ATTR button_isr_handler(void* arg) {
        uint32_t current_time = xTaskGetTickCountFromISR();
        if (current_time - last_btn_pressed_time < pdMS_TO_TICKS(200)) 
        { // Debounce for 200ms
            return;
        }
        last_btn_pressed_time = current_time;
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        vTaskNotifyGiveFromISR(button_task_handle, &xHigherPriorityTaskWoken);
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
            
    }

    void button_task(void* pvParameter) 
    {
        uint32_t counter=0;
        while (1) 
        {
            if(ulTaskNotifyTake(pdFALSE, pdMS_TO_TICKS(500)))// Wait for notification
            {
                counter++;
                printf("Button Pressed %ld times!\n", counter);
            }
            else
            {
                if(counter==1)
                {
                    printf("SINGLE TIME BUTTON PRESS DETECTED!\n");
                }
                else if(counter==2)
                {
                    printf("DOUBLE TIME BUTTON PRESS DETECTED!\n");
                }
                else if(counter >=3)
                {
                    printf("TRIPLE TIME BUTTON PRESS DETECTED!\n");
                }
                counter=0; // Reset counter after processing
            }

        }
    }

    void app_main() 
    {
        gpio_config_t io_conf = {
            .intr_type = GPIO_INTR_NEGEDGE, // Interrupt on falling edge
            .mode = GPIO_MODE_INPUT,        // Set as input mode
            .pin_bit_mask = (1ULL << BUTTON), // Bit mask for the button pin
            .pull_down_en = 0,              // Disable pull-down
            .pull_up_en = 1                 // Enable pull-up
        };
        gpio_config(&io_conf); // Configure the GPIO with the specified settings

        xTaskCreate(button_task, "button_task", 2048, NULL, 10, &button_task_handle); // Create the button task

        gpio_install_isr_service(0); // Install ISR service with default configuration
        gpio_isr_handler_add(BUTTON, button_isr_handler, NULL); // Add ISR handler for the button pin
    }