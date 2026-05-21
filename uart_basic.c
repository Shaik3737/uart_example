#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/gpio.h"
#include "driver/uart.h"

#define UART_NUM UART_NUM_1
#define BUF_SIZE (1024)
#define TXD_PIN     17
#define RXD_PIN     16
QueueHandle_t uart_queue;

void uart_init(void)
{
    const uart_config_t uartConfig = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_APB, 
    };
    uart_driver_install(UART_NUM, BUF_SIZE, BUF_SIZE, 10, &uart_queue, 0);
    uart_param_config(UART_NUM, &uartConfig);
    uart_set_pin(UART_NUM, TXD_PIN, RXD_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
}

void send_AT_command(const char* command)
{
    char buffer[128]={0};
    uint8_t len = strlen(command);
    if(len >0 && command[len-2] =='\r' && command[len-1] =='\n')
    {
        snprintf(buffer, sizeof(buffer), "%s", command);
        uart_write_bytes(UART_NUM, buffer, strlen(buffer));
    }
    else if(len >0 && command[len-2] !='\r' && command[len-1] !='\n')
    {
        snprintf(buffer, sizeof(buffer), "%s\r\n", command);
        uart_write_bytes(UART_NUM, buffer, strlen(buffer));
    }
    else
    {
        printf("Invalid AT command format. Command should end with \\r\\n.\n");
    }
}

void uart_receive_task(void *pvParameter)
{
    uart_event_t event;
    uint8_t buf[1024];
    while(1)
    {
        if(xQueueReceive(uart_queue, (void *)&event, portMAX_DELAY))  //wait for ever if queue is empty
        {
            if(event.type == UART_DATA)
            {
                int len = uart_read_bytes(UART_NUM, buf, event.size, pdMS_TO_TICKS(100)); //wait for 100ms to receive data, if no data received during this period, return what has been received so far
                if (len > 0) 
                {
                    buf[len] = '\0';  // null-terminate so printf works
                    printf("Received data len: %d\n", len);
                    printf("Received data: %s\n", buf);
                }
            }
        }
    }
}
void app_main(void)
{
    uart_init();
    const char* at_command = "AT+GMR";
    xTaskCreate(uart_receive_task, "uart_receive_task", 4096, NULL, 10, NULL);
    send_AT_command(at_command);
    const char* at_command2 = "HELLO, JAAVID! HOW CAN YOU SEE ME?";
    send_AT_command(at_command2);
}