#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "driver/gpio.h"

// 更改UART_RECV_MODE的值来开启不同的功能（值 0 ， 1）  需要短接16，17引脚
//  0 = 查询(轮询)接收方式 代码循环主动去问串口硬件有没有收到数据
//  1 = 中断+队列接收方式  串口收到 1 字节数据，立刻触发 UART 硬件中断；CPU 立刻停下当前工作去接收数据

#define UART_RECV_MODE 1

#define UART_NUM UART_NUM_2
#define UART_TX_PIN (GPIO_NUM_16)
#define UART_RX_PIN (GPIO_NUM_17)
#define UART_BUF_SIZE (1024)
#define UART_BAUD_RATE (115200)

static QueueHandle_t uart_rx_queue;
// 初始化UART(串口)
static void uart_init(void)
{
    uart_config_t uart_config = {
        .baud_rate = UART_BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    ESP_ERROR_CHECK(uart_param_config(UART_NUM, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(UART_NUM, UART_TX_PIN, UART_RX_PIN,
                                 UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    gpio_set_pull_mode(UART_RX_PIN, GPIO_PULLUP_ONLY);

#if (UART_RECV_MODE == 1)
    ESP_ERROR_CHECK(uart_driver_install(UART_NUM, UART_BUF_SIZE, UART_BUF_SIZE,
                                        10, &uart_rx_queue, 0));
#else
    ESP_ERROR_CHECK(uart_driver_install(UART_NUM, UART_BUF_SIZE, UART_BUF_SIZE,
                                        0, NULL, 0));
#endif
}

// 轮询模式
static void uart_poll_rx_task(void *arg)
{
    uint8_t data[UART_BUF_SIZE];
    while (1)
    {
        int len = uart_read_bytes(UART_NUM, data, sizeof(data), 0);
        if (len > 0)
        {
            printf("[查询方式]收到数据,长度=%d\r\n", len);
        }
        else
        {
            printf("正在轮询查询串口\r\n");
        }
        vTaskDelay(pdMS_TO_TICKS(200));
    }
}

// 中断
static void uart_rx_task(void *arg)
{
    uint8_t data[UART_BUF_SIZE];
    uart_event_t event;
    while (1)
    {
        // 永久阻塞
        if (xQueueReceive(uart_rx_queue, &event, portMAX_DELAY))
        {
            switch (event.type)
            {
            case UART_DATA:
            {
                int len = uart_read_bytes(UART_NUM, data, sizeof(data),
                                          pdMS_TO_TICKS(20));
                if (len > 0)
                {
                    printf("[中断方式]收到中断！数据长度 = %d\r\n", len);
                }
                break;
            }
            case UART_FIFO_OVF:
                uart_flush_input(UART_NUM);
                xQueueReset(uart_rx_queue);
                break;
            case UART_BUFFER_FULL:
                uart_flush_input(UART_NUM);
                xQueueReset(uart_rx_queue);
                break;
            default:
                break;
            }
        }
    }
}

void app_main(void)
{
    uart_init();

#if (UART_RECV_MODE == 1)
    xTaskCreate(uart_rx_task, "uart_rx_task", 4096, NULL, 10, NULL);
#else
    xTaskCreate(uart_poll_rx_task, "uart_poll_rx_task", 4096, NULL, 10, NULL);
#endif

    // 每隔1秒发送一包测试数据
    while (1)
    {
        uart_write_bytes(UART_NUM, "test\r\n", strlen("test\r\n"));
        printf("UART2 已经发送 test\r\n");
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
