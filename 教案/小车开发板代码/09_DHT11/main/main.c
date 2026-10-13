#include <stdio.h>
#include "dht11.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

/**
 * @brief   主任务：周期性读取 DHT11 温湿度并通过串口打印
 * @note    本实验仅演示 DHT11 温湿度采集，不涉及其他外设
 */
void app_main(void)
{
    /* 初始化 DHT11 */
    if (dht11_init() != 0)
    {
        printf("DHT11初始化失败，请检查接线\n");
    }
    else
    {
        printf("DHT11初始化成功，开始采集温湿度\n");
    }

    while (1)
    {
        short temp = 0, humi = 0;

        /* 读取一次温湿度，成功返回 0 */
        uint8_t ret = dht11_read_data(&temp, &humi);
        if (ret == 0)
        {
            /* 温度、湿度均为放大 10 倍后的整数，除以 10 还原成小数 */
            printf("温度: %.1f °C   湿度: %.1f %%\n", temp / 10.0, humi / 10.0);
        }
        else
        {
            printf("读取DHT11失败\n");
        }

        /* 每 2 秒采集一次 */
        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}