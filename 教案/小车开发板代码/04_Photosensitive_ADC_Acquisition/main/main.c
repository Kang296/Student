#include <stdio.h>
#include "driver/gpio.h"
#include "esp_adc/adc_oneshot.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

/**
 * @brief   光敏传感器（光照强度）ADC 采集实验
 * @note    光敏传感器信号接至 ADC1 通道0。
 *          在 ESP32-S3 上，ADC1_CH0 对应 GPIO1，请按实际接线确认。
 */
#define LIGHT_ADC_CHANNEL ADC_CHANNEL_0
#define LED_GPIO GPIO_NUM_2
#define GPIO_LEVEL 1

void app_main(void)
{
    // LED初始化
    gpio_reset_pin(LED_GPIO);
    gpio_set_direction(LED_GPIO, GPIO_MODE_OUTPUT);

    /* ========== ADC（光敏）初始化 ========== */
    adc_oneshot_unit_handle_t adc_handle = NULL;

    adc_oneshot_unit_init_cfg_t init_cfg = {
        .unit_id = ADC_UNIT_1,
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_cfg, &adc_handle));

    adc_oneshot_chan_cfg_t chan_cfg = {
        .atten = ADC_ATTEN_DB_12,   /* 量程 0~3.3V，最灵敏 */
        .bitwidth = ADC_BITWIDTH_12, /* 12 位分辨率，0~4095 */
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc_handle, LIGHT_ADC_CHANNEL, &chan_cfg));

    printf("光敏传感器 ADC 采集实验已启动\n");

    while (1)
    {
        int light_raw = 0;
        uint32_t light_mv = 0;

        /* 读取一次光敏 ADC 原始值 */
        if (adc_oneshot_read(adc_handle, LIGHT_ADC_CHANNEL, &light_raw) == ESP_OK)
        {
            /* 由原始值换算电压：raw * 3300 / 4095（mV） */
            light_mv = (uint32_t)light_raw * 3300 / 4095;
            printf("光照强度 - ADC_raw: %d, 电压: %lu mV\n", light_raw, (unsigned long)light_mv);
        }
        else
        {
            printf("读取光敏 ADC 失败\n");
        }
        if(light_raw <= 1000)
        {
            gpio_set_level(LED_GPIO,GPIO_LEVEL);     
            printf("LED:TURN ON\n");
        }
        else
        {
             gpio_set_level(LED_GPIO,!GPIO_LEVEL);
            printf("LED:TURN OFF\n");
        }

        /* 每 1 秒采集一次 */
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}