#include "driver/gpio.h"
#include "driver/gptimer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define LED_GPIO        GPIO_NUM_2
#define LED_ON_LEVEL    1

static volatile bool led_state = false;

// GPTimer 中断回调
static bool IRAM_ATTR gptimer_callback(gptimer_handle_t timer, const gptimer_alarm_event_data_t *edata, void *user_ctx)
{
    led_state = !led_state;
    return true;
}

void app_main(void)
{
    // GPIO初始化LED
    gpio_reset_pin(LED_GPIO);
    gpio_set_direction(LED_GPIO, GPIO_MODE_OUTPUT);
    gpio_set_level(LED_GPIO, !LED_ON_LEVEL);

    // 1. 创建GPTimer实例
    gptimer_handle_t gptimer = NULL;
    gptimer_config_t timer_cfg = {
        .clk_src = GPTIMER_CLK_SRC_DEFAULT,
        .direction = GPTIMER_COUNT_UP,
        .resolution_hz = 1000 * 1000, // 1MHz，1us每tick
    };
    ESP_ERROR_CHECK(gptimer_new_timer(&timer_cfg, &gptimer));

    // 2. 注册中断回调
    gptimer_event_callbacks_t cb_cfg = {
        .on_alarm = gptimer_callback,
    };
    ESP_ERROR_CHECK(gptimer_register_event_callbacks(gptimer, &cb_cfg, NULL));

    // 3. 设置闹钟，500ms触发一次
    gptimer_alarm_config_t alarm_cfg = {
        .alarm_count = 500 * 1000,   // 500 000 us = 500ms
        .reload_count = 0,
        .flags.auto_reload_on_alarm = true, // 自动重装，循环定时
    };
    ESP_ERROR_CHECK(gptimer_set_alarm_action(gptimer, &alarm_cfg));

    // 4. 启动定时器
    ESP_ERROR_CHECK(gptimer_enable(gptimer));
    ESP_ERROR_CHECK(gptimer_start(gptimer));


    // 主循环只做IO输出，中断只改标记
    while (1)
    {
        if (led_state)
        {
            gpio_set_level(LED_GPIO, LED_ON_LEVEL);
        }
        else
        {
            gpio_set_level(LED_GPIO, !LED_ON_LEVEL);
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}