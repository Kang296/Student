#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define LED_GPIO1 GPIO_NUM_2
#define LED_GPIO2 GPIO_NUM_42
#define LED_GPIO3 GPIO_NUM_41
#define LED_ON_LEVEL 1

void app_main(void)
{
    gpio_reset_pin(LED_GPIO1);
    gpio_reset_pin(LED_GPIO2);
    gpio_reset_pin(LED_GPIO3);
    gpio_set_direction(LED_GPIO1, GPIO_MODE_OUTPUT);
    gpio_set_direction(LED_GPIO2, GPIO_MODE_OUTPUT);
    gpio_set_direction(LED_GPIO3, GPIO_MODE_OUTPUT);

    while (1) {
        gpio_set_level(LED_GPIO1, LED_ON_LEVEL);
        gpio_set_level(LED_GPIO2, LED_ON_LEVEL);
        gpio_set_level(LED_GPIO3, LED_ON_LEVEL);
         vTaskDelay(pdMS_TO_TICKS(500));

        gpio_set_level(LED_GPIO1, !LED_ON_LEVEL);
        gpio_set_level(LED_GPIO2, !LED_ON_LEVEL);
        gpio_set_level(LED_GPIO3, !LED_ON_LEVEL);
        vTaskDelay(pdMS_TO_TICKS(500));
     }
}