#include <stdio.h>
#include "motor.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

/**
 * @brief   L298N 电机驱动控制实验
 * @note    先演示 PWM 调速（前进 0%→100%→0%），
 *          再依次演示：前进 → 后退 → 左转 → 右转 → 停止
 *          每个动作持续 3 秒，动作之间停止 1 秒，便于观察。
 */
void app_main(void)
{
    motor_init();
    printf("========== L298N 电机驱动控制实验开始 ==========\n");

    while (1)
    {
        /* ---- 前进：20秒 0% → 100% ---- */
        motor_run(MOTOR_FORWARD);
        printf("[调速] 开始20s升速 0%% →100%%\n");
        for (int speed = 0; speed <= 100; speed += 1)
        {
            motor_set_speed(speed);
            printf("[调速] 前进 速度 %d%%\n", speed);
            vTaskDelay(pdMS_TO_TICKS(200));
        }
        printf("[调速] 到达100%%\n");

        /* ---- 前进：20秒 100% → 0% ---- */
        printf("[调速] 开始20s降速 100%% →0%%\n");
        for (int speed = 100; speed >= 0; speed -= 1)
        {
            motor_set_speed(speed);
            printf("[调速] 前进 速度 %d%%\n", speed);
            vTaskDelay(pdMS_TO_TICKS(200));
        }
        printf("[调速] 降到0%%\n");

        motor_run(MOTOR_STOP);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
