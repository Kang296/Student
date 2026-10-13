#include <stdio.h>
#include "motor.h"
#include "driver/ledc.h"

/* ========== PWM 调速参数 ========== */
#define MOTOR_PWM_MODE  LEDC_LOW_SPEED_MODE /* 低速模式 */
#define MOTOR_PWM_TIMER LEDC_TIMER_0        /* 定时器 */
#define MOTOR_PWM_CH_A  LEDC_CHANNEL_0      /* ENA 通道 */
#define MOTOR_PWM_CH_B  LEDC_CHANNEL_1      /* ENB 通道 */
#define MOTOR_PWM_RES   LEDC_TIMER_8_BIT    /* 8 位分辨率（0~255） */
#define MOTOR_PWM_FREQ  1000                /* 1kHz */

/**
 * @brief       初始化电机控制引脚为推挽输出，并初始化 ENA/ENB 的 PWM
 * @retval      无
 */
void motor_init(void)
{
    gpio_config_t io_conf = {
        .intr_type = GPIO_INTR_DISABLE,
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = (1ULL << MOTOR_IN1) |
                        (1ULL << MOTOR_IN2) |
                        (1ULL << MOTOR_IN3) |
                        (1ULL << MOTOR_IN4),
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .pull_up_en = GPIO_PULLUP_DISABLE,
    };
    gpio_config(&io_conf);

    /* 初始全部拉低 → 电机停止 */
    gpio_set_level(MOTOR_IN1, 0);
    gpio_set_level(MOTOR_IN2, 0);
    gpio_set_level(MOTOR_IN3, 0);
    gpio_set_level(MOTOR_IN4, 0);

    /* 初始化 PWM 定时器（ENA/ENB 调速共用） */
    ledc_timer_config_t timer_conf = {
        .speed_mode = MOTOR_PWM_MODE,
        .timer_num = MOTOR_PWM_TIMER,
        .duty_resolution = MOTOR_PWM_RES,
        .freq_hz = MOTOR_PWM_FREQ,
        .clk_cfg = LEDC_AUTO_CLK,
    };
    ledc_timer_config(&timer_conf);

    /* 通道 A → ENA */
    ledc_channel_config_t ch_a = {
        .speed_mode = MOTOR_PWM_MODE,
        .channel = MOTOR_PWM_CH_A,
        .timer_sel = MOTOR_PWM_TIMER,
        .intr_type = LEDC_INTR_DISABLE,
        .gpio_num = MOTOR_ENA,
        .duty = 0,
        .hpoint = 0,
    };
    ledc_channel_config(&ch_a);

    /* 通道 B → ENB */
    ledc_channel_config_t ch_b = {
        .speed_mode = MOTOR_PWM_MODE,
        .channel = MOTOR_PWM_CH_B,
        .timer_sel = MOTOR_PWM_TIMER,
        .intr_type = LEDC_INTR_DISABLE,
        .gpio_num = MOTOR_ENB,
        .duty = 0,
        .hpoint = 0,
    };
    ledc_channel_config(&ch_b);

    printf("电机驱动 GPIO + PWM 初始化完成\n");
}

/**
 * @brief       通过设置 L298N 的 IN1~IN4 电平控制电机动作方向
 * @param       dir: 动作方向 MOTOR_STOP/FORWARD/BACK/LEFT/RIGHT
 * @retval      无
 *
 * L298N 连接方式（以 小车电机通道 A、B 为例）：
 *   IN1/IN2 控制通道 A，IN3/IN4 控制通道 B。
 *   两通道同时正转 → 前进；同时反转 → 后退；差分 → 左转/右转。
 */
void motor_run(motor_dir_t dir)
{
    switch (dir)
    {
    case MOTOR_FORWARD: /* 前进 */
        gpio_set_level(MOTOR_IN1, 1);
        gpio_set_level(MOTOR_IN2, 0);
        gpio_set_level(MOTOR_IN3, 1);
        gpio_set_level(MOTOR_IN4, 0);
        printf("[电机] 前进\n");
        break;

    case MOTOR_BACK: /* 后退 */
        gpio_set_level(MOTOR_IN1, 0);
        gpio_set_level(MOTOR_IN2, 1);
        gpio_set_level(MOTOR_IN3, 0);
        gpio_set_level(MOTOR_IN4, 1);
        printf("[电机] 后退\n");
        break;

    case MOTOR_LEFT: /* 左转 */
        gpio_set_level(MOTOR_IN1, 1);
        gpio_set_level(MOTOR_IN2, 0);
        gpio_set_level(MOTOR_IN3, 0);
        gpio_set_level(MOTOR_IN4, 1);
        printf("[电机] 左转\n");
        break;

    case MOTOR_RIGHT: /* 右转 */
        gpio_set_level(MOTOR_IN1, 0);
        gpio_set_level(MOTOR_IN2, 1);
        gpio_set_level(MOTOR_IN3, 1);
        gpio_set_level(MOTOR_IN4, 0);
        printf("[电机] 右转\n");
        break;

    case MOTOR_STOP: /* 停止 */
    default:
        gpio_set_level(MOTOR_IN1, 0);
        gpio_set_level(MOTOR_IN2, 0);
        gpio_set_level(MOTOR_IN3, 0);
        gpio_set_level(MOTOR_IN4, 0);
        printf("[电机] 停止\n");
        break;
    }
}

/**
 * @brief       通过 PWM 设置电机转速（同时作用于 ENA、ENB）
 * @param       percent: 速度百分比 0~100，0 为停止，100 为全速
 * @retval      无
 */
void motor_set_speed(int percent)
{
    if (percent < 0)
        percent = 0;
    if (percent > 100)
        percent = 100;

    /* 8 位分辨率：占空比 0~255 */
    uint32_t duty = (uint32_t)(percent * 255 / 100);

    ledc_set_duty(MOTOR_PWM_MODE, MOTOR_PWM_CH_A, duty);
    ledc_update_duty(MOTOR_PWM_MODE, MOTOR_PWM_CH_A);

    ledc_set_duty(MOTOR_PWM_MODE, MOTOR_PWM_CH_B, duty);
    ledc_update_duty(MOTOR_PWM_MODE, MOTOR_PWM_CH_B);
}
