#ifndef __MOTOR_H
#define __MOTOR_H

#include "driver/gpio.h"

/* ========== L298N 电机驱动引脚（根据实际接线修改） ========== */
#define MOTOR_IN1 GPIO_NUM_47
#define MOTOR_IN2 GPIO_NUM_21
#define MOTOR_IN3 GPIO_NUM_20
#define MOTOR_IN4 GPIO_NUM_19

/* ========== L298N PWM 调速引脚（ENA/ENB） ========== */
#define MOTOR_ENA GPIO_NUM_17 /* 通道 A 使能（PWM 调速） */
#define MOTOR_ENB GPIO_NUM_16 /* 通道 B 使能（PWM 调速） */

/* 电机动作方向定义 */
typedef enum
{
    MOTOR_STOP = 0,    /* 停止 */
    MOTOR_FORWARD = 1, /* 前进 */
    MOTOR_BACK = 2,    /* 后退 */
    MOTOR_LEFT = 3,    /* 左转 */
    MOTOR_RIGHT = 4,   /* 右转 */
} motor_dir_t;

/* 函数声明 */
void motor_init(void);           /* 初始化电机控制引脚为推挽输出 */
void motor_run(motor_dir_t dir); /* 控制电机按指定方向动作 */
void motor_set_speed(int percent); /* 设置电机 PWM 速度（0~100） */

#endif
