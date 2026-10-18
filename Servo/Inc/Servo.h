#ifndef SERVO_H
#define SERVO_H

#include "main.h"

#ifdef __cplusplus
extern "C" {
#endif

/* 舵机控制：信号线 PB9（TIM4_CH4），50 Hz PWM，脉宽 500~2500 μs */
void servo_init(void);

/* 设置角度 0~180 度（超出范围会被限幅） */
void servo_set_angle(float degree);

/* 在两个预设角度之间切换 */
void servo_toggle(void);

/* 当前角度 */
float servo_get_angle(void);

#ifdef __cplusplus
}
#endif

#endif // SERVO_H
