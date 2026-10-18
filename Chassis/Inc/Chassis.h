#ifndef CHASSIS_H
#define CHASSIS_H

#ifdef __cplusplus
extern "C" {
#endif
#include "main.h"
#include "stm32f1xx_hal_gpio.h"
#include "stm32f1xx_hal_tim.h"

void chassis_init();
/* 差速控制（保留备用）：v = 线速度，turn = 转向量 */
void update(float v, float turn);
/* 麦轮控制：vx = 前进(+)/后退(-)，vy = 右移(+)/左移(-)，w = 自转(+逆时针) */
void chassis_move(float vx, float vy, float w);

#ifdef __cplusplus
}
#endif

#ifdef __cplusplus

/* 单路直流电机：IN1 接定时器 PWM 通道，IN2 接普通 GPIO。
 *   正转：IN1 输出占空比，IN2 = 低
 *   反转：IN1 输出占空比，IN2 = 高
 *   停止：IN1 占空比 0，   IN2 = 低
 * 对应板上的 TB6612 类驱动（IN1/IN2 + STBY）。 */
class Motor {
public:
  Motor() = default;

  Motor(TIM_HandleTypeDef *htim, uint32_t channel, GPIO_TypeDef *in2Port,
        uint16_t in2Pin)
      : htim_(htim), channel_(channel), in2Port_(in2Port), in2Pin_(in2Pin) {}

  /* 某侧轮子实际转向与预期相反时置 true，无需改接线。
   * 注意：原理图上右侧电机 O1/O2 在排针上的顺序与左侧相反，
   * 若 FR/BR 反转方向不对，把对应电机 set_reversed(true) 即可。 */
  void set_reversed(bool reversed) { reversed_ = reversed; }

  /* speed 取值 -PWM_PERIOD ~ +PWM_PERIOD，正为前进 */
  void set_speed(int16_t speed);

private:
  TIM_HandleTypeDef *htim_ = nullptr;
  uint32_t channel_ = 0;
  GPIO_TypeDef *in2Port_ = nullptr;
  uint16_t in2Pin_ = 0;
  bool reversed_ = false;
};

class ChassisCtrl {
public:
  ChassisCtrl();
  ~ChassisCtrl() = default;

  /* 差速：v 线速度、turn 转向量 */
  void update(float v, float turn);

  /* 麦轮逆运动学：vx 前后、vy 左右、w 自转，输出到四个轮子 */
  void update_mecanum(float vx, float vy, float w);

  Motor &motor(int index) { return _m[index]; }

private:
  Motor _m[4];
};

#endif

#endif // CHASSIS_H
