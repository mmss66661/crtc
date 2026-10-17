#include "Chassis.h"
#include "stm32f1xx_hal_gpio.h"
#include <stdint.h>

extern "C" {
#include "main.h"
#include "stm32f1xx_hal_tim.h"
#include "tim.h"
static ChassisCtrl chassis_ctrl;
void chassis_init() {
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_SET);

  HAL_TIM_Base_Start(&htim2);

  HAL_TIM_Base_Start(&htim3);

  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);

  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);

  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);

  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);

  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);

  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);
}

void update(float v, float turn) { chassis_ctrl.update(v, turn); }
}

void Motor::set_speed(int16_t speed) {
  if (speed > 3600) {
    speed = 3600;
  } else if (speed < -3600) {
    speed = -3600;
  }
  if (speed < -28) {
    set_compare(true, -speed);
  }

  if (speed > 28) {

    set_compare(false, speed);
  }
  if (speed > -28 && speed < 28) {
    set_compare(false, 0);
  }
}
ChassisCtrl::ChassisCtrl() {
  _m[0] = Motor(&htim2, &htim2, TIM_CHANNEL_1, TIM_CHANNEL_2);

  _m[1] = Motor(&htim2, &htim2, TIM_CHANNEL_3, TIM_CHANNEL_4);

  _m[2] = Motor(&htim3, &htim3, TIM_CHANNEL_1, TIM_CHANNEL_2);

  _m[4] = Motor(&htim3, &htim3, TIM_CHANNEL_3, TIM_CHANNEL_4);
}
void ChassisCtrl::update(float v, float turn) {
  int16_t left_speed = v - turn / 2.0;
  int16_t right_speed = v + turn / 2.0;

  _m[0].set_speed(left_speed);
  _m[1].set_speed(left_speed);
  _m[2].set_speed(right_speed);
  _m[3].set_speed(right_speed);
}
