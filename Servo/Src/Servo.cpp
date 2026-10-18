#include "Servo.h"
#include "stm32f1xx_hal_gpio.h"
#include "stm32f1xx_hal_tim.h"
#include <stdint.h>

/* ===================== 舵机（PB9 / TIM4_CH4） =====================
 * 普通模拟舵机要求 50 Hz（周期 20 ms），有效脉宽 0.5~2.5 ms 对应 0~180°。
 *
 * 时基设计：
 *   TIM4 挂 APB1，定时器时钟 72 MHz
 *   PSC = 72-1  -> 计数频率 1 MHz，即 1 个计数 = 1 μs
 *   ARR = 20000-1 -> 周期 20000 μs = 20 ms = 50 Hz
 *   CCR 直接就是脉宽微秒数（500~2500）
 * ================================================================ */

#define SERVO_TIM_CLK_ENABLE() __HAL_RCC_TIM4_CLK_ENABLE()

#define SERVO_PULSE_MIN 500U  /* 0°   */
#define SERVO_PULSE_MAX 2500U /* 180° */

/* 按键切换的两个位置，按实际机构行程调整 */
#define SERVO_ANGLE_A 30.0f
#define SERVO_ANGLE_B 150.0f

static TIM_HandleTypeDef htim4_servo;
static float servo_angle = 90.0f; /* 上电居中 */
static bool servo_at_a = false;

static uint16_t angle_to_pulse(float degree) {
  if (degree < 0.0f) {
    degree = 0.0f;
  } else if (degree > 180.0f) {
    degree = 180.0f;
  }

  float span = (float)(SERVO_PULSE_MAX - SERVO_PULSE_MIN);
  return (uint16_t)((float)SERVO_PULSE_MIN + span * degree / 180.0f);
}

static void servo_gpio_init(void) {
  GPIO_InitTypeDef gpio = {0};

  __HAL_RCC_GPIOB_CLK_ENABLE();

  /* PB9 复用推挽输出，交给 TIM4_CH4 */
  gpio.Pin = GPIO_PIN_9;
  gpio.Mode = GPIO_MODE_AF_PP;
  gpio.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOB, &gpio);
}

static void servo_tim_init(void) {
  TIM_ClockConfigTypeDef clock = {0};
  TIM_MasterConfigTypeDef master = {0};
  TIM_OC_InitTypeDef oc = {0};

  SERVO_TIM_CLK_ENABLE();

  htim4_servo.Instance = TIM4;
  htim4_servo.Init.Prescaler = 72 - 1;        /* 1 MHz，1 计数 = 1 μs */
  htim4_servo.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4_servo.Init.Period = 20000 - 1;        /* 20 ms */
  htim4_servo.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4_servo.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  HAL_TIM_PWM_Init(&htim4_servo);

  clock.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  HAL_TIM_ConfigClockSource(&htim4_servo, &clock);

  HAL_TIM_PWM_Init(&htim4_servo);

  master.MasterOutputTrigger = TIM_TRGO_RESET;
  master.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  HAL_TIMEx_MasterConfigSynchronization(&htim4_servo, &master);

  oc.OCMode = TIM_OCMODE_PWM1;
  oc.Pulse = angle_to_pulse(servo_angle);
  oc.OCPolarity = TIM_OCPOLARITY_HIGH;
  oc.OCFastMode = TIM_OCFAST_DISABLE;
  HAL_TIM_PWM_ConfigChannel(&htim4_servo, &oc, TIM_CHANNEL_4);

  HAL_TIM_PWM_Start(&htim4_servo, TIM_CHANNEL_4);
}

void servo_init(void) {
  servo_gpio_init();
  servo_tim_init();
  servo_set_angle(servo_angle);
}

void servo_set_angle(float degree) {
  if (degree < 0.0f) {
    degree = 0.0f;
  } else if (degree > 180.0f) {
    degree = 180.0f;
  }

  servo_angle = degree;
  __HAL_TIM_SetCompare(&htim4_servo, TIM_CHANNEL_4, angle_to_pulse(degree));
}

void servo_toggle(void) {
  servo_at_a = !servo_at_a;
  servo_set_angle(servo_at_a ? SERVO_ANGLE_A : SERVO_ANGLE_B);
}

float servo_get_angle(void) { return servo_angle; }
