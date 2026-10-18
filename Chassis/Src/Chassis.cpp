#include "Chassis.h"
#include "stm32f1xx_hal_gpio.h"
#include "stm32f1xx_hal_tim.h"
#include <stdint.h>

/* ===================== 板级引脚定义（与原理图一致） =====================
 * 电机驱动 IN1 走 TIM1 PWM，IN2 走普通 GPIO：
 *   FL: IN1 = PA8  (TIM1_CH1)   IN2 = PB12
 *   FR: IN1 = PA9  (TIM1_CH2)   IN2 = PB13
 *   BL: IN1 = PA10 (TIM1_CH3)   IN2 = PB14
 *   BR: IN1 = PA11 (TIM1_CH4)   IN2 = PB15
 * 驱动使能：
 *   STBY1 = PB0（前级：FL + FR）
 *   STBY2 = PB1（后级：BL + BR）
 *
 * 注意：原固件用的是 TIM2(PA0~PA3) / TIM3(PA6/PA7/PB0/PB1)，
 * 与这块板子完全对不上（那些脚在板上是空脚、PS2 的 CS/CLK、STBY）。
 * ====================================================================== */

#define PWM_PERIOD 3600U /* 72 MHz / 3600 = 20 kHz */
#define MOTOR_DEADZONE 28

static TIM_HandleTypeDef htim1_pwm;

/* IN1：PA8~PA11 复用推挽，交给 TIM1 CH1~CH4 */
static void in1_gpio_init(void) {
  GPIO_InitTypeDef gpio = {0};

  __HAL_RCC_GPIOA_CLK_ENABLE();

  gpio.Pin = GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10 | GPIO_PIN_11;
  gpio.Mode = GPIO_MODE_AF_PP;
  gpio.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOA, &gpio);
}

/* IN2 + STBY：PB12~PB15 输出（默认低 = 正转方向），PB0/PB1 拉高使能驱动 */
static void in2_stby_gpio_init(void) {
  GPIO_InitTypeDef gpio = {0};

  __HAL_RCC_GPIOB_CLK_ENABLE();

  gpio.Pin = GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15;
  gpio.Mode = GPIO_MODE_OUTPUT_PP;
  gpio.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &gpio);
  HAL_GPIO_WritePin(GPIOB,
                    GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15,
                    GPIO_PIN_RESET);

  gpio.Pin = GPIO_PIN_0 | GPIO_PIN_1;
  gpio.Mode = GPIO_MODE_OUTPUT_PP;
  gpio.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &gpio);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0 | GPIO_PIN_1, GPIO_PIN_SET);
}

/* TIM1 四路 PWM，20 kHz，向上计数 */
static void tim1_pwm_init(void) {
  TIM_ClockConfigTypeDef clock = {0};
  TIM_MasterConfigTypeDef master = {0};
  TIM_OC_InitTypeDef oc = {0};
  TIM_BreakDeadTimeConfigTypeDef bdt = {0};

  __HAL_RCC_TIM1_CLK_ENABLE();

  htim1_pwm.Instance = TIM1;
  htim1_pwm.Init.Prescaler = 0;
  htim1_pwm.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1_pwm.Init.Period = PWM_PERIOD - 1;
  htim1_pwm.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1_pwm.Init.RepetitionCounter = 0;
  htim1_pwm.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  HAL_TIM_PWM_Init(&htim1_pwm);

  clock.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  HAL_TIM_ConfigClockSource(&htim1_pwm, &clock);

  master.MasterOutputTrigger = TIM_TRGO_RESET;
  master.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  HAL_TIMEx_MasterConfigSynchronization(&htim1_pwm, &master);

  oc.OCMode = TIM_OCMODE_PWM1;
  oc.Pulse = 0;
  oc.OCPolarity = TIM_OCPOLARITY_HIGH;
  oc.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  oc.OCFastMode = TIM_OCFAST_DISABLE;
  oc.OCIdleState = TIM_OCIDLESTATE_RESET;
  oc.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  HAL_TIM_PWM_ConfigChannel(&htim1_pwm, &oc, TIM_CHANNEL_1);
  HAL_TIM_PWM_ConfigChannel(&htim1_pwm, &oc, TIM_CHANNEL_2);
  HAL_TIM_PWM_ConfigChannel(&htim1_pwm, &oc, TIM_CHANNEL_3);
  HAL_TIM_PWM_ConfigChannel(&htim1_pwm, &oc, TIM_CHANNEL_4);

  bdt.OffStateRunMode = TIM_OSSR_DISABLE;
  bdt.OffStateIDLEMode = TIM_OSSI_DISABLE;
  bdt.LockLevel = TIM_LOCKLEVEL_OFF;
  bdt.DeadTime = 0;
  bdt.BreakState = TIM_BREAK_DISABLE;
  bdt.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  bdt.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  HAL_TIMEx_ConfigBreakDeadTime(&htim1_pwm, &bdt);

  /* TIM1 是高级定时器，必须置 MOE 主输出使能才会出波形 */
  __HAL_TIM_MOE_ENABLE(&htim1_pwm);

  HAL_TIM_PWM_Start(&htim1_pwm, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim1_pwm, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim1_pwm, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim1_pwm, TIM_CHANNEL_4);
}

extern "C" {
static ChassisCtrl chassis_ctrl;

void chassis_init() {
  in2_stby_gpio_init(); /* 先把 STBY 拉高、IN2 拉低 */
  in1_gpio_init();
  tim1_pwm_init();
}

void update(float v, float turn) { chassis_ctrl.update(v, turn); }

void chassis_move(float vx, float vy, float w) {
  chassis_ctrl.update_mecanum(vx, vy, w);
}
}

void Motor::set_speed(int16_t speed) {
  if (htim_ == nullptr || in2Port_ == nullptr) {
    return;
  }

  if (reversed_) {
    speed = (int16_t)(-speed);
  }

  if (speed > (int16_t)PWM_PERIOD) {
    speed = (int16_t)PWM_PERIOD;
  } else if (speed < -(int16_t)PWM_PERIOD) {
    speed = -(int16_t)PWM_PERIOD;
  }

  if (speed > MOTOR_DEADZONE) {
    HAL_GPIO_WritePin(in2Port_, in2Pin_, GPIO_PIN_RESET);
    __HAL_TIM_SetCompare(htim_, channel_, (uint32_t)speed);
  } else if (speed < -MOTOR_DEADZONE) {
    HAL_GPIO_WritePin(in2Port_, in2Pin_, GPIO_PIN_SET);
    __HAL_TIM_SetCompare(htim_, channel_, (uint32_t)(-speed));
  } else {
    HAL_GPIO_WritePin(in2Port_, in2Pin_, GPIO_PIN_RESET);
    __HAL_TIM_SetCompare(htim_, channel_, 0);
  }
}

ChassisCtrl::ChassisCtrl() {
  _m[0] = Motor(&htim1_pwm, TIM_CHANNEL_1, GPIOB, GPIO_PIN_12); /* 左前 FL */
  _m[1] = Motor(&htim1_pwm, TIM_CHANNEL_2, GPIOB, GPIO_PIN_13); /* 右前 FR */
  _m[2] = Motor(&htim1_pwm, TIM_CHANNEL_3, GPIOB, GPIO_PIN_14); /* 左后 BL */
  _m[3] = Motor(&htim1_pwm, TIM_CHANNEL_4, GPIOB, GPIO_PIN_15); /* 右后 BR */

  /* 左右电机镜像安装时，若某一侧转向反了，把这里改成 true 即可 */
  _m[0].set_reversed(false);
  _m[1].set_reversed(false);
  _m[2].set_reversed(false);
  _m[3].set_reversed(false);
}

void ChassisCtrl::update(float v, float turn) {
  int16_t left_speed = (int16_t)(v - turn / 2.0f);
  int16_t right_speed = (int16_t)(v + turn / 2.0f);

  _m[0].set_speed(left_speed);  /* 左前 FL */
  _m[1].set_speed(right_speed); /* 右前 FR */
  _m[2].set_speed(left_speed);  /* 左后 BL */
  _m[3].set_speed(right_speed); /* 右后 BR */
}

/* 麦轮逆运动学（X 型辊子布局）
 *
 *   vx > 0 前进      vy > 0 右移      w > 0 逆时针自转
 *
 *   FL = vx - vy - w      FR = vx + vy + w
 *   BL = vx + vy - w      BR = vx - vy + w
 *
 * 自检：
 *   前进   (1,0,0) -> 四轮全 +     后退 ( -1,0,0) -> 四轮全 -
 *   右移   (0,1,0) -> FL/BR 反，FR/BL 正（对角线同向）
 *   左移   (0,-1,0)-> FL/BR 正，FR/BL 反
 *   逆时针 (0,0,1) -> 左侧两轮反，右侧两轮正
 */
void ChassisCtrl::update_mecanum(float vx, float vy, float w) {
  float fl = vx - vy - w;
  float fr = vx + vy + w;
  float bl = vx + vy - w;
  float br = vx - vy + w;

  /* 合成后可能超过满量程，按最大项等比缩放，保证合速度方向不变 */
  float peak = 0.0f;
  const float v[4] = {fl, fr, bl, br};
  for (uint8_t i = 0; i < 4; ++i) {
    float a = (v[i] < 0.0f) ? -v[i] : v[i];
    if (a > peak) {
      peak = a;
    }
  }
  if (peak > (float)PWM_PERIOD) {
    float scale = (float)PWM_PERIOD / peak;
    fl *= scale;
    fr *= scale;
    bl *= scale;
    br *= scale;
  }

  _m[0].set_speed((int16_t)fl); /* 左前 FL */
  _m[1].set_speed((int16_t)fr); /* 右前 FR */
  _m[2].set_speed((int16_t)bl); /* 左后 BL */
  _m[3].set_speed((int16_t)br); /* 右后 BR */
}
