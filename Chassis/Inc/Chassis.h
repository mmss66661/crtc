#ifndef CHASSIS_H
#define CHASSIS_H


#include <sys/types.h>
#ifdef __cplusplus

extern "C" {
#endif
#include "main.h"
#include "stm32_hal_legacy.h"
#include "stm32f103xb.h"
#include "stm32f1xx_hal_gpio.h"
#include "stm32f1xx_hal_tim.h"
void chassis_init();
void update(float v, float turn);

#ifdef __cplusplus
}
#endif

#ifdef __cplusplus

class Motor {
public:
  Motor()
      : _htim_pwm1(nullptr), _pwm_channel1(0), _htim_pwm2(nullptr),
        _pwm_channel2(0) {}
  Motor(TIM_HandleTypeDef *hp1, TIM_HandleTypeDef *hp2, uint32_t c1,
        uint32_t c2)
      : _htim_pwm1(hp1), _htim_pwm2(hp2), _pwm_channel1(c1), _pwm_channel2(c2) {
  }

  // 拷贝构造函数
  Motor(const Motor &other)
      : _htim_pwm1(other._htim_pwm1), _pwm_channel1(other._pwm_channel1),
        _htim_pwm2(other._htim_pwm2), _pwm_channel2(other._pwm_channel2) {}

  // 拷贝赋值运算符
  Motor &operator=(const Motor &other) {
    if (this != &other) {
      _htim_pwm1 = other._htim_pwm1;
      _pwm_channel1 = other._pwm_channel1;
      _htim_pwm2 = other._htim_pwm2;
      _pwm_channel2 = other._pwm_channel2;
    }
    return *this;
  }

  // 移动构造函数
  Motor(Motor &&other) noexcept
      : _htim_pwm1(other._htim_pwm1), _pwm_channel1(other._pwm_channel1),
        _htim_pwm2(other._htim_pwm2), _pwm_channel2(other._pwm_channel2) {
    other._htim_pwm1 = nullptr;
    other._htim_pwm2 = nullptr;
  }

  // 移动赋值运算符
  Motor &operator=(Motor &&other) noexcept {
    if (this != &other) {
      _htim_pwm1 = other._htim_pwm1;
      _pwm_channel1 = other._pwm_channel1;
      _htim_pwm2 = other._htim_pwm2;
      _pwm_channel2 = other._pwm_channel2;
      other._htim_pwm1 = nullptr;
      other._htim_pwm2 = nullptr;
    }
    return *this;
  }
  void set_compare(bool is_converse, uint32_t cnt) {
    if (!is_converse) {
      __HAL_TIM_SetCompare(_htim_pwm1, _pwm_channel1, cnt);
      __HAL_TIM_SetCompare(_htim_pwm2, _pwm_channel2, 0);
    } else {
      __HAL_TIM_SetCompare(_htim_pwm1, _pwm_channel1, 0);
      __HAL_TIM_SetCompare(_htim_pwm2, _pwm_channel2, cnt);
    }
  };

  ~Motor() {
    _htim_pwm1 = nullptr;
    _htim_pwm2 = nullptr;
  }

  void set_speed(int16_t speed);

private:
  TIM_HandleTypeDef *_htim_pwm1;
  uint32_t _pwm_channel1;
  TIM_HandleTypeDef *_htim_pwm2;
  uint32_t _pwm_channel2;
};
class ChassisCtrl {
public:
  ChassisCtrl();
  ~ChassisCtrl() = default;

  void update(float v, float turn);

private:
  Motor _m[4];
};
#endif

#endif // !CHASSIS_H
