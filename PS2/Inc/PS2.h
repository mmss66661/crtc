#ifndef C8T6_PS2_H
#define C8T6_PS2_H

#include "main.h"
#include <stdint.h>

typedef struct {
  uint8_t mode;

  uint8_t buttonLow;
  uint8_t buttonHigh;

  uint8_t rightX;
  uint8_t rightY;

  uint8_t leftX;
  uint8_t leftY;

} PS2RawData;

#ifdef __cplusplus

/* PS2 手柄接收器驱动（软件模拟 SPI 时序）
 *
 * 板级接法：CLK=PA7  CS=PA6  CMD=PA5（MCU→手柄）  DI=PA4（手柄→MCU）
 * 这四根线的顺序与 STM32 SPI1 的硬件功能对不上（CLK 落在 MOSI、
 * CMD 落在 SCK、DI 落在 NSS），因此不能用硬件 SPI，只能 GPIO 翻转模拟。
 */
class PS2 {
public:
  PS2(GPIO_TypeDef *clkPort, uint16_t clkPin, GPIO_TypeDef *csPort,
      uint16_t csPin, GPIO_TypeDef *cmdPort, uint16_t cmdPin,
      GPIO_TypeDef *diPort, uint16_t diPin);

  void init();

  /* 成功读到一帧（模式字节合法）返回 true */
  bool read(PS2RawData &data);

  bool isOnline() const { return online_; }

private:
  void pins_init();
  void send(const uint8_t *tx, uint8_t *rx);
  uint8_t transfer(uint8_t tx);
  bool enable_analog();

  GPIO_TypeDef *clkPort_;
  uint16_t clkPin_;
  GPIO_TypeDef *csPort_;
  uint16_t csPin_;
  GPIO_TypeDef *cmdPort_;
  uint16_t cmdPin_;
  GPIO_TypeDef *diPort_;
  uint16_t diPin_;

  bool online_ = false;
};

#endif

#ifdef __cplusplus
extern "C" {
#endif

void PS2_Init(void);

uint8_t PS2_ReadRaw(PS2RawData *data);

#ifdef __cplusplus
}
#endif

#endif // C8T6_PS2_H
