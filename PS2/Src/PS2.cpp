#include "../Inc/PS2.h"
#include "stm32f1xx_hal_gpio.h"
#include <stdint.h>

/* ============ 板级接法（见原理图，与 SPI1 硬件功能不兼容） ============
 *   CLK = PA7   时钟，空闲高电平
 *   CS  = PA6   片选，低有效（整个 9 字节期间保持低）
 *   CMD = PA5   MCU → 手柄（相当于 MOSI）
 *   DI  = PA4   手柄 → MCU（相当于 MISO）
 * ==================================================================== */

#define PS2_CLK_PORT GPIOA
#define PS2_CLK_PIN GPIO_PIN_7
#define PS2_CS_PORT GPIOA
#define PS2_CS_PIN GPIO_PIN_6
#define PS2_CMD_PORT GPIOA
#define PS2_CMD_PIN GPIO_PIN_5
#define PS2_DI_PORT GPIOA
#define PS2_DI_PIN GPIO_PIN_4

/* 半周期 3 μs ≈ 166 kHz，低于手柄 250 kHz 的上限，留足余量 */
#define PS2_HALF_PERIOD_US 3
/* 字节之间给手柄留出准备时间 */
#define PS2_BYTE_GAP_US 30

static void DWT_Delay_Init() {
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;

  DWT->CYCCNT = 0;

  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

static void delay_us(uint32_t us) {
  uint32_t start = DWT->CYCCNT;

  uint32_t ticks = us * (HAL_RCC_GetHCLKFreq() / 1000000U);

  while ((DWT->CYCCNT - start) < ticks) {
  }
}

PS2::PS2(GPIO_TypeDef *clkPort, uint16_t clkPin, GPIO_TypeDef *csPort,
         uint16_t csPin, GPIO_TypeDef *cmdPort, uint16_t cmdPin,
         GPIO_TypeDef *diPort, uint16_t diPin)
    : clkPort_(clkPort), clkPin_(clkPin), csPort_(csPort), csPin_(csPin),
      cmdPort_(cmdPort), cmdPin_(cmdPin), diPort_(diPort), diPin_(diPin) {}

void PS2::pins_init() {
  GPIO_InitTypeDef gpio = {0};

  __HAL_RCC_GPIOA_CLK_ENABLE();

  /* CLK / CS / CMD 推挽输出 */
  gpio.Pin = (uint32_t)(clkPin_ | csPin_ | cmdPin_);
  gpio.Mode = GPIO_MODE_OUTPUT_PP;
  gpio.Pull = GPIO_NOPULL;
  gpio.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(clkPort_, &gpio);

  /* CLK 与 CS 空闲为高 */
  HAL_GPIO_WritePin(clkPort_, clkPin_, GPIO_PIN_SET);
  HAL_GPIO_WritePin(csPort_, csPin_, GPIO_PIN_SET);
  HAL_GPIO_WritePin(cmdPort_, cmdPin_, GPIO_PIN_SET);

  /* DI 上拉输入：手柄没插时保持高电平，不会读到随机值 */
  gpio.Pin = diPin_;
  gpio.Mode = GPIO_MODE_INPUT;
  gpio.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(diPort_, &gpio);
}

void PS2::init() {
  DWT_Delay_Init();
  pins_init();
  online_ = false;

  /* 等手柄上电完成（接收器冷启动约需 100~500 ms） */
  HAL_Delay(300);

  enable_analog();
}

/* 单个字节：LSB 先行，数据在 CLK 下降沿前建立、上升沿采样 */
uint8_t PS2::transfer(uint8_t tx) {
  uint8_t rx = 0;

  for (uint8_t i = 0; i < 8; ++i) {
    HAL_GPIO_WritePin(cmdPort_, cmdPin_,
                      (tx & (uint8_t)(1U << i)) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    delay_us(PS2_HALF_PERIOD_US);

    HAL_GPIO_WritePin(clkPort_, clkPin_, GPIO_PIN_RESET); /* 下降沿 */
    delay_us(PS2_HALF_PERIOD_US);

    HAL_GPIO_WritePin(clkPort_, clkPin_, GPIO_PIN_SET); /* 上升沿，此沿采样 */
    if (HAL_GPIO_ReadPin(diPort_, diPin_) == GPIO_PIN_SET) {
      rx = (uint8_t)(rx | (uint8_t)(1U << i));
    }
    delay_us(PS2_HALF_PERIOD_US);
  }

  return rx;
}

/* 完整一发一收：9 字节，CS 全程拉低 */
void PS2::send(const uint8_t *tx, uint8_t *rx) {
  HAL_GPIO_WritePin(csPort_, csPin_, GPIO_PIN_RESET);
  delay_us(10);

  for (uint8_t i = 0; i < 9; ++i) {
    uint8_t v = transfer(tx[i]);
    if (rx != nullptr) {
      rx[i] = v;
    }
    if (i < 8) {
      delay_us(PS2_BYTE_GAP_US);
    }
  }

  HAL_GPIO_WritePin(csPort_, csPin_, GPIO_PIN_SET);
  delay_us(PS2_BYTE_GAP_US);
}

/* 让手柄进入模拟模式（只有模拟模式才会返回摇杆数据） */
bool PS2::enable_analog() {
  static const uint8_t enter_config[9] = {0x01, 0x43, 0x00, 0x01, 0x00,
                                          0x00, 0x00, 0x00, 0x00};
  static const uint8_t set_analog[9] = {0x01, 0x44, 0x00, 0x01, 0x03,
                                        0x00, 0x00, 0x00, 0x00};
  static const uint8_t exit_config[9] = {0x01, 0x43, 0x00, 0x00, 0x5A,
                                         0x5A, 0x5A, 0x5A, 0x5A};

  send(enter_config, nullptr);
  HAL_Delay(10);
  send(set_analog, nullptr);
  HAL_Delay(10);
  send(exit_config, nullptr);
  HAL_Delay(10);

  return true;
}

bool PS2::read(PS2RawData &data) {
  static const uint8_t tx[9] = {0x01, 0x42, 0x00, 0x00, 0x00,
                                0x00, 0x00, 0x00, 0x00};
  uint8_t rx[9] = {0};

  send(tx, rx);

  /* rx[1] = 0x41 数字模式 / 0x73 模拟模式 / 0x79 配置模式 */
  uint8_t mode = rx[1];
  online_ = (mode == 0x41 || mode == 0x73 || mode == 0x79);

  if (!online_) {
    return false;
  }

  data.mode = mode;
  data.buttonLow = rx[3];
  data.buttonHigh = rx[4];
  data.rightX = rx[5];
  data.rightY = rx[6];
  data.leftX = rx[7];
  data.leftY = rx[8];

  return true;
}

extern "C" {
static PS2 ps2(PS2_CLK_PORT, PS2_CLK_PIN, PS2_CS_PORT, PS2_CS_PIN, PS2_CMD_PORT,
               PS2_CMD_PIN, PS2_DI_PORT, PS2_DI_PIN);

void PS2_Init(void) { ps2.init(); }

uint8_t PS2_ReadRaw(PS2RawData *data) {
  if (data == nullptr) {
    return 0;
  }

  return ps2.read(*data) ? 1U : 0U;
}
}
