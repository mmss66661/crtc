#include "../Inc/PS2.h"

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

PS2::PS2(SPI_HandleTypeDef *hspi, GPIO_TypeDef *csPort, uint16_t csPin)
    : hspi_(hspi), csPort_(csPort), csPin_(csPin) {}

void PS2::init() {
  HAL_GPIO_WritePin(csPort_, csPin_, GPIO_PIN_SET);

  DWT_Delay_Init();

  online_ = false;
}

uint8_t PS2::transfer(uint8_t tx) {
  uint8_t rx = 0xFF;

  auto status = HAL_SPI_TransmitReceive(hspi_, &tx, &rx, 1, 2);

  return rx;
}

bool PS2::read(PS2RawData &data) {
  static const uint8_t tx[9] = {0x01, 0x42, 0x00, 0x00, 0x00,
                                0x00, 0x00, 0x00, 0x00};

  uint8_t rx[9] = {0};

  HAL_GPIO_WritePin(csPort_, csPin_, GPIO_PIN_RESET);

  delay_us(10);

  for (uint8_t i = 0; i < 9; ++i) {
    rx[i] = transfer(tx[i]);

    if (i < 8) {
      delay_us(60);
    }
  }

  HAL_GPIO_WritePin(csPort_, csPin_, GPIO_PIN_SET);

  data.mode = rx[1];

  data.buttonLow = rx[3];
  data.buttonHigh = rx[4];

  data.rightX = rx[5];
  data.rightY = rx[6];

  data.leftX = rx[7];
  data.leftY = rx[8];

  return true;
}

extern "C" {
static PS2 ps2(&hspi1, GPIOB, GPIO_PIN_6);

void PS2_Init(void) { ps2.init(); }

uint8_t PS2_ReadRaw(PS2RawData *data) {
  if (data == nullptr) {
    return 0;
  }

  return ps2.read(*data) ? 1U : 0U;
}
}
