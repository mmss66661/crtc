/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2026 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "gpio.h"
#include "spi.h"
#include "stm32f1xx_hal_gpio.h"
#include "tim.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "Chassis.h"
#include "PS2.h"
#include "Servo.h"
#include <stdint.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* PS2 手柄遥控（麦轮）
 *   左摇杆：平移 —— 上下 = 前进/后退，左右 = 左右横移
 *   右摇杆 X：原地旋转
 * 摇杆量程 0~255、中位 128，映射到轮速 ±3600。 */
#define JOY_CENTER 128
#define JOY_DEADZONE 12      /* 约 ±10% 死区，防止摇杆回中不准导致漂移 */
#define JOY_SCALE (3600.0f / 128.0f) /* 摇杆偏移量 → 轮速 */
#define W_SCALE (1800.0f / 128.0f)   /* 旋转降半速，更好控 */
#define CTRL_PERIOD_MS 20U    /* 50 Hz 控制周期 */
#define PS2_LOST_LIMIT 5U     /* 连续读取失败多少次判定为掉线 */

/* 舵机控制按键：手柄「×」键（A 键），每次按下在两个预设角度间切换。
 * 按键位参见 PS2 协议：btn = ~(buttonLow | buttonHigh<<8)
 *   bit10 = L1   bit11 = R1   bit14 = ×   bit15 = □
 * 想换成别的键，把下面这一行改成对应位即可。 */
#define SERVO_BTN_MASK (1U << 14)

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {

  /* USER CODE BEGIN 1 */
  PS2RawData raw = {0};
  uint8_t ps2_lost = 0U;
  uint8_t servo_btn_prev = 0U;
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick.
   */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  /* 注意：MX_TIM2_Init / MX_TIM3_Init / MX_SPI1_Init 已停用。
   * 它们占用的是 PA0~PA3、PA6/PA7、PB0/PB1，与这块板子的实际接线冲突
   * （PA6/PA7 = PS2 的 CS/CLK，PB0/PB1 = STBY1/STBY2）。
   * 电机 PWM 现由 chassis_init() 用 TIM1(PA8~PA11) 自行初始化。 */
  /* USER CODE BEGIN 2 */
  chassis_init();
  PS2_Init();
  servo_init();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1) {

    if (PS2_ReadRaw(&raw)) {
      ps2_lost = 0U;

      int16_t lx = (int16_t)raw.leftX - JOY_CENTER;
      int16_t ly = (int16_t)raw.leftY - JOY_CENTER;
      int16_t rx = (int16_t)raw.rightX - JOY_CENTER;

      /* 死区内一律归零 */
      if (lx > -JOY_DEADZONE && lx < JOY_DEADZONE) {
        lx = 0;
      }
      if (ly > -JOY_DEADZONE && ly < JOY_DEADZONE) {
        ly = 0;
      }
      if (rx > -JOY_DEADZONE && rx < JOY_DEADZONE) {
        rx = 0;
      }

      /* 左摇杆上推 = 前进（ly 减小），右推 = 右移；右摇杆右推 = 顺时针 */
      chassis_move(-(float)ly * JOY_SCALE, (float)lx * JOY_SCALE,
                   -(float)rx * W_SCALE);

      /* 舵机：按键上升沿触发一次切换（按住不会连续翻） */
      {
        uint16_t btn =
            (uint16_t)(raw.buttonLow | ((uint16_t)raw.buttonHigh << 8));
        btn = (uint16_t)(~btn); /* 手柄按键低有效 */

        uint8_t pressed = (btn & SERVO_BTN_MASK) ? 1U : 0U;
        if (pressed && !servo_btn_prev) {
          servo_toggle();
        }
        servo_btn_prev = pressed;
      }

    } else {
      /* 手柄掉线保护：连续多次读失败则停车，避免保持最后速度冲出去 */
      if (ps2_lost < PS2_LOST_LIMIT) {
        ps2_lost++;
      }
      if (ps2_lost >= PS2_LOST_LIMIT) {
        chassis_move(0.0f, 0.0f, 0.0f);
      }
    }

    HAL_Delay(CTRL_PERIOD_MS);

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
   * in the RCC_OscInitTypeDef structure.
   */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
   */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
                                RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1) {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
 * @brief  Reports the name of the source file and the source line number
 *         where the assert_param error has occurred.
 * @param  file: pointer to the source file name
 * @param  line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t *file, uint32_t line) {
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line
     number, ex: printf("Wrong parameters value: file %s on line %d\r\n", file,
     line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
