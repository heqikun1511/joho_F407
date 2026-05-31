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
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "usart.h"
#include "ring_buffer.h"
#include "uart_servo_lite.h"

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
/* USART1 直接寄存器方式发送一个字节（绕过HAL，纯硬件测试） */
static void USART1_SendByte(uint8_t byte)
{
    /* 等待TXE（发送数据寄存器空） */
    while (!(USART1->SR & USART_SR_TXE)) {}
    /* 写入数据寄存器 */
    USART1->DR = byte;
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
  /* === LED 初始化（正点原子F407: DS0=PF9, 低电平亮）=== */
  __HAL_RCC_GPIOF_CLK_ENABLE();
  GPIO_InitTypeDef led = {0};
  led.Pin = GPIO_PIN_9;
  led.Mode = GPIO_MODE_OUTPUT_PP;
  led.Pull = GPIO_NOPULL;
  led.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOF, &led);

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART1_UART_Init();
  MX_USART3_UART_Init();
  MX_USART6_UART_Init();
  /* USER CODE BEGIN 2 */

  /* === USART1 硬件测试：直接寄存器发送一个字节 ===
     如果这一步 PA9 有波形/串口能看到 'U'(0x55)，说明USART1硬件没问题
     如果看不到，CH340接线可能错了 */
  HAL_Delay(50);  // 等USART稳定
  USART1_SendByte('U');  // 0x55 -> 01010101，示波器看最清楚
  HAL_Delay(10);
  USART1_SendByte('A');
  HAL_Delay(10);
  USART1_SendByte('R');
  HAL_Delay(10);
  USART1_SendByte('T');
  HAL_Delay(10);
  USART1_SendByte('\n');
  HAL_Delay(10);

  USART_InitServoUsart(&huart3);
  JOHO_STATUS statusCode;

  printf("====================================\r\n");
  printf("F407_JOHO Servo Debug Starting...\r\n");
  printf("USART1(PA9-TX) = Debug Console @115200\r\n");
  printf("USART3(PB10-TX -> Servo RX, PB11-RX <- Servo TX) @115200\r\n");
  printf("====================================\r\n");

  /* 半双工模式：只发送不接收，直接控制舵机 */
  printf("\r\n=== Send-only mode: Sweeping Servo ID=1 ===\r\n");
  printf("If servo moves, USART3 wiring is OK!\r\n");

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  int16_t angle = 0;
  int8_t direction = 1;
  uint32_t loopCount = 0;

  while (1)
  {
    loopCount++;

    HAL_GPIO_TogglePin(GPIOF, GPIO_PIN_9);

    // 发送角度指令（不等待回复）
    USL_SetServoAngle(servoUsart, 1, angle, 800);

    if (angle % 500 == 0 || angle == 0 || angle == 4095) {
        printf("[Loop %lu] Sending angle=%d\r\n", loopCount, angle);
    }

    angle += direction * 50;
    if (angle >= 4095) {
        angle = 4095;
        direction = -1;
        printf(">>> Direction DOWN\r\n");
    } else if (angle <= 0) {
        angle = 0;
        direction = 1;
        printf(">>> Direction UP\r\n");
    }

    SysTick_DelayMs(30);
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
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
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
