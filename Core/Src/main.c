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
#include "test_servo.h"

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
extern volatile uint32_t usart3_rx_count;  // USART3接收字节计数(调试用)

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

// ========== 角度换算：以中心(2048)为0°, 向左为负, 向右为正 ==========

// 舵机原始值(0-4095) → 相对中心的角度(°) 范围 -180°~+180°
static float RawToRelativeDegree(uint16_t raw)
{
    if (raw > 4095) return -999.0f;
    return raw * 360.0f / 4095.0f - 180.0f;
}

// 相对角度(°) → 舵机原始值(0-4095)
static uint16_t RelativeDegreeToRaw(float relDeg)
{
    float absDeg = relDeg + 180.0f;  // 转为 0~360°
    if (absDeg < 0.0f) absDeg = 0.0f;
    if (absDeg > 360.0f) absDeg = 360.0f;
    return (uint16_t)(absDeg * 4095.0f / 360.0f);
}

// 清空舵机接收缓冲区
static void ClearServoRxBuf(void)
{
    RingBuffer_Reset(servoUsart->recvBuf);
}

// 读取单个寄存器(通用), 返回读取到的值, 失败返回0xFFFF
static uint16_t ReadReg(uint8_t addr, uint8_t len)
{
    uint8_t content[2];
    content[0] = addr;
    content[1] = len;
    JOHO_PackageBuild_Send(servoUsart, 1, 4, CMDType_Read, content);

    SysTick_DelayMs(5);
    PackageTypeDef pkg;
    if (USL_RecvPackage(servoUsart, &pkg) == JOHO_STATUS_SUCCESS) {
        if (len == 1) return pkg.content[0];
        // 大端模式: content[0]=高字节, content[1]=低字节
        return (pkg.content[0] << 8) | pkg.content[1];
    }
    return 0xFFFF;
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

  /* USER 
  
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
  

  USART_InitServoUsart(&huart3);

  // ====== 裸数据测试 ======
  RunServoTest();

  JOHO_STATUS statusCode;
  uint8_t servoId = 0;            // 检测到的舵机ID

  // ====== 1. Ping 舵机，自动检测ID（轮询1~3）==========
  printf("\r\n[Init] Scanning servo IDs 1~3...\r\n");
  for (uint8_t tryId = 1; tryId <= 3; tryId++) {
      printf("  Pinging ID=%d... ", tryId);
      statusCode = US_Ping(servoUsart, tryId);
      if (statusCode == JOHO_STATUS_SUCCESS) {
          printf("OK!\r\n");
          servoId = tryId;
          break;
      }
      printf("err=%d\r\n", statusCode);
  }

  if (servoId == 0) {
      printf("[Init] No servo found! Check wiring & power.\r\n");
      printf("       USART3 full-duplex: PB10=TX, PB11=RX\r\n");
      printf("[Init] RX count before Ping loop = %lu\r\n", usart3_rx_count);
      printf("[Init] Forcing servoId=1 (write may work but read will fail if ID mismatch)\r\n");
      servoId = 1;
  } else {
      printf("[Init] Ping OK, total RX bytes = %lu\r\n", usart3_rx_count);
  }

  // ====== 2. 使能扭矩 ======
  printf("\r\n[Init] Torque enable for ID=%d...\r\n", servoId);
  SET_Torque(servoUsart, servoId, 1);
  SysTick_DelayMs(200);

  // ====== 3. 通过USART3发送角度信号，转到中间位置(2048=0°)，停顿5秒后读取实际角度 ======
  printf("\r\n[Move] Send angle command via USART3 to center (2048 = 0°)...\r\n");
  ClearServoRxBuf();
  USL_SetServoAngle(servoUsart, servoId, 2048.0f, 1000);

  // 停顿5秒等待舵机到达
  printf("[Wait] 5 seconds for servo to reach center...\r\n");
  for (uint8_t i = 5; i > 0; i--) {
      printf("  %d...\r\n", i);
      SysTick_DelayMs(1000);
  }

  // 读取实际角度，核对是否到达中心
  printf("\r\n[Verify] Reading actual position...\r\n");
  ClearServoRxBuf();
  uint16_t pos;
  uint8_t statusErr = USL_GetServoStatus(servoUsart, servoId, &pos, NULL, NULL, NULL);
  if (statusErr == JOHO_STATUS_SUCCESS) {
      printf("\r\n[Verify] Actual position = %u (raw), %+.1f° (relative)\r\n",
             pos, RawToRelativeDegree(pos));
      if (pos >= 2020 && pos <= 2076) {
          printf("[Verify] ✓ At center position!\r\n");
      } else {
          printf("[Verify] ✗ Not at center, delta = %d\r\n", (int16_t)pos - 2048);
      }
  } else {
      printf("\r\n[Verify] Read position failed, err=%d\r\n", statusErr);
  }

  // 到达中心位置，通过USART1发送串口信号
  printf("\r\n*** Center position reached! (2048 = 0°) ***\r\n");
  HAL_UART_Transmit(&huart1, (uint8_t *)"CENTER_REACHED\r\n", 16, HAL_MAX_DELAY);

  // ====== 4. 循环：左转20° → 右转20°，每次延时1秒并读取状态 ======
  printf("\r\n========== Servo Motion Test (20° left/right) ==========\r\n");

  uint16_t leftRaw  = RelativeDegreeToRaw(-20.0f);   // 左转20°  ≈ 1820
  uint16_t rightRaw = RelativeDegreeToRaw(20.0f);    // 右转20°  ≈ 2275

  while (1)
  {
    HAL_GPIO_TogglePin(GPIOF, GPIO_PIN_9);

    // ----- 左转20° -----
    printf("\r\n[Left] 20° (raw=%u)...\r\n", leftRaw);
    ClearServoRxBuf();
    USL_SetServoAngle(servoUsart, servoId, (float)leftRaw, 500);
    SysTick_DelayMs(1000);

    // 读取电压/温度/电流
    printf("  [DBG] RX count before read = %lu\r\n", usart3_rx_count);
    uint32_t rx_before = usart3_rx_count;
    ClearServoRxBuf();
    uint16_t pos, volt;
    int16_t curr;
    int8_t temp;
    uint8_t err = USL_GetServoStatus(servoUsart, servoId, &pos, &volt, &curr, &temp);
    uint32_t rx_delta = usart3_rx_count - rx_before;
    printf("  [DBG] RX count after read = %lu (delta=%lu)\r\n", usart3_rx_count, rx_delta);
    if (err == JOHO_STATUS_SUCCESS) {
        printf("  Angle=%+6.1fdeg | Voltage=%5.1fV | Current=%+5dmA | Temp=%+3dC\r\n",
               RawToRelativeDegree(pos), volt * 0.1f, curr, temp);
    } else {
        printf("  Read status err=%d\r\n", err);
    }

    // ----- 右转20° -----
    printf("\r\n[Right] 20° (raw=%u)...\r\n", rightRaw);
    ClearServoRxBuf();
    USL_SetServoAngle(servoUsart, servoId, (float)rightRaw, 500);
    SysTick_DelayMs(1000);

    // 读取电压/温度/电流
    printf("  [DBG] RX count before read = %lu\r\n", usart3_rx_count);
    rx_before = usart3_rx_count;
    ClearServoRxBuf();
    err = USL_GetServoStatus(servoUsart, servoId, &pos, &volt, &curr, &temp);
    rx_delta = usart3_rx_count - rx_before;
    printf("  [DBG] RX count after read = %lu (delta=%lu)\r\n", usart3_rx_count, rx_delta);
    if (err == JOHO_STATUS_SUCCESS) {
        printf("  Angle=%+6.1fdeg | Voltage=%5.1fV | Current=%+5dmA | Temp=%+3dC\r\n",
               RawToRelativeDegree(pos), volt * 0.1f, curr, temp);
    } else {
        printf("  Read status err=%d\r\n", err);
    }

    
    printf("TEST IS BE INTERRUPT ACCIDENTLLY");
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
