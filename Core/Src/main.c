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
#include "math.h"
#include "step.h"
#include "gait.h"

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
// 调试: 打印USART3硬件寄存器状态
static void USART3_DumpStatus(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/* USART1 直接寄存器方式发送一个字节（绕过HAL，纯硬件测试） */
// static void USART1_SendByte(uint8_t byte)
// {
//     /* 等待TXE（发送数据寄存器空） */
//     while (!(USART1->SR & USART_SR_TXE)) {}
//     /* 写入数据寄存器 */
//     USART1->DR = byte;
// }

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

// // 调试: 打印USART3硬件寄存器状态和缓冲区内容
// static void USART3_DumpStatus(void)
// {
//     uint32_t sr = USART3->SR;
//     uint32_t cr1 = USART3->CR1;

//     printf("\r\n=== USART3 DEBUG ===\r\n");
//     printf("SR=0x%08lX", sr);
//     if (sr & USART_SR_PE)   printf(" PE");
//     if (sr & USART_SR_FE)   printf(" FE");
//     if (sr & USART_SR_NE)   printf(" NE");
//     if (sr & USART_SR_ORE)  printf(" ORE");
//     if (sr & USART_SR_IDLE) printf(" IDLE");
//     if (sr & USART_SR_RXNE) printf(" RXNE");
//     if (sr & USART_SR_TC)   printf(" TC");
//     if (sr & USART_SR_TXE)  printf(" TXE");
//     printf("\r\n");

//     printf("CR1=0x%08lX", cr1);
//     if (cr1 & USART_CR1_RE)     printf(" RE");
//     if (cr1 & USART_CR1_TE)     printf(" TE");
//     if (cr1 & USART_CR1_RXNEIE) printf(" RXNEIE");
//     if (cr1 & USART_CR1_PEIE)   printf(" PEIE");
//     printf("\r\n");

//     /* RXNE置位时，直接读DR看是否有数据卡住 */
//     if (sr & USART_SR_RXNE) {
//         uint8_t stuck_byte = (uint8_t)(USART3->DR & 0xFF);
//         printf("DR has stuck byte: 0x%02X\r\n", stuck_byte);
//         /* 注意: 读DR会清除RXNE */
//     }

//     /* 查看环形缓冲区状态 */
//     uint16_t used = RingBuffer_GetByteUsed(servoUsart->recvBuf);
//     uint16_t free = RingBuffer_GetByteFree(servoUsart->recvBuf);
//     printf("RingBuf: used=%u free=%u rx_count=%lu\r\n",
//            used, free, usart3_rx_count);

//     /* 打印缓冲区内容 */
//     if (used > 0) {
//         printf("RingBuf data:");
//         for (uint16_t i = 0; i < used && i < 64; i++) {
//             printf(" %02X", RingBuffer_GetValueByIndex(servoUsart->recvBuf, i));
//         }
//         printf("\r\n");
//     }
//     printf("==================\r\n");
// }

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

  // ====== 3. 转到中间位置(2048=0°)，停顿5秒后读取状态 ======
  printf("\r\n[Move] Send angle command to center (2048 = 0°)...\r\n");
  ClearServoRxBuf();
  USL_SetServoAngle(servoUsart, servoId, 2048.0f, 1000);
  printf("[Wait] 5 seconds for servo to reach center...\r\n");
  for (uint8_t i = 5; i > 0; i--) {
      printf("  %d...\r\n", i);
      SysTick_DelayMs(1000);
  }

  // 读取中心位置和各状态
 

// 
  // ====== 4. 步态控制器初始化 ======
  printf("\r\n========== Gait Controller Demo ==========\r\n");

  /* 定义步态控制器 */
  GaitController gc;
  uint8_t legCount = 2;  /* 2条腿, 每条腿2个舵机(偏航+俯仰) */

  /* 初始化 */
  Gait_Init(&gc, legCount);

  /* ========== ID映射配置 ==========
   *
   * 方案A: 手动映射 (灵活, 支持任意ID分配)
   *   每条腿指定: 偏航舵机ID, 俯仰舵机ID, 安装偏移补偿
   */
  Gait_SetMapping(&gc, 0,          /* 逻辑腿0 */
                  servoId,         /* 偏航=检测到的ID */
                  servoId + 1,     /* 俯仰=ID+1 */
                  0.0f,            /* 偏航安装偏移 0° */
                  0.0f);           /* 俯仰安装偏移 0° */

  Gait_SetMapping(&gc, 1,          /* 逻辑腿1 */
                  servoId + 2,     /* 偏航=ID+2 */
                  servoId + 3,     /* 俯仰=ID+3 */
                  0.0f,
                  0.0f);

  /* 方案B: 顺序映射 (舵机ID连续分配时更简洁) */
  // Gait_SetMappingSequential(&gc, servoId);
  // 效果同上: Leg0(Yaw=1,Pitch=2), Leg1(Yaw=3,Pitch=4), ...

  /* ========== 选择步态 ========== */
  Gait_SetParams(&gc, &GAIT_SPIRAL);  /* 螺旋翻滚 */
  // Gait_SetParams(&gc, &GAIT_TRIPOD);  /* 三角步态 */
  // Gait_SetParams(&gc, &GAIT_WAVE);    /* 波浪步态 */

  /* ========== 使能所有舵机扭矩 ========== */
  for (uint8_t i = 0; i < legCount; i++) {
      if (gc.mapping[i].yaw_servo_id != 0)
          SET_Torque(servoUsart, gc.mapping[i].yaw_servo_id, 1);
      if (gc.mapping[i].pitch_servo_id != 0)
          SET_Torque(servoUsart, gc.mapping[i].pitch_servo_id, 1);
  }
  SysTick_DelayMs(500);

  while (1)
  {
    /* === 核心: 步态更新 ===
     * 传入 HAL_GetTick() 作为时间基准
     * 内部自动计算每条腿的角度并发送指令到对应的物理舵机
     */
    Gait_Update(&gc, HAL_GetTick());

    // 每1秒打印一次
    static uint32_t lastPrint = 0;
    if (HAL_GetTick() - lastPrint > 1000) {
        lastPrint = HAL_GetTick();

        float t = Gait_GetTime(&gc, HAL_GetTick());
        printf("t=%.1fs\r\n", t);

        for (uint8_t leg = 0; leg < legCount; leg++) {
            float theta_y, theta_p;
            Gait_CalcLeg(&gc, leg, t, &theta_y, &theta_p);

            float theta_y_deg = theta_y * (180.0f / M_PI);
            float theta_p_deg = theta_p * (180.0f / M_PI);
            uint16_t raw_y = Gait_RadianToRaw(theta_y);
            uint16_t raw_p = Gait_RadianToRaw(theta_p);

            printf("  Leg%u: Yaw=%+5.1f°(raw=%4u, ID=%u)  "
                   "Pitch=%+5.1f°(raw=%4u, ID=%u)\r\n",
                   leg,
                   theta_y_deg, raw_y, gc.mapping[leg].yaw_servo_id,
                   theta_p_deg, raw_p, gc.mapping[leg].pitch_servo_id);
        }
    }

    SysTick_DelayMs(10);  /* 控制周期 ≈10ms */

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
