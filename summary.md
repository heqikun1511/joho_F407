# F407_JOHO 舵机UART通讯工程总结

## 项目概述
STM32F407 通过 USART3 控制 JOHO 35kg 总线舵机。
- 舵机协议：单线半双工 TTL（S + VCC + GND）
- 转接板：将舵机单线S转为双线 TX/RX 连接 STM32
- 波特率：115200，8N1

---

## 问题1：PB11收不到数据

### 现象
RX count 始终为0，[RAW RX] 0 bytes，完全收不到任何数据。

### 原因
PB11 被配置为 `GPIO_MODE_INPUT`（普通输入）。STM32 的 USART 外设通过**复用功能（Alternate Function）**连接引脚，普通 GPIO 输入模式会断开 USART 连接。

### 修改

**修改前（❌ 错误）：**
```c
// usart.c - PB11配置
GPIO_InitStruct.Pin = GPIO_PIN_11;
GPIO_InitStruct.Mode = GPIO_MODE_INPUT;    // ← 错误！USART连接断开
GPIO_InitStruct.Pull = GPIO_NOPULL;
HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
```

**修改后（✅ 正确）：**
```c
// usart.c - PB11配置
GPIO_InitStruct.Pin = GPIO_PIN_11;
GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;    // ← 必须用复用模式
GPIO_InitStruct.Pull = GPIO_NOPULL;
GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
GPIO_InitStruct.Alternate = GPIO_AF7_USART3; // ← USART3复用功能
HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
```

### 教训
STM32 所有外设的 IO 引脚必须用 `GPIO_MODE_AF_xx`（复用功能模式），普通 GPIO 模式无法连接到外设。

---

## 问题2：Ping显示"OK"但实际没通

### 现象
US_Ping 返回 `SUCCESS`，但实际舵机没有回复 `FF F5`。
测试输出显示 [RAW RX] 0 bytes 但仍返回成功。

### 原因
帧头检测代码错误地接受了 `0xFFFF`（请求帧头）作为有效响应帧头。
回显 `FF FF 01 02 01 FB` 被解析为有效响应，校验和也恰好匹配。

### 修改

**修改前（❌ 错误——接受0xFFFF）：**
```c
// uart_servo_lite.c - 帧头检测
if (hdrBE == JOHO_PACK_RESPONSE_HEADER || hdrLE == JOHO_PACK_RESPONSE_HEADER ||
    hdrBE == JOHO_PACK_REQUEST_HEADER || hdrLE == JOHO_PACK_REQUEST_HEADER) {
    // 帧头匹配成功 ← 0xFFFF也被接受！
    pkg->header = hdrBE;
    pkg->status = JOHO_RECV_FLAG_HEADER;
}
```

**修改后（✅ 正确——只接受0xF5FF）：**
```c
// uart_servo_lite.c - 帧头检测
if (hdrBE == JOHO_PACK_RESPONSE_HEADER || hdrLE == JOHO_PACK_RESPONSE_HEADER) {
    // 只接受响应帧头 0xF5FF
    pkg->header = JOHO_PACK_RESPONSE_HEADER;
    pkg->status = JOHO_RECV_FLAG_HEADER;
}
```

### 教训
请求帧头和响应帧头必须严格区分。
JOHO协议：请求 `0xFFFF`，响应 `0xFFF5`（小端）/ `0xF5FF`（大端）。

---

## 问题3：舵机不回复（推挽输出导致S线被强拉高）

### 现象
STM32发送后舵机能执行指令，但舵机无法回复数据。
转接板S线测量电压约4V。

### 原因
PB10（TX）配置为 `AF_PP`（推挽输出），空闲时STM32强拉高电平。
舵机的驱动器无法克服STM32的推挽驱动，无法把S线拉低发送数据。

### 修改

**修改前（❌ 错误——一直推挽输出）：**
```c
// usart.c
void Usart_SendAll(Usart_DataTypeDef *usart)
{
    // ... 直接发送，发完后PB10仍然是AF_PP推挽输出
    HAL_UART_Transmit(usart->huart, data, len, HAL_MAX_DELAY);
}
```

**修改后（✅ 正确——发完释放S线）：**
```c
// usart.c - 新增TX_Enable/TX_Release
static void TX_Enable(void)
{
    GPIO_InitTypeDef gpio = {0};
    gpio.Pin = GPIO_PIN_10;
    gpio.Mode = GPIO_MODE_AF_PP;
    gpio.Pull = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    gpio.Alternate = GPIO_AF7_USART3;
    HAL_GPIO_Init(GPIOB, &gpio);
}

static void TX_Release(void)
{
    GPIO_InitTypeDef gpio = {0};
    gpio.Pin = GPIO_PIN_10;
    gpio.Mode = GPIO_MODE_INPUT;    // ← 切换到输入，释放S线
    gpio.Pull = GPIO_NOPULL;        // ← 转接板已有外置上拉
    HAL_GPIO_Init(GPIOB, &gpio);
}

void Usart_SendAll(Usart_DataTypeDef *usart)
{
    TX_Enable();                      // 发前切推挽
    // ... 发送数据 ...
    HAL_UART_Transmit(usart->huart, data, len, HAL_MAX_DELAY);
    TX_Release();                     // 发后切输入，释放S线
}
```

### 教训
半双工通信必须发送后**释放总线**。STM32推挽输出在空闲时强拉高电平，
从机无法驱动总线。切为INPUT高阻后，外部上拉保持电平，从机能轻松拉低。

---

## 问题4：噪声淹没舵机回复

### 现象
主循环中 Read 操作返回 err=4 或 err=5，[RAW RX] 显示大量噪声。
测试中（无角度指令时）Read正常。

### 原因
主循环中角度指令发完后等待1秒，期间S线积累大量噪声。
ClearServoRxBuf 清空后，立即发送Read指令，回显+噪声再次涌入，
舵机的 `FF F5` 回复被噪声淹没。

### 修改

**读取函数内部修改（添加清空+等待）：**
```c
// US_Ping (Ping命令)
JOHO_PackageBuild_Send(usart, servo_id, 2, CMDType_Ping, NULL);
SysTick_DelayMs(5);
// 不清空缓冲区，帧头检测自动跳过回显FF FF，只认FF F5
PackageTypeDef pkg;
statusCode = USL_RecvPackage(usart, &pkg);
```

```c
// USL_GETPositionVal, USL_GetServoStatus 等读函数
JOHO_PackageBuild_Send(usart, servo_id, 4, CMDType_Read, content);
SysTick_DelayMs(5);       // 等待回显到齐
ClearServoRxBuf_Safe(usart->recvBuf);  // 清掉回显+噪声
SysTick_DelayMs(20);      // 等待舵机回复
USL_RecvPackage(usart, &pkg);
```

**主循环中清空缓冲区：**
```c
// main.c - while(1)循环中
ClearServoRxBuf();                    // 清掉之前的噪声
USL_SetServoAngle(...);               // 发角度指令
SysTick_DelayMs(1000);                // 等舵机到达

ClearServoRxBuf();                    // 清掉角度指令的回显+噪声
err = USL_GetServoStatus(...);        // 读状态
```

### 教训
- 测试环境干净但主循环有大量积累噪声
- 读操作需要：发命令→等回显→清缓冲区→等回复→接收
- 写操作和读操作之间必须清缓冲区

---

## 问题5：AF_OD（开漏）不适合做舵机TX

### 尝试的方案（最终未采用）
```c
// usart.c - PB10配置为AF_OD
GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;  // 开漏输出
GPIO_InitStruct.Pull = GPIO_PULLUP;
```

### 失败原因
开漏输出只能拉低不能推高，靠上拉电阻恢复高电平。
上升沿太慢（RC时间常数），在115200波特率下信号完整性差。
舵机收不到完整指令，有时动有时不动。

### 结论
半双工TX必须用 **AF_PP推挽** 发送，发完后**切INPUT释放**。
不能用AF_OD一直开漏。

---

## 问题6：HDSEL半双工模式也不适合

### 尝试的方案
```c
USART3->CR3 |= USART_CR3_HDSEL;  // 开启硬件半双工
```

### 失败原因
- HDSEL要求TX引脚配置为AF_OD开漏（信号弱，问题同上）
- 空闲时引脚高阻，但外部上拉到4V，USART输入不稳定
- PB11被HDSEL内部断开，只能靠TX pin接收

### 结论
手动GPIO切换（发送时AF_PP，发完INPUT）比HDSEL更可控。

---

## 问题7：中断编译导致ELF文件损坏

### 现象
```bash
$ pyocd flash build/F407_JOHO.elf --target stm32f407zgtx
Error: Magic number does not match
```

### 原因
用户在 `make` 编译过程中按取消/中断，生成的 .elf 文件不完整。
下次 `make` 时 Makefile 认为文件已最新，不重新链接。

### 解决
```bash
make clean   # 删除所有编译中间文件
make -j4     # 强制全部重新编译
```

---

## 问题8：S线电压4V是否正常

### 现象
转接板S线对GND测量约4V（STM32工作电压3.3V）。

### 原因
转接板自带外部上拉到舵机供电电压（通过电平转换芯片），
导致S线空闲时被拉高到4V。

### 结论
**正常。** STM32F407的PB10/PB11是5V耐受引脚，4V在安全范围内。
内部上拉设为`NOPULL`避免和外部上拉打架。

---

## 最终代码状态总结

### usart.c - 核心收发函数
```c
// PB10: 发前AF_PP推挽 → 发完INPUT释放
// PB11: 一直AF_PP连接USART3接收器
// Usart_SendAll: TX_Enable → 发送 → TX_Release
```

### uart_servo_lite.c - 协议层
```c
// 帧头检测: 只接受0xF5FF，不接0xFFFF
// Ping: 发→等5ms→收（帧头跳回显）
// Read: 发→等5ms→清→等20ms→收
```

### uart_servo_lite.h - 配置
```c
#define JOHO_TIMEOUT_MS 500    // 超时从100ms提到500ms
// #define DEBUG_SERVO_RAW     // 调试输出原始接收字节
```

### main.c - 主循环
```c
// 每次发指令前 ClearServoRxBuf（RingBuffer_Reset）
// 角度指令后等1秒再读状态
// 添加了RX count调试输出
```

---

## 调试工具和方法

### 1. DEBUG_SERVO_RAW宏
启用后超时时打印收到的所有原始字节，判断是否有数据进来。
```c
#define DEBUG_SERVO_RAW  // uart_servo_lite.h中取消注释
```

### 2. RX count计数器
```c
volatile uint32_t usart3_rx_count;  // usart.c中定义
// 在HAL_UART_RxCpltCallback中++，在main.c中打印delta
```

### 3. 裸数据测试函数
`test_servo.c` 中的 `RunServoTest()`：
- 发0xFF → 看噪声
- 发Ping → 看回复
- 扭矩使能后再Ping → 看回复
- 发Read → 看数据

### 4. 万用表测S线电压
- 空闲：约3.3-5V（取决于外部上拉）
- 通信时：应有0V-高电平的波动
- 如果悬空或电压不稳 → 上拉问题
