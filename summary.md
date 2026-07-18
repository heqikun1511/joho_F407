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

---

# 步态控制器模块文档

## 概述

`APP/gait.h` + `APP/gait.c` 实现了基于三角函数的舵机步态协调控制。

核心思想：**用数学公式计算每一条腿在每一时刻的角度，然后自动发给对应的物理舵机。**

---

## 什么是"映射" (Mapping)？

### 问题由来

你有 **N 条腿**，每条腿有 **2 个舵机**（一个控制偏航 Yaw，一个控制俯仰 Pitch）。  

但舵机本身不认识"腿"，只认识自己的 **ID 号**（比如 ID=1、ID=2 ...）。

所以需要一个 **映射表** 来回答这个问题：

> "逻辑上的第 0 条腿的偏航舵机，到底是物理上 ID 为几的那个舵机？"

```
┌──────────────────────────────────────────────────┐
│                  逻辑层 (你关心的)                    │
│                                                    │
│  腿0  ─── 偏航(Yaw)                                │
│        ─── 俯仰(Pitch)                              │
│  腿1  ─── 偏航(Yaw)                                │
│        ─── 俯仰(Pitch)                              │
│  ...                                               │
│                          │                          │
│                     ▼ 映射 ▼                       │
│                          │                          │
│                  物理层 (舵机通讯)                    │
│                                                    │
│  舵机ID=1  ← 腿0 的偏航                              │
│  舵机ID=2  ← 腿0 的俯仰                              │
│  舵机ID=3  ← 腿1 的偏航                              │
│  舵机ID=4  ← 腿1 的俯仰                              │
│  ...                                               │
└──────────────────────────────────────────────────┘
```

### 在代码中的体现

**结构体 `LegServoMap`（`gait.h` 中定义）：**
```c
typedef struct {
    uint8_t  yaw_servo_id;      // 这条腿的偏航舵机，物理ID是多少？
    uint8_t  pitch_servo_id;    // 这条腿的俯仰舵机，物理ID是多少？
    float    yaw_offset_deg;    // 偏航安装偏移补偿（度）
    float    pitch_offset_deg;  // 俯仰安装偏移补偿（度）
} LegServoMap;
```

**配置映射的两种方式：**

**方式A — 手动映射（灵活）：**
```c
// 腿0: 偏航用ID=1，俯仰用ID=2，无安装偏移
Gait_SetMapping(&gc, 0, 1, 2, 0.0f, 0.0f);
// 腿1: 偏航用ID=3，俯仰用ID=4，偏航装偏了3°所以补偿
Gait_SetMapping(&gc, 1, 3, 4, 3.0f, 0.0f);
```

**方式B — 顺序映射（舵机ID连续时最简洁）：**
```c
// base_id=1, leg_count=4
// 自动: Leg0(Yaw=1,Pitch=2), Leg1(Yaw=3,Pitch=4) ...
Gait_SetMappingSequential(&gc, 1);
```

### 为什么需要安装偏移补偿？

舵机装在机器人腿上时，机械结构可能有安装误差。比如腿2的偏航舵机装歪了 5°，如果不补偿，腿2走出来的轨迹就会偏。

解决办法：在映射表中记下这个偏移，`Gait_Update` 发角度时会自动加上它。

```c
Gait_SetMapping(&gc, 2, 5, 6, 5.0f, 0.0f);
//                      ↑   ↑    ↑
//                偏航ID=5  俯仰ID=6  偏航装偏了5°，自动补偿
```

---

## 步态公式

整个步态控制基于两个余弦函数：

```
θ_yi = α_y · cos(ω_y · t + (i-1) · β_y) + γ_y
θ_pi = α_p · cos(ω_p · t + (i-1) · β_p + φ) + γ_p
```

**变量说明：**
| 符号 | 含义 | 单位 |
|------|------|------|
| θ_yi | 第 i 条腿的偏航角 | 弧度 |
| θ_pi | 第 i 条腿的俯仰角 | 弧度 |
| i | 腿编号（从1开始） | — |
| t | 时间 | 秒 |

**参数说明（通过 `GaitParams` 结构体设置）：**
| 参数 | 含义 | 作用 |
|------|------|------|
| α_y, α_p | 振幅 | 腿摆动幅度有多大 |
| ω_y, ω_p | 角频率 | 腿摆动有多快（2π=每秒转一圈） |
| β_y, β_p | 腿间相位差 | 相邻腿之间差多少相位（决定步态模式） |
| φ | 偏航-俯仰相位差 | 腿画圆还是画椭圆的关键 |
| γ_y, γ_p | 中心偏移 | 整体角度偏移（调重心） |

### 三角步态示例（6腿机器人）

假设 β = π，6条腿的相位分布：
```
腿0: cos(ωt + 0)       → 与腿3同相（差3π）
腿1: cos(ωt + π)       → 与腿0反相
腿2: cos(ωt + 2π) = cos(ωt) → 与腿0同相
腿3: cos(ωt + 3π) = cos(ωt+π) → 与腿0反相
腿4: cos(ωt + 4π) = cos(ωt) → 与腿0同相
腿5: cos(ωt + 5π) = cos(ωt+π) → 与腿0反相
```
结果：腿0/2/4 一组，腿1/3/5 一组，交替抬起 → **三角步态**

---

## 预设步态

代码中内置了三种预设步态，可直接使用：

| 步态名 | 宏 | 说明 | 适用 |
|--------|-----|------|------|
| 螺旋翻滚 | `GAIT_SPIRAL` | 腿画圆轨迹，做翻滚动作 | 单腿/双腿测试 |
| 三角步态 | `GAIT_TRIPOD` | 6腿分成两组交替摆动 | 6腿快速行走 |
| 波浪步态 | `GAIT_WAVE` | 6腿依次抬起，像波浪 | 6腿慢速稳定行走 |

切换步态只需一行：
```c
Gait_SetParams(&gc, &GAIT_TRIPOD);  // 换三角步态
```

你也可以定义自己的步态：
```c
GaitParams myGait = {
    .alpha_y = 0.5f,       // ≈28.6°
    .alpha_p = 0.4f,       // ≈22.9°
    .omega_y = 4.0f,       // 更快
    .omega_p = 4.0f,
    .beta_y  = 1.57f,      // π/2
    .beta_p  = 1.57f,
    .phi     = 1.57f,      // π/2
    .gamma_y = 0.0f,
    .gamma_p = -0.17f,     // 俯仰偏-10°，重心前倾
};
Gait_SetParams(&gc, &myGait);
```

---

## 代码文件结构

```
F407_JOHO/
├── APP/
│   ├── gait.h              ← 步态控制器头文件（API + 数据结构）
│   ├── gait.c              ← 步态控制器实现
│   ├── step.h              ← 旧版单腿螺旋翻滚（保留）
│   └── step.c              ← 旧版实现（保留）
├── Core/Src/
│   └── main.c              ← 主程序，演示步态控制器用法
├── HARDWARE/
│   └── uart_servo/
│       ├── uart_servo_lite.h  ← 底层舵机通讯协议
│       └── uart_servo_lite.c  ← 发送/接收/解析
├── Makefile                ← 已添加 APP/gait.c
└── summary.md              ← 本文件
```

---

## 典型使用流程

```c
// === 1. 定义控制器 ===
GaitController gc;

// === 2. 初始化（告诉它有几条腿） ===
Gait_Init(&gc, 4);        // 4条腿

// === 3. 设置ID映射（逻辑腿 → 物理舵机） ===
Gait_SetMappingSequential(&gc, 1);  // 舵机ID 1~8

// === 4. 选择步态 ===
Gait_SetParams(&gc, &GAIT_TRIPOD);

// === 5. 使能扭矩 ===
for (int i = 0; i < 4; i++) {
    SET_Torque(servoUsart, gc.mapping[i].yaw_servo_id, 1);
    SET_Torque(servoUsart, gc.mapping[i].pitch_servo_id, 1);
}

// === 6. 主循环 ===
while (1) {
    Gait_Update(&gc, HAL_GetTick());  // 自动计算+发送
    SysTick_DelayMs(10);
}
```

---

## Gait_Update 内部工作流程

```
Gait_Update(&gc, HAL_GetTick())
│
├─ 计算当前时间 t = (now - start_tick) / 1000
│
├─ 对每条腿 i:
│   ├─ 查映射表 → 得到 yaw_servo_id, pitch_servo_id, offset
│   │
│   ├─ 计算 θ_yi = α_y·cos(ω_y·t + (i-1)·β_y) + γ_y
│   ├─ 计算 θ_pi = α_p·cos(ω_p·t + (i-1)·β_p + φ) + γ_p
│   │
│   ├─ 加上安装偏移: θ_y += yaw_offset
│   │                   θ_p += pitch_offset
│   │
│   ├─ 弧度 → 度 → 舵机原始值(0~4095)
│   │
│   └─ 发送: USL_SetServoAngle(servoUsart, yaw_servo_id, raw_y, 100)
│            USL_SetServoAngle(servoUsart, pitch_servo_id, raw_p, 100)
│
└─ 更新 last_update = current_tick
```

---

## 常见问题

### Q: 为什么腿不动？
1. 检查舵机ID映射是否正确 — 用 `US_Ping` 测试每个舵机ID
2. 检查扭矩是否使能 — `SET_Torque(servoUsart, id, 1)`
3. 检查步态振幅是否太小 — α 至少 10° (0.17 rad) 才能看到明显运动

### Q: 腿动了但轨迹不对？
- 检查安装偏移 `offset_deg` 是否设置正确
- 检查 β（腿间相位差）是否符合你想要的步态模式
- 检查 φ（yaw-pitch相位差），φ=0 是直线，φ=π/2 是圆

### Q: 如何让机器人走得更快？
增大 ω（角频率）即可：
```c
// 原: ω=2π ≈ 6.28 (T=1s)
// 改: ω=4π ≈ 12.56 (T=0.5s)，速度快一倍
gc.params.omega_y = 12.56f;
gc.params.omega_p = 12.56f;
```

### Q: 如何让机器人走得更高（抬腿更高）？
增大 α（振幅）：
```c
gc.params.alpha_y = 0.7f;  // ≈40°
gc.params.alpha_p = 0.7f;  // ≈40°
```

### Q: 新加的 gait.c 编译不过？
确保 Makefile 的 C_SOURCES 中包含 `APP/gait.c \`（末尾反斜杠不能少）。

