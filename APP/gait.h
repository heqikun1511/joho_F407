#ifndef __GAIT_H
#define __GAIT_H

#include <stdint.h>
#include "usart.h"     /* Usart_DataTypeDef, servoUsart */

/* ================================================================
 * 步态控制模块
 *
 * 核心公式 (弧度制):
 *   θ_yi = α_y · cos(ω_y · t + (i-1) · β_y) + γ_y
 *   θ_pi = α_p · cos(ω_p · t + (i-1) · β_p + φ) + γ_p
 *
 * 其中 i = 腿编号(1-based), t = 时间(秒)
 *
 * ID映射: 将"逻辑腿编号"映射到"物理舵机ID"
 *   例如: 4腿机器人, 每条腿2个舵机(偏航+俯仰)
 *   Leg0 → Yaw舵机ID=1, Pitch舵机ID=2
 *   Leg1 → Yaw舵机ID=3, Pitch舵机ID=4
 *   ...
 *
 * 使用方法:
 *   1. Gait_Init() 初始化
 *   2. Gait_SetMapping() 设置每条腿的物理舵机ID
 *   3. Gait_SetParams() 设置步态参数
 *   4. 在主循环中周期调用 Gait_Update( HAL_GetTick() )
 * ================================================================ */

/* ========== 常量 ========== */

/** 最大支持的腿数 */
#define GAIT_MAX_LEGS  8

/* ========== 步态参数 ========== */

/**
 * 步态参数结构体
 * 所有角度单位: 弧度 (rad)
 * 频率单位: rad/s
 */
typedef struct {
    float alpha_y;   /**< 偏航振幅 α_y (rad) */
    float alpha_p;   /**< 俯仰振幅 α_p (rad) */
    float omega_y;   /**< 偏航角频率 ω_y (rad/s) */
    float omega_p;   /**< 俯仰角频率 ω_p (rad/s) */
    float beta_y;    /**< 偏航腿间相位差 β_y (rad) */
    float beta_p;    /**< 俯仰腿间相位差 β_p (rad) */
    float phi;       /**< 偏航-俯仰相位差 φ (rad) */
    float gamma_y;   /**< 偏航中心偏移 γ_y (rad) */
    float gamma_p;   /**< 俯仰中心偏移 γ_p (rad) */
} GaitParams;

/* ========== 预设步态 ========== */

/** 螺旋翻滚 (1腿测试用) */
extern const GaitParams GAIT_SPIRAL;

/** 三角步态示例 (6腿机器人) */
extern const GaitParams GAIT_TRIPOD;

/** 波浪步态示例 (6腿机器人) */
extern const GaitParams GAIT_WAVE;
/*s水平扭动步态*/
extern const GaitParams GAITFLAT;
/*test*/
extern const GaitParams  GAIT_TEST;
/* ========== ID映射 ========== */

/**
 * 单腿舵机ID映射
 * 每条逻辑腿对应2个物理舵机: 偏航(Yaw) + 俯仰(Pitch)
 */
typedef struct {
    uint8_t  yaw_servo_id;     /**< 偏航舵机物理ID (1~254) */
    uint8_t  pitch_servo_id;   /**< 俯仰舵机物理ID (1~254) */
    float    yaw_offset_deg;   /**< 偏航安装偏移补偿 (度), 用于机械结构校准 */
    float    pitch_offset_deg; /**< 俯仰安装偏移补偿 (度) */
} LegServoMap;

/* ========== 步态控制器 ========== */

typedef struct {
    GaitParams  params;                /**< 当前步态参数 */
    LegServoMap mapping[GAIT_MAX_LEGS];/**< ID映射表 */
    uint8_t     leg_count;             /**< 实际腿数 */
    uint32_t    start_tick;            /**< 步态起始时间戳 (ms) */
    uint32_t    last_update;           /**< 上次更新时间戳 (ms) */
} GaitController;

/* ========== API 函数 ========== */

/**
 * @brief 初始化步态控制器
 * @param gc        控制器指针
 * @param leg_count 腿数 (≤ GAIT_MAX_LEGS)
 */
void Gait_Init(GaitController *gc, uint8_t leg_count);

/**
 * @brief 设置单条腿的舵机ID映射
 * @param gc            控制器指针
 * @param leg_index     逻辑腿编号 (0-based)
 * @param yaw_servo_id  偏航舵机物理ID
 * @param pitch_servo_id 俯仰舵机物理ID
 * @param yaw_offset_deg   偏航安装偏移 (度)
 * @param pitch_offset_deg 俯仰安装偏移 (度)
 */
void Gait_SetMapping(GaitController *gc, uint8_t leg_index,
                     uint8_t yaw_servo_id, uint8_t pitch_servo_id,
                     float yaw_offset_deg, float pitch_offset_deg);

/**
 * @brief 批量设置所有腿的ID映射 (顺序映射)
 * @param gc            控制器指针
 * @param base_id       起始舵机ID (第0腿偏航=base_id, 俯仰=base_id+1, 以此类推)
 *
 * 例如 base_id=1, leg_count=4:
 *   Leg0: Yaw=1,  Pitch=2
 *   Leg1: Yaw=3,  Pitch=4
 *   Leg2: Yaw=5,  Pitch=6
 *   Leg3: Yaw=7,  Pitch=8
 */
void Gait_SetMappingSequential(GaitController *gc, uint8_t base_id);

/**
 * @brief 设置步态参数
 */
void Gait_SetParams(GaitController *gc, const GaitParams *params);

/**
 * @brief 获取当前时间t (从步态启动开始的秒数)
 */
float Gait_GetTime(GaitController *gc, uint32_t current_tick_ms);

/**
 * @brief 计算指定腿的偏航角和俯仰角
 * @param gc        控制器指针
 * @param leg_index 逻辑腿编号 (0-based)
 * @param t         时间 (秒)
 * @param theta_y   输出: 偏航角 (度)
 * @param theta_p   输出: 俯仰角 (度)
 */
void Gait_CalcLeg(GaitController *gc, uint8_t leg_index, float t,
                  float *theta_y, float *theta_p);

/**
 * @brief 更新所有腿的舵机角度 (核心函数, 在主循环中周期性调用)
 * @param gc              控制器指针
 * @param current_tick_ms 当前时间戳 (ms), 传入 HAL_GetTick()
 *
 * 内部流程:
 *   1. 计算当前时间 t
 *   2. 对每条腿计算 θ_yi, θ_pi (加上安装偏移补偿)
 *   3. 通过串口发送角度指令到对应的物理舵机
 *
 * 注意: 此函数会阻塞发送, 每条腿耗时约 5~10ms
 *       建议控制周期 ≥ 20ms (50Hz)
 */
void Gait_Update(GaitController *gc, uint32_t current_tick_ms);

/**
 * @brief 同步写模式更新 - 所有舵机角度一次通讯发送 (更高效)
 * @param gc              控制器指针
 * @param current_tick_ms 当前时间戳 (ms), 传入 HAL_GetTick()
 *
 * 与 Gait_Update 的区别:
 *   - Gait_Update:     逐个发送, 每个舵机一条指令 (N条腿 × 2次串口通讯)
 *   - Gait_UpdateSync: 收集所有角度, 通过同步写一次发送 (只需1~2次串口通讯)
 *
 * 优势: 所有舵机几乎同时接收到指令, 运动更同步
 *       串口通讯次数大幅减少, 释放CPU时间
 *
 * 注意: 需要舵机固件支持 SyncWrite 指令 (CMDType_SyncWrite = 0x83)
 *       每批最多 SYNC_WRITE_MAX_SERVOS 个舵机, 超长自动分批
 */
void Gait_UpdateSync(GaitController *gc, uint32_t current_tick_ms);

/**
 * @brief 重启步态 (重置计时器)
 */
void Gait_Restart(GaitController *gc);

/**
 * @brief 停止所有舵机 (关闭扭矩)
 */
void Gait_StopAll(GaitController *gc, Usart_DataTypeDef *usart);

/**
 * @brief 弧度 → 舵机原始值 (0~4095, 中心2048=0°)
 */
uint16_t Gait_RadianToRaw(float rad);

/**
 * @brief 角度(度) → 舵机原始值 (0~4095)
 */
uint16_t Gait_DegToRaw(float deg);

#endif /* __GAIT_H */
