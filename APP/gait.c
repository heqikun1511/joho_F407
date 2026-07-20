/**
 * gait.c - 步态控制实现
 *
 * 基于三角函数的舵机步态协调控制
 * 支持多腿机器人步态规划、ID映射、安装偏移补偿
 */

#include "gait.h"
#include "uart_servo_lite.h"
#include "math.h"
#include <stdio.h>

/* ==================================================================
 * 预设步态参数
 *
 * 注意: 以下参数仅供参考, 需要根据实际机器人结构调优
 *       所有角度单位为 弧度 (rad)
 * ================================================================== */

/** 螺旋翻滚: 2条腿(实际上1条腿的2个舵机做圆轨迹) */
const GaitParams GAIT_SPIRAL = {
    .alpha_y   = 35.0f * (M_PI / 180.0f),   /* 35° 振幅 */
    .alpha_p   = 35.0f * (M_PI / 180.0f),   /* 35° 振幅 */
    .omega_y   = 2.0f * M_PI,                /* 2π rad/s (T=1s) */
    .omega_p   = 2.0f * M_PI,                /* 2π rad/s (T=1s) */
    .beta_y    = M_PI / 2.0f,                /* π/2 腿间相位 */
    .beta_p    = M_PI / 2.0f,                /* π/2 腿间相位 */
    .phi       = M_PI / 2.0f,                /* π/2 yaw-pitch 相位差 */
    .gamma_y   = 0.0f,
    .gamma_p   = 0.0f,
};



/** 波浪步态示例 (6腿: 依次抬起) */
const GaitParams GAIT_WAVE = {
    .alpha_y   = 20.0f * (M_PI / 180.0f),   /* 20° */
    .alpha_p   = 30.0f * (M_PI / 180.0f),   /* 30° */
    .omega_y   = M_PI,                       /* π rad/s (T=2s, 更慢) */
    .omega_p   = M_PI,                       /* π rad/s */
    .beta_y    = M_PI / 3.0f,                /* π/3: 6腿均匀分布相位 */
    .beta_p    = M_PI / 3.0f,                /* π/3 */
    .phi       = M_PI / 2.0f,
    .gamma_y   = 0.0f,
    .gamma_p   = 0.0f,
};
/*平面蜿蜒*/
const GaitParams GAITFLAT={
    .alpha_y   = 45.0f * (M_PI / 180.0f),   /* 20° */
    .alpha_p   = 0.0f,   
    .omega_y   = M_PI,                       /* π rad/s (T=2s, 更慢) */
    .omega_p   = M_PI,                       /* π rad/s */
    .beta_y    = M_PI / 3.0f,                /* π/3: 6腿均匀分布相位 */
    .beta_p    = 0.0f,                /* π/3 */
    .phi       = 0.0f,
    .gamma_y   = 0.0f,
    .gamma_p   = 0.0f,

};
const GaitParams GAIT_TEST={
    .alpha_y   = 
    .alpha_p   = 
    .omega_y   = 
    .omega_p   = 
    .beta_y    = 
    .beta_p    =               
    .phi       = 
    .gamma_y   = 
    .gamma_p   = 

}
/*行波步态*/
//const Gait
/* ==================================================================
 * 内部辅助函数
 * ================================================================== */

/**
 * 弧度 → 舵机原始值 (0~4095, 中心2048=0°)
 * 范围: -π ~ +π → 0~4095
 */
uint16_t Gait_RadianToRaw(float rad)
{
    /* 弧度转度 */
    float deg = rad * (180.0f / M_PI);
    return Gait_DegToRaw(deg);
}

/**
 * 角度(度) → 舵机原始值 (0~4095)
 */
uint16_t Gait_DegToRaw(float deg)
{
    if (deg > 180.0f)  deg = 180.0f;
    if (deg < -180.0f) deg = -180.0f;

    float raw = (deg + 180.0f) * 4095.0f / 360.0f;
    if (raw < 0.0f)    raw = 0.0f;
    if (raw > 4095.0f) raw = 4095.0f;

    return (uint16_t)(raw + 0.5f);
}

/* ==================================================================
 * 步态控制器 API 实现
 * ================================================================== */

void Gait_Init(GaitController *gc, uint8_t leg_count)
{
    if (!gc) return;
    if (leg_count > GAIT_MAX_LEGS) leg_count = GAIT_MAX_LEGS;

    gc->leg_count   = leg_count;
    gc->start_tick  = 0;   /* 等待首次 Update 时设置 */
    gc->last_update = 0;

    /* 清空映射表 */
    for (uint8_t i = 0; i < GAIT_MAX_LEGS; i++) {
        gc->mapping[i].yaw_servo_id    = 0;
        gc->mapping[i].pitch_servo_id  = 0;
        gc->mapping[i].yaw_offset_deg  = 0.0f;
        gc->mapping[i].pitch_offset_deg = 0.0f;
    }

    /* 默认使用平面蜿蜒参数 */
    gc->params =GAIT_TEST;
}

void Gait_SetMapping(GaitController *gc, uint8_t leg_index,
                     uint8_t yaw_servo_id, uint8_t pitch_servo_id,
                     float yaw_offset_deg, float pitch_offset_deg)
{
    if (!gc || leg_index >= gc->leg_count) return;

    gc->mapping[leg_index].yaw_servo_id    = yaw_servo_id;
    gc->mapping[leg_index].pitch_servo_id  = pitch_servo_id;
    gc->mapping[leg_index].yaw_offset_deg  = yaw_offset_deg;
    gc->mapping[leg_index].pitch_offset_deg = pitch_offset_deg;
}

void Gait_SetMappingSequential(GaitController *gc, uint8_t base_id)
{
    if (!gc) return;

    for (uint8_t i = 0; i < gc->leg_count; i++) {
        gc->mapping[i].yaw_servo_id     = base_id + i * 2;
        gc->mapping[i].pitch_servo_id   = base_id + i * 2 + 1;
        gc->mapping[i].yaw_offset_deg   = 0.0f;
        gc->mapping[i].pitch_offset_deg = 0.0f;
    }
}

void Gait_SetParams(GaitController *gc, const GaitParams *params)
{
    if (!gc || !params) return;
    gc->params = *params;
}

float Gait_GetTime(GaitController *gc, uint32_t current_tick_ms)
{
    if (!gc) return 0.0f;
    return (float)(current_tick_ms - gc->start_tick) / 1000.0f;
}

void Gait_CalcLeg(GaitController *gc, uint8_t leg_index, float t,
                  float *theta_y, float *theta_p)
{
    if (!gc || !theta_y || !theta_p) return;
    if (leg_index >= gc->leg_count) return;

    const GaitParams *p = &gc->params;
    uint8_t i = leg_index + 1;  /* 公式中 i 从 1 开始 */

    /* ========== 核心步态公式 ==========
     * θ_yi = α_y · cos(ω_y·t + (i-1)·β_y) + γ_y
     * θ_pi = α_p · cos(ω_p·t + (i-1)·β_p + φ) + γ_p
     */
    float phase_y = p->omega_y * t + (float)(i - 1) * p->beta_y;
    float phase_p = p->omega_p * t + (float)(i - 1) * p->beta_p + p->phi;

    *theta_y = p->alpha_y * cosf(phase_y) + p->gamma_y;
    *theta_p = p->alpha_p * cosf(phase_p) + p->gamma_p;
}

void Gait_Update(GaitController *gc, uint32_t current_tick_ms)
{
    if (!gc || gc->leg_count == 0) return;

    /* 首次调用时记录起始时间 */
    if (gc->start_tick == 0) {
        gc->start_tick = current_tick_ms;
    }

    /* 计算当前时间 t (秒) */
    float t = Gait_GetTime(gc, current_tick_ms);

    /* 遍历每条腿, 计算并发送角度 */
    for (uint8_t leg = 0; leg < gc->leg_count; leg++) {
        const LegServoMap *map = &gc->mapping[leg];

        /* 跳过未映射的腿 */
        if (map->yaw_servo_id == 0 || map->pitch_servo_id == 0) continue;

        /* 计算偏航角和俯仰角 (结果单位为度) */
        float theta_y_rad, theta_p_rad;
        Gait_CalcLeg(gc, leg, t, &theta_y_rad, &theta_p_rad);

        /* 弧度 → 度 (公式输出为弧度, 安装偏移为度) */
        float theta_y_deg = theta_y_rad * (180.0f / M_PI) + map->yaw_offset_deg;
        float theta_p_deg = theta_p_rad * (180.0f / M_PI) + map->pitch_offset_deg;

        /* 度 → 舵机原始值 */
        uint16_t raw_y = Gait_DegToRaw(theta_y_deg);
        uint16_t raw_p = Gait_DegToRaw(theta_p_deg);

        /* 发送偏航舵机角度指令 */
        USL_SetServoAngle(servoUsart, map->yaw_servo_id, (float)raw_y, 100);

        /* 发送俯仰舵机角度指令 */
        USL_SetServoAngle(servoUsart, map->pitch_servo_id, (float)raw_p, 100);
    }

    gc->last_update = current_tick_ms;
}

/**
 * @brief 同步写模式更新 - 所有舵机角度一次通讯发送
 *
 * 收集所有腿的偏航+俯仰角度, 通过 USL_SyncWriteAngles 一次性发送
 * 所有舵机几乎同时收到指令, 运动更同步
 */
void Gait_UpdateSync(GaitController *gc, uint32_t current_tick_ms)
{
    if (!gc || gc->leg_count == 0) return;

    /* 首次调用时记录起始时间 */
    if (gc->start_tick == 0) {
        gc->start_tick = current_tick_ms;
    }

    /* 计算当前时间 t (秒) */
    float t = Gait_GetTime(gc, current_tick_ms);

    /* 收集所有舵机数据 (每条腿2个舵机: 偏航+俯仰) */
    uint8_t  servo_ids[GAIT_MAX_LEGS * 2];
    uint16_t positions[GAIT_MAX_LEGS * 2];
    uint16_t intervals[GAIT_MAX_LEGS * 2];
    uint8_t  servo_count = 0;

    for (uint8_t leg = 0; leg < gc->leg_count; leg++) {
        const LegServoMap *map = &gc->mapping[leg];

        /* 跳过未映射的腿 */
        if (map->yaw_servo_id == 0 && map->pitch_servo_id == 0) continue;

        /* 计算偏航角和俯仰角 */
        float theta_y_rad, theta_p_rad;
        Gait_CalcLeg(gc, leg, t, &theta_y_rad, &theta_p_rad);

        /* 弧度 → 度 + 安装偏移补偿 */
        float theta_y_deg = theta_y_rad * (180.0f / M_PI) + map->yaw_offset_deg;
        float theta_p_deg = theta_p_rad * (180.0f / M_PI) + map->pitch_offset_deg;

        /* 度 → 舵机原始值 */
        uint16_t raw_y = Gait_DegToRaw(theta_y_deg);
        uint16_t raw_p = Gait_DegToRaw(theta_p_deg);

        /* 收集偏航舵机 */
        if (map->yaw_servo_id != 0) {
            servo_ids[servo_count]   = map->yaw_servo_id;
            positions[servo_count]   = raw_y;
            intervals[servo_count]   = 100;  /* 100ms 到达 */
            servo_count++;
        }

        /* 收集俯仰舵机 */
        if (map->pitch_servo_id != 0) {
            servo_ids[servo_count]   = map->pitch_servo_id;
            positions[servo_count]   = raw_p;
            intervals[servo_count]   = 100;
            servo_count++;
        }
    }

    /* 一次性同步发送所有角度 (超长自动分批) */
    if (servo_count > 0) {
        USL_SyncWriteAngles(servoUsart, servo_ids, positions, intervals, servo_count);
    }

    gc->last_update = current_tick_ms;
}

void Gait_Restart(GaitController *gc)
{
    if (!gc) return;
    gc->start_tick = 0;  /* 下次 Update 自动重置 */
}

void Gait_StopAll(GaitController *gc, Usart_DataTypeDef *usart)
{
    if (!gc || !usart) return;

    for (uint8_t leg = 0; leg < gc->leg_count; leg++) {
        if (gc->mapping[leg].yaw_servo_id != 0) {
            SET_Torque(usart, gc->mapping[leg].yaw_servo_id, 0);
        }
        if (gc->mapping[leg].pitch_servo_id != 0) {
            SET_Torque(usart, gc->mapping[leg].pitch_servo_id, 0);
        }
    }
}
