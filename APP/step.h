#ifndef __STEP_H
#define __STEP_H

#include <stdint.h>

/**
 * @brief 螺旋翻滚角度计算
 * @param t      时间 (秒)
 * @param theta_y 输出: 偏航角 (度)
 * @param theta_p 输出: 俯仰角 (度)
 *
 * 公式:
 *   θ_y = α · cos(ω·t)
 *   θ_p = α · cos(ω·t + φ)
 *
 * 参数: α=35°, ω=2π (T=1s), φ=π/2
 * 偏航和俯仰组成圆形轨迹
 */
void steptheta_spiral(float t, float *theta_y, float *theta_p);

/**
 * @brief 将角度(度)转换为舵机原始值 (0~4095, 中心2048=0°)
 * @param deg 角度，范围 -180° ~ +180°
 * @return 舵机原始值 (0~4095)
 */
uint16_t DegToServoRaw(float deg);

#endif /* __STEP_H */