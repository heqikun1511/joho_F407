#include "step.h"
#include "math.h"

/* ====== 螺旋翻滚参数 ======
 * α=35° (≤π/2≈90°), ω=2π (T=1s), φ=π/2
 * 偏航和俯仰组成圆形轨迹
 */
#define SPIRAL_A   35.0f
#define SPIRAL_W   ((float)( M_PI/2.0f))
#define SPIRAL_PHI ((float)(M_PI / 2.0f))
#define SPIRAL_BETA ((float)(M_PI/2.0f))
//#define PHI ((float)(M_PI/2.0f))1

/**
 * @brief 螺旋翻滚角度计算
 *
 * 公式:
 *   θ_y = α · cos(ω·t+beta)
 *   θ_p = α · cos(ω·t + φ)
 *
 * φ=π/2 → θ_p = -α·sin(ω·t), 与θ_y组成圆形轨迹
 */
void steptheta_spiral(float t, float *theta_y, float *theta_p)
{
    if (!theta_y || !theta_p) return;

    float phase = SPIRAL_W * t;
    *theta_y = SPIRAL_A * cosf(phase+SPIRAL_BETA);
    *theta_p = SPIRAL_A * cosf(phase + SPIRAL_BETA+SPIRAL_PHI);
}

/**
 * @brief 将角度(度)转换为舵机原始值 (0~4095, 中心2048=0°)
 * @param deg 角度, 范围 -180° ~ +180°
 * @return 舵机原始值
 */
uint16_t DegToServoRaw(float deg)
{
    if (deg > 180.0f)  deg = 180.0f;
    if (deg < -180.0f) deg = -180.0f;

    float raw = (deg + 180.0f) * 4095.0f / 360.0f;

    if (raw < 0.0f)    raw = 0.0f;
    if (raw > 4095.0f) raw = 4095.0f;

    return (uint16_t)(raw + 0.5f);
}



