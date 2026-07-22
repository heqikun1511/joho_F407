#ifndef __TEST_SERVO_H
#define __TEST_SERVO_H

#include <stdint.h>

void RunServoTest(void);

/**
 * @brief 扫描舵机ID（1~254），报告所有响应舵机
 * @param found_ids    输出：找到的舵机ID数组（需至少254字节）
 * @param max_scan     最大扫描ID（通常254）
 * @return 找到的舵机数量
 */
uint8_t ScanAllServoIDs(uint8_t *found_ids, uint8_t max_scan);

#endif
