/**
 * test_servo.c - 舵机通讯裸数据测试
 * 使用方法: 在 main.c 中 #include 此文件，在 USER CODE 中调用 RunServoTest()
 * 
 * 此测试会:
 * 1. 发送一个0xFF字节到舵机（唤醒/同步）
 * 2. 发送一个Ping指令
 * 3. 打印接收到的所有原始字节（不解析、不清除）
 */

#include "main.h"
#include "usart.h"
#include "ring_buffer.h"
#include "uart_servo_lite.h"
#include <stdio.h>

extern Usart_DataTypeDef *servoUsart;
extern volatile uint32_t usart3_rx_count;

// 读取环形缓冲区中所有可用字节并打印
static void DumpAllRxBytes(const char *label)
{
    uint16_t available = RingBuffer_GetByteUsed(servoUsart->recvBuf);
    printf("[%s] RX count=%lu, buffer has %u bytes: ", 
           label, usart3_rx_count, available);
    
    if (available == 0) {
        printf("(empty)\r\n");
        return;
    }
    
    // 读取所有字节
    for (uint16_t i = 0; i < available; i++) {
        uint8_t b = RingBuffer_ReadByte(servoUsart->recvBuf);
        printf("%02X ", b);
    }
    printf("\r\n");
}

// 裸数据测试函数
void RunServoTest(void)
{
    printf("\r\n========================================\r\n");
    printf("  舵机通讯裸数据测试\r\n");
    printf("========================================\r\n");
    
    // 先清空缓冲区
    RingBuffer_Reset(servoUsart->recvBuf);
    uint32_t old_count = usart3_rx_count;
    
    // === 测试1: 发送单个0xFF字节 ===
    printf("\r\n[Test 1] Send single 0xFF byte...\r\n");
    uint8_t syncByte = 0xFF;
    USL_Send_HEX(servoUsart, 1, &syncByte);
    SysTick_DelayMs(50);
    DumpAllRxBytes("After 0xFF");
    
    // === 测试2: 发送Ping指令 (ID=1) ===
    printf("\r\n[Test 2] Send Ping ID=1 (no buffer clear)...\r\n");
    RingBuffer_Reset(servoUsart->recvBuf);
    
    // 手动构建并发送Ping包: FF FF 01 02 01 <cs>
    uint8_t pingPkt[6];
    pingPkt[0] = 0xFF;
    pingPkt[1] = 0xFF;
    pingPkt[2] = 0x01;  // ID=1
    pingPkt[3] = 0x02;  // size=2
    pingPkt[4] = 0x01;  // CMDType_Ping
    // 计算校验和: ~(0x01 + 0x02 + 0x01) & 0xFF = ~0x04 & 0xFF = 0xFB
    pingPkt[5] = (~(pingPkt[2] + pingPkt[3] + pingPkt[4])) & 0xFF;
    
    printf("  Sending: ");
    for (int i = 0; i < 6; i++) printf("%02X ", pingPkt[i]);
    printf("\r\n");
    
    USL_Send_HEX(servoUsart, 6, pingPkt);
    
    // 等待50ms让舵机回复
    SysTick_DelayMs(50);
    
    DumpAllRxBytes("After Ping");
    
    // === 测试3: 等待更长时间(100ms)再读一次 ===
    SysTick_DelayMs(100);
    DumpAllRxBytes("After +100ms");
    
    // === 测试4: 等待更长时间(200ms)再读一次 ===
    SysTick_DelayMs(200);
    DumpAllRxBytes("After +200ms");
    
    printf("\r\n========================================\r\n");
    printf("  裸数据测试完成\r\n");
    printf("========================================\r\n");
}
