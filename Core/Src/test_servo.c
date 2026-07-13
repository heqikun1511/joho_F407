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
    
    SysTick_DelayMs(50);
    DumpAllRxBytes("After Ping");
    
    SysTick_DelayMs(100);
    DumpAllRxBytes("After +100ms");
    
    // === 测试3: 先广播扭矩使能,再Ping ===
    printf("\r\n[Test 3] Torque enable (broadcast ID=254), then Ping ID=1...\r\n");
    RingBuffer_Reset(servoUsart->recvBuf);
    
    // 扭矩使能: FF FF FE 04 03 28 01 00(checksum)
    uint8_t torquePkt[8];
    torquePkt[0] = 0xFF; torquePkt[1] = 0xFF;
    torquePkt[2] = 0xFE;  // 广播ID=254
    torquePkt[3] = 0x04;  // size=4
    torquePkt[4] = 0x03;  // CMDType_Write
    torquePkt[5] = 0x28;  // Torque register
    torquePkt[6] = 0x01;  // Torque ON
    torquePkt[7] = (~(torquePkt[2] + torquePkt[3] + torquePkt[4] + torquePkt[5] + torquePkt[6])) & 0xFF;
    
    printf("  Torque cmd: ");
    for (int i = 0; i < 8; i++) printf("%02X ", torquePkt[i]);
    printf("\r\n");
    USL_Send_HEX(servoUsart, 8, torquePkt);
    SysTick_DelayMs(200);  // 等扭矩生效
    
    // 清空缓存,重新Ping
    RingBuffer_Reset(servoUsart->recvBuf);
    USL_Send_HEX(servoUsart, 6, pingPkt);
    SysTick_DelayMs(50);
    DumpAllRxBytes("After Torque+Ping");
    SysTick_DelayMs(200);
    DumpAllRxBytes("After Torque+Ping+200ms");
    
    // === 测试4: 扭矩开后发送Read指令 ===
    printf("\r\n[Test 4] Send Read ID=1 reg=0x38 len=2 (after torque ON)...\r\n");
    RingBuffer_Reset(servoUsart->recvBuf);
    
    uint8_t readPkt[8];
    readPkt[0] = 0xFF; readPkt[1] = 0xFF;
    readPkt[2] = 0x01;  // ID=1
    readPkt[3] = 0x04;  // size=4
    readPkt[4] = 0x02;  // CMDType_Read
    readPkt[5] = 0x38;  // 寄存器地址(角度)
    readPkt[6] = 0x02;  // 读取2字节
    readPkt[7] = (~(readPkt[2] + readPkt[3] + readPkt[4] + readPkt[5] + readPkt[6])) & 0xFF;
    
    printf("  Sending: ");
    for (int i = 0; i < 8; i++) printf("%02X ", readPkt[i]);
    printf("\r\n");
    
    USL_Send_HEX(servoUsart, 8, readPkt);
    
    SysTick_DelayMs(50);
    DumpAllRxBytes("After Read +50ms");
    SysTick_DelayMs(200);
    DumpAllRxBytes("After Read +250ms");
    
    printf("\r\n========================================\r\n");
    printf("  裸数据测试完成\r\n");
    printf("========================================\r\n");
}
