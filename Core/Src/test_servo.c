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

/**
 * @brief 快速Ping - 用短超时(50ms)扫描用，避免全局JOHO_TIMEOUT_MS(500ms)拖慢
 */
static JOHO_STATUS FastPing(Usart_DataTypeDef *usart, uint8_t servo_id)
{
    /* 发送Ping请求 */
    JOHO_PackageBuild_Send(usart, servo_id, 2, CMDType_Ping, NULL);
    SysTick_DelayMs(5);

    /* 用短超时50ms快速检查响应 */
    PackageTypeDef pkg;
    pkg.status = 0;
    uint8_t bIdx = 0;
    uint16_t header = 0;

    SysTick_CountdownBegin(50);  /* 短超时50ms */

    while (!SysTick_CountdownIsTimeout()) {
        if (RingBuffer_GetByteUsed(usart->recvBuf) == 0) continue;

        if (pkg.status == JOHO_RECV_FLAG_CONTENT) {
            pkg.checksum = RingBuffer_ReadByte(usart->recvBuf);
            pkg.status |= JOHO_RECV_FLAG_CHECKSUM;
            if (JOHO_CalcChecksum(&pkg) == pkg.checksum) {
                SysTick_CountdownCancel();
                return (pkg.usId == servo_id) ? JOHO_STATUS_SUCCESS : JOHO_STATUS_ID_NOT_MATCH;
            }
            return JOHO_STATUS_CHECKSUM_ERROR;
        } else if (pkg.status == JOHO_RECV_FLAG_SSTAT) {
            pkg.content[bIdx] = RingBuffer_ReadByte(usart->recvBuf);
            bIdx++;
            if (bIdx == (pkg.size - 2)) pkg.status = JOHO_RECV_FLAG_CONTENT;
        } else if (pkg.status == JOHO_RECV_FLAG_SIZE) {
            pkg.sstat = RingBuffer_ReadByte(usart->recvBuf);
            pkg.status = JOHO_RECV_FLAG_SSTAT;
            if ((pkg.size - 2) == 0) pkg.status = JOHO_RECV_FLAG_CONTENT;
        } else if (pkg.status == JOHO_RECV_FLAG_US_ID) {
            pkg.size = RingBuffer_ReadByte(usart->recvBuf);
            if (pkg.size > (JOHO_PACK_RESPONSE_MAX_SIZE - 5)) {
                SysTick_CountdownCancel();
                return JOHO_STATUS_SIZE_TOO_BIG;
            }
            pkg.status = JOHO_RECV_FLAG_SIZE;
        } else if (pkg.status == JOHO_RECV_FLAG_HEADER) {
            pkg.usId = RingBuffer_ReadByte(usart->recvBuf);
            if (pkg.usId > JOHO_US_NUM) {
                SysTick_CountdownCancel();
                return JOHO_STATUS_UNKOWN_US_ID;
            }
            pkg.status = JOHO_RECV_FLAG_US_ID;
        } else {
            uint8_t byte = RingBuffer_ReadByte(usart->recvBuf);
            if (header == 0) {
                if (byte == 0xF5 || byte == 0xFF) header = byte;
            } else {
                uint16_t hdrBE = (header << 8) | byte;
                uint16_t hdrLE = (byte << 8) | header;
                if (hdrBE == JOHO_PACK_RESPONSE_HEADER || hdrLE == JOHO_PACK_RESPONSE_HEADER) {
                    pkg.header = JOHO_PACK_RESPONSE_HEADER;
                    pkg.status = JOHO_RECV_FLAG_HEADER;
                } else if (byte == 0xF5 || byte == 0xFF) {
                    header = byte;
                } else {
                    header = 0;
                }
            }
        }
    }
    return JOHO_STATUS_TIMEOUT;
}

/**
 * @brief 扫描舵机ID（1~max_scan），报告所有响应舵机
 *
 * 使用快速Ping（50ms超时），254个ID约12秒完成扫描。
 *
 * @param found_ids 输出：已找到的舵机ID数组（需至少 max_scan 字节）
 * @param max_scan  最大扫描ID（通常254）
 * @return 找到的舵机数量
 */
uint8_t ScanAllServoIDs(uint8_t *found_ids, uint8_t max_scan)
{
    uint8_t count = 0;
    
    printf("\r\n========================================\r\n");
    printf("  舵机ID全面扫描 (1~%u)\r\n", max_scan);
    printf("========================================\r\n");
    printf("  请确保所有舵机已上电并连接到总线\r\n");
    printf("  使用快速扫描(每ID等待50ms)...\r\n\r\n");
    
    for (uint8_t id = 1; id <= max_scan; id++) {
        /* Ping前清空缓冲区，避免残留数据干扰 */
        RingBuffer_Reset(servoUsart->recvBuf);
        
        JOHO_STATUS status = FastPing(servoUsart, id);
        
        if (status == JOHO_STATUS_SUCCESS) {
            found_ids[count++] = id;
            
            /* 读取电压 */
            uint16_t voltage = USL_GetVoltage(servoUsart, id);
            
            printf("  >>> ID=%3u: 响应成功!", id);
            if (voltage != 0xFFFF) {
                printf(" 电压=%u.%uV", voltage / 10, voltage % 10);
            } else {
                printf(" 电压=读取失败");
            }
            printf("\r\n");
        } else {
            /* 每50个ID输出一个'.'作为进度指示 */
            if (id % 50 == 0) {
                printf("  已扫描到 ID=%u...\r\n", id);
            }
        }
    }
    
    printf("\r\n========================================\r\n");
    printf("  扫描完成！共找到 %u 个舵机\r\n", count);
    if (count > 0) {
        printf("  找到的舵机ID: ");
        for (uint8_t i = 0; i < count; i++) {
            printf("%u", found_ids[i]);
            if (i < count - 1) printf(", ");
        }
        printf("\r\n");
        
        printf("\r\n  各舵机详细信息:\r\n");
        for (uint8_t i = 0; i < count; i++) {
            uint8_t sid = found_ids[i];
            uint16_t voltage = USL_GetVoltage(servoUsart, sid);
            int16_t  current = USL_GetCurrent(servoUsart, sid);
            uint16_t pos     = USL_GETPositionVal(servoUsart, sid);
            
            printf("    ID=%2u: ", sid);
            if (voltage != 0xFFFF) printf("电压=%u.%uV  ", voltage / 10, voltage % 10);
            else                   printf("电压=N/A  ");
            if (current != 0x7FFF && current != 0xFFFF) printf("电流=%d  ", current);
            else                                        printf("电流=N/A  ");
            if (pos != 0xFFFF) printf("角度=%u(0~4095)", pos);
            else               printf("角度=N/A");
            printf("\r\n");
        }
    } else {
        printf("  未找到任何舵机，请检查：\r\n");
        printf("    1. 舵机供电是否正常\r\n");
        printf("    2. USART3接线(PB10=TX, PB11=RX)\r\n");
        printf("    3. 转接板是否正常工作\r\n");
    }
    printf("========================================\r\n");
    
    return count;
}
