#include "stm32f4xx.h"
#include "usart.h"
#include "sys_tick.h"
#include "ring_buffer.h"

// 状态码
#define JOHO_STATUS uint8_t
#define JOHO_STATUS_SUCCESS 0 // 发送/获取成功
#define JOHO_STATUS_FAIL 1 // 发送/获取失败
#define JOHO_STATUS_TIMEOUT 2 // 等待超时 
#define JOHO_STATUS_WRONG_RESPONSE_HEADER 3 // 响应头错误
#define JOHO_STATUS_UNKOWN_US_ID 4 // 未知的舵机ID
#define JOHO_STATUS_SIZE_TOO_BIG 5 // 接收的size超出JOHO_PACK_RESPONSE_MAX_SIZE范围
#define JOHO_STATUS_CHECKSUM_ERROR 6 // 校验和错误
#define JOHO_STATUS_ID_NOT_MATCH 7 // 接收的舵机ID与期望的舵机ID不匹配


//指令类型
#define CMDType_Ping 1
#define CMDType_Read 2
#define CMDType_Write 3

#define JOHO_US_NUM 254 //舵机ID最大值

// 串口通讯超时时间 (原100ms, 增加到200ms保证充分等待)
#define JOHO_TIMEOUT_MS 200

// 在串口舵机通讯系统协议中, 使用的字节序为Little Endian(小端字节序/小端格式)
// STM32系统默认值存储模式为Little Endian
// 比如0xfeff 为帧头值, 在实际发送的时候低位在前: 0xff, 0xfe
#define JOHO_PACK_REQUEST_HEADER		0xffff
#define JOHO_PACK_RESPONSE_HEADER		0xf5ff

// 接收的响应数据包最长的长度
#define JOHO_PACK_RESPONSE_MAX_SIZE 50

// 帧头接收完成的标志位
#define JOHO_RECV_FLAG_HEADER 0x01
// 舵机ID接收完成的标志位
#define JOHO_RECV_FLAG_US_ID 0x02
// 数据长度接收完成的标志位
#define JOHO_RECV_FLAG_SIZE 0x04

//状态指令接收完成标志位
#define JOHO_RECV_FLAG_SSTAT 0x06

// 数据接收完成的标志位
#define JOHO_RECV_FLAG_CONTENT 0x08
// 校验和接收的标志位
#define JOHO_RECV_FLAG_CHECKSUM 0x10

// ��������֡�Ľṹ��
typedef struct{
    uint16_t header; // 帧头
    uint8_t usId; // 舵机ID
    uint8_t size; //数据长度：该值=ID号+指令类型+实际数据字节数，即=content+2
		uint8_t sstat; //1.作为响应包时表示舵机状态   2.作为请求包时表示指令类型
    uint8_t content[JOHO_PACK_RESPONSE_MAX_SIZE]; // 数据内容
    uint8_t checksum; // 校验和

    // 协议帧的接收进度状态 flag标志位
    uint8_t status; 
}PackageTypeDef;

// 发送原始数据帧-HEX
void USL_Send_HEX(Usart_DataTypeDef *usart, uint8_t size, uint8_t *content);

//接收协议帧
JOHO_STATUS USL_RecvPackage(Usart_DataTypeDef *usart,PackageTypeDef *pkg);


void JOHO_PackageBuild_Send(Usart_DataTypeDef *usart, uint8_t usId, uint8_t size,uint8_t cmdType, uint8_t *content);



void JOHO_Package2RingBuffer(PackageTypeDef *pkg,  RingBufferTypeDef *ringBuf);
uint8_t JOHO_CalcChecksum(PackageTypeDef *pkg);


JOHO_STATUS US_Ping(Usart_DataTypeDef *usart, uint8_t servo_id);
void USL_SetServoAngle(Usart_DataTypeDef *usart, uint8_t servo_id, \
				float posi, uint16_t interval);

uint16_t USL_GETPositionVal(Usart_DataTypeDef *usart, uint8_t servo_id);

// 读取舵机供电电压 (返回 0.1V 单位, 失败返回 0xFFFF)
uint16_t USL_GetVoltage(Usart_DataTypeDef *usart, uint8_t servo_id);

// 读取舵机电流 (有符号, 返回 0.1A? 单位取决于具体舵机, 失败返回 0xFFFF)
int16_t USL_GetCurrent(Usart_DataTypeDef *usart, uint8_t servo_id);

// 批量读取: 一次通讯获取角度+电压+温度+电流 (更高效)
// position[0-4095], voltage[0.1V], current[mA], temperature[°C]
// 返回 0=成功, 非0=错误码
uint8_t USL_GetServoStatus(Usart_DataTypeDef *usart, uint8_t servo_id,
                           uint16_t *position, uint16_t *voltage,
                           int16_t *current, int8_t *temperature);

void SET_Torque(Usart_DataTypeDef *usart, uint8_t servo_id,uint8_t isopen);