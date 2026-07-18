/**
 * JOHO UART舵机控制_精简版
 * JOHO SDK
 ***/
#include "uart_servo_lite.h"

extern volatile uint32_t usart3_rx_count;

//构建并发送需要的协议帧: 帧头	ID号	数据长度	指令类型	内容	校验和
void JOHO_PackageBuild_Send(Usart_DataTypeDef *usart, uint8_t usId, uint8_t size,uint8_t cmdType, uint8_t *content){
  // 使用栈上分配的内存，避免动态分配，此处数据包大小固定
    PackageTypeDef pkg;
	
    // 设置帧头
    pkg.header = JOHO_PACK_REQUEST_HEADER;
    // 设置ID号
    pkg.usId = usId;
    // 数据长度		
    pkg.size = size;
	
	  //指令类型
		pkg.sstat = cmdType;
		// 将内容复制到数据区
		for(int i=0; i<size-2; i++){
			pkg.content[i] = content[i];
		}
    // 将pkg写入到发送缓冲区sendBuf中
    JOHO_Package2RingBuffer(&pkg, usart->sendBuf);
		// 通过串口将数据发送出去
    Usart_SendAll(usart);
}




// 发送原始数据帧-HEX
void USL_Send_HEX(Usart_DataTypeDef *usart, uint8_t size, uint8_t *content){

	
	RingBuffer_WriteByteArray( usart->sendBuf,content,size);
	
	// 通过串口将数据发送出去
    Usart_SendAll(usart);
}


//接收协议帧 - 带调试信息输出
JOHO_STATUS USL_RecvPackage(Usart_DataTypeDef *usart,PackageTypeDef *pkg){
	
		pkg->status = 0; // Package状态初始化	
    uint8_t bIdx = 0; // 接收的数据字节索引
    uint16_t header = 0; // 帧头
	
#ifdef DEBUG_SERVO_RAW
		uint8_t rawBytes[128];
		uint8_t rawIdx = 0;
#endif
	
		// 启动超时计时,200ms(原100ms,增加余量)
    SysTick_CountdownBegin(JOHO_TIMEOUT_MS);
		// 如果没有超时
    while (!SysTick_CountdownIsTimeout()){
			if (RingBuffer_GetByteUsed(usart->recvBuf) == 0){
			// 没有新的字节可读, 则等待
            continue;
        }

        if(pkg->status == JOHO_RECV_FLAG_CONTENT){
            // 数据内容接收完毕
            // 接收校验和
            pkg->checksum = RingBuffer_ReadByte(usart->recvBuf);
            // 更新状态-校验和已经接收
            pkg->status = pkg->status | JOHO_RECV_FLAG_CHECKSUM;
            // 直接进行校验和验证
            if (JOHO_CalcChecksum(pkg) != pkg->checksum){
                // 取消超时计时
                SysTick_CountdownCancel();
                // 校验和错误
                return JOHO_STATUS_CHECKSUM_ERROR;
            }else{
                // 取消超时计时
                SysTick_CountdownCancel();
                // 协议帧接收成功
                return JOHO_STATUS_SUCCESS;
            }
        }else if(pkg->status == JOHO_RECV_FLAG_SSTAT){
            // 状态已经接收完毕
            // 接收数据字节
				
            pkg->content[bIdx] = RingBuffer_ReadByte(usart->recvBuf);
#ifdef DEBUG_SERVO_RAW
						if (rawIdx < sizeof(rawBytes)) rawBytes[rawIdx++] = pkg->content[bIdx];
#endif
            bIdx ++;
            // 判断是否收完
            if (bIdx == (pkg->size-2)){
                // 如果收完则进入数据区
                pkg->status = JOHO_RECV_FLAG_CONTENT;
            }
        }
				else if(pkg->status == JOHO_RECV_FLAG_SIZE){
				// Size已经接收完毕
					//接收状态
					pkg->sstat = RingBuffer_ReadByte(usart->recvBuf);
					pkg->status = JOHO_RECV_FLAG_SSTAT;

					if((pkg->size-2)==0)
					{// 如果收完则进入数据区
                pkg->status = JOHO_RECV_FLAG_CONTENT;}
				}
				else if(pkg->status == JOHO_RECV_FLAG_US_ID){
            // 舵机ID接收完毕
            // 接收尺寸信息
            pkg->size = RingBuffer_ReadByte(usart->recvBuf);
            // 判断长度size是否合法
            // 如果size超出JOHO_PACK_RESPONSE_MAX_SIZE的范围
            if (pkg->size > (JOHO_PACK_RESPONSE_MAX_SIZE - 5)){
                // 取消超时计时
                SysTick_CountdownCancel();
                return JOHO_STATUS_SIZE_TOO_BIG;
            }
            // 设置尺寸接收完成的标志位
            pkg->status = JOHO_RECV_FLAG_SIZE;
        }else if(pkg->status == JOHO_RECV_FLAG_HEADER){
            // 帧头已接收 
            // 接收舵机ID
            pkg->usId = RingBuffer_ReadByte(usart->recvBuf);
            // 判断舵机ID是否合法
            // 判断舵机ID是否有效 指令范围验证
            if (pkg->usId > JOHO_US_NUM){
                // 取消超时计时
                SysTick_CountdownCancel();
                return JOHO_STATUS_UNKOWN_US_ID;
            }
            // 设置usId已经接收的标志位
            pkg->status = JOHO_RECV_FLAG_US_ID;
        }else{
            // 接收帧头 - 同时支持两种字节序:
            //   - 大端(协议标准): F5 FF
            //   - 小端(MCU原生):  FF F5
            // 注意: 响应帧头是FF F5, 回显FF FF会被自动跳过
            uint8_t byte = RingBuffer_ReadByte(usart->recvBuf);
#ifdef DEBUG_SERVO_RAW
            if (rawIdx < sizeof(rawBytes)) rawBytes[rawIdx++] = byte;
#endif

            if (header == 0) {
                // 第一个字节: 0xF5(大端F5FF首字节) 或 0xFF(小端FFF5首字节)
                if (byte == 0xF5 || byte == 0xFF) {
                    header = byte;
                } // else 不匹配,跳过该字节
            } else {
                // 已有第一个字节,读取第二个字节后组合判断
                uint16_t hdrBE = (header << 8) | byte;   // 大端组合: header=F5 → F5 FF
                uint16_t hdrLE = (byte << 8) | header;   // 小端组合: header=FF → FF F5

                // 只接受响应帧头 0xF5FF
                if (hdrBE == JOHO_PACK_RESPONSE_HEADER || hdrLE == JOHO_PACK_RESPONSE_HEADER) {
                    // 帧头匹配成功
                    pkg->header = JOHO_PACK_RESPONSE_HEADER;
                    pkg->status = JOHO_RECV_FLAG_HEADER;
                } else if (byte == 0xF5 || byte == 0xFF) {
                    // 当前字节也可能是下一个帧头的首字节,保留它
                    header = byte;
                } else {
                    header = 0;  // 完全复位,重新搜索
                }
            }
        }
			
		}
	// 等待超时
#ifdef DEBUG_SERVO_RAW
	printf("\r\n[RAW RX] %d bytes:", rawIdx);
	for (uint8_t i = 0; i < rawIdx; i++) printf(" %02X", rawBytes[i]);
	printf("\r\n");
#endif
    return JOHO_STATUS_TIMEOUT;

}


// 将协议帧转换为字节流
void JOHO_Package2RingBuffer(PackageTypeDef *pkg,  RingBufferTypeDef *ringBuf){
    uint8_t checksum; // 校验和
//    // 写入帧头 不参与校验，在计算校验和时排除
    RingBuffer_WriteUShort(ringBuf, pkg->header);
    // 写入舵机ID
    RingBuffer_WriteByte(ringBuf, pkg->usId);
    // 写入数据的长度
    RingBuffer_WriteByte(ringBuf, pkg->size);
	//写入状态码or 指令类型
	RingBuffer_WriteByte(ringBuf, pkg->sstat);
    // 写入数据内容
	if(pkg->size != 2)
    RingBuffer_WriteByteArray(ringBuf, pkg->content, pkg->size-2);
    // 计算校验和
    checksum = RingBuffer_GetChecksum(ringBuf);
    // 写入校验和
    RingBuffer_WriteByte(ringBuf, checksum);
	
//	
//	printf("Auto get ServoID usId %d \r\n", pkg->usId);
//	printf("Auto get ServoID pkg->size %d \r\n", pkg->size);
//	printf("Auto get ServoID pkg->sstat %d \r\n", pkg->sstat);
//	printf("Auto get ServoID pkg->content %d \r\n", pkg->content[0]);
//	printf("Auto get ServoID checksum %d \r\n", checksum);

}



// 计算Package的校验和
uint8_t JOHO_CalcChecksum(PackageTypeDef *pkg){
    uint8_t checksum;
	// 初始化临时环形队列
	RingBufferTypeDef ringBuf;
	uint8_t pkgBuf[JOHO_PACK_RESPONSE_MAX_SIZE+1];
	RingBuffer_Init(&ringBuf, JOHO_PACK_RESPONSE_MAX_SIZE, pkgBuf);
    // 将Package转换为ringbuffer
	// 在转换过程中,会自动的计算checksum
    JOHO_Package2RingBuffer(pkg, &ringBuf);
	// 读取环形队列中最后一个元素(即校验和的位置)
	checksum = RingBuffer_GetValueByIndex(&ringBuf, RingBuffer_GetByteUsed(&ringBuf)-1);
    return checksum;
}



/**
 * 舵机控制SDK
 **/

// 清空接收缓冲区(全双工模式,回显字节已在发送完成时到达)
// 注意: 无需延迟等待,立即复位即可。剩余未到的回显字节会被后续
//       帧头检测逻辑自动跳过(见USL_RecvPackage帧头处理)
static void ClearServoRxBuf_Safe(RingBufferTypeDef *recvBuf)
{
    RingBuffer_Reset(recvBuf);
}

//Ping 舵机 状态查询
JOHO_STATUS US_Ping(Usart_DataTypeDef *usart, uint8_t servo_id){
	uint8_t statusCode; // 状态码
	uint8_t ehcoServoId; // PING得到的舵机ID
//	printf("[PING]Send Ping Package\r\n");
	// 发送Ping请求
	JOHO_PackageBuild_Send(usart, servo_id, 2,CMDType_Ping, NULL);
	// 等回显到达后直接接收(帧头检测自动跳过回显,只认FF F5)
	SysTick_DelayMs(5);
	PackageTypeDef pkg;
	statusCode = USL_RecvPackage(usart, &pkg);
	if(statusCode == JOHO_STATUS_SUCCESS){
		// 校验返回的ID号是否匹配
		ehcoServoId = (uint8_t)pkg.usId;
		
		if (ehcoServoId != servo_id){
			// 如果得到的舵机ID号不匹配
			return JOHO_STATUS_ID_NOT_MATCH;
		}
//		printf("[succ]Auto get ServoID %d \r\n", ehcoServoId);
	}
	return statusCode;
}


//控制舵机变换位置
void USL_SetServoAngle(Usart_DataTypeDef *usart, uint8_t servo_id, \
				float posi, uint16_t interval){
//参数校验
if(posi > 4095)posi = 4095;
if(posi <0 )		posi  = 0;			

uint16_t posit = posi;
uint8_t content[5];
					//示例：角度0~4095  FF FF 01 07 03 2A 00 00 03 E8 DF
content[0] =0x2A;
content[1] = posit	>> 8&0XFF;			
content[2] = posit	&0XFF;						
content[3] = interval	>> 8&0XFF;			
content[4] = interval	&0XFF;	
	/// 发送协议帧
	JOHO_PackageBuild_Send(usart, servo_id, 7,CMDType_Write,content);
	
	// 指令发送完毕后等待5ms，确保UART发送完成且回显数据到达
	SysTick_DelayMs(5);

}

//角度查询  返回值为0-4096 为有效值
uint16_t USL_GETPositionVal(Usart_DataTypeDef *usart, uint8_t servo_id){
uint16_t value = 0xffff; 
	uint8_t statusCode;// 状态码
	uint8_t content[2];
					//示例：角度0~4095  FF FF 01 07 03 2A 00 00 03 E8 DF
	content[0] =0x38;
	content[1] =0x02;
	/// 发送协议帧
	JOHO_PackageBuild_Send(usart, servo_id, 4,CMDType_Read,content);
	// 等回显后直接接收(帧头检测跳过回显)
	SysTick_DelayMs(5);
	PackageTypeDef pkg;
	statusCode = USL_RecvPackage(usart, &pkg);
	if(statusCode == 0) {
		// 大端模式: content[0]=高字节, content[1]=低字节
		value = (pkg.content[0] << 8) | pkg.content[1];
	} else {
		value = 0xFFFF;
	}
	return value;
}

//设置扭矩控制 0关闭 1开启
void SET_Torque(Usart_DataTypeDef *usart, uint8_t servo_id,uint8_t isopen){
//	uint8_t statusCode; // 状态码
	uint8_t content[2];
	content[0] =0x28;content[1] =0x01;
	if(isopen == 0){content[1] =0x00;}
	// 发送协议帧-示例：FF FF 01 04 03 28 01 CE
	JOHO_PackageBuild_Send(usart, servo_id, 4,CMDType_Write, content);
	// 等待5ms确保UART发送完成
	SysTick_DelayMs(5);

}

/**
 * @brief 读取舵机供电电压
 * @param usart 串口句柄
 * @param servo_id 舵机ID
 * @return 电压值(0.1V单位)，如 120 = 12.0V；失败返回0xFFFF
 * @note 读取寄存器 0x3A (1字节)
 */
uint16_t USL_GetVoltage(Usart_DataTypeDef *usart, uint8_t servo_id)
{
    uint8_t statusCode;
    uint8_t content[2];
    content[0] = 0x3A;  // 电压寄存器地址
    content[1] = 0x01;  // 读取1字节

    JOHO_PackageBuild_Send(usart, servo_id, 4, CMDType_Read, content);
	// 等回显后直接接收(帧头检测跳过回显)
	SysTick_DelayMs(5);

    PackageTypeDef pkg;
    statusCode = USL_RecvPackage(usart, &pkg);
    if (statusCode == JOHO_STATUS_SUCCESS) {
        return (uint16_t)pkg.content[0];
    }
    return 0xFFFF;
}

/**
 * @brief 读取舵机电流
 * @param usart 串口句柄
 * @param servo_id 舵机ID
 * @return 电流值(有符号)，失败返回0xFFFF
 * @note 读取寄存器 0x3C (2字节)
 */
int16_t USL_GetCurrent(Usart_DataTypeDef *usart, uint8_t servo_id)
{
    uint8_t statusCode;
    uint8_t content[2];
    content[0] = 0x3C;  // 电流寄存器地址
    content[1] = 0x02;  // 读取2字节

    JOHO_PackageBuild_Send(usart, servo_id, 4, CMDType_Read, content);
	// 等回显后直接接收(帧头检测跳过回显)
	SysTick_DelayMs(5);

    PackageTypeDef pkg;
    statusCode = USL_RecvPackage(usart, &pkg);
    if (statusCode == JOHO_STATUS_SUCCESS) {
        // 大端模式: content[0]=高字节, content[1]=低字节
        return (int16_t)((pkg.content[0] << 8) | pkg.content[1]);
    }
    return 0xFFFF;
}

/**
 * @brief 批量读取舵机状态: 角度+电压+电流+温度 (一次通讯)
 * @param usart 串口句柄
 * @param servo_id 舵机ID
 * @param position 输出: 位置值 (0-4095)
 * @param voltage  输出: 电压值 (0.1V单位)
 * @param current  输出: 电流值 (有符号)
 * @param temperature 输出: 温度值 (°C)
 * @return 0=成功, 非0=错误码
 * @note 从寄存器 0x38 开始连续读取6字节:
 *       0x38-0x39: 位置(2字节)
 *       0x3A:       电压(1字节)
 *       0x3B:       温度(1字节)
 *       0x3C-0x3D: 电流(2字节)
 */
uint8_t USL_GetServoStatus(Usart_DataTypeDef *usart, uint8_t servo_id,
                           uint16_t *position, uint16_t *voltage,
                           int16_t *current, int8_t *temperature)
{
    uint8_t content[2];

    // 注意: 该舵机不支持一次读超过2字节，必须逐个读取

    // 1. 读取位置: 2字节, 寄存器 0x38
    content[0] = 0x38;
    content[1] = 0x02;
    JOHO_PackageBuild_Send(usart, servo_id, 4, CMDType_Read, content);
    SysTick_DelayMs(30);
    {
        uint16_t n = RingBuffer_GetByteUsed(usart->recvBuf);
        if (n >= 8) {
            RingBuffer_ReadByte(usart->recvBuf); // h0 (0xFF)
            RingBuffer_ReadByte(usart->recvBuf); // h1 (0xF5)
            uint8_t id = RingBuffer_ReadByte(usart->recvBuf);
            RingBuffer_ReadByte(usart->recvBuf); // size
            RingBuffer_ReadByte(usart->recvBuf); // sstat
            uint8_t d0 = RingBuffer_ReadByte(usart->recvBuf);
            uint8_t d1 = RingBuffer_ReadByte(usart->recvBuf);
            uint8_t cs = RingBuffer_ReadByte(usart->recvBuf);
            // 校验和验证: ~(ID + size + sstat + d0 + d1) & 0xFF
            // 注意前面已读取了id, 用回传的id验证
            if (id == servo_id) {
                if (position) *position = (d0 << 8) | d1;
            } else {
                return JOHO_STATUS_ID_NOT_MATCH;
            }
        } else {
            return JOHO_STATUS_TIMEOUT;
        }
    }

    // 2. 读取电压: 1字节, 寄存器 0x3F
    if (voltage) {
        content[0] = 0x3F;
        content[1] = 0x01;
        JOHO_PackageBuild_Send(usart, servo_id, 4, CMDType_Read, content);
        SysTick_DelayMs(20);
        uint16_t n = RingBuffer_GetByteUsed(usart->recvBuf);
        if (n >= 7) { // 1字节内容: 帧头2+ID1+size1+sstat1+data1+cs1=7字节
            RingBuffer_ReadByte(usart->recvBuf); // h0
            RingBuffer_ReadByte(usart->recvBuf); // h1
            RingBuffer_ReadByte(usart->recvBuf); // ID
            RingBuffer_ReadByte(usart->recvBuf); // size
            RingBuffer_ReadByte(usart->recvBuf); // sstat
            *voltage = RingBuffer_ReadByte(usart->recvBuf); // 电压值(0.1V)
            RingBuffer_ReadByte(usart->recvBuf); // checksum
        } else {
            *voltage = 0;
        }
    }

    // 3. 读取温度: 1字节, 寄存器 0x3E
    if (temperature) {
        content[0] = 0x3E;
        content[1] = 0x01;
        JOHO_PackageBuild_Send(usart, servo_id, 4, CMDType_Read, content);
        SysTick_DelayMs(20);
        uint16_t n = RingBuffer_GetByteUsed(usart->recvBuf);
        if (n >= 7) {
            RingBuffer_ReadByte(usart->recvBuf); // h0
            RingBuffer_ReadByte(usart->recvBuf); // h1
            RingBuffer_ReadByte(usart->recvBuf); // ID
            RingBuffer_ReadByte(usart->recvBuf); // size
            RingBuffer_ReadByte(usart->recvBuf); // sstat
            *temperature = (int8_t)RingBuffer_ReadByte(usart->recvBuf); // 温度
            RingBuffer_ReadByte(usart->recvBuf); // checksum
        } else {
            *temperature = 0;
        }
    }

    // 4. 读取电流: 2字节, 寄存器 0x2E
    if (current) {
        content[0] = 0x2E;
        content[1] = 0x02;
        JOHO_PackageBuild_Send(usart, servo_id, 4, CMDType_Read, content);
        SysTick_DelayMs(20);
        uint16_t n = RingBuffer_GetByteUsed(usart->recvBuf);
        if (n >= 8) {
            RingBuffer_ReadByte(usart->recvBuf); // h0
            RingBuffer_ReadByte(usart->recvBuf); // h1
            RingBuffer_ReadByte(usart->recvBuf); // ID
            RingBuffer_ReadByte(usart->recvBuf); // size
            RingBuffer_ReadByte(usart->recvBuf); // sstat
            uint8_t d0 = RingBuffer_ReadByte(usart->recvBuf);
            uint8_t d1 = RingBuffer_ReadByte(usart->recvBuf);
            RingBuffer_ReadByte(usart->recvBuf); // checksum
            *current = (int16_t)((d0 << 8) | d1);
        } else {
            *current = 0;
        }
    }

    return JOHO_STATUS_SUCCESS;
}
