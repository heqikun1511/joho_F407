/**
 * JOHO UART舵机控制_精简版
 * JOHO SDK
 ***/
#include "uart_servo_lite.h"



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


//接收协议帧
JOHO_STATUS USL_RecvPackage(Usart_DataTypeDef *usart,PackageTypeDef *pkg){
	
		pkg->status = 0; // Package状态初始化	
    uint8_t bIdx = 0; // 接收的数据字节索引
    uint16_t header = 0; // 帧头
	
	
		// 启动超时计时,100ms
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
            // 接收帧头
            if (header == 0){
                // 接收第一个字节
                header = RingBuffer_ReadByte(usart->recvBuf);

                // 判断接收的第一个字节是否正确
                if (header != (JOHO_PACK_RESPONSE_HEADER&0xff)){
										// 如果第一个字节错误 header重置为0
                    header = 0;
                }
            }else if(header == (JOHO_PACK_RESPONSE_HEADER&0xFF)){
                // 接收帧头第二个字节
                header =  header | (RingBuffer_ReadByte(usart->recvBuf) << 8);
								// 判断第二个字节是否正确
                if(header != JOHO_PACK_RESPONSE_HEADER){
                    header = 0;
                }else{
                    pkg->header = header;
                    // 帧头接收成功
                    pkg->status = JOHO_RECV_FLAG_HEADER; 
                }
            }else{
                header = 0;
            }
        }
			
		}
	// 等待超时
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

//Ping 舵机 状态查询
JOHO_STATUS US_Ping(Usart_DataTypeDef *usart, uint8_t servo_id){
	uint8_t statusCode; // 状态码
	uint8_t ehcoServoId; // PING得到的舵机ID
//	printf("[PING]Send Ping Package\r\n");
	// ���������
	JOHO_PackageBuild_Send(usart, servo_id, 2,CMDType_Ping, NULL);
	// ���շ��ص�Ping
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
uint16_t time;			
uint8_t content[5];
					//示例：角度0~4095  FF FF 01 07 03 2A 00 00 03 E8 DF
content[0] =0x2A;
content[1] = posit	>> 8&0XFF;			
content[2] = posit	&0XFF;						
content[3] = interval	>> 8&0XFF;			
content[4] = interval	&0XFF;	
	/// 发送协议帧
	JOHO_PackageBuild_Send(usart, servo_id, 7,CMDType_Write,content);
	

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
	// 接收返回的pkg
	PackageTypeDef pkg;
	statusCode = USL_RecvPackage(usart, &pkg);
	if(statusCode==0){
//			printf("[succ]position %d \r\n", pkg.content[1]|(pkg.content[0]<<8));
			value = pkg.content[1]|(pkg.content[0]<<8);
	}
	else {value = value - statusCode;}
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

}
