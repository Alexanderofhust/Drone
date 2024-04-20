#ifndef __PC_SERIAL_H
#define __PC_SERIAL_H

#include "stm32f4xx_hal.h"


#pragma pack(push, 1)     //不进行字节对齐

typedef struct PCSendData //数据顺序不能变,注意32字节对齐
{
    int8_t start_flag;
    uint8_t robot_color : 1;
    uint8_t shoot_level : 2;
    uint8_t mode : 2;
    uint8_t which_balance : 2;
    uint8_t change_priority_flag : 1;
    uint8_t frame_id;
    short pitch;
    float yaw;
    int16_t crc16;
} PCSendData;

typedef struct PCRecvData
{
    int8_t start_flag;
	
    uint8_t enemy_id : 3;   
    uint8_t shoot_flag : 1;
    uint8_t mode : 2;       
    uint8_t _ : 2;
	
    uint8_t frame_id;
	
    short pitch;
    float yaw;
		//float distance;
    int16_t crc16;
} PCRecvData;


#pragma pack(pop) //不进行字节对齐

#define PC_SENDBUF_SIZE sizeof(PCSendData)
#define PC_RECVBUF_SIZE sizeof(PCRecvData)


extern PCRecvData 						PC_Recv;

void PCReceive(unsigned char *PCbuffer);
void SendtoPC(void);
#endif
