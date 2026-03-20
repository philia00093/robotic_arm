#ifndef __VISION_H
#define __VISION_H

#include "main.h"
#include "struct_typedef.h"

#include "bsp_usart.h"

#define VISION_BUFFER_LEN 200
#define NOW 0
#define LAST 1

#define ATTACK_NONE 0 //不识别
#define ATTACK_RED 1  //识别红方
#define ATTACK_BLUE 2 //识别蓝方

#define VISION_DATA_ERROR 0	  //视觉数据错误
#define VISION_DATA_CORRECT 1 //视觉数据错误

#define VISION_LEN_HEADER 3		  //帧头长
#define VISION_LEN_DATA 15		  //数据段长度,可自定义

#define VISION_SEND_LEN_PACKED 16 //发送数据包长度
#define VISION_READ_LEN_PACKED 18 //接受数据包长度

#define VISION_SEND_LEN_PACKED_PC 15 //发送数据包长度
#define VISION_READ_LEN_PACKED_PC 16 //接受数据包长度

#define VISION_OFF (0x00)			  //关闭视觉
#define VISION_RED (0x01)			  //识别红色
#define VISION_BLUE (0x02)			  //识别蓝色
#define VISION_RBUFF_ANTI (0x03)	  //红逆 大符
#define VISION_BBUFF_ANTI (0x04)	  //蓝逆 大符
#define VISION_RBUFF_CLOCKWISE (0x05) //红顺 大符
#define VISION_BBUFF_CLOCKWISE (0x06) //蓝顺 大符
#define VISION_RBUFF_STAND (0x07)	  //红 小符
#define VISION_BBUFF_STAND (0x08)	  //蓝 小符

//起始字节,协议固定为0xA5
#define VISION_BEGIN (0xA5) //可更改
#define VISION_END (0xFF)	//帧尾

/*-------视觉分辨率预编译--------*/
#define VISION_MID_YAW 444	 // 640
#define VISION_MID_PITCH 500 // 360

/*------------------自瞄预编译,角度初始化补偿------------------------*/
#define COMPENSATION_YAW 0
#define COMPENSATION_PITCH 0
#define COMPENSATION_PITCH_DIST 0

/* 	STM32 -> PC

	CmdID   0x00   关闭视觉
	CmdID   0x01   识别红色装甲
	CmdID   0x02   识别蓝色装甲
	CmdID   0x03   红符
	CmdID   0x04   蓝符
*/

/* 	PC -> STM32

	CmdID   0x00   关闭视觉
	CmdID   0x01   识别红色装甲
	CmdID   0x02   识别蓝色装甲
	CmdID   0x03   小符
	CmdID   0x04   大符
*/

//可利用收和发的指令码进行比较,当收和发的指令码相同时,可判定为数据可用

// PC收发与STM32收发成镜像关系,以下结构体适用于STM32,PC需稍作修改



extern uint8_t CmdID;
extern UART_HandleTypeDef huart1;
extern DMA_HandleTypeDef hdma_usart1_rx;

extern uint8_t Vision_Buffer[2][VISION_BUFFER_LEN]; //视觉数据暂存

extern void vision_init();
extern bool_t vision_if_find_target();

/*----------------------------------------------导航-------------------------------------------------*/

typedef __packed struct
{
	uint8_t header;

	fp32 linear_x;
	fp32 linear_y;
	fp32 linear_z;
	
	fp32 vacant1;
	fp32 vacant2;
	float angular_z;
	uint8_t mode;

}SendPacketTwist_t;

#ifdef __cplusplus
extern "C" {
#endif
void vision_read_SendPacketTwist(uint8_t *ReadFormUart);
#ifdef __cplusplus
};
#endif

/*----------------------------------------------英雄辅助瞄准-------------------------------------------------*/

typedef __packed struct
{
	uint8_t startflag;
	uint8_t flag;
	int16_t pitch;
	int32_t yaw;
	uint8_t color;
	int32_t time_stamp;
	int16_t roll;
	uint8_t right_clicked;
	int8_t user_time_bias;
	int8_t outpost_state;
	uint8_t crc;
}SendHelpedAim_t;

typedef __packed struct
{

	uint8_t flag; //帧头起始位,暂定0xA5
	int16_t pitch;
	int32_t yaw;
	uint8_t shootStatus;
	int32_t time_stamp;
	uint8_t state;
	uint8_t num;
	uint8_t crc;

}ReadHelpedAim_t;

extern SendHelpedAim_t SendHelpedAim;
extern ReadHelpedAim_t ReadHelpedAim; 

void vision_read_data_HA(uint8_t *ReadFormUart,uint8_t data_len);

#ifdef __cplusplus
extern "C" {
#endif

void vision_send_data_HA(uint8_t CmdID);

#ifdef __cplusplus
};
#endif

extern void vision_error_angle_HA(float *yaw_angle_error, float *pitch_angle_error);

#endif
