#include "vision.h"
#include "Remote_control.h"
#include "struct_typedef.h"
#include "string.h"
#include "INS.h"
#include "tim.h"
#include "gimbal.h"



#ifdef __cplusplus
extern "C"
{
#endif

#include "CRC.h"

#ifdef __cplusplus
}
#endif

// #include "referee.h"

extern UART_HandleTypeDef huart1;
extern DMA_HandleTypeDef hdma_usart1_rx;

uint8_t Vision_Buffer[2][VISION_BUFFER_LEN]; //视觉数据暂存

extern RC_ctrl_t rc_ctrl;
//extern INS imu;
extern Gimbal gimbal;



//打符是否换装甲了
uint8_t Vision_Armor = FALSE;

//是否识别到装甲板
bool_t if_identify_target = FALSE;


static uint64_t send_cnt = 0;

// 添加计数器变量定义
uint32_t target_detect_count = 0;

void vision_init()
{
  usart1_init(Vision_Buffer[0], Vision_Buffer[1], VISION_BUFFER_LEN);
}


/**
  * @brief  读取视觉信息
  * @param  uart1缓存数据
  * @retval void
  * @attention  IRQ执行
  */
uint8_t Vision_Time_Test[2] = {0}; //当前数据和上一次数据
uint8_t Vision_Ping = 0;           //发送时间间隔

/**
  * @brief          定时器周期给视觉发送陀螺仪数据
  * @param[in]      htim:定时器指针
  * @retval         none
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim == &htim1)
  {
    HAL_GPIO_TogglePin(CRAMA_TRI_GPIO_Port, CRAMA_TRI_Pin);
    if(send_cnt++ % 2 == 0)
    {

    }
  }
}




uint8_t CmdID = 0;

/**
  * @brief  判断是否识别到装甲板
  * @param  void
  * @retval TRUE识别到   FALSE未识别到
  * @attention  为自瞄做准备
  */
bool_t vision_if_find_target(void)
{
  return if_identify_target;
}

/**
  * @brief  判断换装甲板了吗
  * @param  void
  * @retval TRUE换了   FALSE没换
  * @attention  为自动打符做准备,串口空闲中断每触发一次且通过校验,则Vision_Armor置TRUE
  */
bool_t vision_if_armor(void)
{
  return Vision_Armor;
}

/**
  * @brief  换装甲标志位手动置0(false)
  * @param  void
  * @retval void
  * @attention  记得要清零,在哪清零自己选,调用这个函数就行
  */
void vision_clean_ammorflag(void)
{
  Vision_Armor = FALSE;
}


/*----------------------------------------------导航-------------------------------------------------*/
//拍视频用

SendPacketTwist_t SendPacketTwist;  //视觉发送导航数据体

#define NAVIGATION_BEGIN 0xA4

void  vision_read_SendPacketTwist(uint8_t *ReadFormUart){
  if(ReadFormUart[0] == NAVIGATION_BEGIN)
  {
    //swapEndian(ReadFormUart,25);
     memcpy(&SendPacketTwist, ReadFormUart,25);
  }
}
/*----------------------------------------------英雄辅助瞄准-------------------------------------------------*/

SendHelpedAim_t SendHelpedAim;
ReadHelpedAim_t ReadHelpedAim;

int32_t time_stamp_; 

int test_bias = 1;
uint8_t vision_send_pack_HA[sizeof(SendHelpedAim)] = {0};
unsigned char *framepoint;                      //读写指针
void vision_send_data_HA(uint8_t CmdID)
{
     
 
      SendHelpedAim.startflag = VISION_BEGIN;

      if (gimbal.attack_target_change_flag == 1)
      {
        SendHelpedAim.flag = 0x06;
      }
      else
      {
        SendHelpedAim.flag = 0x05;
      }
      

      SendHelpedAim.pitch = (int16_t)(gimbal.gimbal_pitch.gyro_angle*5730);
      SendHelpedAim.yaw = (int32_t)(gimbal.gimbal_yaw_motor.gyro_angle*5730);
  
      SendHelpedAim.color = 1-can_receive.gimbal_receive.color;
      
      SendHelpedAim.time_stamp = time_stamp_;

   //   SendHelpedAim.roll =(int16_t)(imu.INS_angle[2]*5730);
      
      if(gimbal.right_click_flag == 1)
      {
        SendHelpedAim.right_clicked = 1;
      }
      else
      {
        SendHelpedAim.right_clicked = 0;
      }
      
    
      //SendHelpedAim.user_time_bias = 1;
      SendHelpedAim.user_time_bias = test_bias;
      SendHelpedAim.outpost_state = 1;

      framepoint = (uint8_t *)&SendHelpedAim;
      SendHelpedAim.crc = Get_CRC8_Check_Sum(framepoint,sizeof(SendHelpedAim),0XFF);
      
    memcpy(vision_send_pack_HA, &SendHelpedAim,sizeof(SendHelpedAim));
  
  HAL_UART_Transmit(&huart1, vision_send_pack_HA, sizeof(vision_send_pack_HA), 0xFFF);
}

float read_pitch_HA;
float read_yaw_HA; 

void vision_read_data_HA(uint8_t *ReadFormUart,uint8_t data_len)
{
  uint8_t data_tmp[15];
  memcpy(data_tmp,ReadFormUart,data_len);

 //判断帧头数据是否为0xA5
  if (data_tmp[0] == VISION_BEGIN)
  {
    // if(verify_CRC8_check_sum(data_tmp,data_len))
    // {
      ReadHelpedAim.flag = data_tmp[1];
      ReadHelpedAim.pitch = (data_tmp[2] << 8) | data_tmp[3];
      ReadHelpedAim.yaw = ( ( (int)data_tmp[4])<<24)
                          |(( (int)data_tmp[5])<<16)
                          |(( (int)data_tmp[6])<<8 )
                          |   (data_tmp[7]);

      
      //read_pitch_HA = -(ReadHelpedAim.pitch/5730.f - gimbal.gimbal_pitch_motor.gyro_angle);
      read_pitch_HA = -(ReadHelpedAim.pitch/5730.f - gimbal.gimbal_pitch.gyro_angle);
      read_yaw_HA = -(ReadHelpedAim.yaw/5730.f - gimbal.gimbal_yaw_motor.gyro_angle);
      
      //read_pitch_HA = -ReadHelpedAim.pitch/5730.f;
      //read_yaw_HA = -ReadHelpedAim.yaw/5730.f;

			if(ReadHelpedAim.yaw  == 0 && ReadHelpedAim.pitch == 0)
      {
        if_identify_target = FALSE;
      }
      else
      {
        if_identify_target = TRUE;
      }

      ReadHelpedAim.shootStatus = data_tmp[8];

      
      ReadHelpedAim.time_stamp = ( ( (int)data_tmp[9])<<24)
                                |(( (int)data_tmp[10])<<16)
                                |(( (int)data_tmp[11])<<8 )
                                |   (data_tmp[12]);
      ReadHelpedAim.state = data_tmp[13];
      ReadHelpedAim.num = data_tmp[14];
    //}
  }
}


uint16_t yaw_para_HA = 125;
uint16_t pitch_para_HA = 150;
fp32 error_pitch_HA = 0 ;
fp32 error_yaw_HA = 0;

void vision_error_angle_HA(float *yaw_angle_error, float *pitch_angle_error)
{
  
  *yaw_angle_error = -(read_yaw_HA + error_yaw_HA )/ yaw_para_HA;
  *pitch_angle_error = -(read_pitch_HA + error_pitch_HA) / pitch_para_HA;

  if (ReadHelpedAim.yaw == 0)
  {
    *yaw_angle_error = 0;
  }
  if (ReadHelpedAim.pitch == 0)
  {
    *pitch_angle_error = 0;
  }
}
