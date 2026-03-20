#include "Can_receive.h"
#include "cmsis_os.h"
#include "main.h"
#ifdef __cplusplus
extern "C"
{
#endif

#include "bsp_delay.h"

#ifdef __cplusplus
}
#endif
#include "bsp_can.h"
#include "fdcan.h"

#include "struct_typedef.h"

#include "vision.h"

extern FDCAN_HandleTypeDef hfdcan1; // CAN����1
extern FDCAN_HandleTypeDef hfdcan2;


void Can_receive::init()
{
    can_filter_init();
}

void can_filter_init(void)
{
   FDCAN_FilterTypeDef filter;                   	//< 声明局部变量 can过滤器结构体
	filter.IdType       = FDCAN_STANDARD_ID;       	//< id设置为标准id
	filter.FilterIndex  = 0;                      	//< 设值筛选器的编号，标准id选择0-127
	filter.FilterType   = FDCAN_FILTER_MASK;       	//< 设置工作模式为掩码模式
	filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0; 	//< 将经过过滤的数据存储到 fifo0
	filter.FilterID1    = 0x000;                   	//< 筛选器的id
	filter.FilterID2    = 0x000;
	
	HAL_FDCAN_ConfigFilter(&hfdcan1, &filter);   //< 配置过滤器	
  HAL_FDCAN_Start(&hfdcan1);                   //< 使能can
    //该check来测试can控制器是否使能，可以把该赋值去掉	
	HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);  // 使能fifo0接收到新信息中断

  HAL_FDCAN_ConfigFilter(&hfdcan2, &filter);
  HAL_FDCAN_Start(&hfdcan2);
  HAL_FDCAN_ActivateNotification(&hfdcan2, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);
}

/*-----------------------------发送函数------------------------------*/
void Can_receive::can_cmd_yaw_motor(int16_t yaw)
{
  can_tx_message.Identifier = CAN_GIMBAL_ALL_ID;
	can_tx_message.IdType = FDCAN_STANDARD_ID;
  can_tx_message.TxFrameType = FDCAN_DATA_FRAME;
  can_tx_message.DataLength = FDCAN_DLC_BYTES_8;
  can_send_data[0] = yaw >> 8;
  can_send_data[1] = yaw;
  can_send_data[2] = 0 ;
  can_send_data[3] = 0 ;
  can_send_data[4] = 0 ;
  can_send_data[5] = 0 ;
  can_send_data[6] = 0 ;
  can_send_data[7] = 0 ;

  HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &can_tx_message, can_send_data);

}

void Can_receive::can_cmd_pitch_motor(int16_t pitch)
{
  can_tx_message.Identifier = CAN_GIMBAL_ALL_ID;
	can_tx_message.IdType = FDCAN_STANDARD_ID;
  can_tx_message.TxFrameType = FDCAN_DATA_FRAME;
  can_tx_message.DataLength = FDCAN_DLC_BYTES_8;
  can_send_data[0] = pitch >> 8;
  can_send_data[1] = pitch;
  can_send_data[2] = 0 ;
  can_send_data[3] = 0 ;
  can_send_data[4] = 0 ;
  can_send_data[5] = 0 ;
  can_send_data[6] = 0 ;
  can_send_data[7] = 0 ;
  HAL_FDCAN_AddMessageToTxFifoQ(&GIMBAL_PITCH_CAN, &can_tx_message, can_send_data);

}

//----------------------------------------------- 有关云台Pitch电机的---------------------------------------------------------//
//浮点转整形算法
static uint16_t float_to_uint(float x, float x_min, float x_max, uint8_t bits)
{
    float span = x_max - x_min;
    float offset = x_min;
    
    return (uint16_t) ((x-offset)*((float)((1<<bits)-1))/span);
}

//整形转浮点算法
static float uint_to_float(int x_int, float x_min, float x_max, int bits)
{
    float span = x_max - x_min;
    float offset = x_min;
    return ((float)x_int)*span/((float)((1<<bits)-1)) + offset;
}

void Can_receive::enable_pitch_motor()
{
  uint16_t id = CAN_PITCH_CMD_ID + MIT_MODE;

  can_tx_message.Identifier = id;
	can_tx_message.IdType = FDCAN_STANDARD_ID;
  can_tx_message.TxFrameType = FDCAN_DATA_FRAME;
  can_tx_message.DataLength = FDCAN_DLC_BYTES_8;

  can_send_data[0] = 0xFF;
	can_send_data[1] = 0xFF;
	can_send_data[2] = 0xFF;
	can_send_data[3] = 0xFF;
	can_send_data[4] = 0xFF;
	can_send_data[5] = 0xFF;
	can_send_data[6] = 0xFF;
	can_send_data[7] = 0XFC;

  HAL_FDCAN_AddMessageToTxFifoQ(&GIMBAL_PITCH_CAN, &can_tx_message, can_send_data);

}

void Can_receive::save_motor_zero()
{
  uint16_t id = CAN_PITCH_CMD_ID + MIT_MODE;

  can_tx_message.Identifier = id;
	can_tx_message.IdType = FDCAN_STANDARD_ID;
  can_tx_message.TxFrameType = FDCAN_DATA_FRAME;
  can_tx_message.DataLength = FDCAN_DLC_BYTES_8;

  can_send_data[0] = 0xFF;
	can_send_data[1] = 0xFF;
	can_send_data[2] = 0xFF;
	can_send_data[3] = 0xFF;
	can_send_data[4] = 0xFF;
	can_send_data[5] = 0xFF;
	can_send_data[6] = 0xFF;
	can_send_data[7] = 0XFE;

  HAL_FDCAN_AddMessageToTxFifoQ(&GIMBAL_PITCH_CAN, &can_tx_message, can_send_data);
}

uint16_t call_back_cnt = 0;
uint8_t ERR = 0;

void Can_receive::can_cmd_pitch_motor_mit(fp32 torq)
{
    uint16_t pos_tmp,vel_tmp,kp_tmp,kd_tmp,tor_tmp;
	uint16_t id = CAN_PITCH_CMD_ID + MIT_MODE;

    fp32 pos = 0;
    fp32 vel = 0;
    fp32 kp  = 0;
    fp32 kd  = 0;

	pos_tmp = float_to_uint(pos,  P_MIN,  P_MAX,  16);
	vel_tmp = float_to_uint(vel,  V_MIN,  V_MAX,  12);
	kp_tmp  = float_to_uint(kp,   KP_MIN, KP_MAX, 12);
	kd_tmp  = float_to_uint(kd,   KD_MIN, KD_MAX, 12);
	tor_tmp = float_to_uint(torq, T_MIN,  T_MAX,  12);

  can_tx_message.Identifier = id;
	can_tx_message.IdType = FDCAN_STANDARD_ID;
  can_tx_message.TxFrameType = FDCAN_DATA_FRAME;
  can_tx_message.DataLength = FDCAN_DLC_BYTES_8;

  can_send_data[0] = (pos_tmp >> 8);
	can_send_data[1] = pos_tmp;
	can_send_data[2] = (vel_tmp >> 4);
	can_send_data[3] = ((vel_tmp&0xF)<<4)|(kp_tmp>>8);
	can_send_data[4] = kp_tmp;
	can_send_data[5] = (kd_tmp >> 4);
	can_send_data[6] = ((kd_tmp&0xF)<<4)|(tor_tmp>>8);
	can_send_data[7] = tor_tmp;

  HAL_FDCAN_AddMessageToTxFifoQ(&GIMBAL_PITCH_CAN, &can_tx_message, can_send_data);
	
}

void Can_receive::get_pitch_motor_measure(uint8_t data[8])
{
    uint16_t p_int,v_int,t_int;

    pitch_motor.master_id = (data[0])&0x0F;
	pitch_motor.state = (data[0])>>4;

	p_int=(data[1]<<8)|data[2];
	v_int=(data[3]<<4)|(data[4]>>4);
	t_int=((data[4]&0xF)<<8)|data[5];

	pitch_motor.angle = uint_to_float(p_int, P_MIN, P_MAX, 16); // (-12.5,12.5)
	pitch_motor.velocity = uint_to_float(v_int, V_MIN, V_MAX, 12); // (-30.0,30.0)
	pitch_motor.torque = uint_to_float(t_int, T_MIN, T_MAX, 12);  // (-10.0,10.0)

    ERR = pitch_motor.state;
    if(ERR == 1)
    {
        call_back_cnt = 0;
    }
}

# define DM_LOSE_ERROR_CNT 10

uint8_t Can_receive::DM_disable_dog()
{
    call_back_cnt ++;
    if (call_back_cnt >= DM_LOSE_ERROR_CNT)
    {
        return 0;
        /* code */
    }
    return 1;
}



/*-----------------------------接收函数------------------------------*/
void Can_receive::get_gimbal_motor_measure(uint8_t num, uint8_t data[8])
{
    gimbal_motor[num].last_ecd = gimbal_motor[num].ecd;
    gimbal_motor[num].ecd = (uint16_t)(data[0] << 8 | data[1]);
    gimbal_motor[num].speed_rpm = (uint16_t)(data[2] << 8 | data[3]);
    gimbal_motor[num].given_current = (uint16_t)(data[4] << 8 | data[5]);
    gimbal_motor[num].temperate = data[6];
}

// void Can_receive::get_can2_motor_measure(uint8_t num, uint8_t data[8])
// {
//     can2_motor[num].last_ecd = can2_motor[num].ecd;
//     can2_motor[num].ecd = (uint16_t)(data[0] << 8 | data[1]);
//     can2_motor[num].speed_rpm = (uint16_t)(data[2] << 8 | data[3]);
//     can2_motor[num].given_current = (uint16_t)(data[4] << 8 | data[5]);
//     can2_motor[num].temperate = data[6];
// }


/**
 * @brief          返回云台电机 6020电机数据指针
 * @param[in]      i: 电机编号,范围[0,1]
 * @retval         电机数据指针
 */
const motor_measure_t *Can_receive::get_gimbal_motor_measure_point(uint8_t i)
{
    return &gimbal_motor[i];
}

const DM_motor_measure_t *Can_receive::get_pitch_motor_measure_point()
{
    return &pitch_motor;
}

void Can_receive::get_shoot_motor_measure(uint8_t num, uint8_t data[8])
{
 
    shoot_motor[num].ecd = (uint16_t)(data[0] << 8 | data[1]);
    shoot_motor[num].speed_rpm = (uint16_t)(data[2] << 8 | data[3]);
    shoot_motor[num].given_current = (uint16_t)(data[4] << 8 | data[5]);
    shoot_motor[num].temperate = data[6];
    
    if (shoot_motor[num].ecd - shoot_motor[num].last_ecd > 4096)
    {
        shoot_motor[num].round_cnt--;
    }
    else if (shoot_motor[num].ecd - shoot_motor[num].last_ecd < -4096)
    {
        shoot_motor[num].round_cnt++;
    }
    shoot_motor[num].last_ecd = shoot_motor[num].ecd;
}

void Can_receive::can_cmd_shoot_motor(int16_t left_fric, int16_t right_fric, int16_t up_fric, int16_t scope)
{
    can_tx_message.Identifier = CAN_SHOOT_ADD_ID;
    can_tx_message.IdType = FDCAN_STANDARD_ID;
    can_tx_message.TxFrameType = FDCAN_DATA_FRAME;
    can_tx_message.DataLength = FDCAN_DLC_BYTES_8;
    can_send_data[0] = left_fric >> 8;
    can_send_data[1] = left_fric;
    can_send_data[2] = right_fric >> 8;
    can_send_data[3] = right_fric;
    can_send_data[4] = up_fric>>8;
    can_send_data[5] = up_fric;
    can_send_data[6] = scope >> 8;
    can_send_data[7] = scope;

  HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &can_tx_message, can_send_data);
}

void Can_receive::can_cmd_shoot_add_motor(int16_t out_left_fric ,int16_t out_right_fric, int16_t out_up_fric)
{
    can_tx_message.Identifier = CAN_SHOOT_ADD_ID;
    can_tx_message.IdType = FDCAN_STANDARD_ID;
    can_tx_message.TxFrameType = FDCAN_DATA_FRAME;
    can_tx_message.DataLength = FDCAN_DLC_BYTES_8;
    can_send_data[0] = out_left_fric >> 8;
    can_send_data[1] = out_left_fric;
    can_send_data[2] = out_right_fric >> 8;
    can_send_data[3] = out_right_fric;
    can_send_data[4] = out_up_fric>>8;
    can_send_data[5] = out_up_fric;
    can_send_data[6] = 0;
    can_send_data[7] = 0;

  HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &can_tx_message, can_send_data);
}


/**
 * @brief          发送Trigger电机电流
 * @param[in]      trigger轴电流 trigger轴电流
 */
void Can_receive::can_cmd_trigger_motor(int16_t trigger)
{
    can_tx_message.Identifier = 0x200;
    can_tx_message.IdType = FDCAN_STANDARD_ID;
    can_tx_message.TxFrameType = FDCAN_DATA_FRAME;
    can_tx_message.DataLength = FDCAN_DLC_BYTES_8;

    can_send_data[0] = 0;
    can_send_data[1] = 0;
    can_send_data[2] = 0;
    can_send_data[3] = 0;
    can_send_data[4] = trigger >> 8;
    can_send_data[5] = trigger;
    can_send_data[6] = 0;
    can_send_data[7] = 0;

  HAL_FDCAN_AddMessageToTxFifoQ(&TRIGGER_CAN, &can_tx_message, can_send_data);
}

/**
 * @brief          发送ID为0x700的CAN包,它会设置3508电机进入快速设置ID
 * @param[in]      none
 * @retval         none
 */
void Can_receive::can_cmd_shoot_motor_reset_ID(void)
{
    can_tx_message.Identifier = 0x700;
    can_tx_message.IdType = FDCAN_STANDARD_ID;
    can_tx_message.TxFrameType = FDCAN_DATA_FRAME;
    can_tx_message.DataLength = FDCAN_DLC_BYTES_8;
    can_send_data[0] = 0;
    can_send_data[1] = 0;
    can_send_data[2] = 0;
    can_send_data[3] = 0;
    can_send_data[4] = 0;
    can_send_data[5] = 0;
    can_send_data[6] = 0;
    can_send_data[7] = 0;

  HAL_FDCAN_AddMessageToTxFifoQ(&SHOOT_CAN, &can_tx_message, can_send_data);
}

/**
 * @brief          返回云台电机 6020电机数据指针
 * @param[in]      i: 电机编号,范围[0,1]
 * @retval         电机数据指针
 */
const motor_measure_t *Can_receive::get_shoot_motor_measure_point(uint8_t i)
{
    return &shoot_motor[i];
}

void Can_receive::receive_cooling_and_id_board_com(uint8_t data[8])
{
    gimbal_receive.shoot_cooling_limit = (uint16_t)(data[0] << 8 | data[1]);
    gimbal_receive.shoot_cooling_heat = (uint16_t)(data[4] << 8 | data[5]);
    gimbal_receive.color = (data[6]);
    gimbal_receive.robot_id = (data[7]);
}

void Can_receive::receive_shoot_speed_and_mode_board_com(uint8_t data[8])
{
    gimbal_receive.game_progress = data[5];

}

/*----------------------------------------- 板间通讯 ---------------------------------------------*/

void Can_receive::send_rc_board_com(int16_t ch_0, int16_t ch_2, int16_t ch_3, uint16_t v)
{
    //数据填充
    gimbal_send.ch_0 = ch_0;
    gimbal_send.ch_2 = ch_2;
    gimbal_send.ch_3 = ch_3;
    gimbal_send.v = v;

    can_tx_message.Identifier = CAN_RC_BOARM_COM_ID;
    can_tx_message.IdType = FDCAN_STANDARD_ID;
    can_tx_message.TxFrameType = FDCAN_DATA_FRAME;
    can_tx_message.DataLength = FDCAN_DLC_BYTES_8;
    can_send_data[0] = ch_0 >> 8;
    can_send_data[1] = ch_0;
    can_send_data[2] = ch_2 >> 8;
    can_send_data[3] = ch_2;
    can_send_data[4] = ch_3 >> 8;
    can_send_data[5] = ch_3;
    can_send_data[6] = v >> 8;
    can_send_data[7] = v;

    HAL_FDCAN_AddMessageToTxFifoQ(&BOARD_COM_CAN, &can_tx_message, can_send_data);
}

void Can_receive::send_gimbal_board_com(uint8_t s0, uint8_t gimbal_behaviour, fp32 gimbal_yaw_angle)
{
    int32_t temp_gimbal_yaw_angle = (int32_t)(gimbal_yaw_angle * 10000);

    //数据填充
    gimbal_send.s0 = s0;
    gimbal_send.gimbal_yaw_angle = gimbal_yaw_angle;

    can_tx_message.Identifier = CAN_GIMBAL_BOARD_COM_ID;
    can_tx_message.IdType = FDCAN_STANDARD_ID;
    can_tx_message.TxFrameType = FDCAN_DATA_FRAME;
    can_tx_message.DataLength = FDCAN_DLC_BYTES_8;
    can_send_data[0] = s0;
    can_send_data[1] = 0;
    can_send_data[2] = (uint8_t)((int32_t)temp_gimbal_yaw_angle >> 24);
    can_send_data[3] = (uint8_t)((int32_t)temp_gimbal_yaw_angle >> 16);
    can_send_data[4] = (uint8_t)((int32_t)temp_gimbal_yaw_angle >> 8);
    can_send_data[5] = (uint8_t)((int32_t)temp_gimbal_yaw_angle);
    can_send_data[6] = 0;
    can_send_data[7] = 0;

    HAL_FDCAN_AddMessageToTxFifoQ(&BOARD_COM_CAN, &can_tx_message, can_send_data);
}

void Can_receive::send_UI_com(bool_t auto_s, bool_t aim_s, bool_t fric_s, bool_t HA_mode_s,fp32 gimbal_pitch_angle)
{
    int32_t temp_gimbal_pitch_angle = (int32_t)(gimbal_pitch_angle * 10000);

    //数据填充
    gimbal_send.auto_state = auto_s;
    gimbal_send.aim_state = aim_s;
    gimbal_send.fric_state = fric_s;
    gimbal_send.HA_mode = HA_mode_s;

    gimbal_send.gimbal_pitch_angle = gimbal_pitch_angle;

    can_tx_message.Identifier = CAN_UI_COM_ID;
    can_tx_message.IdType = FDCAN_STANDARD_ID;
    can_tx_message.TxFrameType = FDCAN_DATA_FRAME;
    can_tx_message.DataLength = FDCAN_DLC_BYTES_8;
    can_send_data[0] = auto_s;
    can_send_data[1] = aim_s;
    can_send_data[2] = fric_s;
    can_send_data[3] = HA_mode_s;
    can_send_data[4] = (uint8_t)((int32_t)temp_gimbal_pitch_angle >> 24);
    can_send_data[5] = (uint8_t)((int32_t)temp_gimbal_pitch_angle >> 16);
    can_send_data[6] = (uint8_t)((int32_t)temp_gimbal_pitch_angle >> 8);
    can_send_data[7] = (uint8_t)((int32_t)temp_gimbal_pitch_angle );

    HAL_FDCAN_AddMessageToTxFifoQ(&BOARD_COM_CAN, &can_tx_message, can_send_data);
}


///*------------------------------------------------  拍视频专用  -------------------------------------------------------------*/

////导航发送数据
//fp32 u_X;
//fp32 u_Y;
//fp32 u_Z;

//void Can_receive::send_SendPacket(SendPacketTwist_t& SendPacketTwist)
//{
//    can_tx_message.Identifier = CAN_Sendpark;
//    can_tx_message.IdType = FDCAN_STANDARD_ID;
//    can_tx_message.TxFrameType = FDCAN_DATA_FRAME;
//    can_tx_message.DataLength = FDCAN_DLC_BYTES_8;
//    u_X=SendPacketTwist.linear_x;
//    u_Y=SendPacketTwist.linear_y;
//    u_Z=SendPacketTwist.angular_z;

//    // 将 float 转换为 uint32_t
//    uint32_t u_x = *reinterpret_cast<uint32_t*>(&u_X);
//    uint32_t u_y = *reinterpret_cast<uint32_t*>(&u_Y);
//    uint32_t u_z = *reinterpret_cast<uint32_t*>(&u_Z);

//    // 将转换后的 uint32_t 数据按字节写入 can_send_data 数组
//    can_send_data[1] = (uint8_t)(u_x >> 24);
//    can_send_data[2] = (uint8_t)(u_x >> 16);
//    can_send_data[3] = (uint8_t)(u_y >> 24);
//    can_send_data[4] = (uint8_t)(u_y >> 16);

//    can_send_data[5] = (uint8_t)(u_z >> 24);
//    can_send_data[6] = (uint8_t)(u_z >> 16);
//    can_send_data[7] = SendPacketTwist.mode;

//    HAL_FDCAN_AddMessageToTxFifoQ(&BOARD_COM_CAN, &can_tx_message, can_send_data);
//    
//}

