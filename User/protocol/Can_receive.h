#ifndef CAN_RECEIVE_H
#define CAN_RECEIVE_H

#include "main.h"

#include "struct_typedef.h"

#include "vision.h"

extern FDCAN_HandleTypeDef hfdcan1; // CAN����1
extern FDCAN_HandleTypeDef hfdcan2;


#define SHOOT_CAN hfdcan2
#define GIMBAL_PITCH_CAN hfdcan2
#define GIMBAL_YAW_CAN hfdcan1
#define TELESCOPES_CAN hfdcan2
#define TRIGGER_CAN hfdcan1
#define BOARD_COM_CAN hfdcan1
#define FDCAN_1 hfdcan1
#define FDCAN_2 hfdcan2

#define MIT_MODE 			0x000


//改变编码器的获取
#define P_MIN -12.5  //位置最小值
#define P_MAX  12.5f  //位置最大值
#define V_MIN -30.0f  //速度最小值
#define V_MAX  30.0f  //速度最大值
#define KP_MIN 0.0f   //Kp最小值
#define KP_MAX 500.0f //Kp最大值
#define KD_MIN 0.0f   //Kd最小值
#define KD_MAX 5.0f   //Kd最大值
#define T_MIN -10.0f  //转矩最大值
#define T_MAX  10.0f  //转矩最小值


//云台电机编号
enum gimbal_motor_id_e
{
    //底盘动力电机接收
    YAW_MOTOR = 0,
    PITCH_MOTOR,
};

//发射机构电机编号
enum shoot_motor_id_e
{
    //底盘动力电机接收
    LEFT_FRIC_MOTOR = 0,
    RIGHT_FRIC_MOTOR,
    TRIGGER_MOTOR,

};

typedef enum
{
    //发射机构和Pitch电机接受ID CAN2
    CAN_LEFT_FRIC_MOTOR_ID = 0x201,
    CAN_RIGHT_FRIC_MOTOR_ID = 0x202,
    CAN_UP_FRIC_MOTOR_ID = 0x208,//0X203,

    CAN_OUT_LEFT_FRIC_MOTOR_ID = 0x205,
    CAN_OUT_RIGHT_FRIC_MOTOR_ID = 0x206,
    CAN_OUT_UP_FRIC_MOTOR_ID = 0X207,
  

    //Pitch控制ID
    CAN_PITCH_CMD_ID = 0x301,
    //Pitch接收ID
    CAN_PITCH_REC_ID = 0X300,


    CAN_SHOOT_ALL_ID = 0x200,
    CAN_SHOOT_ADD_ID = 0X1FF,
    
    //云台电机接收ID CAN1
    CAN_TRIGGER_MOTOR_ID = 0x203,
    CAN_YAW_MOTOR_ID = 0x205,
    CAN_GIMBAL_ALL_ID = 0x1FF,

    //板间通信ID
    CAN_RC_BOARM_COM_ID = 0x041,
    CAN_GIMBAL_BOARD_COM_ID = 0x302,
    CAN_COOLING_BOARM_COM_ID = 0x303,
    CAN_SHOOT_SPEED_BOARD_COM_ID = 0x304,
    CAN_UI_COM_ID = 0x305,

    //传输 导航数据
    CAN_Sendpark= 0x306,

} can_msg_id_e;

typedef struct 
{
  uint16_t ecd; // 读出的编码器角度
  int16_t speed_rpm; // 读出的当前转速
  int16_t given_current; // 给的电流值
  uint8_t temperate; // 读出的温度
  int16_t last_ecd; // 上一次的编码器值

  uint8_t round_cnt;  //圈数
    /* data */
}motor_measure_t;

// DM motor data
typedef struct 
{
    uint16_t master_id;       //电机ID值,master_id
    uint16_t state ;          //电机的状态码
    fp32 angle;               //转子位置，单位rad
    fp32 velocity;            //转子速度，单位rad/s
    fp32 torque;              //电机扭矩

}DM_motor_measure_t;


//云台发送数据结构体
typedef struct
{
    //遥控器数据
    int16_t ch_0;
    int16_t ch_2;
    int16_t ch_3;
    uint16_t v;

    uint8_t s0;

    //云台数据
    fp32 gimbal_yaw_angle;
    fp32 gimbal_pitch_angle;

    //云台传输UI数据
    bool_t auto_state;
    bool_t aim_state;
    bool_t fric_state;
    bool_t HA_mode;

    
} gimbal_send_t;

//云台接收数据结构体
typedef struct
{
    //测试热量及ID
    uint16_t shoot_cooling_limit; // shoot测速热量上限
    uint16_t shoot_cooling_heat;  // shoot测速实时热量
    uint8_t color;                //判断红蓝方
    uint8_t robot_id;             //机器人编号



    uint8_t game_progress;

} gimbal_receive_t;

class Can_receive
{
public:

    motor_measure_t gimbal_motor[3];
    //motor_measure_t can2_motor[8];

    DM_motor_measure_t pitch_motor;

    //发射机构电机反馈数据结构体
    motor_measure_t shoot_motor[7];

    FDCAN_TxHeaderTypeDef can_tx_message; // can1前八个

    uint8_t can_send_data[8]; // can1前八个

    //板间通信
    //云台接收信息
    gimbal_receive_t gimbal_receive;
    //云台发送
    gimbal_send_t gimbal_send;

    void init();

    /*--------------------云台电机数据接收----------------------*/
    void can_cmd_yaw_motor(int16_t yaw);
    void can_cmd_pitch_motor(int16_t pitch);

    void get_gimbal_motor_measure(uint8_t num, uint8_t data[8]);
    const motor_measure_t *get_gimbal_motor_measure_point(uint8_t i);
 
    /*--------------------DM_pitch_电机数据接收----------------------*/
    void enable_pitch_motor();             //使能
    void save_motor_zero();                                                        
    uint8_t DM_disable_dog();              //dm 失能看门口
    void can_cmd_pitch_motor_mit(fp32 torq);      //mit控制函数
    void get_pitch_motor_measure(uint8_t data[8]);
    const DM_motor_measure_t *get_pitch_motor_measure_point();
    
    /*-------------------发射机构电机数据接收--------------------*/
    void get_shoot_motor_measure(uint8_t num, uint8_t data[8]);
   
    void can_cmd_shoot_motor(int16_t left_fric, int16_t right_fric, int16_t up_fric, int16_t scope);
    void can_cmd_shoot_add_motor(int16_t out_left_fric ,int16_t out_right_fric, int16_t out_up_fric);
 
    void can_cmd_trigger_motor(int16_t trigger);
    void can_cmd_shoot_motor_reset_ID();
    const motor_measure_t *get_shoot_motor_measure_point(uint8_t i);

    /*-----------------------板间通信函数------------------------*/
    void receive_cooling_and_id_board_com(uint8_t data[8]);
    void receive_shoot_speed_and_mode_board_com(uint8_t data[8]);
    void send_rc_board_com(int16_t ch_0, int16_t ch_2, int16_t ch_3, uint16_t v);            //发送遥控器数据
    void send_gimbal_board_com(uint8_t s0, uint8_t gimbal_behaviour, fp32 gimbal_yaw_angle); //发送云台模式及状态
    void send_UI_com(bool_t auto_s, bool_t aim_s, bool_t fric_s, bool_t HA_mode_s,fp32 gimbal_pitch_angle); //发送UI数据
    //发送导航数据
  //  void send_SendPacket(SendPacketTwist_t& SendPacketTwist);//发送视觉数据

};
void can_filter_init(void);


#endif