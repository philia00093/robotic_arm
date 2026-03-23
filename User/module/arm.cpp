/**
  ******************************************************************************
  * @file    arm.cpp
  * @brief   机械臂控制模块，包含机械臂的运动学、动力学、控制算法等相关函数的实现
  * @author  wzk
  ******************************************************************************
  */

#include "arm.h"
#include "Pid_1.h"

#ifdef __cplusplus

//PID对象实例化
Pid speed_pid;
Pid omega_pid;

/*机械臂对象实例化*/
Arm arm;

/*机械臂控制函数实现*/
/**
 * @brief 机械臂初始化函数
 * @details 初始化机械臂相关资源，如CAN通信、PID控制器等
 * @param void 
 * @return void
 */
void Arm::init()
{
    // 初始化CAN通信
    can_Init();
    
    // 初始化PID控制器,L一开始使能时回到零点
    for(int i = 0; i < 6; i++)
    {
        speed_pid.Init(&arm_motor_pid[i]);
        omega_pid.Init(&arm_motor_pid[i]);
    }
}

/**
 * @brief 机械臂数据反馈更新函数
 * @details 从CAN总线接收机械臂电机的反馈数据，并更新
 * 机械臂的状态信息，如电机编码值、速度、电流等
 * @param void
 * @return void
 */
void Arm::arm_offline_detect(){

}

/**
 * @brief 机械臂模式设置函数
 * @details 根据当前的控制模式（如遥操作、自动控制等），设置机械臂的控制参数和状态
 * @param void
 * @return void
 */
void Arm::mode_set(){
    
}

/**
 * @brief 机械臂解算函数
 * @details 根据机械臂当前状态和目标状态，计算每个关节的目标速度和电流，
 * 并进行限幅处理六轴机械臂解算机械臂前五轴为步进电机，最后一轴为舵机
 * @param void
 * @return void
 */
void Arm::arm_slove(){

}

/**
 * @brief 机械臂输出函数
 * @details 将计算得到的目标速度和电流通过CAN总线发送给机械臂的电机控制器，
 * 实现对机械臂的控制
 * @param void
 * @return void
 */

void Arm::arm_output(){

}
