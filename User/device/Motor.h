/**
  ******************************************************************************
  * @file    Motor.h
  * @brief   电机控制模块头文件，包含电机的控制算法、状态监测等相关函数的声明
  * @author  wzk
  ******************************************************************************
  */

#ifndef MOTOR_H
#define MOTOR_H

#include "Pid_1.h"


class Step_motor
{
public:
    uint16_t ecd; //电机编码器值
    int16_t speed; //电机速度
    int16_t current; //电机电流
    int16_t speed_set; //电机目标速度
    int16_t current_set; //电机目标电流
    int16_t current_give; //电机实际输出电流
    pid_data_t pid_speed; //电机速度pid控制器

};

class Servo
{
public:

   uint16_t pwm;

   void servo_motor_init();

};
#endif
