/**
******************************************************************************
  * @file    Motor.cpp
  * @brief   电机控制模块实现文件,包括电机与舵机的控制
  * @author  wzk
******************************************************************************
*/

#include "Motor.h"
#include "bsp_can.h"
#include "main.h"
#include "pid.h"


extern "C"
{
    #include "bsp_can.h"
    #include "main.h"
}

//张大头驱动