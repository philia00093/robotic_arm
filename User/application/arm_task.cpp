/**
******************************************************************************
  * @file    arm_task.cpp
  * @brief   机械臂freertos分任务
  * @author  wzk
  * @history 2026-03-21 v1.0 初始版本 done
******************************************************************************
*/

#include "arm_task.h"
#include "FreeRTOS.h"
#include "task.h"
#include "Can_receive.h"
#include "arm.h"

void ArmTask(void *argument)
{
    vTaskDelay(ARM_TASK_INIT_TIME);
    //机械臂初始化
    arm.init();
    //机械臂数据反馈
    arm.feedback_update();
    while (1)
    {
        // 机械臂控制逻辑
        arm.offline_detect();
        //模式选择
        arm.mode_set(); //遥操作等模式
        //数据反馈更新
        arm.feedback_update();
        //解算
        arm.arm_slove();
        //机械臂输出
        arm.arm_output();
        
        vTaskDelay(ARM_CONTROL_TIME); // 机械臂控制延时
    }
}
