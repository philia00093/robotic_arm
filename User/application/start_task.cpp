/**
******************************************************************************
  * @file    start_task.cpp
  * @brief   系统启动任务
  * @author  wzk
  * @history 2026-03-21 v1.0 初始版本 done
******************************************************************************
*/

#include "start_task.h"
#include "FreeRTOS.h"
#include "task.h"

TaskHandle_t arm_task_handle;
void StartTask(void *argument)
{
    vTaskDelay(START_TASK_INIT_TIME);
    // 系统初始化
    // ...
    while (1)
    {        
        // 创建其他任务
        xTaskCreate(arm_task, "arm_task", Large_Stack_Size, NULL, PriorityHigh, &arm_task_handle);
    }
}

}