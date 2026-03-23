/**
  ******************************************************************************
  * @file    start_task.h
  * @brief   机械臂启动任务头文件，包含机械臂启动任务的相关函数声明和宏定义
  * @author  wzk
  ******************************************************************************
  */
#ifndef START_TASK_H
#define START_TASK_H

#define START_TASK_INIT_TIME 1000 // 启动任务初始化时间，单位为毫秒

#include "arm_task.h"

void StartTask(void *argument);

#endif
