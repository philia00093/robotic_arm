#include "bsp_can.h"
#include "main.h"
extern FDCAN_HandleTypeDef hfdcan1;
extern FDCAN_HandleTypeDef hfdcan2;

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






//*******************************************************************�¼�
