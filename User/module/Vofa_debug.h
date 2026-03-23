#ifndef VOFA_DEBUG_H
#define VOFA_DEBUG_H
#endif


#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include "struct_typedef.h"
#include "usart.h"

#ifdef __cplusplus
}
#endif


/*  
 * 碎碎念：其实最好的办法是设定基类，基类内部所有函数都是虚函数，然后在继承的子类中实现功能。
*/

#pragma once
class vofa_float//浮点数调试
{
    private:
        int data_quantity;// 数据数量
        uint8_t tail[4];
        fp32* tx_buffer_point;// 发送缓存变量
        size_t data_type;// 数据类型表示
        fp32 temp_buffer;// 临时发送缓存
        UART_HandleTypeDef* huart_piont;// 串口句柄

    public:
        void vofa_init(UART_HandleTypeDef* huart);// 通用初始化

        void float_buffer_init(int data_quantity_in , size_t data_type_in = sizeof(fp32));// 多点数初始化

        void float_write(fp32* data_point);// 数据写入

        void float_send(void);// 浮点数 数据发送

};


// #pragma once
// class vofa_fast_float:vofa_float// 快速浮点数调试
// { 
//     void single_float_fast(UART_HandleTypeDef* huart,fp32* data_point);// 快速浮点数发送
// };

