#include "Vofa_debug.h"
#include <stdio.h>
#include <stdlib.h>


/*-------------以下为多浮点数调试模块-------------*/
/**
 * @brief 初始化VOFA调试模块
 * @param huart UART句柄指针，用于串口通信
 * @note 该函数初始化帧尾，并保存UART句柄指针
 */
void vofa_float::vofa_init(UART_HandleTypeDef* huart) 
{
    // 帧尾初始化
    tail[0]=0x00;
    tail[1]=0x00;
    tail[2]=0x80;
    tail[3]=0x07f;

    // 串口指针传递
    huart_piont = huart;
}

/**
 * @brief 初始化浮点数缓存
 * @param data_quantity_in 发送的数据数量
 * @param data_type 数据类型大小，默认为sizeof(fp32)
 * 
 * @details 该函数用于初始化vofa_float对象，分配指定数量的浮点数内存空间
 */
void vofa_float::float_buffer_init(int data_quantity_in , size_t data_type_in)
{
    //数据类型存储
    data_type = data_type_in;

    // 保存数据数量并分配内存
    data_quantity = data_quantity_in;
    tx_buffer_point = (fp32*)calloc(data_quantity,data_type);
}

/**
 * @brief 写入浮点数数据
 * 
 * 该函数用于将多个浮点数数据写入发送缓存区。通过循环遍历数据数组，
 * 将每个数据元素从源位置复制到缓存区。
 * 
 * @param fp32* data_point 写入数据指针，指向待写入数据的起始位置
 * @return void 无返回值
 */
void vofa_float::float_write(fp32* data_point) 
{
    // 遍历所有数据项并进行赋值操作
    for(int i = 0; i < data_quantity; i++)
    {
        tx_buffer_point[i] = data_point[i];
    }
}

/**
 * @brief 发送多个浮点数数据
 * @details 通过UART接口循环发送多个浮点数数据，每个数据后跟随帧尾报文
 * @param void 无参数
 * @return void 无返回值
 */
void vofa_float::float_send(void)
{
    for(int i = 0; i < data_quantity; i++)
    {
        temp_buffer = tx_buffer_point[i];

        // 发送调试数据内容
        HAL_UART_Transmit(huart_piont,(uint8_t*)&temp_buffer,sizeof(data_type * data_quantity),0xff);
    }
    
    // 发送帧尾报文
    HAL_UART_Transmit(huart_piont,tail,4,0xff);

    // 释放动态分配的缓冲区内存
    free(tx_buffer_point);
}



/*-------------以下为单浮点快速数调试模块---------------*/
/**
 * @brief 快速发送单精度浮点数数据通过UART接口
 * 
 * 
 * @param huart 指向UART句柄结构体的指针，用于指定通信接口
 * @param data_point 指向要发送的单精度浮点数数据的指针
 * 
 * @note 该函数无返回值
 */
// void vofa_fast_float::single_float_fast(UART_HandleTypeDef* huart,fp32* data_point)
// {
//     vofa_init(huart);
//     float_buffer_init(1);
//     float_write(data_point);
//     float_send();
// }

