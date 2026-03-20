#ifndef REMOTE_CONTROL_H
#define REMOTE_CONTROL_H


#ifdef __cplusplus             //���߱��������ⲿ�ִ��밴C���Եĸ�ʽ���б��룬������C++��
extern "C"{
#include "struct_typedef.h"
#include "main.h"


extern UART_HandleTypeDef huart5;
extern DMA_HandleTypeDef hdma_uart5_rx;

extern void RC_Init(uint8_t *rx1_buf, uint8_t *rx2_buf, uint16_t dma_buf_num);
extern void RC_unable(void);
extern void RC_restart(uint16_t dma_buf_num);
}
#endif

//遥控器出错数据上限
#define RC_CHANNAL_ERROR_VALUE 700

#define SBUS_RX_BUF_NUM 36u

#define RC_FRAME_LENGTH 18u

#define RC_CH_VALUE_MIN ((uint16_t)364)
#define RC_CH_VALUE_OFFSET ((uint16_t)1024)
#define RC_CH_VALUE_MAX ((uint16_t)1684)

/*---------------------按键--------------------*/
// yaw,pitch控制通道以及状态开关通道
#define YAW_CHANNEL 0
#define PITCH_CHANNEL 1
#define GIMBAL_MODE_CHANNEL 0

//云台 遥控器速度
#define YAW_RC_SEN -0.00002f // 右手系 z轴逆时针为正 但是遥控器通道向右为正 故加负号
#define PITCH_RC_SEN 0.000005f

//云台 鼠标速度
#define YAW_MOUSE_SEN   -0.00004f
#define PITCH_MOUSE_SEN -0.00003f

/* ----------------------- RC Switch Definition----------------------------- */
#define RC_SW_UP ((uint16_t)1)
#define RC_SW_MID ((uint16_t)3)
#define RC_SW_DOWN ((uint16_t)2)
#define switch_is_down(s) (s == RC_SW_DOWN)
#define switch_is_mid(s) (s == RC_SW_MID)
#define switch_is_up(s) (s == RC_SW_UP)
/* ----------------------- PC Key Definition-------------------------------- */
#define KEY_PRESSED_OFFSET_W ((uint16_t)1 << 0)
#define KEY_PRESSED_OFFSET_S ((uint16_t)1 << 1)
#define KEY_PRESSED_OFFSET_A ((uint16_t)1 << 2)
#define KEY_PRESSED_OFFSET_D ((uint16_t)1 << 3)
#define KEY_PRESSED_OFFSET_SHIFT ((uint16_t)1 << 4)
#define KEY_PRESSED_OFFSET_CTRL ((uint16_t)1 << 5)
#define KEY_PRESSED_OFFSET_Q ((uint16_t)1 << 6)
#define KEY_PRESSED_OFFSET_E ((uint16_t)1 << 7)
#define KEY_PRESSED_OFFSET_R ((uint16_t)1 << 8)
#define KEY_PRESSED_OFFSET_F ((uint16_t)1 << 9)
#define KEY_PRESSED_OFFSET_G ((uint16_t)1 << 10)
#define KEY_PRESSED_OFFSET_Z ((uint16_t)1 << 11)
#define KEY_PRESSED_OFFSET_X ((uint16_t)1 << 12)
#define KEY_PRESSED_OFFSET_C ((uint16_t)1 << 13)
#define KEY_PRESSED_OFFSET_V ((uint16_t)1 << 14)
#define KEY_PRESSED_OFFSET_B ((uint16_t)1 << 15)

/* ----------------------- Data Struct ------------------------------------- */
typedef __packed struct
{
        __packed struct
        {
                int16_t ch[5];
                char s[2];
        } rc;
        __packed struct
        {
                int16_t x;
                int16_t y;
                int16_t z;
                uint8_t press_l;
                uint8_t press_r;
        } mouse;
        __packed struct
        {
                uint16_t v;
        } key;

} RC_ctrl_t;

/*-----相比于官方的版本,将宏定义布尔值转化为可传参的函数,方便不同任务内的遥控器调用按键--*/
//是否按下鼠标
uint16_t if_mouse_pessed(const RC_ctrl_t *_rc_ctrl, char mouse_num);
//是否按下对应按键
uint16_t if_key_pessed(uint16_t key_value, char key_num);
//是否单击对于按键
uint16_t if_key_singal_pessed(uint16_t key_value, uint16_t last_key_value, char key_num);
/* ----------------------- Internal Data ----------------------------------- */

#ifdef __cplusplus
extern "C" {
#endif

//遥控器控制
class Remote_control
{
public:
        //接收原始数据，为18个字节，给了36个字节长度，防止DMA传输越界
        uint8_t sbus_rx_buf[2][SBUS_RX_BUF_NUM];
        RC_ctrl_t rc_ctrl;
        RC_ctrl_t last_rc_ctrl;
        const RC_ctrl_t *gimbal_RC;

        void init();

        void remote();

        const RC_ctrl_t *get_remote_control_point();
        RC_ctrl_t *get_last_remote_control_point();

        void unpack(uint8_t num);

        uint8_t RC_data_is_error();
        void slove_RC_lost();
        void slove_data_error();
        int16_t RC_abs(int16_t value); //取正函数
        // fp32 *yaw,*pitch;
};

#ifdef __cplusplus
}
#endif

// extern Remote_control remote_control;

    #define rc_deadband_limit(input, output, dealine)        \
    {                                                    \
        if ((input) > (dealine) || (input) < -(dealine)) \
        {                                                \
            (output) = (input);                          \
        }                                                \
        else                                             \
        {                                                \
            (output) = 0;                                \
        }                                                \
    }




#endif

