/*********************************************************************************************************************
* RT1064DVL6A Opensourec Library 即（RT1064DVL6A 开源库）是一个基于官方 SDK 接口的第三方开源库
* Copyright (c) 2022 SEEKFREE 逐飞科技
* 
* 本文件是 RT1064DVL6A 开源库的一部分
* 
* RT1064DVL6A 开源库 是免费软件
* 您可以根据自由软件基金会发布的 GPL（GNU General Public License，即 GNU通用公共许可证）的条款
* 即 GPL 的第3版（即 GPL3.0）或（您选择的）任何后来的版本，重新发布和/或修改它
* 
* 本开源库的发布是希望它能发挥作用，但并未对其作任何的保证
* 甚至没有隐含的适销性或适合特定用途的保证
* 更多细节请参见 GPL
* 
* 您应该在收到本开源库的同时收到一份 GPL 的副本
* 如果没有，请参阅<https://www.gnu.org/licenses/>
* 
* 额外注明：
* 本开源库使用 GPL3.0 开源许可证协议 以上许可申明为译文版本
* 许可申明英文版在 libraries/doc 文件夹下的 GPL3_permission_statement.txt 文件中
* 许可证副本在 libraries 文件夹下 即该文件夹下的 LICENSE 文件
* 欢迎各位使用并传播本程序 但修改内容时必须保留逐飞科技的版权声明（即本声明）
* 
* 文件名称          isr
* 公司名称          成都逐飞科技有限公司
* 版本信息          查看 libraries/doc 文件夹内 version 文件 版本说明
* 开发环境          IAR 8.32.4 or MDK 5.33
* 适用平台          RT1064DVL6A
* 店铺链接          https://seekfree.taobao.com/
* 
* 修改记录
* 日期              作者                备注
* 2022-09-21        SeekFree            first version
********************************************************************************************************************/

#include "zf_common_headfile.h"
#include "zf_common_debug.h"
#include "isr.h"
#include "path_executor.h"
#include "mission_controller.h"
#include "test.h"


extern int error;

volatile int photo_data[12][16];
volatile int car_x, car_y;
volatile int flag = 0;
volatile uint32_t camera_frame_sequence = 0u;
volatile uint32_t camera_pose_sequence = 0u;
volatile uint16_t camera_pose_x100 = POSE_INVALID_COORD;
volatile uint16_t camera_pose_y100 = POSE_INVALID_COORD;
volatile uint8_t camera_pose_confidence = 0u;
volatile uint8_t camera_pose_valid = 0u;
volatile uint8_t camera_pose_frame_sequence = 0u;

//volatile uint8_t g_frame_ready = 0;
//volatile uint8_t g_frame_buf[FRAME_LEN];



void CSI_IRQHandler(void)   
{
    CSI_DriverIRQHandler();     // 调用SDK自带的中断函数 这个函数最后会调用我们设置的回调函数
    __DSB();                    // 数据同步隔离
}

void PIT_IRQHandler(void)
{
    if(pit_flag_get(PIT_CH0))
    {
        encoder_get_speed();
#if WHEEL_SPEED_TUNE_MODE
        wheel_speed_tune_update_10ms();
#elif POSITION_STEP_TEST_MODE
        position_step_test_update_10ms();
#elif GYRO_SPEED_TUNE_MODE
        gyro_speed_tune_update_10ms();
#else
        path_executor_update_10ms();
        if(path_executor_is_fault() || mission_controller_should_stop())
        {
            MecanumCarStop();
        }
        else
        {
            MecanumCarSpeedControl();
            MecanumMotorSpeedControl();
        }
#endif
    //    get_encoder_total();
        pit_flag_clear(PIT_CH0);
    }
    

    if(pit_flag_get(PIT_CH1))
    {   
        get_yaw();
        pit_flag_clear(PIT_CH1);
    }
    
#if !WHEEL_SPEED_TUNE_MODE && !GYRO_SPEED_TUNE_MODE
    if(pit_flag_get(PIT_CH2))
    {
        mission_controller_update_10ms();
        pit_flag_clear(PIT_CH2);
    }
#endif

    
    if(pit_flag_get(PIT_CH3))
    {
        pit_flag_clear(PIT_CH3);
    }

    __DSB();
}


static uint8_t camera_pose_crc8(const uint8_t *data, uint8_t length)
{
    uint8_t crc = 0u;
    uint8_t index;
    for(index = 0u; index < length; index++)
    {
        uint8_t bit;
        crc ^= data[index];
        for(bit = 0u; bit < 8u; bit++)
        {
            if((crc & 0x80u) != 0u)
            {
                crc = (uint8_t)((crc << 1u) ^ 0x07u);
            }
            else
            {
                crc <<= 1u;
            }
        }
    }
    return crc;
}

void LPUART1_IRQHandler(void)
{
    static uint8_t frame_type = 0u;
    static uint16_t frame_index = 0u;
    static uint8_t map_data[FRAME_LEN];
    static uint8_t pose_data[POSE_FRAME_LEN];

    if(kLPUART_RxDataRegFullFlag & LPUART_GetStatusFlags(LPUART1))
    {
        uint8_t value = LPUART_ReadByte(LPUART1);

        if(frame_index == 0u)
        {
            if(value == FRAME_HEAD0)
            {
                frame_type = 1u;
                map_data[0] = value;
                frame_index = 1u;
            }
            else if(value == POSE_FRAME_HEAD0)
            {
                frame_type = 2u;
                pose_data[0] = value;
                frame_index = 1u;
            }
        }
        else if(frame_type == 1u)
        {
            map_data[frame_index++] = value;
            if((frame_index == 2u) && (map_data[1] != FRAME_HEAD1))
            {
                frame_index = 0u;
                frame_type = 0u;
            }
            else if(frame_index == FRAME_LEN)
            {
                if(map_data[TAIL_IDX] == FRAME_TAIL)
                {
                    uint8_t row;
                    uint8_t col;
                    camera_frame_sequence++;
                    for(row = 0u; row < 12u; row++)
                    {
                        for(col = 0u; col < 16u; col++)
                        {
                            photo_data[row][col] = map_data[IMG_OFFSET + row * 16u + col];
                        }
                    }
                    car_x = map_data[CAR_X_IDX];
                    car_y = map_data[CAR_Y_IDX];
                    camera_frame_sequence++;
                    flag = 1;
                }
                frame_index = 0u;
                frame_type = 0u;
            }
        }
        else
        {
            pose_data[frame_index++] = value;
            if((frame_index == 2u) && (pose_data[1] != POSE_FRAME_HEAD1))
            {
                frame_index = 0u;
                frame_type = 0u;
            }
            else if(frame_index == POSE_FRAME_LEN)
            {
                uint8_t crc = camera_pose_crc8(&pose_data[2], 8u);
                if((pose_data[2] == POSE_FRAME_VERSION) &&
                   (pose_data[10] == crc) &&
                   (pose_data[11] == POSE_FRAME_TAIL))
                {
                    uint16_t x100 = (uint16_t)(pose_data[5] |
                                               ((uint16_t)pose_data[6] << 8u));
                    uint16_t y100 = (uint16_t)(pose_data[7] |
                                               ((uint16_t)pose_data[8] << 8u));
                    uint8_t valid = (uint8_t)(((pose_data[4] & POSE_FLAG_VALID) != 0u) &&
                                              (x100 != POSE_INVALID_COORD) &&
                                              (y100 != POSE_INVALID_COORD));
                    camera_pose_sequence++;
                    camera_pose_frame_sequence = pose_data[3];
                    camera_pose_x100 = x100;
                    camera_pose_y100 = y100;
                    camera_pose_confidence = pose_data[9];
                    camera_pose_valid = valid;
                    camera_pose_sequence++;
                }
                frame_index = 0u;
                frame_type = 0u;
            }
        }
    }
    LPUART_ClearStatusFlags(LPUART1, kLPUART_RxOverrunFlag);
}

//void LPUART1_IRQHandler(void)
//{
//    static uint16_t rx_idx = 0;
//    static uint8_t  work_buf[FRAME_LEN];

//    if(kLPUART_RxDataRegFullFlag & LPUART_GetStatusFlags(LPUART1))
//    {
//        uint8_t b = LPUART_ReadByte(LPUART1);

//        if(rx_idx == 0)
//        {
//            if(b == FRAME_HEAD0) work_buf[rx_idx++] = b;
//        }
//        else if(rx_idx == 1)
//        {
//            if(b == FRAME_HEAD1)
//            {
//                work_buf[rx_idx++] = b;
//            }
//            else
//            {
//                rx_idx = (b == FRAME_HEAD0) ? 1 : 0;
//                if(rx_idx == 1) work_buf[0] = b;
//            }
//        }
//        else
//        {
//            work_buf[rx_idx++] = b;
//            if(rx_idx == FRAME_LEN)
//            {
//                if(work_buf[TAIL_IDX] == FRAME_TAIL && g_frame_ready == 0)
//                {
//                    for(uint16_t i = 0; i < FRAME_LEN; i++)
//                    {
//                        g_frame_buf[i] = work_buf[i];
//                    }
//                    g_frame_ready = 1;
//                }
//                rx_idx = 0;
//            }
//        }
//    }

//    LPUART_ClearStatusFlags(LPUART1, kLPUART_RxOverrunFlag); 
//}
//void openart_frame_process(void)
//{
//    uint8_t local_buf[FRAME_LEN];

//    if(g_frame_ready == 0) return;

//    __disable_irq();
//    for(uint16_t i = 0; i < FRAME_LEN; i++) local_buf[i] = g_frame_buf[i];
//    g_frame_ready = 0;
//    __enable_irq();

//    for(uint16_t k = 0; k < IMG_SIZE; k++)
//    {
//        photo_data[k / 16][k % 16] = local_buf[IMG_OFFSET + k];
//    }

//    car_x = local_buf[CAR_X_IDX];
//    car_y = local_buf[CAR_Y_IDX];
//    flag = 1;
//		
//}

void LPUART2_IRQHandler(void)
{
    if(kLPUART_RxDataRegFullFlag & LPUART_GetStatusFlags(LPUART2))
    {
        // 接收中断
        
    }
        
    LPUART_ClearStatusFlags(LPUART2, kLPUART_RxOverrunFlag);    // 不允许删除
}

void LPUART3_IRQHandler(void)
{
    if(kLPUART_RxDataRegFullFlag & LPUART_GetStatusFlags(LPUART3))
    {
        // 接收中断
        
    }
        
    LPUART_ClearStatusFlags(LPUART3, kLPUART_RxOverrunFlag);    // 不允许删除
}

volatile uint8_t label_rx_ready;
volatile uint8_t label_rx_type;
volatile uint8_t label_rx_index;
volatile uint8_t label_rx_value;
volatile uint8_t label_rx_frame_seen;
volatile uint8_t label_rx_tail_ok;
volatile uint8_t label_rx_check_ok;



void LPUART4_IRQHandler(void)
{
    static uint8_t buffer[7];
    static uint8_t index = 0u;
    uint8_t byte;

    if(kLPUART_RxDataRegFullFlag &
       LPUART_GetStatusFlags(LPUART4))
    {
        byte = LPUART_ReadByte(LPUART4);

        if(index == 0u)
        {
            if(byte == 0xA4u)
            {
                buffer[index++] = byte;
            }
        }
        else if(index == 1u)
        {
            if(byte == 0xB4u)
            {
                buffer[index++] = byte;
            }
            else
            {
                index = 0u;
            }
        }
        else
        {
            buffer[index++] = byte;

            if(index == sizeof(buffer))
            {
                uint8_t type = buffer[2];
                uint8_t object_index = buffer[3];
                uint8_t label = buffer[4];
                uint8_t check = buffer[5];
                uint8_t expected_check = (uint8_t)(
                    type ^ object_index ^ label);

                label_rx_frame_seen = 1u;
                label_rx_tail_ok =
                    (buffer[6] == 0xC4u) ? 1u : 0u;
                label_rx_check_ok =
                    (check == expected_check) ? 1u : 0u;

                if((label_rx_tail_ok != 0u) &&
                   (label_rx_check_ok != 0u))
                {
                    if(label_rx_ready == 0u)
                    {
                        label_rx_type = type;
                        label_rx_index = object_index;
                        label_rx_value = label;
                        label_rx_ready = 1u;
                    }
                }

                index = 0u;
            }
        }
    }

    LPUART_ClearStatusFlags(LPUART4,kLPUART_RxOverrunFlag);
}

void LPUART5_IRQHandler(void)
{
    if(kLPUART_RxDataRegFullFlag & LPUART_GetStatusFlags(LPUART5))
    {
        // 接收中断
        camera_uart_handler();
    }
        
    LPUART_ClearStatusFlags(LPUART5, kLPUART_RxOverrunFlag);    // 不允许删除
}

void LPUART6_IRQHandler(void)
{
    if(kLPUART_RxDataRegFullFlag & LPUART_GetStatusFlags(LPUART6))
    {
        // 接收中断
        
    }
        
    LPUART_ClearStatusFlags(LPUART6, kLPUART_RxOverrunFlag);    // 不允许删除
}


void LPUART8_IRQHandler(void)
{
    if(kLPUART_RxDataRegFullFlag & LPUART_GetStatusFlags(LPUART8))
    {
        // 接收中断
        wireless_module_uart_handler();
        
    }
        
    LPUART_ClearStatusFlags(LPUART8, kLPUART_RxOverrunFlag);    // 不允许删除
}


void GPIO1_Combined_0_15_IRQHandler(void)
{
    if(exti_flag_get(B0))
    {
        exti_flag_clear(B0);// 清除中断标志位
    }
    
}


void GPIO1_Combined_16_31_IRQHandler(void)
{
    wireless_module_spi_handler();
    if(exti_flag_get(B16))
    {
        exti_flag_clear(B16); // 清除中断标志位
    }

    
}

void GPIO2_Combined_0_15_IRQHandler(void)
{
    flexio_camera_vsync_handler();
    
    if(exti_flag_get(C0))
    {
        exti_flag_clear(C0);// 清除中断标志位
    }

}

void GPIO2_Combined_16_31_IRQHandler(void)
{
    // -----------------* ToF INT 更新中断 预置中断处理函数 *-----------------
    tof_module_exti_handler();
    // -----------------* ToF INT 更新中断 预置中断处理函数 *-----------------
    
    if(exti_flag_get(C16))
    {
        exti_flag_clear(C16); // 清除中断标志位
    }
    
}




void GPIO3_Combined_0_15_IRQHandler(void)
{

    if(exti_flag_get(D4))
    {
        exti_flag_clear(D4);// 清除中断标志位
    }
}









/*
中断函数名称，用于设置对应功能的中断函数
Sample usage:当前启用了周期定时器中断
void PIT_IRQHandler(void)
{
    //务必清除标志位
    __DSB();
}
记得进入中断后清除标志位
CTI0_ERROR_IRQHandler
CTI1_ERROR_IRQHandler
CORE_IRQHandler
FLEXRAM_IRQHandler
KPP_IRQHandler
TSC_DIG_IRQHandler
GPR_IRQ_IRQHandler
LCDIF_IRQHandler
CSI_IRQHandler
PXP_IRQHandler
WDOG2_IRQHandler
SNVS_HP_WRAPPER_IRQHandler
SNVS_HP_WRAPPER_TZ_IRQHandler
SNVS_LP_WRAPPER_IRQHandler
CSU_IRQHandler
DCP_IRQHandler
DCP_VMI_IRQHandler
Reserved68_IRQHandler
TRNG_IRQHandler
SJC_IRQHandler
BEE_IRQHandler
PMU_EVENT_IRQHandler
Reserved78_IRQHandler
TEMP_LOW_HIGH_IRQHandler
TEMP_PANIC_IRQHandler
USB_PHY1_IRQHandler
USB_PHY2_IRQHandler
ADC1_IRQHandler
ADC2_IRQHandler
DCDC_IRQHandler
Reserved86_IRQHandler
Reserved87_IRQHandler
GPIO1_INT0_IRQHandler
GPIO1_INT1_IRQHandler
GPIO1_INT2_IRQHandler
GPIO1_INT3_IRQHandler
GPIO1_INT4_IRQHandler
GPIO1_INT5_IRQHandler
GPIO1_INT6_IRQHandler
GPIO1_INT7_IRQHandler
GPIO1_Combined_0_15_IRQHandler
GPIO1_Combined_16_31_IRQHandler
GPIO2_Combined_0_15_IRQHandler
GPIO2_Combined_16_31_IRQHandler
GPIO3_Combined_0_15_IRQHandler
GPIO3_Combined_16_31_IRQHandler
GPIO4_Combined_0_15_IRQHandler
GPIO4_Combined_16_31_IRQHandler
GPIO5_Combined_0_15_IRQHandler
GPIO5_Combined_16_31_IRQHandler
WDOG1_IRQHandler
RTWDOG_IRQHandler
EWM_IRQHandler
CCM_1_IRQHandler
CCM_2_IRQHandler
GPC_IRQHandler
SRC_IRQHandler
Reserved115_IRQHandler
GPT1_IRQHandler
GPT2_IRQHandler
PWM1_0_IRQHandler
PWM1_1_IRQHandler
PWM1_2_IRQHandler
PWM1_3_IRQHandler
PWM1_FAULT_IRQHandler
SEMC_IRQHandler
USB_OTG2_IRQHandler
USB_OTG1_IRQHandler
XBAR1_IRQ_0_1_IRQHandler
XBAR1_IRQ_2_3_IRQHandler
ADC_ETC_IRQ0_IRQHandler
ADC_ETC_IRQ1_IRQHandler
ADC_ETC_IRQ2_IRQHandler
ADC_ETC_ERROR_IRQ_IRQHandler
PIT_IRQHandler
ACMP1_IRQHandler
ACMP2_IRQHandler
ACMP3_IRQHandler
ACMP4_IRQHandler
Reserved143_IRQHandler
Reserved144_IRQHandler
ENC1_IRQHandler
ENC2_IRQHandler
ENC3_IRQHandler
ENC4_IRQHandler
TMR1_IRQHandler
TMR2_IRQHandler
TMR3_IRQHandler
TMR4_IRQHandler
PWM2_0_IRQHandler
PWM2_1_IRQHandler
PWM2_2_IRQHandler
PWM2_3_IRQHandler
PWM2_FAULT_IRQHandler
PWM3_0_IRQHandler
PWM3_1_IRQHandler
PWM3_2_IRQHandler
PWM3_3_IRQHandler
PWM3_FAULT_IRQHandler
PWM4_0_IRQHandler
PWM4_1_IRQHandler
PWM4_2_IRQHandler
PWM4_3_IRQHandler
PWM4_FAULT_IRQHandler
Reserved171_IRQHandler
GPIO6_7_8_9_IRQHandler*/
