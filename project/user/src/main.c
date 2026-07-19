#include "zf_common_headfile.h"
#include "camera_sokoban.h"
#include "path_executor.h"
#include "mission_controller.h"
#include "isr.h"
#include "test.h"
#include "uart.h"
 
extern volatile int flag;
extern volatile int photo_data[12][16];

extern volatile uint32_t camera_frame_sequence;

#if !POSITION_STEP_TEST_MODE && !GYRO_SPEED_TUNE_MODE
static uint8_t debug_screen_tick = 0u;

static void debug_screen_init(void)
{
    ips200_clear();

    ips200_show_string(0,   0, "STATE");
    ips200_show_string(0,  16, "STATUS");
    ips200_show_string(0,  32, "YAW");
    ips200_show_string(0,  48, "T_YAW");
    ips200_show_string(0,  64, "GYRO_X");
    ips200_show_string(0,  80, "T_VX");
    ips200_show_string(0,  96, "T_VY");
    ips200_show_string(0, 112, "ENC_VX");
    ips200_show_string(0, 128, "ENC_VY");
    ips200_show_string(0, 144, "PROGRESS");
    ips200_show_string(0, 160, "CAM_SEQ");
    ips200_show_string(0, 176, "BLAST");
    ips200_show_string(0, 192, "LABEL");
}

static void debug_screen_update(void)
{
    int fl;
    int fr;
    int bl;
    int br;

    float debug_yaw;
    float debug_target_yaw;
    float debug_gyro_x;
    float debug_target_vx;
    float debug_target_vy;
    float encoder_vx;
    float encoder_vy;
    float progress;

    uint32_t camera_sequence;
    uint8_t blast_index;
    uint8_t blast_count;

    /*
     * 主循环约10ms执行一次，每10次刷新一次屏幕，
     * 即大约100ms刷新一次。
     */
    if(++debug_screen_tick < 10u)
    {
        return;
    }
    debug_screen_tick = 0u;

    /*
     * 快速复制中断共享数据。
     * 屏幕操作绝对不能放在关中断区域。
     */
    __disable_irq();

    fl = encoder_fl;
    fr = encoder_fr;
    bl = encoder_bl;
    br = encoder_br;

    debug_yaw = yaw;
    debug_target_yaw = target_yaw;
    debug_gyro_x = Gyro.x;
    debug_target_vx = target_vx;
    debug_target_vy = target_vy;

    camera_sequence = camera_frame_sequence;
    progress = path_executor_get_last_step_distance();

    __enable_irq();

    encoder_vx = (float)(fl + fr + bl + br) / 4.0f;
    encoder_vy = (float)(-fl + fr + bl - br) / 4.0f;

    blast_index = mission_controller_get_blast_event_index();
    blast_count = mission_controller_get_blast_count();

    ips200_show_uint(
        96, 0,
        (uint32_t)mission_controller_get_state(),
        2);

    ips200_show_uint(
        96, 16,
        (uint32_t)mission_controller_get_last_status(),
        2);

    ips200_show_float(96,  32, debug_yaw,        6, 1);
    ips200_show_float(96,  48, debug_target_yaw, 6, 1);
    ips200_show_float(96,  64, debug_gyro_x,     6, 1);
    ips200_show_float(96,  80, debug_target_vx,  6, 1);
    ips200_show_float(96,  96, debug_target_vy,  6, 1);
    ips200_show_float(96, 112, encoder_vx,       6, 1);
    ips200_show_float(96, 128, encoder_vy,       6, 1);
    ips200_show_float(96, 144, progress,         6, 1);

    ips200_show_uint(96, 160, camera_sequence, 10);

    ips200_show_uint(96, 176, blast_index, 1);
    ips200_show_string(112, 176, "/");
    ips200_show_uint(128, 176, blast_count, 1);
    ips200_show_uint(96, 192, label, 2);
}
#endif

int main(void)
{
    clock_init(SYSTEM_CLOCK_600M);  
   debug_init(); 
    UART_init();
    label_uart_init();
    system_delay_ms(1000); 

    ips200_init(IPS200_TYPE_SPI); 
#if POSITION_STEP_TEST_MODE
    position_step_test_screen_init();
#elif GYRO_SPEED_TUNE_MODE
    gyro_speed_tune_screen_init();
#else
    debug_screen_init();
#endif
    encoder_init();
#if POSITION_STEP_TEST_MODE || GYRO_SPEED_TUNE_MODE
    key_init(10);
#endif
    mecanum_pid_init();
    imu_init();

//    wireless_uart_init();

    camera_sokoban_init();
    path_executor_init();
    mission_controller_init();
	
//电机初始化
                                
    gpio_init(MOTOR1_DIR, GPO, 0, GPO_PUSH_PULL);                            // GPIO 初始化为输出 默认上拉输出高
    pwm_init(MOTOR1_PWM, 17000, 0);                                                  // PWM 通道初始化频率 17KHz 占空比初始为 0
    gpio_init(MOTOR2_DIR, GPO, 0, GPO_PUSH_PULL);                            // GPIO 初始化为输出 默认上拉输出高
    pwm_init(MOTOR2_PWM, 17000, 0);                                                  // PWM 通道初始化频率 17KHz 占空比初始为 0
    
    gpio_init(MOTOR3_DIR, GPO, 0, GPO_PUSH_PULL);                           
    pwm_init(MOTOR3_PWM, 17000, 0); 
    gpio_init(MOTOR4_DIR, GPO, 0, GPO_PUSH_PULL);                           
    pwm_init(MOTOR4_PWM, 17000, 0);

#if POSITION_STEP_TEST_MODE
    position_step_test_init();
#elif GYRO_SPEED_TUNE_MODE
    gyro_speed_tune_init();
#endif

    pit_ms_init(PIT_CH0, 10);
    pit_ms_init(PIT_CH1, 1);
    pit_ms_init(PIT_CH2, 10);		
    char buffer[100];
//		ips200_clear();
//自用屏幕显示
  
    while(1)
    {
        system_delay_ms(10);
    //    sprintf(&buffer[0],"%d,%d,%d,%d,%f,%f,%f,%f,%f,%f\n",encoder_fl,encoder_fr,encoder_bl,encoder_br,
    //                                                   pwm_fl,pwm_fr,pwm_bl,pwm_br,yaw,(encoder_fl + encoder_fr + encoder_bl + encoder_br) / 4.0f);
    //    wireless_uart_send_string(buffer);



			
//			uint16_t arr[3][4] = {
//    {11,22,33,44},
//    {55,66,77,88},
//    {99,100,111,122}
//};
//			  uint8_t i,j;
//    for(i=0; i<12; i++)  // 遍历行
//    {
//        for(j=0; j<16; j++) // 遍历列
//        {
//            printf("%d ", photo_data[i][j]); // 逗号分隔
//        }
//        printf("\n"); // 每行结束换行
//    }
//    printf("#\n"); // VOFA结束标识符

#if POSITION_STEP_TEST_MODE
        position_step_test_process_keys();
        position_step_test_screen_update();
#elif GYRO_SPEED_TUNE_MODE
        gyro_speed_tune_process_keys();
        gyro_speed_tune_screen_update();
#else
        mission_controller_process();
        mission_turn_process();
        mission_label_process();
        debug_screen_update();
#endif
			
			
			
			
			





				
//				if (flag == 1)
//				{
//						ips200_show_string(0, 0, "OOK");
//				}
//				else
//				{
//						ips200_show_string(0, 0, "KOO");
//				}
//				
//        system_delay_ms(2);

//				for (int i = 0;i < 12;i ++)
//				{
//						for (int j = 0;j < 16;j ++)
//						{
//								if (photo_data[i][j] == 0)
//								{
//										ips200_show_char(12*(j+0.6),13*(i+1), ' ');
//								}
//								else if (photo_data[i][j] == 1)
//								{
//										ips200_show_char(12*(j+0.6),13*(i+1), 'O');
//								}
//								else if (photo_data[i][j] == 2)
//								{
//										ips200_show_char(12*(j+0.6),13*(i+1), '@');
//								}
//								else if (photo_data[i][j] == 3)
//								{
//										ips200_show_char(12*(j+0.6),13*(i+1), 'X');
//								}
//								else if (photo_data[i][j] == 4)
//								{
//										ips200_show_char(12*(j+0.6),13*(i+1), 'C');
//								}
//						}
//				}
				
//        encoder_show();
					
//       ips200_show_int(0, 176,  encoder_total, 5);
    }
}
