#include "zf_common_headfile.h"
int encoder_fl = 0;
int encoder_fr = 0;
int encoder_bl = 0;
int encoder_br = 0;
int encoder_space=0;


void encoder_get_speed(void)
{
	encoder_fl = encoder_get_count(ENCODER_4) * 2;
	encoder_fr = -encoder_get_count(ENCODER_3) * 2;
	encoder_bl = -encoder_get_count(ENCODER_1);
	encoder_br = -encoder_get_count(ENCODER_2);
	encoder_clear_count(ENCODER_1);
	encoder_clear_count(ENCODER_2);
	encoder_clear_count(ENCODER_3);
	encoder_clear_count(ENCODER_4);

}
// void encoder_update(void){
// 	encoder_bl = encoder_get_count(ENCODER_1);
// 	encoder_br = encoder_get_count(ENCODER_2);
// 	encoder_fl = encoder_get_count(ENCODER_4)*2;
// 	encoder_fr = encoder_get_count(ENCODER_3)*2;
// }

void encoder_init(void)
{
	encoder_quad_init(ENCODER_1, ENCODER_1_A, ENCODER_1_B);
    encoder_quad_init(ENCODER_2, ENCODER_2_A, ENCODER_2_B);
    encoder_quad_init(ENCODER_3, ENCODER_3_A, ENCODER_3_B);
    encoder_quad_init(ENCODER_4, ENCODER_4_A, ENCODER_4_B);
    
}
		
void encoder_show(){
	ips200_show_int(0, 0,  encoder_fl, 5);
	ips200_show_int(0, 16, encoder_fr, 5);
	ips200_show_int(0, 32, encoder_bl, 5);
	ips200_show_int(0, 48, encoder_br, 5);
  ips200_show_float(0, 64, pwm_fl, 5,2);
	ips200_show_float(0, 80, pwm_fl, 5,2);
	ips200_show_float(0, 96, pwm_fl, 5,2);
	ips200_show_float(0, 112, pwm_fl, 5,2);
//	ips200_show_int(0, 128, encoder_space, 5);
}
void encoder_I(){
	encoder_br = encoder_get_count(ENCODER_2);
	encoder_space+=encoder_br;
	encoder_clear_count(ENCODER_2);
}