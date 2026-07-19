#ifndef __ENCODER_H
#define __ENCODER_H

//编码器
#define ENCODER_1                   (QTIMER1_ENCODER1)              // 左后轮编码器
#define ENCODER_1_A                 (QTIMER1_ENCODER1_CH1_C0)
#define ENCODER_1_B                 (QTIMER1_ENCODER1_CH2_C1)

#define ENCODER_2                   (QTIMER1_ENCODER2)              // 右后轮编码器
#define ENCODER_2_A                 (QTIMER1_ENCODER2_CH1_C2)
#define ENCODER_2_B                 (QTIMER1_ENCODER2_CH2_C24)

#define ENCODER_3                   (QTIMER2_ENCODER1)              // 右前轮编码器
#define ENCODER_3_A                 (QTIMER2_ENCODER1_CH1_C3)
#define ENCODER_3_B                 (QTIMER2_ENCODER1_CH2_C4)

#define ENCODER_4                   (QTIMER2_ENCODER2)              // 左前轮编码器
#define ENCODER_4_A                 (QTIMER2_ENCODER2_CH1_C5)
#define ENCODER_4_B                 (QTIMER2_ENCODER2_CH2_C25)

extern int encoder_fl;
extern int encoder_fr;
extern int encoder_bl;
extern int encoder_br;
extern int encoder_space;

// void encoder_update(void);
void encoder_get_speed(void);
void encoder_init(void);
void encoder_show();
void encoder_I();
#endif
