////// 引入RT1064开发板的核心驱动头文件（包含时钟、GPIO、摄像头、LCD等底层功能）
////#include "zf_common_headfile.h"
////// 引入内存/字符串操作头文件（用到拷贝、清空数组的函数）
////#include <string.h>

/////************************** 硬件参数宏定义（根据自己的硬件修改） **************************/
////#define CAM_W    MT9V03X_W        // 摄像头采集图像的宽度（比如160像素，由摄像头驱动定义）
////#define CAM_H    MT9V03X_H        // 摄像头采集图像的高度（比如120像素）
////#define LCD_W    240              // LCD屏幕的宽度（固定240像素）
////#define LCD_H    320              // LCD屏幕的高度（固定320像素）
////#define DISP_H   (LCD_H / 2)      // 只显示屏幕上半部分（160像素），减少计算量
////#define GrayScale 256             // 灰度等级（0-255，共256级，图像的基础属性）
////#define MID_BASE (CAM_W / 2)      // 赛道中线基准点（摄像头宽度的一半，比如160的话就是80）

/////************************** 全局变量定义（存放算法的所有数据） **************************/
////// 原始图像数组：存储摄像头采集的灰度图像（__attribute__((aligned(4)))是为了RT1064内存对齐，不用改）
////uint8 base_image[CAM_H][CAM_W] __attribute__((aligned(4)));
////// 二值化图像数组：处理后只有0（黑色，赛道外）和255（白色，赛道）两种值
////uint8 image[CAM_H][CAM_W] __attribute__((aligned(4)));

////// 大津法（自动算阈值）相关变量
////uint16 hist[GrayScale]={0};      // 灰度直方图：hist[50]表示灰度值为50的像素有多少个
////float P[GrayScale]={0};          // 每个灰度值出现的概率（比如P[50]=hist[50]/总像素数）
////float PK[GrayScale]={0};         // 0~i灰度值的概率总和（比如PK[100]是0-100灰度的总概率）
////float MK[GrayScale]={0};         // 0~i灰度值的均值总和（用于计算最优阈值）
////uint8 img_threshold;             // 大津法算出的最优二值化阈值（核心输出）
////float imgsize;                   // 图像总像素数（宽度×高度）

////// 最长白列巡线核心变量（找赛道边界用）
////int start_column=20;             // 最长白列搜索起始列（避开屏幕边缘20列，防止干扰）
////int end_column=CAM_W-20;         // 最长白列搜索结束列（同上）
////int Longest_White_Column_Left[2];// 左侧最长白列：[0]存长度，[1]存列号（比如[0]=100，[1]=50表示第50列有100个白色像素）
////int Longest_White_Column_Right[2];// 右侧最长白列：和左边同理
////int White_Column[CAM_W];         // 每一列的白色像素高度（从图像最下面往上数）
////int Search_Stop_Line;            // 巡线截止行（只处理最长白列范围内的行，省算力）
////int Boundry_Start_Left;          // 左侧第一个没丢线的行（找赛道起始位置）
////int Boundry_Start_Right;         // 右侧第一个没丢线的行
////int Road_Wide[CAM_H];            // 每一行的赛道宽度（右边界-左边界）
////int Right_Lost_Time;             // 右侧丢线的行数（没找到边界就算丢线）
////int Left_Lost_Time;              // 左侧丢线的行数
////int Both_Lost_Time;              // 左右都丢线的行数（可能是环岛/十字）
////int Lost_Time_cha;               // 左右丢线数的差值（判断左弯/右弯）
////int Left_Lost_Flag[CAM_H];       // 左侧丢线标志：1=丢线，0=没丢线（每行一个标志）
////int Right_Lost_Flag[CAM_H];      // 右侧丢线标志：同上
////uint8 left_line_list[CAM_H];     // 每一行的左边界列号（核心输出，控制舵机用）
////uint8 right_line_list[CAM_H];    // 每一行的右边界列号（核心输出）
////uint8 mid_line_list[CAM_H];      // 每一行的中线列号（左右边界的平均值）
////int last_error_image;            // 上一帧的中线误差（备用，防抖用）

////// 赛道特征标志（预留功能：十字/斑马线，暂时用不到，先定义）
////int Cross_Flag=0;                // 十字标志：0=没十字，1=有十字
////int Left_Down_Find=0;            // 十字左下拐点的行号（找到就存行数，没找到是0）
////int Left_Up_Find=0;              // 十字左上拐点行号
////int Right_Down_Find=0;           // 十字右下拐点行号
////int Right_Up_Find=0;             // 十字右上拐点行号
////int Zebra_Stripes_Flag=0;        // 斑马线标志：0=没斑马线，1=有
////int right_flag_x;                // 右侧丢线的累计计数

////// 误差计算相关（控制舵机的核心）
////int error_image;                 // 当前帧的中线误差（基准点-中线列号，正=偏右，负=偏左）
////int error_image_last;            // 上一帧的误差（备用）
////uint8 zhuan_raw=52;              // 选第52行计算误差（选中间行更稳定，避免抖动）
////int sign_image;                  // 图像处理完成标志：1=处理完，0=没处理

/////************************** 大津法函数：自动计算二值化阈值（不用手动调参数） **************************/
////// 输入：原始灰度图像；输出：最优二值化阈值（把黑白分最开的那个值）
////uint8 Ostu(uint8 index[CAM_H][CAM_W])
////{
////    uint8 threshold=64;                // 默认阈值（防止计算出错，保底用）
////    imgsize = CAM_H * CAM_W;           // 计算图像总像素数（宽度×高度）
////    uint8 images_value_temp;           // 临时变量：存当前像素的灰度值
////    float sumPK = 0;                   // 临时变量：概率累加和
////    float sumMK = 0;                   // 临时变量：均值累加和
////    float var = 0;                     // 最大类间方差（找最优阈值的核心）
////    float vartmp = 0;                  // 当前阈值的类间方差

////    // 第一步：清空所有历史数据（避免上一帧的结果干扰当前帧）
////    for(uint16 i=0;i<GrayScale;i++)
////    {
////        hist[i]=0;    // 灰度直方图清零
////        P[i]=0;       // 概率数组清零
////        PK[i]=0;      // 概率累加和清零
////        MK[i]=0;      // 均值累加和清零
////    }

////    // 第二步：统计每个灰度值的像素数量（画灰度直方图）
////    for(uint8 i = 0;i<CAM_H;i++)       // 遍历每一行
////    {
////        for(uint8 j=0;j<CAM_W;j++)     // 遍历每一列
////        {
////            images_value_temp = index[i][j];  // 取当前像素的灰度值（0-255）
////            hist[images_value_temp]++;         // 对应灰度值的计数+1
////        }
////    }

////    // 第三步：计算每个灰度值的概率、概率累加和、均值累加和
////    for(uint16 i=0;i<GrayScale;i++)
////    {
////        P[i]=(float)hist[i]/imgsize;    // 灰度i的出现概率（个数/总数）
////        PK[i] = sumPK + P[i];           // 0~i灰度的概率总和
////        sumPK=PK[i];                    // 更新累加和
////        MK[i] = sumMK+i*P[i];           // 0~i灰度的均值总和
////        sumMK=MK[i];                    // 更新均值和
////    }

////    // 第四步：遍历所有可能的阈值（5-245），找类间方差最大的（最优阈值）
////    for(uint8 i=5;i<245;i++)
////    {
////        // 跳过无效阈值（全黑/全白，避免除以0错误）
////        if(PK[i] == 0 || PK[i] == 1) continue;
////        // 类间方差公式：值越大，黑白区分越明显（不用理解公式，会用就行）
////        vartmp = ((MK[GrayScale-1] * PK[i] - MK[i]) * (MK[GrayScale - 1] * PK[i] - MK[i])) / (PK[i] * (1 - PK[i]));
////        if(vartmp>var)  // 如果当前方差更大，更新最优阈值
////        {
////            var = vartmp;
////            threshold = i;
////        }
////    }
////    return threshold;  // 返回最优阈值
////}

/////************************** 二值化函数：把灰度图转黑白图 **************************/
////// 输入：大津法算出的阈值；输出：把image数组变成只有0和255的黑白图
////void set_image_twovalues(uint8 value)
////{
////    uint8 temp_value;  // 临时变量：存当前像素的灰度值
////    // 遍历每一行
////    for(uint8 i=0;i<CAM_H;i++)
////    {
////        // 遍历每一列
////        for(uint8 j=0;j<CAM_W;j++)
////        {
////            temp_value=base_image[i][j];  // 取原始灰度值
////            // 灰度值 < 阈值+8 → 判定为黑色（赛道外），存0
////            if(temp_value<value+8)
////            {
////                image[i][j]=0;
////            }
////            // 否则 → 判定为白色（赛道），存255
////            else
////            {
////                image[i][j]=255;
////            }
////        }
////    }
////}

/////************************** 最长白列巡线函数：找赛道的左右边界 **************************/
////void Longest_White_Column()
////{
////    int i, j;  // 循环变量：i=行，j=列
////    // 第一步：初始化所有计数变量（清空上一帧的结果）
////    Left_Lost_Time=0;     // 左侧丢线数清零
////    Both_Lost_Time=0;     // 左右都丢线数清零
////    Right_Lost_Time=0;    // 右侧丢线数清零
////    Lost_Time_cha=0;      // 丢线差清零
////    Boundry_Start_Left=0; // 左边界起始点清零
////    Boundry_Start_Right=0;// 右边界起始点清零
////    uint8 left_border = 0, right_border = 0;  // 临时存当前行的左右边界

////    // 第二步：清空边界/丢线标志数组（初始化）
////    for (i = 0; i <=CAM_H-1; i++)
////    {
////        Right_Lost_Flag[i] = 0;    // 右侧丢线标志置0（没丢线）
////        Left_Lost_Flag[i] = 0;     // 左侧丢线标志置0
////        left_line_list[i] = 0;     // 左边界默认屏幕最左（0列）
////        right_line_list[i] = CAM_W-1;  // 右边界默认屏幕最右（最后一列）
////        mid_line_list[i] = MID_BASE;   // 中线默认基准点（屏幕中间）
////    }
////    // 清空每列白色高度数组
////    for(i=0;i<=CAM_W-1;i++)
////    {
////        White_Column[i] = 0;
////    }

////    // 第三步：统计每一列的白色像素高度（从图像最下面往上数）
////    for (j =start_column; j<=end_column; j++)  // 只统计20~CAM_W-20列（避开边缘）
////    {
////        // 从图像最下方（车头方向）往上数
////        for (i = CAM_H - 1; i >= 0; i--)
////        {
////            if(image[i][j] == 0)  // 遇到黑色，停止计数
////                break;
////            else                  // 遇到白色，计数+1
////                White_Column[j]++;
////        }
////    }

////    // 第四步：找左侧最长白列（从左到右找）
////    Longest_White_Column_Left[0] =0;  // 初始最长长度为0
////    for(i=start_column;i<=end_column;i++)
////    {
////        // 如果当前列的白色高度更长，更新最长白列
////        if (Longest_White_Column_Left[0] < White_Column[i])
////        {
////            Longest_White_Column_Left[0] = White_Column[i];  // 存长度
////            Longest_White_Column_Left[1] = i;                // 存列号
////        }
////    }

////    // 第五步：找右侧最长白列（从右到左找）
////    Longest_White_Column_Right[0] = 0;  // 初始最长长度为0
////    for(i=end_column;i>=start_column;i--)
////    {
////        // 如果当前列的白色高度更长，更新最长白列
////        if (Longest_White_Column_Right[0] < White_Column[i])
////        {
////            Longest_White_Column_Right[0] = White_Column[i];  // 存长度
////            Longest_White_Column_Right[1] = i;                // 存列号
////        }
////    }

////    // 第六步：确定巡线截止行（只处理最长白列范围内的行）
////    Search_Stop_Line = Longest_White_Column_Left[0];
////    // 第七步：逐行检测左右边界（从下往上，只处理截止行内的行）
////    for (i = CAM_H - 1; i >=CAM_H-Search_Stop_Line; i--)
////    {
////        // 检测右边界：从右侧最长白列位置往右找「白黑黑」（255,0,0）
////        for (j = Longest_White_Column_Right[1]; j <= CAM_W - 1 - 2; j++)
////        {
////            // 找到「白黑黑」→ 这就是右边界
////            if (image[i][j] ==255 && image[i][j + 1] == 0 && image[i][j + 2] ==0)
////            {
////                right_border = (uint8)j;    // 存右边界列号
////                Right_Lost_Flag[i] = 0;     // 标记没丢线
////                break;  // 找到就退出循环
////            }
////            // 没找到边界，到屏幕边缘了 → 标记丢线
////            else if(j>=CAM_W-1-2)
////            {
////                right_border = (uint8)j;    // 边界置屏幕最右
////                Right_Lost_Flag[i] = 1;     // 标记丢线
////                break;
////            }
////        }
////        // 检测左边界：从左侧最长白列位置往左找「黑黑白」（0,0,255）
////        for (j = Longest_White_Column_Left[1]; j >= 0 + 2; j--)
////        {
////            // 找到「黑黑白」→ 这就是左边界
////            if (image[i][j] ==255 && image[i][j - 1] ==0 && image[i][j - 2] == 0)
////            {
////                left_border = (uint8)j;     // 存左边界列号
////                Left_Lost_Flag[i] = 0;      // 标记没丢线
////                break;  // 找到就退出循环
////            }
////            // 没找到边界，到屏幕边缘了 → 标记丢线
////            else if(j<=0+2)
////            {
////                left_border = (uint8)j;     // 边界置屏幕最左
////                Left_Lost_Flag[i] = 1;      // 标记丢线
////                break;
////            }
////        }
////        // 保存当前行的边界和中线
////        left_line_list[i] = left_border;       // 存左边界
////        right_line_list[i] = right_border;     // 存右边界
////        mid_line_list[i]=(left_line_list[i]+right_line_list[i])/2;  // 算中线（平均值）
////    }

////    // 第八步：统计丢线数据（判断赛道类型：直道/左弯/右弯）
////    right_flag_x=0;
////    for (i = CAM_H - 1; i >= 0; i--)
////    {
////        if (Left_Lost_Flag[i]  == 1)        // 左侧丢线，计数+1
////            Left_Lost_Time++;
////        if (Right_Lost_Flag[i] == 1)        // 右侧丢线，计数+1
////        {
////            Right_Lost_Time++;
////            right_flag_x++;
////        }

////        if (Left_Lost_Flag[i] == 1 && Right_Lost_Flag[i] == 1)  // 左右都丢线，计数+1
////            Both_Lost_Time++;
////        // 记录左侧第一个没丢线的行（赛道起始点）
////        if (Boundry_Start_Left ==  0 && Left_Lost_Flag[i]  != 1)
////            Boundry_Start_Left = i;
////        // 记录右侧第一个没丢线的行
////        if (Boundry_Start_Right == 0 && Right_Lost_Flag[i] != 1)
////            Boundry_Start_Right = i;

////        Road_Wide[i] = right_line_list[i] - left_line_list[i];  // 算当前行的赛道宽度
////    }
////    Lost_Time_cha=Left_Lost_Time- Right_Lost_Time;  // 算左右丢线差（判断弯道方向）
////}

/////************************** 中线误差计算函数：算舵机需要转的方向 **************************/
////void jisuanzhongzhi()
////{
////    int i;
////    // 第一步：重新计算每一行的中线（确保和最新边界同步）
////    for (i = CAM_H - 1; i >=0; i--)
////    {
////        mid_line_list[i]=(left_line_list[i]+right_line_list[i])/2;
////    }
////    error_image_last=error_image;  // 保存上一帧的误差（防抖用）

////    // 第二步：选稳定的行计算当前帧误差（避免抖动）
////    // 如果最长白列长度 > 图像高度-52 → 取第52行算误差（稳定）
////    if(Search_Stop_Line>CAM_H-zhuan_raw)
////    {
////        // 误差=基准点-中线列号：正=偏右（舵机左打），负=偏左（舵机右打）
////        error_image=MID_BASE-mid_line_list[zhuan_raw];
////    }
////    else  // 最长白列比较短 → 取最长白列+6行（更靠近车头）
////    {
////        if(Search_Stop_Line>10)  // 最长白列长度>10（有效）
////        {
////            error_image=MID_BASE-mid_line_list[ CAM_H-Search_Stop_Line+6];
////        }
////        else  // 最长白列太短（无效），误差置0（直走）
////        {
////            error_image=0;
////        }
////    }
////}

/////************************** 图像处理总函数：封装所有步骤 **************************/
////void image_process(void)
////{
////    // 第一步：把摄像头采集的图像拷贝到base_image数组（原始灰度图）
////    memcpy(&base_image[0][0],mt9v03x_image,CAM_W*CAM_H);
////    // 第二步：用大津法算最优阈值
////    img_threshold = Ostu(base_image);
////    // 第三步：把灰度图转黑白图
////    set_image_twovalues(img_threshold);
////    // 第四步：找赛道的左右边界
////    Longest_White_Column();
////    // 第五步：计算中线误差（控制舵机的核心）
////    jisuanzhongzhi();
////    // 第六步：标记图像处理完成
////    sign_image=1;
////}

/////************************** 图像显示函数：把赛道和边界显示到LCD屏幕（调试用） **************************/
////uint16 frame_buf[CAM_H][CAM_W] = {0};  // 显示缓存（存RGB565格式的图像）
////uint16 lcd_buf[LCD_H][LCD_W] = {0};    // LCD屏幕的显示缓存
////int error;                             // 最终误差值（给舵机控制用）

////// 输入：摄像头图像缓存；输出：缩放到LCD屏幕的图像缓存
////void img_show(uint16 src[][CAM_W], uint16 dest[][LCD_W])
////{
////    // 清空LCD缓存（RGB565格式，每个像素占2字节）
////    memset(dest, 0, LCD_W*LCD_H*2);
////    // 只显示屏幕上半部分
////    for(uint16 y = 0; y < DISP_H; y++)
////    {
////        // 把摄像头图像的行缩放到LCD的行（适配不同分辨率）
////        uint16 sy = (y * CAM_H) / DISP_H;
////        if(sy >= CAM_H) sy = CAM_H - 1;  // 防止越界
////        // 把摄像头图像的列缩放到LCD的列
////        for(uint16 x = 0; x < LCD_W; x++)
////        {
////            uint16 sx = (x * CAM_W) / LCD_W;
////            if(sx >= CAM_W) sx = CAM_W - 1;  // 防止越界
////            dest[y][x] = src[sy][sx];        // 拷贝像素到LCD缓存
////        }
////    }
////}

/////************************** 主函数：程序入口（所有功能从这里开始） **************************/
////int main()
////{
////    // 第一步：初始化所有硬件（RT1064必须先初始化才能用）
////    clock_init(SYSTEM_CLOCK_600M);  // 初始化系统时钟（600MHz）
////    debug_init();                   // 初始化调试串口（打印信息用）

////    ips200_init(IPS200_TYPE_SPI);   // 初始化LCD屏幕（SPI接口）
////    pit_ms_init(PIT_CH0, 10);       // 初始化定时器（10ms）
////    Servo_Init();                   // 初始化舵机
////    MOTOR_INIT();                   // 初始化电机
////    key_init(5);                    // 初始化按键（5ms扫描一次）
////    mt9v03x_init();                 // 初始化摄像头
////    ips200_set_dir(IPS200_PORTAIT); // 设置LCD为竖屏显示
////    ips200_set_color(RGB565_WHITE, RGB565_BLACK); // 设置LCD背景白，字体黑

////    system_delay_ms(300);           // 延时300ms，等硬件稳定

////    // 第二步：主循环（死循环，程序一直运行）
////    while(1)
////    {
////        // 如果摄像头采集完一帧图像（标志位为1）
////        if(mt9v03x_finish_flag)
////        {
////            mt9v03x_finish_flag = 0; // 清空标志位，准备下一次采集
////            __disable_irq();         // 关闭中断（防止处理图像时被打断）
////            image_process();         // 核心：处理图像（算边界和误差）
////            __enable_irq();          // 开启中断

////            // 第三步：绘制赛道、边界、中线（调试用，能在屏幕看到）
////            for(uint16 y = 0; y < CAM_H; y++)
////            {
////                for(uint16 x = 0; x < CAM_W; x++)
////                {
////                    // 二值化图像显示：白色=白，黑色=黑
////                    frame_buf[y][x] = image[y][x] ? RGB565_WHITE : RGB565_BLACK;
////                    if(x == left_line_list[y]) frame_buf[y][x] = RGB565_RED;    // 左边界标红色
////                    if(x == right_line_list[y]) frame_buf[y][x] = RGB565_BLUE;  // 右边界标蓝色
////                    if(x == mid_line_list[y]) frame_buf[y][x] = RGB565_GREEN;   // 中线标绿色
////                }
////            }

////            // 第四步：把图像显示到LCD屏幕
////            img_show(frame_buf, lcd_buf);
////            ips200_show_rgb565_image(0, 0, (uint16*)lcd_buf, LCD_W, LCD_H, LCD_W, LCD_H, 1);

////            // 第五步：在屏幕显示误差值（调试用，看误差对不对）
////            ips200_show_string(10, DISP_H+10, "Error:");
////            ips200_show_int(70, DISP_H+10, error_image, 4);

////            // 第六步：保存误差值（后面可以用这个值控制舵机）
////            error = error_image;
////        }
////    }
////}
//#include "zf_common_headfile.h"
//#include <string.h>

//#define CAM_W    MT9V03X_W
//#define CAM_H    MT9V03X_H
//#define LCD_W    240
//#define LCD_H    320
//#define DISP_H   (LCD_H / 2)
//#define GrayScale 256
//#define MID_BASE (CAM_W / 2)

//uint8 base_image[CAM_H][CAM_W] __attribute__((aligned(4)));
//uint8 image[CAM_H][CAM_W] __attribute__((aligned(4)));
//uint16 hist[GrayScale]={0};
//float P[GrayScale]={0};
//float PK[GrayScale]={0};
//float MK[GrayScale]={0};
//uint8 img_threshold;
//float imgsize;

//int start_column=20;
//int end_column=CAM_W-20;
//int Longest_White_Column_Left[2];
//int Longest_White_Column_Right[2];
//int White_Column[CAM_W];
//int Search_Stop_Line;
//int Boundry_Start_Left;
//int Boundry_Start_Right;
//int Road_Wide[CAM_H];
//int Right_Lost_Time;
//int Left_Lost_Time;
//int Both_Lost_Time;
//int Lost_Time_cha;
//int Left_Lost_Flag[CAM_H];
//int Right_Lost_Flag[CAM_H];
//uint8 left_line_list[CAM_H];
//uint8 right_line_list[CAM_H];
//uint8 mid_line_list[CAM_H];
//int last_error_image;
//int Cross_Flag=0;
//int Left_Down_Find=0;
//int Left_Up_Find=0;
//int Right_Down_Find=0;
//int Right_Up_Find=0;
//int Zebra_Stripes_Flag=0;
//int right_flag_x;
//int error_image;
//int error_image_last;
//uint8 zhuan_raw=52;
//int sign_image;

//uint8 Ostu(uint8 index[CAM_H][CAM_W])
//{
//    uint8 threshold=64;
//    imgsize = CAM_H * CAM_W;
//    uint8 images_value_temp;
//    float sumPK = 0;
//    float sumMK = 0;
//    float var = 0;
//    float vartmp = 0;

//    for(uint16 i=0;i<GrayScale;i++)
//    {
//        hist[i]=0;
//        P[i]=0;
//        PK[i]=0;
//        MK[i]=0;
//    }

//    for(uint8 i = 0;i<CAM_H;i++)
//    {
//        for(uint8 j=0;j<CAM_W;j++)
//        {
//            images_value_temp = index[i][j];
//            hist[images_value_temp]++;
//        }
//    }

//    for(uint16 i=0;i<GrayScale;i++)
//    {
//        P[i]=(float)hist[i]/imgsize;
//        PK[i] = sumPK + P[i];
//        sumPK=PK[i];
//        MK[i] = sumMK+i*P[i];
//        sumMK=MK[i];
//    }

//    for(uint8 i=5;i<245;i++)
//    {
//        if(PK[i] == 0 || PK[i] == 1) continue;
//        vartmp = ((MK[GrayScale-1] * PK[i] - MK[i]) * (MK[GrayScale - 1] * PK[i] - MK[i])) / (PK[i] * (1 - PK[i]));
//        if(vartmp>var)
//        {
//            var = vartmp;
//            threshold = i;
//        }
//    }
//    return threshold;
//}

//void set_image_twovalues(uint8 value)
//{
//    uint8 temp_value;
//    for(uint8 i=0;i<CAM_H;i++)
//    {
//        for(uint8 j=0;j<CAM_W;j++)
//        {
//            temp_value=base_image[i][j];
//            if(temp_value<value+8)
//            {
//                image[i][j]=0;
//            }
//            else
//            {
//                image[i][j]=255;
//            }
//        }
//    }
//}

//void Longest_White_Column()
//{
//    int i, j;
//    Left_Lost_Time=0;
//    Both_Lost_Time=0;
//    Right_Lost_Time=0;
//    Lost_Time_cha=0;
//    Boundry_Start_Left=0;
//    Boundry_Start_Right=0;
//    uint8 left_border = 0, right_border = 0;

//    for (i = 0; i <=CAM_H-1; i++)
//    {
//        Right_Lost_Flag[i] = 0;
//        Left_Lost_Flag[i] = 0;
//        left_line_list[i] = 0;
//        right_line_list[i] = CAM_W-1;
//        mid_line_list[i] = MID_BASE;
//    }
//    for(i=0;i<=CAM_W-1;i++)
//    {
//        White_Column[i] = 0;
//    }

//    for (j =start_column; j<=end_column; j++)
//    {
//        for (i = CAM_H - 1; i >= 0; i--)
//        {
//            if(image[i][j] == 0)
//                break;
//            else
//                White_Column[j]++;
//        }
//    }

//    Longest_White_Column_Left[0] =0;
//    for(i=start_column;i<=end_column;i++)
//    {
//        if (Longest_White_Column_Left[0] < White_Column[i])
//        {
//            Longest_White_Column_Left[0] = White_Column[i];
//            Longest_White_Column_Left[1] = i;
//        }
//    }

//    Longest_White_Column_Right[0] = 0;
//    for(i=end_column;i>=start_column;i--)
//    {
//        if (Longest_White_Column_Right[0] < White_Column[i])
//        {
//            Longest_White_Column_Right[0] = White_Column[i];
//            Longest_White_Column_Right[1] = i;
//        }
//    }

//    Search_Stop_Line = Longest_White_Column_Left[0];
//    for (i = CAM_H - 1; i >=CAM_H-Search_Stop_Line; i--)
//    {
//        for (j = Longest_White_Column_Right[1]; j <= CAM_W - 1 - 2; j++)
//        {
//            if (image[i][j] ==255 && image[i][j + 1] == 0 && image[i][j + 2] ==0)
//            {
//                right_border = (uint8)j;
//                Right_Lost_Flag[i] = 0;
//                break;
//            }
//            else if(j>=CAM_W-1-2)
//            {
//                right_border = (uint8)j;
//                Right_Lost_Flag[i] = 1;
//                break;
//            }
//        }
//        for (j = Longest_White_Column_Left[1]; j >= 0 + 2; j--)
//        {
//            if (image[i][j] ==255 && image[i][j - 1] ==0 && image[i][j - 2] == 0)
//            {
//                left_border = (uint8)j;
//                Left_Lost_Flag[i] = 0;
//                break;
//            }
//            else if(j<=0+2)
//            {
//                left_border = (uint8)j;
//                Left_Lost_Flag[i] = 1;
//                break;
//            }
//        }
//        left_line_list[i] = left_border;
//        right_line_list[i] = right_border;
//        mid_line_list[i]=(left_line_list[i]+right_line_list[i])/2;
//    }

//    right_flag_x=0;
//    for (i = CAM_H - 1; i >= 0; i--)
//    {
//        if (Left_Lost_Flag[i]  == 1)
//            Left_Lost_Time++;
//        if (Right_Lost_Flag[i] == 1)
//        {
//            Right_Lost_Time++;
//            right_flag_x++;
//        }

//        if (Left_Lost_Flag[i] == 1 && Right_Lost_Flag[i] == 1)
//            Both_Lost_Time++;
//        if (Boundry_Start_Left ==  0 && Left_Lost_Flag[i]  != 1)
//            Boundry_Start_Left = i;
//        if (Boundry_Start_Right == 0 && Right_Lost_Flag[i] != 1)
//            Boundry_Start_Right = i;

//        Road_Wide[i] = right_line_list[i] - left_line_list[i];
//    }
//    Lost_Time_cha=Left_Lost_Time- Right_Lost_Time;
//}

//void jisuanzhongzhi()
//{
//    int i;
//    for (i = CAM_H - 1; i >=0; i--)
//    {
//        mid_line_list[i]=(left_line_list[i]+right_line_list[i])/2;
//    }
//    error_image_last=error_image;
//    if(Search_Stop_Line>CAM_H-zhuan_raw)
//    {
//        error_image=MID_BASE-mid_line_list[zhuan_raw];
//    }
//    else
//    {
//        if(Search_Stop_Line>10)
//        {
//            error_image=MID_BASE-mid_line_list[ CAM_H-Search_Stop_Line+6];
//        }
//        else
//        {
//            error_image=0;
//        }
//    }
//}

//void image_process(void)
//{
//    memcpy(&base_image[0][0],mt9v03x_image,CAM_W*CAM_H);
//    img_threshold = Ostu(base_image);
//    set_image_twovalues(img_threshold);
//    Longest_White_Column();
//    jisuanzhongzhi();
//    sign_image=1;
//}

//uint16 frame_buf[CAM_H][CAM_W] = {0};
//uint16 lcd_buf[LCD_H][LCD_W] = {0};
//int error;

//void img_show(uint16 src[][CAM_W], uint16 dest[][LCD_W])
//{
//    memset(dest, 0, LCD_W*LCD_H*2);
//    for(uint16 y = 0; y < DISP_H; y++)
//    {
//        uint16 sy = (y * CAM_H) / DISP_H;
//        if(sy >= CAM_H) sy = CAM_H - 1;
//        for(uint16 x = 0; x < LCD_W; x++)
//        {
//            uint16 sx = (x * CAM_W) / LCD_W;
//            if(sx >= CAM_W) sx = CAM_W - 1;
//            dest[y][x] = src[sy][sx];
//        }
//    }
//}

//int main()
//{
//    clock_init(SYSTEM_CLOCK_600M);
//    debug_init();

//    ips200_init(IPS200_TYPE_SPI);
//    pit_ms_init(PIT_CH0, 10);
//    Servo_Init();
//    MOTOR_INIT();
//    key_init(5);
//    mt9v03x_init();
//    ips200_set_dir(IPS200_PORTAIT);
//    ips200_set_color(RGB565_WHITE, RGB565_BLACK);

//    system_delay_ms(300);

//    while(1)
//    {
//        if(mt9v03x_finish_flag)
//        {
//            mt9v03x_finish_flag = 0;
//            __disable_irq();
//            image_process();
//            __enable_irq();

//            for(uint16 y = 0; y < CAM_H; y++)
//            {
//                for(uint16 x = 0; x < CAM_W; x++)
//                {
//                    frame_buf[y][x] = image[y][x] ? RGB565_WHITE : RGB565_BLACK;
//                    if(x == left_line_list[y]) frame_buf[y][x] = RGB565_RED;
//                    if(x == right_line_list[y]) frame_buf[y][x] = RGB565_BLUE;
//                    if(x == mid_line_list[y]) frame_buf[y][x] = RGB565_GREEN;
//                }
//            }

//            img_show(frame_buf, lcd_buf);
//            ips200_show_rgb565_image(0, 0, (uint16*)lcd_buf, LCD_W, LCD_H, LCD_W, LCD_H, 1);

//            ips200_show_string(10, DISP_H+10, "Error:");
//            ips200_show_int(70, DISP_H+10, error_image, 4);

//            error = error_image;
//        }
//    }
//}






















