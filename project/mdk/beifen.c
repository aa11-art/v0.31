//#include "zf_common_headfile.h"
//#include "isr.h"
//#include <math.h>


//#define CAM_W    MT9V03X_W																			// 摄像头分辨率 320×240
//#define CAM_H    MT9V03X_H
//#define LCD_W    240																						// IPS200 屏幕的分辨率 240×320
//#define LCD_H    320
//#define DISP_H   (LCD_H / 2)

//uint8 img_real[CAM_H][CAM_W] = {0};																				// img_real：存储二值化后的摄像头图像
//uint8 img_calc[CAM_H][CAM_W] = {0};																				// img_calc：存储原始灰度图像
//uint16 frame_buf[CAM_H][CAM_W] = {0};																			// frame_buf：存储要显示到屏幕的图像（RGB565 格式，带颜色标记：红 = 原始边线、绿 = 补线、蓝 = 中线）
//uint8 best_th = 64;																												// best_th：初始化默认二值化阈值（64）

//#define SR_NUM  120 																												// SR_NUM：巡线的有效行数（取摄像头画面下方 120 行）
//#define SR_START ((CAM_H - SR_NUM)/2)  // (120-80)/2=20 → 处理第20行~第99行，上下各裁20行  SR_START：有效行的起始行（摄像头最后一行往前数 120 行）
//int16 l[SR_NUM], r[SR_NUM], m[SR_NUM], lc[SR_NUM], rc[SR_NUM];
//uint8 t_type;
//int16 wm = 0;

//// 数组定义：
//// l[]：每行原始左侧边线坐标（-1 表示没检测到）
//// r[]：每行原始右侧边线坐标（-1 表示没检测到）
//// m[]：每行原始中线坐标（(l+r)/2，没边线则 - 1）
//// lc[]：每行补线后的左侧坐标（修复断边）
//// rc[]：每行补线后的右侧坐标（修复断边）

//// wm：最终的加权中线值

//// 频率更新间隔
//#define TH_UPD_INT   40        		 //二值化计算
//#define LINE_UPD_INT 1   					 //边线检测
//#define MID_UPD_INT 1    					 //中线
//#define WGHT_UPD_INT 1   					 //加权值

//uint8 is_island = 0;																	// 是否进入环岛（0 = 否，1 = 是）
//uint8 island_cnt = 0;																	// island_cnt：环岛计数（进入后累加，到 50 则标记 “出环岛”）
//#define ISL_CNT_MAX 80																// 环岛最大计数（可根据车速调整，车速快则调大）


////权重
//const uint8 w_list[SR_NUM] = {
//1,1,1,1,1,1,1,1,1,1,
//1,1,1,1,1,1,1,1,1,1,
//1,1,1,1,1,1,1,1,1,1,
//1,1,1,1,1,1,1,1,1,1,
//6,6,6,6,6,6,6,6,6,6,
//7,8,9,10,11,12,13,14,15,16,
//17,18,19,20,20,20,20,19,18,
//17,16,15,14,13,12,11,10,9,8,
//7,6,6,6,6,6,6,6,6,6,
//1,1,1,1,1,1,1,1,1,1,
//1,1,1,1,1,1,1,1,1,1,
//1,1,1,1,1,1,1,1,1,1
//};

//// 大津法：
//uint8 otsu_fast(uint8 gray[][CAM_W])
//{
//    uint32 hist[256] = {0}, sum = 0, w0 = 0, sum0 = 0;
//    uint32 max_var = 0, best_th = 64;
//    uint32 total = CAM_W * CAM_H;
//    for(uint16 y = 0; y < CAM_H; y++)
//        for(uint16 x = 0; x < CAM_W; x++)
//        {
//            uint8 g = gray[y][x];
//            hist[g]++;
//            sum += g;
//        }
//    for(uint8 th = 20; th <= 130; th++)
//    {
//        w0 += hist[th];
//        if(w0 < total/10 || w0 > total*9/10) continue;
//        sum0 += th * hist[th];
//        uint32 var = (sum0 * (total - w0) - (sum - sum0) * w0);
//        var *= var;
//        if(var > max_var) {max_var = var; best_th = th;}
//    }
//    return (best_th < 20) ? 20 : best_th;
//}

//// 检测当前点是否为白色区域的边界（仅当前点为白色，且相邻背景为黑色）
//static uint8 is_white_edge(uint8 bin[][CAM_W], uint16 y, uint16 x)
//{
//    if(bin[y][x] != 255) return 0;
//    
//    // 八邻域坐标偏移
//    int8 dy[] = {-1, -1, -1, 0, 0, 1, 1, 1};
//    int8 dx[] = {-1, 0, 1, -1, 1, -1, 0, 1};
//    
//    for(uint8 k = 0; k < 8; k++)
//    {
//        uint16 ny = y + dy[k];
//        uint16 nx = x + dx[k];
//        if(ny < CAM_H && nx < CAM_W && bin[ny][nx] == 0)
//        {
//            return 1;
//        }
//    }
//    return 0;
//}

//// 赛道检测与补线（初始化所有数组为 - 1（表示 “未检测到”））
//void line_detect(uint8 bin[][CAM_W])
//{
//    memset(l, -1, sizeof(l));
//    memset(r, -1, sizeof(r));
//    memset(m, -1, sizeof(m));
//    memset(lc, -1, sizeof(lc));
//    memset(rc, -1, sizeof(rc));
//    
//    for(uint8 i = 0; i < SR_NUM; i++)
//    {
//        uint16 y = SR_START + i;
//        // 左侧边线检测：从左到右扫描，找到白色区域的左边界
//        for(uint16 x = 0; x < CAM_W; x++)
//        {
//            if(is_white_edge(bin, y, x))
//            {l[i] = x; break;}
//        }
//        // 右侧边线检测：从右到左扫描，找到白色区域的右边界
//        for(int16 x = CAM_W-1; x >= 0; x--)
//        {
//            if(is_white_edge(bin, y, x))
//            {r[i] = x; break;}
//        }
//        if(l[i]!=-1 && r[i]!=-1) m[i] = (l[i]+r[i])/2;// 如果左右边线都检测到，计算中线（左右坐标的平均值）
//    }
//    
//    uint8 l_jump = 0, r_jump = 0;
//		// 中线
//		// 检测 “跳变”：统计相邻行边线坐标的跳变次数（差值 > 15 视为跳变）正常赛道边线是连续的，跳变多说明是十字交叉、环岛等特殊场景
//		for(uint8 i = 1; i < SR_NUM; i++)
//    {
//        if(l[i]!=-1 && l[i-1]!=-1 && abs(l[i]-l[i-1])>15) l_jump++;
//        if(r[i]!=-1 && r[i-1]!=-1 && abs(r[i]-r[i-1])>15) r_jump++;
//    }
//    
//		// 赛道类型判断：
//		// t_type=0：正常赛道（跳变少）
//		// t_type=1：十字交叉（左右都跳变≥2 次）
//		// t_type=2：环岛进入（单侧跳变≥3 次，计数 < 50）
//		// t_type=3：环岛退出（计数≥50，重置计数）
//    if(l_jump>=2 && r_jump>=2) {t_type=1; is_island=0;}
//    else if(l_jump>=3 || r_jump>=3)
//    {
//        is_island = 1;
//        island_cnt++;
//        if(island_cnt < ISL_CNT_MAX) t_type=2;
//        else {t_type=3; island_cnt=0; is_island=0;}
//    }
//    else {t_type=0; is_island=0; island_cnt=0;}
//    
//    for(uint8 i = 0; i < SR_NUM; i++)
//    {
//        lc[i] = l[i]; rc[i] = r[i];
//				// 补线逻辑（修复断边）：十字交叉场景：如果当前行没检测到边线，用前 3 行的平均值补线（平滑补线，避免突变）
//        if(t_type==1)
//        {
//            if(lc[i]==-1 && i>5) lc[i] = (lc[i-1]+lc[i-2]+lc[i-3])/3;
//            if(rc[i]==-1 && i>5) rc[i] = (rc[i-1]+rc[i-2]+rc[i-3])/3;
//        }
//				// 环岛场景：环岛是弧形，补线时左侧逐行减 2、右侧逐行加 2（模拟弧形轨迹）
//        else if(t_type==2 || t_type==3)
//        {
//            if(lc[i]==-1 && i>3) lc[i] = lc[i-1] - 2;
//            if(rc[i]==-1 && i>3) rc[i] = rc[i-1] + 2;
//        }
//				// 正常赛道：如果当前行没检测到边线，直接继承上一行的坐标（简单补线，保证连续）
//        else
//        {
//            if(lc[i]==-1 && i>0) lc[i] = lc[i-1];
//            if(rc[i]==-1 && i>0) rc[i] = rc[i-1];
//        }
//				// 补线后重新计算中线，保证中线连续
//        if(lc[i]!=-1 && rc[i]!=-1) m[i] = (lc[i]+rc[i])/2;
//    }
//}

//// 加权中线计算
//// 遍历所有行，只计算有中线的行；
//// 加权平均：wm = 总(中线×权重) / 总权重；
//// 兜底：如果所有行都没中线，返回摄像头中心（避免失控）
//int16 wm_calc(void)
//{
//    int32 sum_m = 0, sum_w = 0;                              // sum_m：中线 × 权重的总和
//    for(uint8 i = 0; i < SR_NUM; i++)												 // sum_w：权重总和
//    {
//        if(m[i]==-1) continue;
//        sum_m += (int32)m[i] * w_list[i];
//        sum_w += w_list[i];
//    }
//    return sum_w==0 ? (CAM_W/2) : (int16)(sum_m/sum_w);
//}

//// 图像显示：
//void img_show(uint16 src[][CAM_W], uint16 dest[][LCD_W])
//{
//    memset(dest, 0, LCD_W*LCD_H*2);															   // memset清空屏幕缓存
//    for(uint16 y = 0; y < DISP_H; y++)                             // sy = y×CAM_H/DISP_H（y 轴缩放）、sx = x×CAM_W/LCD_W（x 轴缩放）
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

//int error;
//extern int error;
//int main()
//{
//	  clock_init(SYSTEM_CLOCK_600M);  // 不可删除
//    debug_init();                   // 调试端口初始化
//	
//	
//		ips200_init(IPS200_TYPE_SPI);
//		pit_ms_init(PIT_CH0, 10);  
//		Servo_Init();
//		MOTOR_INIT();
//		key_init(5);
//		mt9v03x_init();
//	  ips200_set_dir(IPS200_PORTAIT);
//    ips200_set_color(RGB565_WHITE, RGB565_BLACK);

//    uint32 cnt = 0;
//    uint16 lcd_buf[LCD_H][LCD_W] = {0};
//	
//		system_delay_ms(300);           //等待主板其他外设上电完成


//	
//	while(1)
//	{
//		if(mt9v03x_finish_flag) 
//		{
//				mt9v03x_finish_flag = 0;
//				// mt9v03x_image是摄像头原始灰度数据，拷贝到img_calc（计算用）和img_real（显示用）
//        __disable_irq();																						
//        memcpy(img_calc, mt9v03x_image, CAM_W*CAM_H);																
//        memcpy(img_real, img_calc, CAM_W*CAM_H);																		
//        __enable_irq();
//			
//				// 二值化
//        if(cnt % TH_UPD_INT == 0) best_th = otsu_fast(img_calc);

//        for(uint16 y = 0; y < CAM_H; y++)
//            for(uint16 x = 0; x < CAM_W; x++)
//                img_real[y][x] = img_real[y][x] >= best_th ? 255 : 0;

//        if(cnt % LINE_UPD_INT == 0) line_detect(img_real);

//        if(cnt % MID_UPD_INT == 0 && cnt % WGHT_UPD_INT == 0) wm = wm_calc();

//				// 每帧计算加权中线（最终循迹值）
//        for(uint16 y = 0; y < CAM_H; y++)
//        {
//            for(uint16 x = 0; x < CAM_W; x++)
//            {
//                frame_buf[y][x] = img_real[y][x] ? RGB565_WHITE : RGB565_BLACK;
//                uint8 i = y - SR_START;
//                if(i < SR_NUM)
//                {
//                    if(l[i]==x || r[i]==x) frame_buf[y][x] = RGB565_BLUE;							// 原始边线颜色
//                    if(lc[i]==x || rc[i]==x) frame_buf[y][x] = RGB565_RED;						// 中线颜色
//                    if(m[i]==x) frame_buf[y][x] = RGB565_GREEN;												// 补线颜色
//                }
//            }
//        }

//				// 绘制显示图像：
//				// 基础：二值化图像（白 = 赛道，黑 = 背景）；
//				// 标记有效行：
//				// 原始边线 = 蓝色；
//				// 补线后边线 = 绿色；
//				// 中线 = 红色；
//				//（RGB565 是屏幕的颜色格式，比如 RGB565_RED=0xF800）
//        img_show(frame_buf, lcd_buf);
//				
//				// 全屏显示
//        ips200_show_rgb565_image(0, 0, (uint16*)lcd_buf, LCD_W, LCD_H, LCD_W, LCD_H, 1);
//        
//				// 赛道类型
//        ips200_show_string(10, DISP_H+10, "Error:");
//        ips200_show_int(70, DISP_H+10, error, 4);
//				
//       
//				
//        cnt++;         //循环计数 + 1，进入下一轮
//				error = wm - 60;
//		}
//        		
//        
//  }
//	
//	
//}










