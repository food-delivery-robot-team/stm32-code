#include "sys.h"
#include "delay.h"
#include "usart.h"
#include "led.h"
#include "includes.h"
#include "key.h"
#include "lcd.h"
#include "ltdc.h" 
#include "sdram.h"
#include "w25qxx.h"
#include "nand.h"  
#include "mpu.h"
#include "sdmmc_sdcard.h"
#include "malloc.h"
#include "ff.h"
#include "exfuns.h"
#include "fontupd.h"
#include "text.h"
#include "ov5640.h" 
#include "dcmi.h"  
#include "pcf8574.h" 
#include "atk_qrdecode.h"
#include "lidar.h"
#include "touch.h"
#include "ultrasonic.h"
#include "timer.h"
#include "crc.h"
#include "motor_drive.h"
#include "usmart.h"
#include "rs485.h"
#include "wm8978.h"	 
#include "audioplay.h"
#include "common.h"
#include "text.h"
#include "math.h"


/************************************************
 ALIENTEK 阿波罗STM32F7开发板 UCOSIII实验
 例4-1 UCOSII移植实验
 
 UCOSIII中以下优先级用户程序不能使用，ALIENTEK
 将这些优先级分配给了UCOSIII的5个系统内部任务
 优先级0：中断服务服务管理任务 OS_IntQTask()
 优先级1：时钟节拍任务 OS_TickTask()
 优先级2：定时任务 OS_TmrTask()
 优先级OS_CFG_PRIO_MAX-2：统计任务 OS_StatTask()
 优先级OS_CFG_PRIO_MAX-1：空闲任务 OS_IdleTask()
 技术支持：www.openedv.com
 淘宝店铺：http://eboard.taobao.com 
 关注微信公众平台微信号："正点原子"，免费获取STM32资料。
 广州市星翼电子科技有限公司  
 作者：正点原子 @ALIENTEK
************************************************/

//任务优先级
#define START_TASK_PRIO		3
//任务堆栈大小	
#define START_STK_SIZE 		512
//任务控制块
OS_TCB StartTaskTCB;
//任务堆栈	
CPU_STK START_TASK_STK[START_STK_SIZE];
//任务函数
void start_task(void *p_arg);

//任务优先级
#define interface_TASK_PRIO		4
//任务堆栈大小	
#define interface_STK_SIZE 		256
//任务控制块
OS_TCB interfaceTaskTCB;
//任务堆栈	
CPU_STK interface_TASK_STK[interface_STK_SIZE];
void interface_task(void *p_arg);

//任务优先级
#define ov5640_TASK_PRIO		5
//任务堆栈大小	
#define ov5640_STK_SIZE 		1024
//任务控制块
OS_TCB ov5640TaskTCB;
//任务堆栈	
CPU_STK ov5640_TASK_STK[ov5640_STK_SIZE];
void ov5640_task(void *p_arg);

//任务优先级
#define motor_drive_TASK_PRIO		4
//任务堆栈大小	
#define motor_drive_STK_SIZE 		256
//任务控制块
OS_TCB motor_driveTaskTCB;
//任务堆栈	
CPU_STK motor_drive_TASK_STK[motor_drive_STK_SIZE];
void motor_drive_task(void *p_arg);


//任务优先级
#define ultrasonic_TASK_PRIO		4
//任务堆栈大小	
#define ultrasonic_STK_SIZE 		256
//任务控制块
OS_TCB ultrasonicTaskTCB;
//任务堆栈	
CPU_STK ultrasonic_TASK_STK[ultrasonic_STK_SIZE];
void ultrasonic_task(void *p_arg);

//任务优先级
#define AGV_guide_TASK_PRIO		5
//任务堆栈大小	
#define AGV_guide_STK_SIZE 		128
//任务控制块
OS_TCB AGV_guideTaskTCB;
//任务堆栈	
CPU_STK AGV_guide_TASK_STK[AGV_guide_STK_SIZE];
void AGV_guide_task(void *p_arg);

//任务优先级
#define lidar_TASK_PRIO		4
//任务堆栈大小	
#define lidar_STK_SIZE       512
//任务控制块
OS_TCB lidarTaskTCB;
//任务堆栈	
CPU_STK lidar_TASK_STK[lidar_STK_SIZE];
//任务函数
void lidar_task(void *p_arg);

//任务优先级
#define RFID_TASK_PRIO		5
//任务堆栈大小	
#define RFID_STK_SIZE       128
//任务控制块
OS_TCB RFIDTaskTCB;
//任务堆栈	
CPU_STK RFID_TASK_STK[RFID_STK_SIZE];
//任务函数
void RFID_task(void *p_arg);

//任务优先级
#define MUSIC_TASK_PRIO		5
//任务堆栈大小	
#define MUSIC_STK_SIZE       256
//任务控制块
OS_TCB MUSICTaskTCB;
//任务堆栈	
CPU_STK MUSIC_TASK_STK[MUSIC_STK_SIZE];
//任务函数
void MUSIC_task(void *p_arg);

//任务优先级
#define SELECT_TABLE_TASK_PRIO		5
//任务堆栈大小	
#define SELECT_TABLE_STK_SIZE       256
//任务控制块
OS_TCB SELECT_TABLETaskTCB;
//任务堆栈	
CPU_STK SELECT_TABLE_TASK_STK[SELECT_TABLE_STK_SIZE];
//任务函数
void SELECT_TABLE_task(void *p_arg);

//任务优先级
#define LETS_RUN_TASK_PRIO		5
//任务堆栈大小	
#define LETS_RUN_STK_SIZE       256
//任务控制块
OS_TCB LETS_RUNTaskTCB;
//任务堆栈	
CPU_STK LETS_RUN_TASK_STK[LETS_RUN_STK_SIZE];
//任务函数
void LETS_RUN_task(void *p_arg);

//任务优先级
#define WIFI_TASK_PRIO		4
//任务堆栈大小	
#define WIFI_STK_SIZE       256
//任务控制块
OS_TCB WIFITaskTCB;
//任务堆栈	
CPU_STK WIFI_TASK_STK[WIFI_STK_SIZE];
//任务函数
void WIFI_task(void *p_arg);

//任务优先级
#define runing_TASK_PRIO		4
//任务堆栈大小
#define runing_STK_SIZE		256
//任务控制块
OS_TCB	runingTaskTCB;
//任务堆栈
__align(8) CPU_STK	runing_TASK_STK[runing_STK_SIZE];
//任务函数
void runing_task(void *p_arg);



////////////////////////////////////////////////////////
//OS_TMR 	tmr1;		//定时器1
//OS_TMR	tmr2;		//定时器2
//void tmr1_callback(void *p_tmr, void *p_arg); 	//定时器1回调函数
//void tmr2_callback(void *p_tmr, void *p_arg);	//定时器2回调函数
u8 Lidar_Start[2]={0xA5,0x60};
u8 Lidar_Stop[2]={0xA5,0x65};  //开始扫描,停止扫描


u8 ult_detection;
u8 music_play;

u8 interface_menu;
u8 TOUCH_flag=0;

u8 Rx_485_BUF[15];
u8 Rx_485_STA;

u8 RFID_485_STA;
u8 RFID_RESULT;

u8 QR_CODE_RESULT;

u8 SELECT_TABLE_RESULT;
u8 LETS_RUN_FLAG;//是否有送餐任务

u8 USART_RX_FLAG;
u16 USART_RX_STORAGE;

u16 qr_image_width;						//输入识别图像的宽度（长度=宽度）
u8 	readok=0;									//采集完一帧数据标识
u32 *dcmi_line_buf[2];				//摄像头采用一行一行读取,定义行缓存  
u16 *rgb_data_buf;						//RGB565帧缓存buf 
u16 dcmi_curline=0;						//摄像头输出数据,当前行编号	

u8 seconfary_menu=0;

void qr_dcmi_rx_callback(void);
void qr_show_image(u16 xoff,u16 yoff,u16 width,u16 height,u16 *imagebuf);
void qr_decode(u16 imagewidth,u16 *imagebuf);
void LCD_Show_Interface(void);



int main(void)
{
    OS_ERR err;
	CPU_SR_ALLOC();
    
	float fac;
	
    Write_Through();                //透写
    Cache_Enable();                 //打开L1-Cache
    HAL_Init();				        //初始化HAL库
    Stm32_Clock_Init(432,25,2,9);   //设置时钟,216Mhz 
    delay_init(216);                //延时初始化
	uart_init(128000);		        //串口初始化
	usart3_init(115200);  						//初始化串口3波特率为115200
	usmart_dev.init(108); 		    //初始化USMART	
    LED_Init();                     //初始化LED
	KEY_Init();                     //初始化按键
	SDRAM_Init();                   //初始化SDRAM
	LCD_Init();                     //初始化LCD
	
	ultrasonic_init();              //初始化超声波传感器
	motor_drive_Init();
	
//	W25QXX_Init();				   	//初始化W25Q256
//	W25QXX_Init();				    //初始化W25Q256
//    WM8978_Init();				    //初始化WM8978
//	WM8978_HPvol_Set(40,40);	    //耳机音量设置
//	WM8978_SPKvol_Set(30);		    //喇叭音量设置
	PCF8574_Init();					//初始化PCF8574
	OV5640_Init();					//初始化OV5640
	RS485_Init(9600);		        //初始化RS485
	tp_dev.init();				    //初始化触摸屏
	my_mem_init(SRAMIN);            //初始化内部内存池
	my_mem_init(SRAMEX);            //初始化外部SDRAM内存池
	my_mem_init(SRAMDTCM);          //初始化内部DTCM内存池
	
	exfuns_init();		            //为fatfs相关变量申请内存  
    f_mount(fs[0],"0:",1);          //挂载SD卡 
 	f_mount(fs[1],"1:",1);          //挂载SPI FLASH.   
	
//	TIM3_Init(9,108-1);             //108M/108=1M的计数频率，自动重装载为100，那么PWM频率为1M/10=100khz
	TIM5_CH1_Cap_Init(0XFFFFFFFF,108-1); //以1MHZ的频率计数
	TIM3_PWM_Init(500-1,108-1);     //108M/108=1M的计数频率，自动重装载为500，那么PWM频率为1M/500=2kHZ
	
	TIM_SetTIM3Compare4(399);	//修改比较值，修改占空比
	TIM_SetTIM3Compare3(399);	//修改比较值，修改占空比
	
	LCD_Draw_Circle(240,400,2);  //标记雷达中心
	
//	HAL_UART_Transmit_IT(&UART1_Handler,Lidar_Start,2);          //串口发送启动指令
	
	POINT_COLOR=RED; 
	LCD_Clear(BLACK); 	
	while(font_init()) 		//检查字库
	{	    
		LCD_ShowString(30,50,200,16,16,(u8*)"Font Error!");
		OSTimeDlyHMSM(0,0,0,200,OS_OPT_TIME_HMSM_STRICT,&err); //延时200ms				  
		LCD_Fill(30,50,240,66,WHITE);//清除显示	     
		OSTimeDlyHMSM(0,0,0,200,OS_OPT_TIME_HMSM_STRICT,&err); //延时200ms				  
	}  	 

	while(OV5640_Init())//初始化OV5640
	{
		Show_Str(30,190,240,16,(u8*)"OV5640 错误!",16,0);
		OSTimeDlyHMSM(0,0,0,200,OS_OPT_TIME_HMSM_STRICT,&err); //延时200ms
	    LCD_Fill(30,190,239,206,WHITE);
		OSTimeDlyHMSM(0,0,0,200,OS_OPT_TIME_HMSM_STRICT,&err); //延时200ms
	}	
	//自动对焦初始化
	OV5640_RGB565_Mode();		//RGB565模式 
	OV5640_Focus_Init(); 
	OV5640_Light_Mode(0);		//自动模式
	OV5640_Color_Saturation(3);	//色彩饱和度0
	OV5640_Brightness(4);		//亮度0
	OV5640_Contrast(3);			//对比度0
	OV5640_Sharpness(33);		//自动锐度
	OV5640_Focus_Constant();//启动持续对焦
	DCMI_Init();						//DCMI配置 

	
	
		
	qr_image_width=lcddev.width;
	if(qr_image_width>480)qr_image_width=480;//这里qr_image_width设置为240的倍数
	if(qr_image_width==320)qr_image_width=240;
	Show_Str(0,(lcddev.height+qr_image_width)/2+4,240,16,(u8*)"识别结果：",16,1);
	
	dcmi_line_buf[0]=mymalloc(SRAMIN,qr_image_width*2);						//为行缓存接收申请内存	
	dcmi_line_buf[1]=mymalloc(SRAMIN,qr_image_width*2);						//为行缓存接收申请内存
	rgb_data_buf=mymalloc(SRAMEX,qr_image_width*qr_image_width*2);//为rgb帧缓存申请内存
	
	dcmi_rx_callback=qr_dcmi_rx_callback;//DMA数据接收中断回调函数
	DCMI_DMA_Init((u32)dcmi_line_buf[0],(u32)dcmi_line_buf[1],qr_image_width/2,DMA_MDATAALIGN_HALFWORD,DMA_MINC_ENABLE);//DCMI DMA配置  
	fac=800/qr_image_width;	//得到比例因子
	OV5640_OutSize_Set((1280-fac*qr_image_width)/2,(800-fac*qr_image_width)/2,qr_image_width,qr_image_width); 
	DCMI_Start(); 					//启动传输	 
		
	printf("SRAM IN:%d\r\n",my_mem_perused(SRAMIN));
	printf("SRAM EX:%d\r\n",my_mem_perused(SRAMEX));
	printf("SRAM DCTM:%d\r\n",my_mem_perused(SRAMDTCM)); 

	
	atk_qr_init();//初始化识别库，为算法申请内存
	
	printf("1SRAM IN:%d\r\n",my_mem_perused(SRAMIN));
	printf("1SRAM EX:%d\r\n",my_mem_perused(SRAMEX));
	printf("1SRAM DCTM:%d\r\n",my_mem_perused(SRAMDTCM));

	
	POINT_COLOR=RED;
	Show_Str_Mid(0,30,"ATK-ESP8266 WIFI模块测试",16,240); 
	while(atk_8266_send_cmd("AT","OK",20))//检查WIFI模块是否在线
	{
		atk_8266_quit_trans();//退出透传
		atk_8266_send_cmd("AT+CIPMODE=0","OK",200);  //关闭透传模式	
//		Show_Str(40,55,200,16,"未检测到模块!!!",16,0);
		OSTimeDlyHMSM(0,0,0,800,OS_OPT_TIME_HMSM_STRICT,&err); //延时200ms
//		LCD_Fill(40,55,200,55+16,WHITE);
//		Show_Str(40,55,200,16,"尝试连接模块...",16,0); 
	} 
	while(atk_8266_send_cmd("ATE0","OK",20));//关闭回显
	atk_8266_msg_show(32,155,0);

	OSInit(&err);		            //初始化UCOSIII
	OS_CRITICAL_ENTER();            //进入临界区
	//创建开始任务
	OSTaskCreate((OS_TCB 	* )&StartTaskTCB,		//任务控制块
				 (CPU_CHAR	* )"start task", 		//任务名字
                 (OS_TASK_PTR )start_task, 			//任务函数
                 (void		* )0,					//传递给任务函数的参数
                 (OS_PRIO	  )START_TASK_PRIO,     //任务优先级
                 (CPU_STK   * )&START_TASK_STK[0],	//任务堆栈基地址
                 (CPU_STK_SIZE)START_STK_SIZE/10,	//任务堆栈深度限位
                 (CPU_STK_SIZE)START_STK_SIZE,		//任务堆栈大小
                 (OS_MSG_QTY  )0,					//任务内部消息队列能够接收的最大消息数目,为0时禁止接收消息
                 (OS_TICK	  )0,					//当使能时间片轮转时的时间片长度，为0时为默认长度，
                 (void   	* )0,					//用户补充的存储区
                 (OS_OPT      )OS_OPT_TASK_STK_CHK|OS_OPT_TASK_STK_CLR|OS_OPT_TASK_SAVE_FP, //任务选项,为了保险起见，所有任务都保存浮点寄存器的值
                 (OS_ERR 	* )&err);				//存放该函数错误时的返回值
	OS_CRITICAL_EXIT();	//退出临界区	 
	OSStart(&err);      //开启UCOSIII
    while(1)
    {
	} 
}

//开始任务函数
void start_task(void *p_arg)
{
	OS_ERR err;
	CPU_SR_ALLOC();
	p_arg = p_arg;

	CPU_Init();
#if OS_CFG_STAT_TASK_EN > 0u
   OSStatTaskCPUUsageInit(&err);  	//统计任务                
#endif
	
#ifdef CPU_CFG_INT_DIS_MEAS_EN		//如果使能了测量中断关闭时间
    CPU_IntDisMeasMaxCurReset();	
#endif

#if	OS_CFG_SCHED_ROUND_ROBIN_EN  //当使用时间片轮转的时候
	 //使能时间片轮转调度功能,设置默认的时间片长度s
	OSSchedRoundRobinCfg(DEF_ENABLED,10,&err);  
#endif		
	
	//创建定时器1
//	OSTmrCreate((OS_TMR		*)&tmr1,		//定时器1
//                (CPU_CHAR	*)"tmr1",		//定时器名字
//                (OS_TICK	 )20,			//20*10=200ms
//                (OS_TICK	 )100,          //100*10=1000ms
//                (OS_OPT		 )OS_OPT_TMR_PERIODIC, //周期模式
//                (OS_TMR_CALLBACK_PTR)tmr1_callback,//定时器1回调函数
//                (void	    *)0,			//参数为0
//                (OS_ERR	    *)&err);		//返回的错误码		
				
	
	OS_CRITICAL_ENTER();	//进入临界区
	//创建运行测试任务
	OSTaskCreate((OS_TCB 	* )&runingTaskTCB,		
				 (CPU_CHAR	* )"runing test task", 		
                 (OS_TASK_PTR )runing_task, 			
                 (void		* )0,					
                 (OS_PRIO	  )runing_TASK_PRIO,     	
                 (CPU_STK   * )&runing_TASK_STK[0],	
                 (CPU_STK_SIZE)runing_STK_SIZE/10,	
                 (CPU_STK_SIZE)runing_STK_SIZE,		
                 (OS_MSG_QTY  )0,					
                 (OS_TICK	  )0,					
                 (void   	* )0,				
                 (OS_OPT      )OS_OPT_TASK_STK_CHK|OS_OPT_TASK_STK_CLR|OS_OPT_TASK_SAVE_FP, 
                 (OS_ERR 	* )&err);
				 
//创建interface任务
	OSTaskCreate((OS_TCB 	* )&interfaceTaskTCB,		
				 (CPU_CHAR	* )"interface task", 		
                 (OS_TASK_PTR )interface_task, 			
                 (void		* )0,					
                 (OS_PRIO	  )interface_TASK_PRIO,     	
                 (CPU_STK   * )&interface_TASK_STK[0],	
                 (CPU_STK_SIZE)interface_STK_SIZE/10,	
                 (CPU_STK_SIZE)interface_STK_SIZE,		
                 (OS_MSG_QTY  )0,					
                 (OS_TICK	  )0,					
                 (void   	* )0,				
                 (OS_OPT      )OS_OPT_TASK_STK_CHK|OS_OPT_TASK_STK_CLR|OS_OPT_TASK_SAVE_FP, 
                 (OS_ERR 	* )&err);
				 
	//创建ov5640任务
	OSTaskCreate((OS_TCB 	* )&ov5640TaskTCB,		
				 (CPU_CHAR	* )"ov5640 task", 		
                 (OS_TASK_PTR )ov5640_task, 			
                 (void		* )0,					
                 (OS_PRIO	  )ov5640_TASK_PRIO,     
                 (CPU_STK   * )&ov5640_TASK_STK[0],	
                 (CPU_STK_SIZE)ov5640_STK_SIZE/10,	
                 (CPU_STK_SIZE)ov5640_STK_SIZE,		
                 (OS_MSG_QTY  )0,					
                 (OS_TICK	  )0,					
                 (void   	* )0,					
                 (OS_OPT      )OS_OPT_TASK_STK_CHK|OS_OPT_TASK_STK_CLR|OS_OPT_TASK_SAVE_FP,
                 (OS_ERR 	* )&err);				
	
				 
	//创建lidar任务
	OSTaskCreate((OS_TCB 	* )&lidarTaskTCB,		
				 (CPU_CHAR	* )"lidar task", 		
                 (OS_TASK_PTR )lidar_task, 			
                 (void		* )0,					
                 (OS_PRIO	  )lidar_TASK_PRIO,     	
                 (CPU_STK   * )&lidar_TASK_STK[0],	
                 (CPU_STK_SIZE)lidar_STK_SIZE/10,	
                 (CPU_STK_SIZE)lidar_STK_SIZE,		
                 (OS_MSG_QTY  )0,					
                 (OS_TICK	  )0,					
                 (void   	* )0,				
                 (OS_OPT      )OS_OPT_TASK_STK_CHK|OS_OPT_TASK_STK_CLR|OS_OPT_TASK_SAVE_FP, 
                 (OS_ERR 	* )&err);
				 
	//创建RFID任务
	OSTaskCreate((OS_TCB 	* )&RFIDTaskTCB,		
				 (CPU_CHAR	* )"RFID task", 		
                 (OS_TASK_PTR )RFID_task, 			
                 (void		* )0,					
                 (OS_PRIO	  )RFID_TASK_PRIO,     	
                 (CPU_STK   * )&RFID_TASK_STK[0],	
                 (CPU_STK_SIZE)RFID_STK_SIZE/10,	
                 (CPU_STK_SIZE)RFID_STK_SIZE,		
                 (OS_MSG_QTY  )0,					
                 (OS_TICK	  )0,					
                 (void   	* )0,				
                 (OS_OPT      )OS_OPT_TASK_STK_CHK|OS_OPT_TASK_STK_CLR|OS_OPT_TASK_SAVE_FP, 
                 (OS_ERR 	* )&err);
	//创建MUSIC任务
	OSTaskCreate((OS_TCB 	* )&MUSICTaskTCB,		
				 (CPU_CHAR	* )"MUSIC task", 		
                 (OS_TASK_PTR )MUSIC_task, 			
                 (void		* )0,					
                 (OS_PRIO	  )MUSIC_TASK_PRIO,     	
                 (CPU_STK   * )&MUSIC_TASK_STK[0],	
                 (CPU_STK_SIZE)MUSIC_STK_SIZE/10,	
                 (CPU_STK_SIZE)MUSIC_STK_SIZE,		
                 (OS_MSG_QTY  )0,					
                 (OS_TICK	  )0,					
                 (void   	* )0,				
                 (OS_OPT      )OS_OPT_TASK_STK_CHK|OS_OPT_TASK_STK_CLR|OS_OPT_TASK_SAVE_FP, 
                 (OS_ERR 	* )&err);
//创建RFID任务
	OSTaskCreate((OS_TCB 	* )&SELECT_TABLETaskTCB,		
				 (CPU_CHAR	* )"SELECT_TABLE task", 		
                 (OS_TASK_PTR )SELECT_TABLE_task, 			
                 (void		* )0,					
                 (OS_PRIO	  )SELECT_TABLE_TASK_PRIO,     	
                 (CPU_STK   * )&SELECT_TABLE_TASK_STK[0],	
                 (CPU_STK_SIZE)SELECT_TABLE_STK_SIZE/10,	
                 (CPU_STK_SIZE)SELECT_TABLE_STK_SIZE,		
                 (OS_MSG_QTY  )0,					
                 (OS_TICK	  )0,					
                 (void   	* )0,				
                 (OS_OPT      )OS_OPT_TASK_STK_CHK|OS_OPT_TASK_STK_CLR|OS_OPT_TASK_SAVE_FP, 
                 (OS_ERR 	* )&err);
//创建lidar任务
	OSTaskCreate((OS_TCB 	* )&LETS_RUNTaskTCB,		
				 (CPU_CHAR	* )"LETS_RUN task", 		
                 (OS_TASK_PTR )LETS_RUN_task, 			
                 (void		* )0,					
                 (OS_PRIO	  )LETS_RUN_TASK_PRIO,     	
                 (CPU_STK   * )&LETS_RUN_TASK_STK[0],	
                 (CPU_STK_SIZE)LETS_RUN_STK_SIZE/10,	
                 (CPU_STK_SIZE)LETS_RUN_STK_SIZE,		
                 (OS_MSG_QTY  )0,					
                 (OS_TICK	  )0,					
                 (void   	* )0,				
                 (OS_OPT      )OS_OPT_TASK_STK_CHK|OS_OPT_TASK_STK_CLR|OS_OPT_TASK_SAVE_FP, 
                 (OS_ERR 	* )&err);
//创建lidar任务
	OSTaskCreate((OS_TCB 	* )&WIFITaskTCB,		
				 (CPU_CHAR	* )"WIFI task", 		
                 (OS_TASK_PTR )WIFI_task, 			
                 (void		* )0,					
                 (OS_PRIO	  )WIFI_TASK_PRIO,     	
                 (CPU_STK   * )&WIFI_TASK_STK[0],	
                 (CPU_STK_SIZE)WIFI_STK_SIZE/10,	
                 (CPU_STK_SIZE)WIFI_STK_SIZE,		
                 (OS_MSG_QTY  )0,					
                 (OS_TICK	  )0,					
                 (void   	* )0,				
                 (OS_OPT      )OS_OPT_TASK_STK_CHK|OS_OPT_TASK_STK_CLR|OS_OPT_TASK_SAVE_FP, 
                 (OS_ERR 	* )&err);
//创建ultrasonic任务
	OSTaskCreate((OS_TCB 	* )&ultrasonicTaskTCB,		
				 (CPU_CHAR	* )"ultrasonic task", 		
                 (OS_TASK_PTR )ultrasonic_task, 			
                 (void		* )0,					
                 (OS_PRIO	  )ultrasonic_TASK_PRIO,     	
                 (CPU_STK   * )&ultrasonic_TASK_STK[0],	
                 (CPU_STK_SIZE)ultrasonic_STK_SIZE/10,	
                 (CPU_STK_SIZE)ultrasonic_STK_SIZE,		
                 (OS_MSG_QTY  )0,					
                 (OS_TICK	  )0,					
                 (void   	* )0,				
                 (OS_OPT      )OS_OPT_TASK_STK_CHK|OS_OPT_TASK_STK_CLR|OS_OPT_TASK_SAVE_FP, 
                 (OS_ERR 	* )&err);				 

//创建AGV_guide任务
	OSTaskCreate((OS_TCB 	* )&AGV_guideTaskTCB,		
				 (CPU_CHAR	* )"AGV_guide task", 		
                 (OS_TASK_PTR )AGV_guide_task, 			
                 (void		* )0,					
                 (OS_PRIO	  )AGV_guide_TASK_PRIO,     	
                 (CPU_STK   * )&AGV_guide_TASK_STK[0],	
                 (CPU_STK_SIZE)AGV_guide_STK_SIZE/10,	
                 (CPU_STK_SIZE)AGV_guide_STK_SIZE,		
                 (OS_MSG_QTY  )0,					
                 (OS_TICK	  )0,					
                 (void   	* )0,				
                 (OS_OPT      )OS_OPT_TASK_STK_CHK|OS_OPT_TASK_STK_CLR|OS_OPT_TASK_SAVE_FP, 
                 (OS_ERR 	* )&err);

//创建motor_drive_guide任务
	OSTaskCreate((OS_TCB 	* )&motor_driveTaskTCB,		
				 (CPU_CHAR	* )"motor_drive task", 		
                 (OS_TASK_PTR )motor_drive_task, 			
                 (void		* )0,					
                 (OS_PRIO	  )motor_drive_TASK_PRIO,     	
                 (CPU_STK   * )&motor_drive_TASK_STK[0],	
                 (CPU_STK_SIZE)motor_drive_STK_SIZE/10,	
                 (CPU_STK_SIZE)motor_drive_STK_SIZE,		
                 (OS_MSG_QTY  )0,					
                 (OS_TICK	  )0,					
                 (void   	* )0,				
                 (OS_OPT      )OS_OPT_TASK_STK_CHK|OS_OPT_TASK_STK_CLR|OS_OPT_TASK_SAVE_FP, 
                 (OS_ERR 	* )&err);				 
				 
	OS_CRITICAL_EXIT();	//进入临界区				 
	OS_TaskSuspend((OS_TCB*)&StartTaskTCB,&err);		//挂起开始任务			 
}



void runing_task(void *p_arg)
{
	u8 key;
	u16 times;
	
	OS_ERR err;
	p_arg = p_arg;
	
	OSTaskSuspend((OS_TCB*)&ov5640TaskTCB,&err);
	OSTaskSuspend((OS_TCB*)&SELECT_TABLETaskTCB,&err);
	OSTaskSuspend((OS_TCB*)&LETS_RUNTaskTCB,&err);
	OSTaskSuspend((OS_TCB*)&lidarTaskTCB,&err);
	
	seconfary_menu=0;
	while(1)
	{
		key=KEY_Scan(0);
		tp_dev.scan(0); 		 
		if(tp_dev.sta&TP_PRES_DOWN)			//触摸屏被按下
		{	
		 	if(tp_dev.x[0]<lcddev.width&&tp_dev.y[0]<lcddev.height)
			{	
//				printf("%d   %d",tp_dev.x[0],tp_dev.y[0]);
				if(tp_dev.x[0]>20&&tp_dev.y[0]>20&&tp_dev.x[0]<160&&tp_dev.y[0]<80&&seconfary_menu==0&&TOUCH_flag==0)
				{
					TOUCH_flag=1;
					seconfary_menu=1;
					
					OSTaskResume((OS_TCB*)&interfaceTaskTCB,&err);				
					OSTaskResume((OS_TCB*)&ov5640TaskTCB,&err);	//有时候无法解挂，多重复几次以确保解挂
					OSTimeDlyHMSM(0,0,0,10,OS_OPT_TIME_HMSM_STRICT,&err);	//没有按键按下的时候 
					OSTaskResume((OS_TCB*)&ov5640TaskTCB,&err);	
					OSTimeDlyHMSM(0,0,0,10,OS_OPT_TIME_HMSM_STRICT,&err);	//没有按键按下的时候 
					OSTaskResume((OS_TCB*)&ov5640TaskTCB,&err);	
					OSTimeDlyHMSM(0,0,0,10,OS_OPT_TIME_HMSM_STRICT,&err);	//没有按键按下的时候 
//					printf("resumeov5640TaskTCB\r\n");
				}
				else if(tp_dev.x[0]>20&&tp_dev.y[0]>100&&tp_dev.x[0]<160&&tp_dev.y[0]<180&&seconfary_menu==0&&TOUCH_flag==0)
				{
					TOUCH_flag=2;
					seconfary_menu=1;
					
					OSTaskResume((OS_TCB*)&interfaceTaskTCB,&err);
					OSTaskResume((OS_TCB*)&SELECT_TABLETaskTCB,&err);	
					
				}
				else if(tp_dev.x[0]>20&&tp_dev.y[0]>200&&tp_dev.x[0]<160&&tp_dev.y[0]<280&&seconfary_menu==0&&TOUCH_flag==0)
				{
					TOUCH_flag=3;
					seconfary_menu=1;
					USART_RX_STA=0;
					HAL_UART_Transmit_IT(&UART1_Handler,Lidar_Start,2);          //串口发送启动指令
					OSTaskResume((OS_TCB*)&interfaceTaskTCB,&err);
					OSTimeDlyHMSM(0,0,0,10,OS_OPT_TIME_HMSM_STRICT,&err);	//没有按键按下的时候 
					OSTaskResume((OS_TCB*)&lidarTaskTCB,&err);	
					OSTimeDlyHMSM(0,0,0,10,OS_OPT_TIME_HMSM_STRICT,&err);	//没有按键按下的时候 
					OSTaskResume((OS_TCB*)&lidarTaskTCB,&err);	
					OSTimeDlyHMSM(0,0,0,10,OS_OPT_TIME_HMSM_STRICT,&err);	//没有按键按下的时候 
					OSTaskResume((OS_TCB*)&lidarTaskTCB,&err);	
					OSTimeDlyHMSM(0,0,0,10,OS_OPT_TIME_HMSM_STRICT,&err);	//没有按键按下的时候 
					
				}
				else if(tp_dev.x[0]>180&&tp_dev.y[0]>20&&tp_dev.x[0]<280&&tp_dev.y[0]<80&&seconfary_menu==1&&TOUCH_flag!=0)
				{
					if(TOUCH_flag==3)
					{
						USART_RX_STA|=0x8000;
					}
					seconfary_menu=0;
					TOUCH_flag=0;
					OSTaskSuspend((OS_TCB*)&ov5640TaskTCB,&err);
					OSTaskSuspend((OS_TCB*)&SELECT_TABLETaskTCB,&err);
					HAL_UART_Transmit_IT(&UART1_Handler,Lidar_Stop,2);          //串口发送停止指令
					OSTaskSuspend((OS_TCB*)&lidarTaskTCB,&err);
//					printf("suspend\r\n");
//					LCD_Clear(WHITE);
					OSTaskResume((OS_TCB*)&interfaceTaskTCB,&err);
					
				}
				else if((tp_dev.x[0]>20&&tp_dev.y[0]>170&&tp_dev.x[0]<120&&tp_dev.y[0]<230&&seconfary_menu==1&&TOUCH_flag==2)||
						(tp_dev.x[0]>20&&tp_dev.y[0]>290&&tp_dev.x[0]<120&&tp_dev.y[0]<350&&seconfary_menu==1&&TOUCH_flag==2)||
						(tp_dev.x[0]>160&&tp_dev.y[0]>120&&tp_dev.x[0]<260&&tp_dev.y[0]<280&&seconfary_menu==1&&TOUCH_flag==2)||
						(tp_dev.x[0]>160&&tp_dev.y[0]>240&&tp_dev.x[0]<260&&tp_dev.y[0]<300&&seconfary_menu==1&&TOUCH_flag==2)||
						(tp_dev.x[0]>160&&tp_dev.y[0]>360&&tp_dev.x[0]<260&&tp_dev.y[0]<420&&seconfary_menu==1&&TOUCH_flag==2))
						{
							if(tp_dev.x[0]>20&&tp_dev.y[0]>170&&tp_dev.x[0]<120&&tp_dev.y[0]<230&&seconfary_menu==1&&TOUCH_flag==2)  SELECT_TABLE_RESULT=1;
							if(tp_dev.x[0]>20&&tp_dev.y[0]>290&&tp_dev.x[0]<120&&tp_dev.y[0]<350&&seconfary_menu==1&&TOUCH_flag==2)  SELECT_TABLE_RESULT=2;
							if(tp_dev.x[0]>160&&tp_dev.y[0]>120&&tp_dev.x[0]<260&&tp_dev.y[0]<280&&seconfary_menu==1&&TOUCH_flag==2) SELECT_TABLE_RESULT=3;
							if(tp_dev.x[0]>160&&tp_dev.y[0]>240&&tp_dev.x[0]<260&&tp_dev.y[0]<300&&seconfary_menu==1&&TOUCH_flag==2) SELECT_TABLE_RESULT=4;
							if(tp_dev.x[0]>160&&tp_dev.y[0]>360&&tp_dev.x[0]<260&&tp_dev.y[0]<420&&seconfary_menu==1&&TOUCH_flag==2) SELECT_TABLE_RESULT=5;					
							
							seconfary_menu=0;
							TOUCH_flag=0;
							OSTaskSuspend((OS_TCB*)&ov5640TaskTCB,&err);
							OSTaskSuspend((OS_TCB*)&SELECT_TABLETaskTCB,&err);
							HAL_UART_Transmit_IT(&UART1_Handler,Lidar_Stop,2);          //串口发送停止指令
							OSTaskSuspend((OS_TCB*)&lidarTaskTCB,&err);
		//					printf("suspend\r\n");
		//					LCD_Clear(WHITE);
							OSTaskResume((OS_TCB*)&interfaceTaskTCB,&err);
							
							OSTimeDlyHMSM(0,0,1,0,OS_OPT_TIME_HMSM_STRICT,&err); //延时200ms
							
							
						}
				else OSTimeDlyHMSM(0,0,0,10,OS_OPT_TIME_HMSM_STRICT,&err);	//没有按键按下的时候 			   
			}
			
		}else OSTimeDlyHMSM(0,0,0,10,OS_OPT_TIME_HMSM_STRICT,&err);	//没有按键按下的时候 
		times++;
		if(times>200) 
		{
//			printf("TOUCH_flag:%d seconfary_menu:%d \r\n",TOUCH_flag,seconfary_menu);
			OSTaskResume((OS_TCB*)&interfaceTaskTCB,&err);//每5秒更新显示
			times=0;
		}
	}
}

//interface任务函数
void interface_task(void *p_arg)
{
	
	OS_ERR err;
	p_arg = p_arg;
	while(1)
	{	
		if(TOUCH_flag==0)
		{
			LCD_Clear(WHITE);//清屏 
			LCD_DrawRectangle(20, 20, 160, 80);
			LCD_ShowString(30,35,200,16,32,(u8*)"QR CODE");
			LCD_DrawRectangle(20, 100, 160, 180);
			LCD_ShowString(30,125,200,16,32,(u8*)"SELECT");
			LCD_DrawRectangle(20, 200, 160, 280);
			LCD_ShowString(30,220,200,16,32,(u8*)"LIDAR");
			
			LCD_ShowString(30,290,200,16,32,(u8*)"WIFI:");
			LCD_ShowString(30,330,200,16,32,(u8*)"RX:");
			LCD_ShowString(30,420,200,16,32,(u8*)"TASK:");
			if(SELECT_TABLE_RESULT==0) LCD_ShowString(120,420,200,16,32,(u8*)"NONE");
			else LCD_ShowNum(120,420,SELECT_TABLE_RESULT,1,32);
//			printf("SELECT_TABLE_RESULT %d",SELECT_TABLE_RESULT);
		}
		if(TOUCH_flag==1)
		{
			LCD_Clear(WHITE);//清屏 
			LCD_DrawRectangle(180, 20, 280, 80);
			LCD_ShowString(195,30,200,16,32,(u8*)"EXIT");		
//			TOUCH_flag=0;
		}
		if(TOUCH_flag==2)
		{
			LCD_Clear(WHITE);//清屏 
			LCD_DrawRectangle(180, 20, 260, 80);
			LCD_ShowString(195,30,200,16,32,(u8*)"EXIT");
			LCD_DrawRectangle(20, 170, 120, 230);
			LCD_ShowString(70,180,200,16,32,(u8*)"1");
			LCD_DrawRectangle(20, 290, 120, 350);
			LCD_ShowString(70,300,200,16,32,(u8*)"2");
			LCD_DrawRectangle(160, 120, 260, 180);
			LCD_ShowString(210,130,200,16,32,(u8*)"3");
			LCD_DrawRectangle(160, 240, 260, 300);
			LCD_ShowString(210,250,200,16,32,(u8*)"4");
			LCD_DrawRectangle(160, 360, 260, 420);
			LCD_ShowString(210,370,200,16,32,(u8*)"5");
//			TOUCH_flag=0;
		}
		if(TOUCH_flag==3)
		{
			LCD_Clear(WHITE);//清屏 
			LCD_DrawRectangle(180, 20, 260, 80);
			LCD_ShowString(195,30,200,16,32,(u8*)"EXIT");
			LCD_Draw_Circle(240,400,2);  //标记雷达中心
//			TOUCH_flag=0;
		}
		
		OSTaskSuspend((OS_TCB*)&interfaceTaskTCB,&err);
	}
	
	
	
}

//ov56400任务函数
void ov5640_task(void *p_arg)
{
//	float fac;
	u8 i;
	
	OS_ERR err;
	p_arg = p_arg;
	

		while(1)
		{

			i++;

			if(readok==1)			//采集到了一帧图像
			{	
				printf("readok: %d\r\n",readok);				
				readok=0;
				qr_show_image((lcddev.width-qr_image_width)/2,(lcddev.height-qr_image_width)/2,qr_image_width,qr_image_width,rgb_data_buf);
				qr_decode(qr_image_width,rgb_data_buf);
			}

			
			OSTimeDlyHMSM(0,0,0,50,OS_OPT_TIME_HMSM_STRICT,&err); //延时200ms
			
			if(i>10)
			{
				i=0;
				printf("running\r\n");
			}
			
//			if(i>0) 
//			{
////				OV5640_Init();					//初始化OV5640
////				
////				while(OV5640_Init())//初始化OV5640
////				{
////					Show_Str(30,190,240,16,(u8*)"OV5640 错误!",16,0);
////					OSTimeDlyHMSM(0,0,0,200,OS_OPT_TIME_HMSM_STRICT,&err); //延时200ms
////					LCD_Fill(30,190,239,206,WHITE);
////					OSTimeDlyHMSM(0,0,0,200,OS_OPT_TIME_HMSM_STRICT,&err); //延时200ms
////				}	
////				//自动对焦初始化
////				OV5640_RGB565_Mode();		//RGB565模式 
////				OV5640_Focus_Init(); 
////				OV5640_Light_Mode(0);		//自动模式
////				OV5640_Color_Saturation(3);	//色彩饱和度0
////				OV5640_Brightness(4);		//亮度0
////				OV5640_Contrast(3);			//对比度0
////				OV5640_Sharpness(33);		//自动锐度
////				OV5640_Focus_Constant();//启动持续对焦
////				DCMI_Init();						//DCMI配置 
////				
////				qr_image_width=lcddev.width;
////				if(qr_image_width>480)qr_image_width=480;//这里qr_image_width设置为240的倍数
////				if(qr_image_width==320)qr_image_width=240;
////				Show_Str(0,(lcddev.height+qr_image_width)/2+4,240,16,(u8*)"识别结果：",16,1);
////				
////				dcmi_line_buf[0]=mymalloc(SRAMIN,qr_image_width*2);						//为行缓存接收申请内存	
////				dcmi_line_buf[1]=mymalloc(SRAMIN,qr_image_width*2);						//为行缓存接收申请内存
////				rgb_data_buf=mymalloc(SRAMEX,qr_image_width*qr_image_width*2);//为rgb帧缓存申请内存
////				
////				dcmi_rx_callback=qr_dcmi_rx_callback;//DMA数据接收中断回调函数
////				DCMI_DMA_Init((u32)dcmi_line_buf[0],(u32)dcmi_line_buf[1],qr_image_width/2,DMA_MDATAALIGN_HALFWORD,DMA_MINC_ENABLE);//DCMI DMA配置  
////				fac=800/qr_image_width;	//得到比例因子
////				OV5640_OutSize_Set((1280-fac*qr_image_width)/2,(800-fac*qr_image_width)/2,qr_image_width,qr_image_width); 
////				DCMI_Start(); 					//启动传输	 
////					
////				printf("SRAM IN:%d\r\n",my_mem_perused(SRAMIN));
////				printf("SRAM EX:%d\r\n",my_mem_perused(SRAMEX));
////				printf("SRAM DCTM:%d\r\n",my_mem_perused(SRAMDTCM)); 
////				
////				atk_qr_init();//初始化识别库，为算法申请内存
////				
////				printf("1SRAM IN:%d\r\n",my_mem_perused(SRAMIN));
////				printf("1SRAM EX:%d\r\n",my_mem_perused(SRAMEX));
////				printf("1SRAM DCTM:%d\r\n",my_mem_perused(SRAMDTCM));
//				i=0;
//			}
			
		}

}


//lidar任务函数
void RFID_task(void *p_arg)
{
	u8 len,i;
	long num;
	u8 receive[15];

	OS_ERR err;
	p_arg = p_arg;
	
	while(1)
	{
		
//		LCD_ShowString(20,160,200,16,32,(u8*)"result:");
//		RS485_Receive_Data(Rx_485_BUF,&Rx_485_STA);
		if(RFID_485_STA)
		{
			if(Rx_485_BUF[0]==0x02)
			{
				if((Rx_485_BUF[11]==0x0d)&&(Rx_485_BUF[12]==0x0a)&&(Rx_485_BUF[13]==0x03))
				{
					for(i=0;i<14;i++)
					{
						receive[i]=Rx_485_BUF[i];
//						printf("%x\r\n",Rx_485_BUF[i]);
					}			
				}
				if((receive[1]==0x31)&&(receive[6]==0x33)&&(receive[10]==0x34)) RFID_RESULT=1;
				else RFID_RESULT=0;
				if((receive[1]==0x31)&&(receive[6]==0x34)&&(receive[10]==0x38)) RFID_RESULT=2;
				if((receive[1]==0x31)&&(receive[6]==0x33)&&(receive[10]==0x36)) RFID_RESULT=3;
				if((receive[1]==0x31)&&(receive[6]==0x35)&&(receive[10]==0x36)) RFID_RESULT=4;
				if((receive[1]==0x33)&&(receive[6]==0x38)&&(receive[10]==0x39)) RFID_RESULT=5;
				if((receive[1]==0x33)&&(receive[6]==0x39)&&(receive[10]==0x39)) RFID_RESULT=6;
				if((receive[1]==0x33)&&(receive[6]==0x33)&&(receive[10]==0x31)) RFID_RESULT=7;
				if((receive[1]==0x33)&&(receive[6]==0x32)&&(receive[10]==0x33)) RFID_RESULT=8;
				if((receive[1]==0x33)&&(receive[6]==0x34)&&(receive[10]==0x39)) RFID_RESULT=9;
				if((receive[1]==0x33)&&(receive[6]==0x38)&&(receive[10]==0x37)) RFID_RESULT=10;
				
//				printf("RFID_RESULT %d\r\n",RFID_RESULT);
			}
			RFID_485_STA=0;
		}
		OSTimeDlyHMSM(0,0,0,50,OS_OPT_TIME_HMSM_STRICT,&err); //延时200ms
	}
}


//lidar任务函数
void SELECT_TABLE_task(void *p_arg)
{
	OS_ERR err;
	p_arg = p_arg;
	
	while(1)
	{
//		if(tp_dev.sta&TP_PRES_DOWN)			//触摸屏被按下
//		{
//			seconfary_menu=1;
//		}
//		if(tp_dev.x[0]>20&&tp_dev.y[0]>170&&tp_dev.x[0]<120&&tp_dev.y[0]<230&&seconfary_menu==1)  SELECT_TABLE_RESULT=1;
//		if(tp_dev.x[0]>20&&tp_dev.y[0]>290&&tp_dev.x[0]<120&&tp_dev.y[0]<350&&seconfary_menu==1)  SELECT_TABLE_RESULT=2;
//		if(tp_dev.x[0]>160&&tp_dev.y[0]>120&&tp_dev.x[0]<260&&tp_dev.y[0]<280&&seconfary_menu==1) SELECT_TABLE_RESULT=3;
//		if(tp_dev.x[0]>160&&tp_dev.y[0]>240&&tp_dev.x[0]<260&&tp_dev.y[0]<300&&seconfary_menu==1) SELECT_TABLE_RESULT=4;
//		if(tp_dev.x[0]>160&&tp_dev.y[0]>360&&tp_dev.x[0]<260&&tp_dev.y[0]<420&&seconfary_menu==1) SELECT_TABLE_RESULT=5;
			
//		printf("SELECT_TABLE_RESULT:%d\r\n",SELECT_TABLE_RESULT);
		OSTaskResume((OS_TCB*)&LETS_RUNTaskTCB,&err);	
		
		OSTimeDlyHMSM(0,0,0,100,OS_OPT_TIME_HMSM_STRICT,&err); //延时200ms
	}
}
void LETS_RUN_task(void *p_arg)
{
	OS_ERR err;
	p_arg = p_arg;
	
	while(1)
	{
//		printf("RFID_RESULT %d SELECT_TABLE_RESULT %d\r\n",RFID_RESULT,SELECT_TABLE_RESULT);
		if((RFID_RESULT==SELECT_TABLE_RESULT)&&(SELECT_TABLE_RESULT>0))
		{
			LETS_RUN_FLAG=1;
		}
//		printf("LETS_RUN_FLAG %d",LETS_RUN_FLAG);
		
		OSTimeDlyHMSM(0,0,0,100,OS_OPT_TIME_HMSM_STRICT,&err); //延时200ms
	}
}

extern u8 Lidar_receive_flag;

//lidar任务函数
void lidar_task(void *p_arg)
{
	
	
	u32 count;
	extern float AngleFSA,AngleLSA,Anglei[100],Lidar_Distance[100],Lidar_x[100],Lidar_y[100]; //初始角,结束角,中间角,距离
	extern u8 lidar_i,lidar_LSN,lidar_b,lidar_n,lidar_c;
	extern u8 lidar_a,lidar_m,Lidar_Data[150];//接收数组
	float k=0.1;
	u8 key;
	
	OS_ERR err;
	p_arg = p_arg;
	
	while(1)
	{
		
		
		LCD_Draw_Circle(240,400,2);  //标记雷达中心
		

		
		key=KEY_Scan(0);
		switch(key)
		{				 
			
			case  KEY2_PRES:	//LCD比例放大
						k=k+0.02f;
						break;
			
			case  KEY0_PRES:	//LCD比例缩小
						k=k-0.02f;
						break;
		}
	
		if(k==0)
		{
			k=0.02;
		}
		

	
		for(lidar_m=0;lidar_m<=lidar_LSN;lidar_m++)
		{
			LCD_DrawPoint(240-(Lidar_x[lidar_m]*k),400-(Lidar_y[lidar_m]*k));

		}
		
		OSTimeDlyHMSM(0,0,0,3,OS_OPT_TIME_HMSM_STRICT,&err); //延时200ms				
		count++;
		if(count>500)
		{
			LCD_Clear(WHITE);
			LCD_DrawRectangle(180, 20, 280, 80);
			LCD_ShowString(195,30,200,16,32,(u8*)"EXIT");		
			count=0;

		}
		
		
	}
}

extern u8 wifi_flag;
	
//lidar任务函数
void WIFI_task(void *p_arg)
{
	u8 i,key;
	u16 rlen;
	
	u8 netpro=0;	//网络模式
	u8 timex=0; 
	u8 ipbuf[16]; 	//IP缓存
	u8 *p;
	u16 t=999;		//加速第一次获取链接状态
	u8 res=0;
	u8 constate=0;	//连接状态
	
	OS_ERR err;
	p_arg = p_arg;
	
	p=mymalloc(SRAMIN,32);							//申请32字节内存
	USART3_RX_STA=0;
	while(1)
	{
		
		
		if(wifi_flag)
		{
			atk_8266_wifiap_test();	//WIFI AP测试

			while(1)
			{
				t++;
				OSTimeDlyHMSM(0,0,0,5,OS_OPT_TIME_HMSM_STRICT,&err); //延时200ms
		  
				if(USART3_RX_STA&0X8000)		//接收到一次数据了
				{ 
					rlen=USART3_RX_STA&0X7FFF;	//得到本次接收到的数据长度
					USART3_RX_BUF[rlen]=0;		//添加结束符 
//					for(i=0;i<rlen;i++)
//					{
//						printf("USART3_RX_BUF:%d\r\n",USART3_RX_BUF[i]);	//发送到串口   
//					}
//					printf("\r\n");	//发送到串口 
					Show_Str(80,340,180,190,USART3_RX_BUF,12,0);//显示接收到的数据  
					if(USART3_RX_BUF[11]==48&&USART3_RX_BUF[12]==49) 
					{
						SELECT_TABLE_RESULT=1;
						OSTaskResume((OS_TCB*)&interfaceTaskTCB,&err);	
						OSTaskResume((OS_TCB*)&LETS_RUNTaskTCB,&err);	
					}
					if(USART3_RX_BUF[11]==48&&USART3_RX_BUF[12]==50) 
					{
						SELECT_TABLE_RESULT=2;
						OSTaskResume((OS_TCB*)&interfaceTaskTCB,&err);	
						OSTaskResume((OS_TCB*)&LETS_RUNTaskTCB,&err);	
					}
					if(USART3_RX_BUF[11]==48&&USART3_RX_BUF[12]==51) 
					{
						SELECT_TABLE_RESULT=3;
						OSTaskResume((OS_TCB*)&interfaceTaskTCB,&err);	
						OSTaskResume((OS_TCB*)&LETS_RUNTaskTCB,&err);	
					}
					if(USART3_RX_BUF[11]==48&&USART3_RX_BUF[12]==53) 
					{
						SELECT_TABLE_RESULT=5;
						OSTaskResume((OS_TCB*)&interfaceTaskTCB,&err);	
						OSTaskResume((OS_TCB*)&LETS_RUNTaskTCB,&err);	
					}
					PCF8574_WriteBit(BEEP_IO,0);//打开蜂鸣器
					OSTimeDlyHMSM(0,0,0,100,OS_OPT_TIME_HMSM_STRICT,&err); //延时200ms
					PCF8574_WriteBit(BEEP_IO,1);
					PCF8574_WriteBit(RS485_RE_IO,0);
//					printf("SELECT_TABLE_RESULT %d\r\n",SELECT_TABLE_RESULT);
					USART3_RX_STA=0;

				}
		
				if(t==1000&&seconfary_menu==0)//连续10秒钟没有收到任何数据,检查连接是不是还存在.
				{
					constate=atk_8266_consta_check();//得到连接状态
					if(constate=='+')Show_Str(120,300,200,12,"连接成功",12,0);  //连接状态
					else Show_Str(120,300,200,24,"连接失败",12,0); 	  	 
					t=0;
				}

				atk_8266_at_response(1);
				
			}
			atk_8266_msg_show(32,155,0);
			OSTaskResume((OS_TCB*)&interfaceTaskTCB,&err);
			
			wifi_flag=0;
		}
	OSTimeDlyHMSM(0,0,0,10,OS_OPT_TIME_HMSM_STRICT,&err); //延时200ms
	}
}

extern u8  TIM5CH1_CAPTURE_STA;		//输入捕获状态		    				
extern u32	TIM5CH1_CAPTURE_VAL;	//输入捕获值 


void ultrasonic_task(void *p_arg)
{
	long long temp=0;
	u8 ult_times0,ult_times1,ult_times2;
	long long ult_temp=0;
	
	OS_ERR err;
	p_arg = p_arg;
	
	while(1)
	{
		if(TIM5CH1_CAPTURE_STA&0X80)        //成功捕获到了一次高电平
		{
			temp=TIM5CH1_CAPTURE_STA&0X3F; 
			temp*=0XFFFFFFFF;		 	    //溢出时间总和
			temp+=TIM5CH1_CAPTURE_VAL;      //得到总的高电平时间
			temp=temp/58;
			
//			printf("HIGH:%lld cm\r\n",temp);//打印总的高点平时间		
			if(temp>60)
			{
				ult_times0++;
				ult_times1=0;
				ult_times2=0;
				if(ult_times0>3)
				{
					ult_detection=0;
					ult_times0=0;
				}
			}
	//		if(temp<100&&temp>50)
	//		{
	//			ult_times++;
	//			if(ult_times>2)
	//			{
	//				ult_detection=1;
	//				ult_times=0;
	//			}
	//		}
			if(temp<60&&temp>20) 
			{	
				ult_times1++;
				ult_times0=0;
				ult_times2=0;
				if(ult_times1>3)
				{
//					music_play=1;
					ult_detection=1;
					ult_times1=0;
				}
			}
			if(temp<20&&temp>0) 
			{
				ult_times2++;
				ult_times1=0;
				ult_times0=0;
				if(ult_times2>3&&LETS_RUN_FLAG==0)
				{
//					music_play=0;
//					printf("boom\r\n");
					ult_detection=2;
					Left_BK(0);Right_BK(0);
					Left_FR(1);Right_FR(0);
					Left_BK(1);Right_BK(1);
					TIM_SetTIM3Compare4(399);	//修改比较值，修改占空比左
					TIM_SetTIM3Compare3(399);	//修改比较值，修改占空比右	
					OSTimeDlyHMSM(0,0,5,0,OS_OPT_TIME_HMSM_STRICT,&err); //延时100ms
					Left_BK(0);Right_BK(0);
					Left_FR(0);Right_FR(1);
					Left_BK(1);Right_BK(1);
					ult_detection=1;
					ult_times2=0;
				}
			}
//			printf("music:%d\r\n\r\n",music_play);

			TIM5CH1_CAPTURE_STA=0;          //开启下一次捕获
		}
		HAL_GPIO_WritePin(GPIOB,GPIO_PIN_15,GPIO_PIN_SET);
		delay_us(15);
		HAL_GPIO_WritePin(GPIOB,GPIO_PIN_15,GPIO_PIN_RESET);

		
		
		
//		printf("%d\r\n",ult_detection);
		
		OSTimeDlyHMSM(0,0,0,100,OS_OPT_TIME_HMSM_STRICT,&err); //延时100ms
	}
}


u8 AGV_INF[8];

void AGV_guide_task(void *p_arg)
{
	u8 len,i;
	
	u16 usart_rx_ss;
	unsigned short crc_check;

	OS_ERR err;
	p_arg = p_arg;
	
	while(1)
	{

		RS485_Receive_Data(Rx_485_BUF,&Rx_485_STA);
		if(Rx_485_STA)
		{
			RFID_485_STA=1;
			if(Rx_485_BUF[0]==1)//磁轨第一位
			{
	//			len=USART_RX_STA&0x3fff;//得到此次接收到的数据长度
				if(Rx_485_STA>8) Rx_485_STA=8;
				crc_check=CRC16((uint8_t*)Rx_485_BUF,6);
				if((Rx_485_BUF[7]==(crc_check>>8))&&(Rx_485_BUF[6]==(crc_check&0X00FF)))
				{
					for(i=0;i<8;i++)
					{
						AGV_INF[i]=Rx_485_BUF[i];
//						printf("%x",Rx_485_BUF[i]);
					}
				}
//				Rx_485_STA=0;
			}
//			Rx_485_STA=0;
		}

		OSTimeDlyHMSM(0,0,0,5,OS_OPT_TIME_HMSM_STRICT,&err); //延时100ms
			
//		if(USART_RX_FLAG==1)
//		{
//			USART_RX_STORAGE++;
//			usart_rx_ss=USART_RX_STORAGE-USART_RX_STA;

//			if(usart_rx_ss>2)
//			{
//				USART_RX_STA|=0x8000;
//				USART_RX_STORAGE=0;
//				USART_RX_FLAG=0;
//			}
//			if(USART_RX_STA==8)
//			{
//				USART_RX_STA|=0x8000;
//				USART_RX_STORAGE=0;
//				USART_RX_FLAG=0;
//			}
//		}
		
	}
}


//motor_drive任务函数
//有轨及超声波避障
void motor_drive_task(void *p_arg)
{
	u16 AGV_feedback;
	u8 ult_flag=0;//检测到前方有障碍物时，控制进入避障流程
	u8 direction_flag=0;//遇到障碍时0选择往左，1选择往右
	u8 process_control=0;
	u8 arrive_process_control=0;
	u8 i;
	OS_ERR err;
	p_arg = p_arg;
	
	while(1)
	{
		AGV_feedback=AGV_INF[4];
		AGV_feedback=(AGV_feedback<<8)+AGV_INF[5];
//		printf("AGV_feedback%x\r\n",AGV_feedback);
		if(ult_detection==0&&ult_flag==0&&LETS_RUN_FLAG==0)
		{
			
			if(((0x0128<AGV_feedback&&AGV_feedback<0x03c8)||(AGV_feedback==0x0108)||(AGV_feedback==0x0060))&&AGV_feedback!=0)
			{
				TIM_SetTIM3Compare4(349);	//修改比较值，修改占空比左
				TIM_SetTIM3Compare3(349);	//修改比较值，修改占空比右	
			}
					
			if((AGV_feedback<0x0128)&&(AGV_feedback!=0x0108)&&(AGV_feedback!=0x0060)&&AGV_feedback!=0)
			{					
				if(process_control==6)
				{
					
					TIM_SetTIM3Compare4(459);	//修改比较值，修改占空比左
					TIM_SetTIM3Compare3(349);	//修改比较值，修改占空比右
					process_control=0;	
					OSTimeDlyHMSM(0,0,3,500,OS_OPT_TIME_HMSM_STRICT,&err); //延时2s	
				}
				if(process_control==7)
				{
					
					TIM_SetTIM3Compare4(498);	//修改比较值，修改占空比左
					TIM_SetTIM3Compare3(349);	//修改比较值，修改占空比右
					process_control=0;	
					OSTimeDlyHMSM(0,0,3,0,OS_OPT_TIME_HMSM_STRICT,&err); //延时2s	
				}
				else
				{
//					printf("aaa\r\n");
					TIM_SetTIM3Compare4(249);	//修改比较值，修改占空比左
					TIM_SetTIM3Compare3(429);	//修改比较值，修改占空比右	
				}					
			}
			
			if(AGV_feedback>0x03c8)
			{
				if(process_control==6)
				{
					TIM_SetTIM3Compare4(349);	//修改比较值，修改占空比左
					TIM_SetTIM3Compare3(429);	//修改比较值，修改占空比右
					process_control=0;	
					OSTimeDlyHMSM(0,0,2,0,OS_OPT_TIME_HMSM_STRICT,&err); //延时2s

				}
				if(process_control==7)
				{
					
					TIM_SetTIM3Compare4(349);	//修改比较值，修改占空比左
					TIM_SetTIM3Compare3(498);	//修改比较值，修改占空比右
					process_control=0;	
					OSTimeDlyHMSM(0,0,3,0,OS_OPT_TIME_HMSM_STRICT,&err); //延时2s	
				}
				else
				{
//					printf("aaa\r\n");
					TIM_SetTIM3Compare4(429);	//修改比较值，修改占空比左
					TIM_SetTIM3Compare3(249);	//修改比较值，修改占空比右
				}
			}
		}
		if(LETS_RUN_FLAG==1)
		{
			if(RFID_RESULT==1||RFID_RESULT==2||RFID_RESULT==3||RFID_RESULT==5)
			{
				if(arrive_process_control==0)
				{
					TIM_SetTIM3Compare4(349);	//修改比较值，修改占空比左
					TIM_SetTIM3Compare3(498);	//修改比较值，修改占空比右
					arrive_process_control=1;
					OSTimeDlyHMSM(0,0,4,0,OS_OPT_TIME_HMSM_STRICT,&err); //延时2s
				}
				if(arrive_process_control==1)
				{
					TIM_SetTIM3Compare4(399);	//修改比较值，修改占空比左
					TIM_SetTIM3Compare3(399);	//修改比较值，修改占空比右
					arrive_process_control=2;
				}
			}
			if(arrive_process_control==2)
			{
				if(RFID_RESULT!=6||RFID_RESULT!=7||RFID_RESULT!=8||RFID_RESULT!=10)
				{
					if((AGV_feedback<0x8000)&&(AGV_feedback!=0))
					{
						Left_BK(0);Right_BK(0);
						Left_FR(1);Right_FR(0);
						Left_BK(1);Right_BK(1);
						TIM_SetTIM3Compare4(399);	//修改比较值，修改占空比左
						TIM_SetTIM3Compare3(498);	//修改比较值，修改占空比右
						OSTimeDlyHMSM(0,0,0,500,OS_OPT_TIME_HMSM_STRICT,&err); //延时2s
						Left_BK(0);Right_BK(0);
						Left_FR(0);Right_FR(1);
						Left_BK(1);Right_BK(1);
						TIM_SetTIM3Compare4(399);	//修改比较值，修改占空比左
						TIM_SetTIM3Compare3(399);	//修改比较值，修改占空比右
					}
					if(AGV_feedback>0x8000)
					{
						Left_BK(0);Right_BK(0);
						Left_FR(1);Right_FR(0);
						Left_BK(1);Right_BK(1);
						TIM_SetTIM3Compare4(498);	//修改比较值，修改占空比左
						TIM_SetTIM3Compare3(399);	//修改比较值，修改占空比右
						OSTimeDlyHMSM(0,0,0,500,OS_OPT_TIME_HMSM_STRICT,&err); //延时2s
						Left_BK(0);Right_BK(0);
						Left_FR(0);Right_FR(1);
						Left_BK(1);Right_BK(1);
						TIM_SetTIM3Compare4(399);	//修改比较值，修改占空比左
						TIM_SetTIM3Compare3(399);	//修改比较值，修改占空比右
					}
				}
				if(RFID_RESULT==6||RFID_RESULT==7||RFID_RESULT==8||RFID_RESULT==10)
				{
					TIM_SetTIM3Compare4(498);	//修改比较值，修改占空比左
					TIM_SetTIM3Compare3(498);	//修改比较值，修改占空比右
					HAL_UART_Transmit_IT(&UART1_Handler,Lidar_Stop,2);          //串口发送停止指令
					LCD_Clear(WHITE);
					OSTimeDlyHMSM(0,0,0,50,OS_OPT_TIME_HMSM_STRICT,&err); //延时5s
					OSTaskResume((OS_TCB*)&ov5640TaskTCB,&err);
					OSTimeDlyHMSM(0,0,0,10,OS_OPT_TIME_HMSM_STRICT,&err); //延时5s
					OSTaskResume((OS_TCB*)&ov5640TaskTCB,&err);
					OSTimeDlyHMSM(0,0,0,10,OS_OPT_TIME_HMSM_STRICT,&err); //延时5s
					OSTaskResume((OS_TCB*)&ov5640TaskTCB,&err);
					qr_show:  if(QR_CODE_RESULT==1) 
					{
						OSTimeDlyHMSM(0,0,2,0,OS_OPT_TIME_HMSM_STRICT,&err); //延时5s
						QR_CODE_RESULT=0;
					}
					else
					{
						OSTimeDlyHMSM(0,0,0,100,OS_OPT_TIME_HMSM_STRICT,&err); //延时5s
						goto qr_show;
					}
					OSTaskSuspend((OS_TCB*)&ov5640TaskTCB,&err);
					OSTaskResume((OS_TCB*)&interfaceTaskTCB,&err);
					PCF8574_WriteBit(RS485_RE_IO,0);
//					Left_BK(0);Right_BK(0);
//					Left_FR(1);Right_FR(0);
//					Left_BK(1);Right_BK(1);
//					TIM_SetTIM3Compare4(399);	//修改比较值，修改占空比左
//					TIM_SetTIM3Compare3(399);	//修改比较值，修改占空比右
//					OSTimeDlyHMSM(0,0,2,0,OS_OPT_TIME_HMSM_STRICT,&err); //延时5s
//					Left_BK(0);Right_BK(0);
//					Left_FR(0);Right_FR(1);
//					Left_BK(1);Right_BK(1);
					TIM_SetTIM3Compare4(498);	//修改比较值，修改占空比左
					TIM_SetTIM3Compare3(399);	//修改比较值，修改占空比右
					OSTimeDlyHMSM(0,0,11,0,OS_OPT_TIME_HMSM_STRICT,&err); //延时11s
					TIM_SetTIM3Compare4(399);	//修改比较值，修改占空比左
					TIM_SetTIM3Compare3(399);	//修改比较值，修改占空比右
					process_control=7;
					SELECT_TABLE_RESULT=0;
					RFID_RESULT=0;
					LETS_RUN_FLAG=0;
					OSTaskSuspend((OS_TCB*)&LETS_RUNTaskTCB,&err);
					OSTaskResume((OS_TCB*)&interfaceTaskTCB,&err);
					arrive_process_control=0;
				}
			}
		}
		
		if(ult_detection==1&LETS_RUN_FLAG==0) ult_flag=1;//检测到前方有障碍
			
		if((ult_flag==1)&&(direction_flag==0)&&LETS_RUN_FLAG==0)
		{
			if(process_control==0)
			{
				TIM_SetTIM3Compare4(399);	//修改比较值，修改占空比左
				TIM_SetTIM3Compare3(498);	//修改比较值，修改占空比右
				process_control=1;
				OSTimeDlyHMSM(0,0,4,0,OS_OPT_TIME_HMSM_STRICT,&err); //延时2s
		
//				printf("a\r\n");
			}
//			printf("ult_detection%d\r\n",ult_detection);
			if((ult_detection==0)&&(direction_flag==0))
			{
				if(process_control==1)
				{
					TIM_SetTIM3Compare4(399);	//修改比较值，修改占空比左
					TIM_SetTIM3Compare3(399);	//修改比较值，修改占空比右
					process_control=2;
					OSTimeDlyHMSM(0,0,5,0,OS_OPT_TIME_HMSM_STRICT,&err); //延时2s
//					printf("aa\r\n");
				}
				if(process_control==2)
				{
					TIM_SetTIM3Compare4(498);	//修改比较值，修改占空比左
					TIM_SetTIM3Compare3(399);	//修改比较值，修改占空比�
					process_control=3;
					OSTimeDlyHMSM(0,0,4,0,OS_OPT_TIME_HMSM_STRICT,&err); //延时2s
//					printf("bb\r\n");
				}
				if(process_control==3)
				{
					TIM_SetTIM3Compare4(399);	//修改比较值，修改占空比左
					TIM_SetTIM3Compare3(399);	//修改比较值，修改占空比右
					process_control=4;
					OSTimeDlyHMSM(0,0,6,0,OS_OPT_TIME_HMSM_STRICT,&err); //延时2s
				}
				if(process_control==4)
				{
					TIM_SetTIM3Compare4(498);	//修改比较值，修改占空比左
					TIM_SetTIM3Compare3(399);	//修改比较值，修改占空比右
					process_control=5;
					AGV_feedback=0;
					OSTimeDlyHMSM(0,0,4,0,OS_OPT_TIME_HMSM_STRICT,&err); //延时2s
				}
				if(process_control==5)
				{
					TIM_SetTIM3Compare4(399);	//修改比较值，修改占空比左
					TIM_SetTIM3Compare3(399);	//修改比较值，修改占空比右
					process_control=6;

					ult_flag=0;
//					process_control=0;
//					OSTimeDlyHMSM(0,0,2,0,OS_OPT_TIME_HMSM_STRICT,&err); //延时2s
//					printf("cc\r\n");

				}
				
			}
			if(ult_detection>0)
			{
				direction_flag=1;
				if(process_control==1)
				{
					Left_BK(0);Right_BK(0);
					Left_FR(1);Right_FR(0);
					Left_BK(1);Right_BK(1);
					TIM_SetTIM3Compare4(399);	//修改比较值，修改占空比左
					TIM_SetTIM3Compare3(498);	//修改比较值，修改占空比右
					OSTimeDlyHMSM(0,0,4,0,OS_OPT_TIME_HMSM_STRICT,&err); //延时2s
					Left_BK(0);Right_BK(0);
					Left_FR(0);Right_FR(1);
					Left_BK(1);Right_BK(1);
					process_control=0;
//					printf("c\r\n");
				}
				if(process_control==2)
				{
					Left_BK(0);Right_BK(0);
					Left_FR(1);Right_FR(0);
					Left_BK(1);Right_BK(1);
					TIM_SetTIM3Compare4(498);	//修改比较值，修改占空比左
					TIM_SetTIM3Compare3(399);	//修改比较值，修改占空比右
					OSTimeDlyHMSM(0,0,4,0,OS_OPT_TIME_HMSM_STRICT,&err); //延时2s
					TIM_SetTIM3Compare4(399);	//修改比较值，修改占空比左
					TIM_SetTIM3Compare3(399);	//修改比较值，修改占空比右
					OSTimeDlyHMSM(0,0,4,0,OS_OPT_TIME_HMSM_STRICT,&err); //延时2s
					TIM_SetTIM3Compare4(399);	//修改比较值，修改占空比左
					TIM_SetTIM3Compare3(498);	//修改比较值，修改占空比右
					OSTimeDlyHMSM(0,0,4,0,OS_OPT_TIME_HMSM_STRICT,&err); //延时2s
					Left_BK(0);Right_BK(0);
					Left_FR(0);Right_FR(1);
					Left_BK(1);Right_BK(1);
					process_control=0;
				}
			}				
		}
		if((ult_flag==1)&&(direction_flag==1)&&(LETS_RUN_FLAG==0))
		{
			if(process_control==0)
			{
				TIM_SetTIM3Compare4(498);	//修改比较值，修改占空比左
				TIM_SetTIM3Compare3(399);	//修改比较值，修改占空比右
				OSTimeDlyHMSM(0,0,4,0,OS_OPT_TIME_HMSM_STRICT,&err); //延时2s
				process_control=1;
//				printf("q\r\n");
			}
//			OSTimeDlyHMSM(0,0,1,0,OS_OPT_TIME_HMSM_STRICT,&err); //延时2s
			if(ult_detection==0)
			{
				if(process_control==1)
				{
					TIM_SetTIM3Compare4(399);	//修改比较值，修改占空比左
					TIM_SetTIM3Compare3(399);	//修改比较值，修改占空比右
					process_control=2;
					OSTimeDlyHMSM(0,0,6,0,OS_OPT_TIME_HMSM_STRICT,&err); //延时2s
//					printf("ww\r\n");
				}
				if(process_control==2)
				{
					TIM_SetTIM3Compare4(399);	//修改比较值，修改占空比左
					TIM_SetTIM3Compare3(498);	//修改比较值，修改占空比右
					process_control=3;
					OSTimeDlyHMSM(0,0,3,500,OS_OPT_TIME_HMSM_STRICT,&err); //延时2s
//					printf("ee\r\n");
				}
				if(process_control==3)
				{
					TIM_SetTIM3Compare4(399);	//修改比较值，修改占空比左
					TIM_SetTIM3Compare3(399);	//修改比较值，修改占空比右
					process_control=4;
					OSTimeDlyHMSM(0,0,6,0,OS_OPT_TIME_HMSM_STRICT,&err); //延时2s
				}
				if(process_control==4)
				{
					TIM_SetTIM3Compare4(399);	//修改比较值，修改占空比左
					TIM_SetTIM3Compare3(498);	//修改比较值，修改占空比右
					process_control=5;
					OSTimeDlyHMSM(0,0,4,0,OS_OPT_TIME_HMSM_STRICT,&err); //延时2s
				}
				if(process_control==5)
				{
					
					TIM_SetTIM3Compare4(399);	//修改比较值，修改占空比左
					TIM_SetTIM3Compare3(399);	//修改比较值，修改占空比右
//					if(AGV_feedback!=0)
//					{
//						printf("AGV_feedbackAGV_feedback:\r\n\r\n%x\r\n\r\n",AGV_feedback);
//						TIM_SetTIM3Compare4(399);	//修改比较值，修改占空比左
//						TIM_SetTIM3Compare3(498);	//修改比较值，修改占空比右
						ult_flag=0;
						direction_flag=0;
						process_control=6;
//						printf("ff\r\n");
//						OSTimeDlyHMSM(0,0,2,0,OS_OPT_TIME_HMSM_STRICT,&err); //延时2s
//					}
//					printf("w\r\n");
				}
			}
			if(ult_detection>0)
			{
				direction_flag=0;
				if(process_control==1)
				{
					Left_BK(0);Right_BK(0);
					Left_FR(1);Right_FR(0);
					Left_BK(1);Right_BK(1);
					TIM_SetTIM3Compare4(498);	//修改比较值，修改占空比左
					TIM_SetTIM3Compare3(399);	//修改比较值，修改占空比右
					OSTimeDlyHMSM(0,0,4,0,OS_OPT_TIME_HMSM_STRICT,&err); //延时2s
					Left_BK(0);Right_BK(0);
					Left_FR(0);Right_FR(1);
					Left_BK(1);Right_BK(1);
					process_control=0;
//					printf("e\r\n");
				}
				if(process_control==3)
				{
					Left_BK(0);Right_BK(0);
					Left_FR(1);Right_FR(0);
					Left_BK(1);Right_BK(1);
					TIM_SetTIM3Compare4(399);	//修改比较值，修改占空比左
					TIM_SetTIM3Compare3(498);	//修改比较值，修改占空比右
					OSTimeDlyHMSM(0,0,4,0,OS_OPT_TIME_HMSM_STRICT,&err); //延时2s
					TIM_SetTIM3Compare4(399);	//修改比较值，修改占空比左
					TIM_SetTIM3Compare3(399);	//修改比较值，修改占空比右
					OSTimeDlyHMSM(0,0,4,0,OS_OPT_TIME_HMSM_STRICT,&err); //延时2s
					TIM_SetTIM3Compare4(498);	//修改比较值，修改占空比左
					TIM_SetTIM3Compare3(399);	//修改比较值，修改占空比右
					OSTimeDlyHMSM(0,0,4,0,OS_OPT_TIME_HMSM_STRICT,&err); //延时2s
					Left_BK(0);Right_BK(0);
					Left_FR(0);Right_FR(1);
					Left_BK(1);Right_BK(1);
					process_control=0;
				}
			}
		}			
//		printf("%d %d %d %d\r\n",ult_detection,direction_flag,ult_flag,LETS_RUN_FLAG);
//		printf("process_control %d\r\n",process_control);
		OSTimeDlyHMSM(0,0,0,10,OS_OPT_TIME_HMSM_STRICT,&err); //延时10ms
	}
}

//lidar任务函数
void MUSIC_task(void *p_arg)
{
	OS_ERR err;
	p_arg = p_arg;
    
	while(1)
	{
		while(music_play)
		{
//			static u16 i=0;
//			i++;
//			if(i==1)
//			{
//				W25QXX_Init();				   	//初始化W25Q256
//				W25QXX_Init();				    //初始化W25Q256
//				WM8978_Init();				    //初始化WM8978
//				WM8978_HPvol_Set(40,40);	    //耳机音量设置
//				WM8978_SPKvol_Set(30);		    //喇叭音量设置
//				PCF8574_Init();					//初始化PCF8574
//				
//			}
//			audio_play();
//			
//			if(i>5) i=2;
//			OSTimeDlyHMSM(0,0,0,50,OS_OPT_TIME_HMSM_STRICT,&err); //延时200ms
		}
		OSTimeDlyHMSM(0,0,0,50,OS_OPT_TIME_HMSM_STRICT,&err); //延时200ms
	}
}

//摄像头数据DMA接收完成中断回调函数
void qr_dcmi_rx_callback(void)
{  

	CPU_SR_ALLOC();
	u32 *pbuf;
	u16 i;
	pbuf=(u32*)(rgb_data_buf+dcmi_curline*qr_image_width);//将rgb_data_buf地址偏移赋值给pbuf
	
	if(DMADMCI_Handler.Instance->CR&(1<<19))//DMA使用buf1,读取buf0
	{ 
		for(i=0;i<qr_image_width/2;i++)
		{
			pbuf[i]=dcmi_line_buf[0][i];
		} 
	}else 										//DMA使用buf0,读取buf1
	{
		for(i=0;i<qr_image_width/2;i++)
		{
			pbuf[i]=dcmi_line_buf[1][i];
		} 
	} 
	dcmi_curline++;
}

//显示图像
void qr_show_image(u16 xoff,u16 yoff,u16 width,u16 height,u16 *imagebuf)
{

	CPU_SR_ALLOC();
	u16 linecnt=yoff;
	
	if(lcdltdc.pwidth!=0)//RGB屏
	{
		for(linecnt=0;linecnt<height;linecnt++)
		{
			LTDC_Color_Fill(xoff,linecnt+yoff,xoff+width-1,linecnt+yoff,imagebuf+linecnt*width);//RGB屏,DM2D填充 
		}
		
	}else LCD_Color_Fill(xoff,yoff,xoff+width-1,yoff+height-1,imagebuf);	//MCU屏,直接显示
}

//imagewidth:<=240;大于240时,是240的整数倍
//imagebuf:RGB图像数据缓冲区
void qr_decode(u16 imagewidth,u16 *imagebuf)
{
	OS_ERR err;
	CPU_SR_ALLOC();
	static u8 bartype=0; 
	u8 *bmp;
	u8 *result=NULL;
	u16 Color;
	u16 i,j;	
	u16 qr_img_width=0;						//输入识别器的图像宽度,最大不超过240!
	u8 qr_img_scale=0;						//压缩比例因子
	
	if(imagewidth>240)
	{
		if(imagewidth%240)return ;	//不是240的倍数,直接退出
		qr_img_width=240;
		qr_img_scale=imagewidth/qr_img_width;
	}else
	{
		qr_img_width=imagewidth;
		qr_img_scale=1;
	}  
	result=mymalloc(SRAMIN,1536);//申请识别结果存放内存
	bmp=mymalloc(SRAMDTCM,qr_img_width*qr_img_width);//DTCM管理内存为120K，这里申请240*240=56K 
	mymemset(bmp,0,qr_img_width*qr_img_width);
	if(lcdltdc.pwidth==0)//MCU屏,无需镜像
	{ 
		for(i=0;i<qr_img_width;i++)		
		{
			for(j=0;j<qr_img_width;j++)		//将RGB565图片转成灰度
			{	
				Color=*(imagebuf+((i*imagewidth)+j)*qr_img_scale); //按照qr_img_scale压缩成240*240
				*(bmp+i*qr_img_width+j)=(((Color&0xF800)>> 8)*76+((Color&0x7E0)>>3)*150+((Color&0x001F)<<3)*30)>>8;
			}		
		}
	}else	//RGB屏,需要镜像
	{
		for(i=0;i<qr_img_width;i++)		
		{
			for(j=0;j<qr_img_width;j++)		//将RGB565图片转成灰度
			{	
				Color=*(imagebuf+((i*imagewidth)+qr_img_width-j-1)*qr_img_scale);//按照qr_img_scale压缩成240*240
				*(bmp+i*qr_img_width+j)=(((Color&0xF800)>> 8)*76+((Color&0x7E0)>>3)*150+((Color&0x001F)<<3)*30)>>8;
			}		
		}		
	}
	atk_qr_decode(qr_img_width,qr_img_width,bmp,bartype,result);//识别灰度图片（注意：单次耗时约0.2S）
	
	if(result[0]==0)//没有识别出来
	{
		QR_CODE_RESULT=0;
		bartype++;
		if(bartype>=5)bartype=0; 
	}
	else if(result[0]!=0)//识别出来了，显示结果
	{	
		QR_CODE_RESULT=1;
		PCF8574_WriteBit(BEEP_IO,0);//打开蜂鸣器
		OSTimeDlyHMSM(0,0,0,100,OS_OPT_TIME_HMSM_STRICT,&err); //延时200ms
		PCF8574_WriteBit(BEEP_IO,1);
//		POINT_COLOR=BLUE; 
		LCD_Fill(0,(lcddev.height+qr_image_width)/2+20,lcddev.width,lcddev.height,BLACK);
		Show_Str(0,(lcddev.height+qr_image_width)/2+20,lcddev.width,
								(lcddev.height-qr_image_width)/2-20,(u8*)result,16,0							
						);//LCD显示识别结果
		printf("\r\nresult:\r\n%s\r\n",result);//串口打印识别结果 		
	}
	myfree(SRAMDTCM,bmp);		//释放灰度图bmp内存
	myfree(SRAMIN,result);	//释放识别结果	
}  


