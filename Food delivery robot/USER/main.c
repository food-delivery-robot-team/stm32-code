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
 ALIENTEK °¢²¨ÂÞSTM32F7¿ª·¢°å UCOSIIIÊµÑé
 Àý4-1 UCOSIIÒÆÖ²ÊµÑé
 
 UCOSIIIÖÐÒÔÏÂÓÅÏÈ¼¶ÓÃ»§³ÌÐò²»ÄÜÊ¹ÓÃ£¬ALIENTEK
 ½«ÕâÐ©ÓÅÏÈ¼¶·ÖÅä¸øÁËUCOSIIIµÄ5¸öÏµÍ³ÄÚ²¿ÈÎÎñ
 ÓÅÏÈ¼¶0£ºÖÐ¶Ï·þÎñ·þÎñ¹ÜÀíÈÎÎñ OS_IntQTask()
 ÓÅÏÈ¼¶1£ºÊ±ÖÓ½ÚÅÄÈÎÎñ OS_TickTask()
 ÓÅÏÈ¼¶2£º¶¨Ê±ÈÎÎñ OS_TmrTask()
 ÓÅÏÈ¼¶OS_CFG_PRIO_MAX-2£ºÍ³¼ÆÈÎÎñ OS_StatTask()
 ÓÅÏÈ¼¶OS_CFG_PRIO_MAX-1£º¿ÕÏÐÈÎÎñ OS_IdleTask()
 ¼¼ÊõÖ§³Ö£ºwww.openedv.com
 ÌÔ±¦µêÆÌ£ºhttp://eboard.taobao.com 
 ¹Ø×¢Î¢ÐÅ¹«ÖÚÆ½Ì¨Î¢ÐÅºÅ£º"ÕýµãÔ­×Ó"£¬Ãâ·Ñ»ñÈ¡STM32×ÊÁÏ¡£
 ¹ãÖÝÊÐÐÇÒíµç×Ó¿Æ¼¼ÓÐÏÞ¹«Ë¾  
 ×÷Õß£ºÕýµãÔ­×Ó @ALIENTEK
************************************************/

//ÈÎÎñÓÅÏÈ¼¶
#define START_TASK_PRIO		3
//ÈÎÎñ¶ÑÕ»´óÐ¡	
#define START_STK_SIZE 		512
//ÈÎÎñ¿ØÖÆ¿é
OS_TCB StartTaskTCB;
//ÈÎÎñ¶ÑÕ»	
CPU_STK START_TASK_STK[START_STK_SIZE];
//ÈÎÎñº¯Êý
void start_task(void *p_arg);

//ÈÎÎñÓÅÏÈ¼¶
#define interface_TASK_PRIO		4
//ÈÎÎñ¶ÑÕ»´óÐ¡	
#define interface_STK_SIZE 		256
//ÈÎÎñ¿ØÖÆ¿é
OS_TCB interfaceTaskTCB;
//ÈÎÎñ¶ÑÕ»	
CPU_STK interface_TASK_STK[interface_STK_SIZE];
void interface_task(void *p_arg);

//ÈÎÎñÓÅÏÈ¼¶
#define ov5640_TASK_PRIO		5
//ÈÎÎñ¶ÑÕ»´óÐ¡	
#define ov5640_STK_SIZE 		1024
//ÈÎÎñ¿ØÖÆ¿é
OS_TCB ov5640TaskTCB;
//ÈÎÎñ¶ÑÕ»	
CPU_STK ov5640_TASK_STK[ov5640_STK_SIZE];
void ov5640_task(void *p_arg);

//ÈÎÎñÓÅÏÈ¼¶
#define motor_drive_TASK_PRIO		4
//ÈÎÎñ¶ÑÕ»´óÐ¡	
#define motor_drive_STK_SIZE 		256
//ÈÎÎñ¿ØÖÆ¿é
OS_TCB motor_driveTaskTCB;
//ÈÎÎñ¶ÑÕ»	
CPU_STK motor_drive_TASK_STK[motor_drive_STK_SIZE];
void motor_drive_task(void *p_arg);


//ÈÎÎñÓÅÏÈ¼¶
#define ultrasonic_TASK_PRIO		4
//ÈÎÎñ¶ÑÕ»´óÐ¡	
#define ultrasonic_STK_SIZE 		256
//ÈÎÎñ¿ØÖÆ¿é
OS_TCB ultrasonicTaskTCB;
//ÈÎÎñ¶ÑÕ»	
CPU_STK ultrasonic_TASK_STK[ultrasonic_STK_SIZE];
void ultrasonic_task(void *p_arg);

//ÈÎÎñÓÅÏÈ¼¶
#define AGV_guide_TASK_PRIO		5
//ÈÎÎñ¶ÑÕ»´óÐ¡	
#define AGV_guide_STK_SIZE 		128
//ÈÎÎñ¿ØÖÆ¿é
OS_TCB AGV_guideTaskTCB;
//ÈÎÎñ¶ÑÕ»	
CPU_STK AGV_guide_TASK_STK[AGV_guide_STK_SIZE];
void AGV_guide_task(void *p_arg);

//ÈÎÎñÓÅÏÈ¼¶
#define lidar_TASK_PRIO		4
//ÈÎÎñ¶ÑÕ»´óÐ¡	
#define lidar_STK_SIZE       512
//ÈÎÎñ¿ØÖÆ¿é
OS_TCB lidarTaskTCB;
//ÈÎÎñ¶ÑÕ»	
CPU_STK lidar_TASK_STK[lidar_STK_SIZE];
//ÈÎÎñº¯Êý
void lidar_task(void *p_arg);

//ÈÎÎñÓÅÏÈ¼¶
#define RFID_TASK_PRIO		5
//ÈÎÎñ¶ÑÕ»´óÐ¡	
#define RFID_STK_SIZE       128
//ÈÎÎñ¿ØÖÆ¿é
OS_TCB RFIDTaskTCB;
//ÈÎÎñ¶ÑÕ»	
CPU_STK RFID_TASK_STK[RFID_STK_SIZE];
//ÈÎÎñº¯Êý
void RFID_task(void *p_arg);

//ÈÎÎñÓÅÏÈ¼¶
#define MUSIC_TASK_PRIO		5
//ÈÎÎñ¶ÑÕ»´óÐ¡	
#define MUSIC_STK_SIZE       256
//ÈÎÎñ¿ØÖÆ¿é
OS_TCB MUSICTaskTCB;
//ÈÎÎñ¶ÑÕ»	
CPU_STK MUSIC_TASK_STK[MUSIC_STK_SIZE];
//ÈÎÎñº¯Êý
void MUSIC_task(void *p_arg);

//ÈÎÎñÓÅÏÈ¼¶
#define SELECT_TABLE_TASK_PRIO		5
//ÈÎÎñ¶ÑÕ»´óÐ¡	
#define SELECT_TABLE_STK_SIZE       256
//ÈÎÎñ¿ØÖÆ¿é
OS_TCB SELECT_TABLETaskTCB;
//ÈÎÎñ¶ÑÕ»	
CPU_STK SELECT_TABLE_TASK_STK[SELECT_TABLE_STK_SIZE];
//ÈÎÎñº¯Êý
void SELECT_TABLE_task(void *p_arg);

//ÈÎÎñÓÅÏÈ¼¶
#define LETS_RUN_TASK_PRIO		5
//ÈÎÎñ¶ÑÕ»´óÐ¡	
#define LETS_RUN_STK_SIZE       256
//ÈÎÎñ¿ØÖÆ¿é
OS_TCB LETS_RUNTaskTCB;
//ÈÎÎñ¶ÑÕ»	
CPU_STK LETS_RUN_TASK_STK[LETS_RUN_STK_SIZE];
//ÈÎÎñº¯Êý
void LETS_RUN_task(void *p_arg);

//ÈÎÎñÓÅÏÈ¼¶
#define WIFI_TASK_PRIO		4
//ÈÎÎñ¶ÑÕ»´óÐ¡	
#define WIFI_STK_SIZE       256
//ÈÎÎñ¿ØÖÆ¿é
OS_TCB WIFITaskTCB;
//ÈÎÎñ¶ÑÕ»	
CPU_STK WIFI_TASK_STK[WIFI_STK_SIZE];
//ÈÎÎñº¯Êý
void WIFI_task(void *p_arg);

//ÈÎÎñÓÅÏÈ¼¶
#define runing_TASK_PRIO		4
//ÈÎÎñ¶ÑÕ»´óÐ¡
#define runing_STK_SIZE		256
//ÈÎÎñ¿ØÖÆ¿é
OS_TCB	runingTaskTCB;
//ÈÎÎñ¶ÑÕ»
__align(8) CPU_STK	runing_TASK_STK[runing_STK_SIZE];
//ÈÎÎñº¯Êý
void runing_task(void *p_arg);



////////////////////////////////////////////////////////
//OS_TMR 	tmr1;		//¶¨Ê±Æ÷1
//OS_TMR	tmr2;		//¶¨Ê±Æ÷2
//void tmr1_callback(void *p_tmr, void *p_arg); 	//¶¨Ê±Æ÷1»Øµ÷º¯Êý
//void tmr2_callback(void *p_tmr, void *p_arg);	//¶¨Ê±Æ÷2»Øµ÷º¯Êý
u8 Lidar_Start[2]={0xA5,0x60};
u8 Lidar_Stop[2]={0xA5,0x65};  //¿ªÊ¼É¨Ãè,Í£Ö¹É¨Ãè


u8 ult_detection;
u8 music_play;

u8 interface_menu;
u8 TOUCH_flag;

u8 Rx_485_BUF[15];
u8 Rx_485_STA;

u8 RFID_485_STA;
u8 RFID_RESULT;

u8 QR_CODE_RESULT;

u8 SELECT_TABLE_RESULT;
u8 LETS_RUN_FLAG;//ÊÇ·ñÓÐËÍ²ÍÈÎÎñ

u8 USART_RX_FLAG;
u16 USART_RX_STORAGE;

u16 qr_image_width;						//ÊäÈëÊ¶±ðÍ¼ÏñµÄ¿í¶È£¨³¤¶È=¿í¶È£©
u8 	readok=0;									//²É¼¯ÍêÒ»Ö¡Êý¾Ý±êÊ¶
u32 *dcmi_line_buf[2];				//ÉãÏñÍ·²ÉÓÃÒ»ÐÐÒ»ÐÐ¶ÁÈ¡,¶¨ÒåÐÐ»º´æ  
u16 *rgb_data_buf;						//RGB565Ö¡»º´æbuf 
u16 dcmi_curline=0;						//ÉãÏñÍ·Êä³öÊý¾Ý,µ±Ç°ÐÐ±àºÅ	

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
	
    Write_Through();                //Í¸Ð´
    Cache_Enable();                 //´ò¿ªL1-Cache
    HAL_Init();				        //³õÊ¼»¯HAL¿â
    Stm32_Clock_Init(432,25,2,9);   //ÉèÖÃÊ±ÖÓ,216Mhz 
    delay_init(216);                //ÑÓÊ±³õÊ¼»¯
	uart_init(128000);		        //´®¿Ú³õÊ¼»¯
	usart3_init(115200);  						//³õÊ¼»¯´®¿Ú3²¨ÌØÂÊÎª115200
	usmart_dev.init(108); 		    //³õÊ¼»¯USMART	
    LED_Init();                     //³õÊ¼»¯LED
	KEY_Init();                     //³õÊ¼»¯°´¼ü
	SDRAM_Init();                   //³õÊ¼»¯SDRAM
	LCD_Init();                     //³õÊ¼»¯LCD
	
	ultrasonic_init();              //³õÊ¼»¯³¬Éù²¨´«¸ÐÆ÷
	motor_drive_Init();
	
//	W25QXX_Init();				   	//³õÊ¼»¯W25Q256
//	W25QXX_Init();				    //³õÊ¼»¯W25Q256
//    WM8978_Init();				    //³õÊ¼»¯WM8978
//	WM8978_HPvol_Set(40,40);	    //¶ú»úÒôÁ¿ÉèÖÃ
//	WM8978_SPKvol_Set(30);		    //À®°ÈÒôÁ¿ÉèÖÃ
	PCF8574_Init();					//³õÊ¼»¯PCF8574
	OV5640_Init();					//³õÊ¼»¯OV5640
	RS485_Init(9600);		        //³õÊ¼»¯RS485
	tp_dev.init();				    //³õÊ¼»¯´¥ÃþÆÁ
	my_mem_init(SRAMIN);            //³õÊ¼»¯ÄÚ²¿ÄÚ´æ³Ø
	my_mem_init(SRAMEX);            //³õÊ¼»¯Íâ²¿SDRAMÄÚ´æ³Ø
	my_mem_init(SRAMDTCM);          //³õÊ¼»¯ÄÚ²¿DTCMÄÚ´æ³Ø
	
	exfuns_init();		            //ÎªfatfsÏà¹Ø±äÁ¿ÉêÇëÄÚ´æ  
    f_mount(fs[0],"0:",1);          //¹ÒÔØSD¿¨ 
 	f_mount(fs[1],"1:",1);          //¹ÒÔØSPI FLASH.   
	
//	TIM3_Init(9,108-1);             //108M/108=1MµÄ¼ÆÊýÆµÂÊ£¬×Ô¶¯ÖØ×°ÔØÎª100£¬ÄÇÃ´PWMÆµÂÊÎª1M/10=100khz
	TIM5_CH1_Cap_Init(0XFFFFFFFF,108-1); //ÒÔ1MHZµÄÆµÂÊ¼ÆÊý
	TIM3_PWM_Init(500-1,108-1);     //108M/108=1MµÄ¼ÆÊýÆµÂÊ£¬×Ô¶¯ÖØ×°ÔØÎª500£¬ÄÇÃ´PWMÆµÂÊÎª1M/500=2kHZ
	
	TIM_SetTIM3Compare4(399);	//ÐÞ¸Ä±È½ÏÖµ£¬ÐÞ¸ÄÕ¼¿Õ±È
	TIM_SetTIM3Compare3(399);	//ÐÞ¸Ä±È½ÏÖµ£¬ÐÞ¸ÄÕ¼¿Õ±È
	
	LCD_Draw_Circle(240,400,2);  //±ê¼ÇÀ×´ïÖÐÐÄ
	
//	HAL_UART_Transmit_IT(&UART1_Handler,Lidar_Start,2);          //´®¿Ú·¢ËÍÆô¶¯Ö¸Áî
	
	POINT_COLOR=RED; 
	LCD_Clear(BLACK); 	
	while(font_init()) 		//¼ì²é×Ö¿â
	{	    
		LCD_ShowString(30,50,200,16,16,(u8*)"Font Error!");
		OSTimeDlyHMSM(0,0,0,200,OS_OPT_TIME_HMSM_STRICT,&err); //ÑÓÊ±200ms				  
		LCD_Fill(30,50,240,66,WHITE);//Çå³ýÏÔÊ¾	     
		OSTimeDlyHMSM(0,0,0,200,OS_OPT_TIME_HMSM_STRICT,&err); //ÑÓÊ±200ms				  
	}  	 

	while(OV5640_Init())//³õÊ¼»¯OV5640
	{
		Show_Str(30,190,240,16,(u8*)"OV5640 ´íÎó!",16,0);
		OSTimeDlyHMSM(0,0,0,200,OS_OPT_TIME_HMSM_STRICT,&err); //ÑÓÊ±200ms
	    LCD_Fill(30,190,239,206,WHITE);
		OSTimeDlyHMSM(0,0,0,200,OS_OPT_TIME_HMSM_STRICT,&err); //ÑÓÊ±200ms
	}	
	//×Ô¶¯¶Ô½¹³õÊ¼»¯
	OV5640_RGB565_Mode();		//RGB565Ä£Ê½ 
	OV5640_Focus_Init(); 
	OV5640_Light_Mode(0);		//×Ô¶¯Ä£Ê½
	OV5640_Color_Saturation(3);	//É«²Ê±¥ºÍ¶È0
	OV5640_Brightness(4);		//ÁÁ¶È0
	OV5640_Contrast(3);			//¶Ô±È¶È0
	OV5640_Sharpness(33);		//×Ô¶¯Èñ¶È
	OV5640_Focus_Constant();//Æô¶¯³ÖÐø¶Ô½¹
	DCMI_Init();						//DCMIÅäÖÃ 

	
	
		
	qr_image_width=lcddev.width;
	if(qr_image_width>480)qr_image_width=480;//ÕâÀïqr_image_widthÉèÖÃÎª240µÄ±¶Êý
	if(qr_image_width==320)qr_image_width=240;
	Show_Str(0,(lcddev.height+qr_image_width)/2+4,240,16,(u8*)"Ê¶±ð½á¹û£º",16,1);
	
	dcmi_line_buf[0]=mymalloc(SRAMIN,qr_image_width*2);						//ÎªÐÐ»º´æ½ÓÊÕÉêÇëÄÚ´æ	
	dcmi_line_buf[1]=mymalloc(SRAMIN,qr_image_width*2);						//ÎªÐÐ»º´æ½ÓÊÕÉêÇëÄÚ´æ
	rgb_data_buf=mymalloc(SRAMEX,qr_image_width*qr_image_width*2);//ÎªrgbÖ¡»º´æÉêÇëÄÚ´æ
	
	dcmi_rx_callback=qr_dcmi_rx_callback;//DMAÊý¾Ý½ÓÊÕÖÐ¶Ï»Øµ÷º¯Êý
	DCMI_DMA_Init((u32)dcmi_line_buf[0],(u32)dcmi_line_buf[1],qr_image_width/2,DMA_MDATAALIGN_HALFWORD,DMA_MINC_ENABLE);//DCMI DMAÅäÖÃ  
	fac=800/qr_image_width;	//µÃµ½±ÈÀýÒò×Ó
	OV5640_OutSize_Set((1280-fac*qr_image_width)/2,(800-fac*qr_image_width)/2,qr_image_width,qr_image_width); 
	DCMI_Start(); 					//Æô¶¯´«Êä	 
		
	printf("SRAM IN:%d\r\n",my_mem_perused(SRAMIN));
	printf("SRAM EX:%d\r\n",my_mem_perused(SRAMEX));
	printf("SRAM DCTM:%d\r\n",my_mem_perused(SRAMDTCM)); 
	
	atk_qr_init();//³õÊ¼»¯Ê¶±ð¿â£¬ÎªËã·¨ÉêÇëÄÚ´æ
	
	printf("1SRAM IN:%d\r\n",my_mem_perused(SRAMIN));
	printf("1SRAM EX:%d\r\n",my_mem_perused(SRAMEX));
	printf("1SRAM DCTM:%d\r\n",my_mem_perused(SRAMDTCM));
	
	POINT_COLOR=RED;
	Show_Str_Mid(0,30,"ATK-ESP8266 WIFIÄ£¿é²âÊÔ",16,240); 
	while(atk_8266_send_cmd("AT","OK",20))//¼ì²éWIFIÄ£¿éÊÇ·ñÔÚÏß
	{
		atk_8266_quit_trans();//ÍË³öÍ¸´«
		atk_8266_send_cmd("AT+CIPMODE=0","OK",200);  //¹Ø±ÕÍ¸´«Ä£Ê½	
//		Show_Str(40,55,200,16,"Î´¼ì²âµ½Ä£¿é!!!",16,0);
		OSTimeDlyHMSM(0,0,0,800,OS_OPT_TIME_HMSM_STRICT,&err); //ÑÓÊ±200ms
//		LCD_Fill(40,55,200,55+16,WHITE);
//		Show_Str(40,55,200,16,"³¢ÊÔÁ¬½ÓÄ£¿é...",16,0); 
	} 
	while(atk_8266_send_cmd("ATE0","OK",20));//¹Ø±Õ»ØÏÔ
	atk_8266_msg_show(32,155,0);

	OSInit(&err);		            //³õÊ¼»¯UCOSIII
	OS_CRITICAL_ENTER();            //½øÈëÁÙ½çÇø
	//´´½¨¿ªÊ¼ÈÎÎñ
	OSTaskCreate((OS_TCB 	* )&StartTaskTCB,		//ÈÎÎñ¿ØÖÆ¿é
				 (CPU_CHAR	* )"start task", 		//ÈÎÎñÃû×Ö
                 (OS_TASK_PTR )start_task, 			//ÈÎÎñº¯Êý
                 (void		* )0,					//´«µÝ¸øÈÎÎñº¯ÊýµÄ²ÎÊý
                 (OS_PRIO	  )START_TASK_PRIO,     //ÈÎÎñÓÅÏÈ¼¶
                 (CPU_STK   * )&START_TASK_STK[0],	//ÈÎÎñ¶ÑÕ»»ùµØÖ·
                 (CPU_STK_SIZE)START_STK_SIZE/10,	//ÈÎÎñ¶ÑÕ»Éî¶ÈÏÞÎ»
                 (CPU_STK_SIZE)START_STK_SIZE,		//ÈÎÎñ¶ÑÕ»´óÐ¡
                 (OS_MSG_QTY  )0,					//ÈÎÎñÄÚ²¿ÏûÏ¢¶ÓÁÐÄÜ¹»½ÓÊÕµÄ×î´óÏûÏ¢ÊýÄ¿,Îª0Ê±½ûÖ¹½ÓÊÕÏûÏ¢
                 (OS_TICK	  )0,					//µ±Ê¹ÄÜÊ±¼äÆ¬ÂÖ×ªÊ±µÄÊ±¼äÆ¬³¤¶È£¬Îª0Ê±ÎªÄ¬ÈÏ³¤¶È£¬
                 (void   	* )0,					//ÓÃ»§²¹³äµÄ´æ´¢Çø
                 (OS_OPT      )OS_OPT_TASK_STK_CHK|OS_OPT_TASK_STK_CLR|OS_OPT_TASK_SAVE_FP, //ÈÎÎñÑ¡Ïî,ÎªÁË±£ÏÕÆð¼û£¬ËùÓÐÈÎÎñ¶¼±£´æ¸¡µã¼Ä´æÆ÷µÄÖµ
                 (OS_ERR 	* )&err);				//´æ·Å¸Ãº¯Êý´íÎóÊ±µÄ·µ»ØÖµ
	OS_CRITICAL_EXIT();	//ÍË³öÁÙ½çÇø	 
	OSStart(&err);      //¿ªÆôUCOSIII
    while(1)
    {
	} 
}

//¿ªÊ¼ÈÎÎñº¯Êý
void start_task(void *p_arg)
{
	OS_ERR err;
	CPU_SR_ALLOC();
	p_arg = p_arg;

	CPU_Init();
#if OS_CFG_STAT_TASK_EN > 0u
   OSStatTaskCPUUsageInit(&err);  	//Í³¼ÆÈÎÎñ                
#endif
	
#ifdef CPU_CFG_INT_DIS_MEAS_EN		//Èç¹ûÊ¹ÄÜÁË²âÁ¿ÖÐ¶Ï¹Ø±ÕÊ±¼ä
    CPU_IntDisMeasMaxCurReset();	
#endif

#if	OS_CFG_SCHED_ROUND_ROBIN_EN  //µ±Ê¹ÓÃÊ±¼äÆ¬ÂÖ×ªµÄÊ±ºò
	 //Ê¹ÄÜÊ±¼äÆ¬ÂÖ×ªµ÷¶È¹¦ÄÜ,ÉèÖÃÄ¬ÈÏµÄÊ±¼äÆ¬³¤¶Ès
	OSSchedRoundRobinCfg(DEF_ENABLED,10,&err);  
#endif		
	
	//´´½¨¶¨Ê±Æ÷1
//	OSTmrCreate((OS_TMR		*)&tmr1,		//¶¨Ê±Æ÷1
//                (CPU_CHAR	*)"tmr1",		//¶¨Ê±Æ÷Ãû×Ö
//                (OS_TICK	 )20,			//20*10=200ms
//                (OS_TICK	 )100,          //100*10=1000ms
//                (OS_OPT		 )OS_OPT_TMR_PERIODIC, //ÖÜÆÚÄ£Ê½
//                (OS_TMR_CALLBACK_PTR)tmr1_callback,//¶¨Ê±Æ÷1»Øµ÷º¯Êý
//                (void	    *)0,			//²ÎÊýÎª0
//                (OS_ERR	    *)&err);		//·µ»ØµÄ´íÎóÂë		
				
	
	OS_CRITICAL_ENTER();	//½øÈëÁÙ½çÇø
	//´´½¨ÔËÐÐ²âÊÔÈÎÎñ
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
				 
//´´½¨interfaceÈÎÎñ
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
				 
	//´´½¨ov5640ÈÎÎñ
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
	
				 
	//´´½¨lidarÈÎÎñ
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
				 
	//´´½¨RFIDÈÎÎñ
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
	//´´½¨MUSICÈÎÎñ
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
//´´½¨RFIDÈÎÎñ
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
//´´½¨lidarÈÎÎñ
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
//´´½¨lidarÈÎÎñ
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
//´´½¨ultrasonicÈÎÎñ
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

//´´½¨AGV_guideÈÎÎñ
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

//´´½¨motor_drive_guideÈÎÎñ
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
				 
	OS_CRITICAL_EXIT();	//½øÈëÁÙ½çÇø				 
	OS_TaskSuspend((OS_TCB*)&StartTaskTCB,&err);		//¹ÒÆð¿ªÊ¼ÈÎÎñ			 
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
		if(tp_dev.sta&TP_PRES_DOWN)			//´¥ÃþÆÁ±»°´ÏÂ
		{	
		 	if(tp_dev.x[0]<lcddev.width&&tp_dev.y[0]<lcddev.height)
			{	
//				printf("%d   %d",tp_dev.x[0],tp_dev.y[0]);
				if(tp_dev.x[0]>20&&tp_dev.y[0]>20&&tp_dev.x[0]<160&&tp_dev.y[0]<80&&seconfary_menu==0)
				{
					OSTaskResume((OS_TCB*)&interfaceTaskTCB,&err);
					OSTaskResume((OS_TCB*)&ov5640TaskTCB,&err);	
					TOUCH_flag=1;
					seconfary_menu=1;
	//				printf("resume\r\n");
				}
				else if(tp_dev.x[0]>20&&tp_dev.y[0]>100&&tp_dev.x[0]<160&&tp_dev.y[0]<180&&seconfary_menu==0)
				{
					OSTaskResume((OS_TCB*)&interfaceTaskTCB,&err);
					OSTaskResume((OS_TCB*)&SELECT_TABLETaskTCB,&err);	
					TOUCH_flag=2;
					seconfary_menu=1;
				}
				else if(tp_dev.x[0]>20&&tp_dev.y[0]>200&&tp_dev.x[0]<160&&tp_dev.y[0]<280&&seconfary_menu==0)
				{
					HAL_UART_Transmit_IT(&UART1_Handler,Lidar_Start,2);          //´®¿Ú·¢ËÍÆô¶¯Ö¸Áî
					OSTaskResume((OS_TCB*)&interfaceTaskTCB,&err);
					OSTaskResume((OS_TCB*)&lidarTaskTCB,&err);	
					
					
//					OSTaskSuspend((OS_TCB*)&ov5640TaskTCB,&err);
//					OSTaskSuspend((OS_TCB*)&SELECT_TABLETaskTCB,&err);
//					OSTaskSuspend((OS_TCB*)&LETS_RUNTaskTCB,&err);
//					OSTaskSuspend((OS_TCB*)&motor_driveTaskTCB,&err);
//					OSTaskSuspend((OS_TCB*)&AGV_guideTaskTCB,&err);
//					OSTaskSuspend((OS_TCB*)&ultrasonicTaskTCB,&err);
//					OSTaskSuspend((OS_TCB*)&WIFITaskTCB,&err);
//					OSTaskSuspend((OS_TCB*)&MUSICTaskTCB,&err);
//					OSTaskSuspend((OS_TCB*)&RFIDTaskTCB,&err);
					
					
					TOUCH_flag=3;
					seconfary_menu=1;
				}
				else if(tp_dev.x[0]>180&&tp_dev.y[0]>20&&tp_dev.x[0]<280&&tp_dev.y[0]<80&&seconfary_menu==1)
				{
					OSTaskSuspend((OS_TCB*)&ov5640TaskTCB,&err);
					OSTaskSuspend((OS_TCB*)&SELECT_TABLETaskTCB,&err);
					HAL_UART_Transmit_IT(&UART1_Handler,Lidar_Stop,2);          //´®¿Ú·¢ËÍÆô¶¯Ö¸Áî
					OSTaskSuspend((OS_TCB*)&lidarTaskTCB,&err);
//					printf("suspend\r\n");
//					LCD_Clear(WHITE);
					OSTaskResume((OS_TCB*)&interfaceTaskTCB,&err);
					seconfary_menu=0;
				}
				else if((tp_dev.x[0]>20&&tp_dev.y[0]>170&&tp_dev.x[0]<120&&tp_dev.y[0]<230&&seconfary_menu==1)||
						(tp_dev.x[0]>20&&tp_dev.y[0]>290&&tp_dev.x[0]<120&&tp_dev.y[0]<350&&seconfary_menu==1)||
						(tp_dev.x[0]>160&&tp_dev.y[0]>120&&tp_dev.x[0]<260&&tp_dev.y[0]<280&&seconfary_menu==1)||
						(tp_dev.x[0]>160&&tp_dev.y[0]>240&&tp_dev.x[0]<260&&tp_dev.y[0]<300&&seconfary_menu==1)||
						(tp_dev.x[0]>160&&tp_dev.y[0]>360&&tp_dev.x[0]<260&&tp_dev.y[0]<420&&seconfary_menu==1))
						{
							if(tp_dev.x[0]>20&&tp_dev.y[0]>170&&tp_dev.x[0]<120&&tp_dev.y[0]<230&&seconfary_menu==1)  SELECT_TABLE_RESULT=1;
							if(tp_dev.x[0]>20&&tp_dev.y[0]>290&&tp_dev.x[0]<120&&tp_dev.y[0]<350&&seconfary_menu==1)  SELECT_TABLE_RESULT=2;
							if(tp_dev.x[0]>160&&tp_dev.y[0]>120&&tp_dev.x[0]<260&&tp_dev.y[0]<280&&seconfary_menu==1) SELECT_TABLE_RESULT=3;
							if(tp_dev.x[0]>160&&tp_dev.y[0]>240&&tp_dev.x[0]<260&&tp_dev.y[0]<300&&seconfary_menu==1) SELECT_TABLE_RESULT=4;
							if(tp_dev.x[0]>160&&tp_dev.y[0]>360&&tp_dev.x[0]<260&&tp_dev.y[0]<420&&seconfary_menu==1) SELECT_TABLE_RESULT=5;					
							
							OSTaskSuspend((OS_TCB*)&ov5640TaskTCB,&err);
							OSTaskSuspend((OS_TCB*)&SELECT_TABLETaskTCB,&err);
							HAL_UART_Transmit_IT(&UART1_Handler,Lidar_Stop,2);          //´®¿Ú·¢ËÍÆô¶¯Ö¸Áî
							OSTaskSuspend((OS_TCB*)&lidarTaskTCB,&err);
		//					printf("suspend\r\n");
		//					LCD_Clear(WHITE);
							OSTaskResume((OS_TCB*)&interfaceTaskTCB,&err);
							
							OSTimeDlyHMSM(0,0,1,0,OS_OPT_TIME_HMSM_STRICT,&err); //ÑÓÊ±200ms
							
							seconfary_menu=0;
						}
				else OSTimeDlyHMSM(0,0,0,10,OS_OPT_TIME_HMSM_STRICT,&err);	//Ã»ÓÐ°´¼ü°´ÏÂµÄÊ±ºò 			   
			}
			
		}else OSTimeDlyHMSM(0,0,0,10,OS_OPT_TIME_HMSM_STRICT,&err);	//Ã»ÓÐ°´¼ü°´ÏÂµÄÊ±ºò 
		times++;
		if(times==500) 
		{
			OSTaskResume((OS_TCB*)&interfaceTaskTCB,&err);//Ã¿Á½Ãë¸üÐÂÏÔÊ¾
			times=0;
		}
	}
}

//interfaceÈÎÎñº¯Êý
void interface_task(void *p_arg)
{
	
	OS_ERR err;
	p_arg = p_arg;
	while(1)
	{	
		if(TOUCH_flag==0)
		{
			LCD_Clear(WHITE);//ÇåÆÁ 
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
			printf("SELECT_TABLE_RESULT %d",SELECT_TABLE_RESULT);
		}
		if(TOUCH_flag==1)
		{
			LCD_Clear(WHITE);//ÇåÆÁ 
			LCD_DrawRectangle(180, 20, 280, 80);
			LCD_ShowString(195,30,200,16,32,(u8*)"EXIT");		
			TOUCH_flag=0;
		}
		if(TOUCH_flag==2)
		{
			LCD_Clear(WHITE);//ÇåÆÁ 
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
			TOUCH_flag=0;
		}
		if(TOUCH_flag==3)
		{
			LCD_Clear(WHITE);//ÇåÆÁ 
			LCD_DrawRectangle(180, 20, 260, 80);
			LCD_ShowString(195,30,200,16,32,(u8*)"EXIT");
			LCD_Draw_Circle(240,400,2);  //±ê¼ÇÀ×´ïÖÐÐÄ
			TOUCH_flag=0;
		}
		
		OSTaskSuspend((OS_TCB*)&interfaceTaskTCB,&err);
	}
	
	
	
}

//ov56400ÈÎÎñº¯Êý
void ov5640_task(void *p_arg)
{
//	float fac;
	u8 i;
	
	OS_ERR err;
	p_arg = p_arg;
	

		while(1)
		{

			seconfary_menu=1;
//			i++;

			OSTimeDlyHMSM(0,0,0,200,OS_OPT_TIME_HMSM_STRICT,&err); //ÑÓÊ±200ms
//			if(i>0) 
//			{
////				OV5640_Init();					//³õÊ¼»¯OV5640
////				
////				while(OV5640_Init())//³õÊ¼»¯OV5640
////				{
////					Show_Str(30,190,240,16,(u8*)"OV5640 ´íÎó!",16,0);
////					OSTimeDlyHMSM(0,0,0,200,OS_OPT_TIME_HMSM_STRICT,&err); //ÑÓÊ±200ms
////					LCD_Fill(30,190,239,206,WHITE);
////					OSTimeDlyHMSM(0,0,0,200,OS_OPT_TIME_HMSM_STRICT,&err); //ÑÓÊ±200ms
////				}	
////				//×Ô¶¯¶Ô½¹³õÊ¼»¯
////				OV5640_RGB565_Mode();		//RGB565Ä£Ê½ 
////				OV5640_Focus_Init(); 
////				OV5640_Light_Mode(0);		//×Ô¶¯Ä£Ê½
////				OV5640_Color_Saturation(3);	//É«²Ê±¥ºÍ¶È0
////				OV5640_Brightness(4);		//ÁÁ¶È0
////				OV5640_Contrast(3);			//¶Ô±È¶È0
////				OV5640_Sharpness(33);		//×Ô¶¯Èñ¶È
////				OV5640_Focus_Constant();//Æô¶¯³ÖÐø¶Ô½¹
////				DCMI_Init();						//DCMIÅäÖÃ 
////				
////				qr_image_width=lcddev.width;
////				if(qr_image_width>480)qr_image_width=480;//ÕâÀïqr_image_widthÉèÖÃÎª240µÄ±¶Êý
////				if(qr_image_width==320)qr_image_width=240;
////				Show_Str(0,(lcddev.height+qr_image_width)/2+4,240,16,(u8*)"Ê¶±ð½á¹û£º",16,1);
////				
////				dcmi_line_buf[0]=mymalloc(SRAMIN,qr_image_width*2);						//ÎªÐÐ»º´æ½ÓÊÕÉêÇëÄÚ´æ	
////				dcmi_line_buf[1]=mymalloc(SRAMIN,qr_image_width*2);						//ÎªÐÐ»º´æ½ÓÊÕÉêÇëÄÚ´æ
////				rgb_data_buf=mymalloc(SRAMEX,qr_image_width*qr_image_width*2);//ÎªrgbÖ¡»º´æÉêÇëÄÚ´æ
////				
////				dcmi_rx_callback=qr_dcmi_rx_callback;//DMAÊý¾Ý½ÓÊÕÖÐ¶Ï»Øµ÷º¯Êý
////				DCMI_DMA_Init((u32)dcmi_line_buf[0],(u32)dcmi_line_buf[1],qr_image_width/2,DMA_MDATAALIGN_HALFWORD,DMA_MINC_ENABLE);//DCMI DMAÅäÖÃ  
////				fac=800/qr_image_width;	//µÃµ½±ÈÀýÒò×Ó
////				OV5640_OutSize_Set((1280-fac*qr_image_width)/2,(800-fac*qr_image_width)/2,qr_image_width,qr_image_width); 
////				DCMI_Start(); 					//Æô¶¯´«Êä	 
////					
////				printf("SRAM IN:%d\r\n",my_mem_perused(SRAMIN));
////				printf("SRAM EX:%d\r\n",my_mem_perused(SRAMEX));
////				printf("SRAM DCTM:%d\r\n",my_mem_perused(SRAMDTCM)); 
////				
////				atk_qr_init();//³õÊ¼»¯Ê¶±ð¿â£¬ÎªËã·¨ÉêÇëÄÚ´æ
////				
////				printf("1SRAM IN:%d\r\n",my_mem_perused(SRAMIN));
////				printf("1SRAM EX:%d\r\n",my_mem_perused(SRAMEX));
////				printf("1SRAM DCTM:%d\r\n",my_mem_perused(SRAMDTCM));
//				i=0;
//			}
			if(readok==1)			//²É¼¯µ½ÁËÒ»Ö¡Í¼Ïñ
			{		
				readok=0;
				qr_show_image((lcddev.width-qr_image_width)/2,(lcddev.height-qr_image_width)/2,qr_image_width,qr_image_width,rgb_data_buf);
				qr_decode(qr_image_width,rgb_data_buf);
			}

		}

}


//lidarÈÎÎñº¯Êý
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
						printf("%x\r\n",Rx_485_BUF[i]);
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
				
				printf("RFID_RESULT %d\r\n",RFID_RESULT);
			}
			RFID_485_STA=0;
		}
		OSTimeDlyHMSM(0,0,0,50,OS_OPT_TIME_HMSM_STRICT,&err); //ÑÓÊ±200ms
	}
}


//lidarÈÎÎñº¯Êý
void SELECT_TABLE_task(void *p_arg)
{
	OS_ERR err;
	p_arg = p_arg;
	
	while(1)
	{
//		if(tp_dev.sta&TP_PRES_DOWN)			//´¥ÃþÆÁ±»°´ÏÂ
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
		
		OSTimeDlyHMSM(0,0,0,100,OS_OPT_TIME_HMSM_STRICT,&err); //ÑÓÊ±200ms
	}
}
void LETS_RUN_task(void *p_arg)
{
	OS_ERR err;
	p_arg = p_arg;
	
	while(1)
	{
		printf("RFID_RESULT %d SELECT_TABLE_RESULT %d\r\n",RFID_RESULT,SELECT_TABLE_RESULT);
		if((RFID_RESULT==SELECT_TABLE_RESULT)&&(SELECT_TABLE_RESULT>0))
		{
			LETS_RUN_FLAG=1;
		}
//		printf("LETS_RUN_FLAG %d",LETS_RUN_FLAG);
		
		OSTimeDlyHMSM(0,0,0,100,OS_OPT_TIME_HMSM_STRICT,&err); //ÑÓÊ±200ms
	}
}

extern u8 Lidar_receive_flag;

//lidarÈÎÎñº¯Êý
void lidar_task(void *p_arg)
{
	
	
	u32 count;
	extern float AngleFSA,AngleLSA,Anglei[100],Lidar_Distance[100],Lidar_x[100],Lidar_y[100]; //³õÊ¼½Ç,½áÊø½Ç,ÖÐ¼ä½Ç,¾àÀë
	extern u8 lidar_i,lidar_LSN,lidar_b,lidar_n,lidar_c;
	extern u8 lidar_a,lidar_m,Lidar_Data[150];//½ÓÊÕÊý×é
	float k;
	u8 key;
	
	OS_ERR err;
	p_arg = p_arg;
	
	while(1)
	{
		
//		if(tp_dev.sta&TP_PRES_DOWN)			//´¥ÃþÆÁ±»°´ÏÂ
//		{
//			seconfary_menu=1;
//		}
		
		LCD_Draw_Circle(240,400,2);  //±ê¼ÇÀ×´ïÖÐÐÄ
		
//		if(Lidar_receive_flag==1)
//		{
//			Lidar_receive_flag=0;

//				
//			
//		}
		
		key=KEY_Scan(0);
			switch(key)
			{				 
				
				case  KEY2_PRES:	//LCD±ÈÀý·Å´ó
							k=k+0.02f;
							break;
				
				case  KEY0_PRES:	//LCD±ÈÀýËõÐ¡
							k=k-0.02f;
							break;
				case  KEY1_PRES:	//LCD±ÈÀý·Å´ó
							HAL_UART_Transmit_IT(&UART1_Handler,Lidar_Start,2);          //´®¿Ú·¢ËÍÆô¶¯Ö¸Áî
				
							PCF8574_WriteBit(BEEP_IO,0);//´ò¿ª·äÃùÆ÷
							OSTimeDlyHMSM(0,0,0,100,OS_OPT_TIME_HMSM_STRICT,&err); //ÑÓÊ±200ms
							PCF8574_WriteBit(BEEP_IO,1);
							PCF8574_WriteBit(RS485_RE_IO,0);
							break;
			}
		
		if(k==0)
		{
			k=0.02;
		}
		
//		evade();
			
//		LCD_ShowString(0,80,240,32,32,"FSA=");	  //ÏÔÊ¾ÆðÊ¼½Ç,½áÊø½Ç
//		LCD_ShowNum(80,80,AngleFSA,3,32);
//		LCD_ShowString(0,120,240,32,32,"LSA=");	
//		LCD_ShowNum(80,120,AngleLSA,3,32);
	
		for(lidar_m=0;lidar_m<=lidar_LSN;lidar_m++)
		{
//			LCD_DrawPoint(240-(Lidar_x[lidar_m]*k),400-(Lidar_y[lidar_m]*k));
//			LCD_ShowNum(200,20*lidar_m,Anglei[lidar_m],6,16);                //LCDÏÔÊ¾½Ç¶È
//			LCD_ShowNum(300,20*lidar_m,Distance[lidar_m],6,16);	       //LCDÏÔÊ¾¾àÀë
//			printf("%f,%f        ",Anglei[m],Distance[m]);  //´®¿ÚÏÔÊ¾
		}
		
		OSTimeDlyHMSM(0,0,0,3,OS_OPT_TIME_HMSM_STRICT,&err); //ÑÓÊ±200ms				
		count++;
		if(count==250)
		{
			
			LCD_Clear(WHITE);
			count=0;
		}
		
		
	}
}

extern u8 wifi_flag;
	
//lidarÈÎÎñº¯Êý
void WIFI_task(void *p_arg)
{
	u8 i,key;
	u16 rlen;
	
	u8 netpro=0;	//ÍøÂçÄ£Ê½
	u8 timex=0; 
	u8 ipbuf[16]; 	//IP»º´æ
	u8 *p;
	u16 t=999;		//¼ÓËÙµÚÒ»´Î»ñÈ¡Á´½Ó×´Ì¬
	u8 res=0;
	u8 constate=0;	//Á¬½Ó×´Ì¬
	
	OS_ERR err;
	p_arg = p_arg;
	
	p=mymalloc(SRAMIN,32);							//ÉêÇë32×Ö½ÚÄÚ´æ
	USART3_RX_STA=0;
	while(1)
	{
		
		
		if(wifi_flag)
		{
			atk_8266_wifiap_test();	//WIFI AP²âÊÔ

			while(1)
			{
				t++;
				OSTimeDlyHMSM(0,0,0,5,OS_OPT_TIME_HMSM_STRICT,&err); //ÑÓÊ±200ms
		  
				if(USART3_RX_STA&0X8000)		//½ÓÊÕµ½Ò»´ÎÊý¾ÝÁË
				{ 
					rlen=USART3_RX_STA&0X7FFF;	//µÃµ½±¾´Î½ÓÊÕµ½µÄÊý¾Ý³¤¶È
					USART3_RX_BUF[rlen]=0;		//Ìí¼Ó½áÊø·û 
					for(i=0;i<rlen;i++)
					{
						printf("USART3_RX_BUF:%d\r\n",USART3_RX_BUF[i]);	//·¢ËÍµ½´®¿Ú   
					}
					printf("\r\n");	//·¢ËÍµ½´®¿Ú 
					Show_Str(80,340,180,190,USART3_RX_BUF,12,0);//ÏÔÊ¾½ÓÊÕµ½µÄÊý¾Ý  
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
					PCF8574_WriteBit(BEEP_IO,0);//´ò¿ª·äÃùÆ÷
					OSTimeDlyHMSM(0,0,0,100,OS_OPT_TIME_HMSM_STRICT,&err); //ÑÓÊ±200ms
					PCF8574_WriteBit(BEEP_IO,1);
					PCF8574_WriteBit(RS485_RE_IO,0);
					printf("SELECT_TABLE_RESULT %d\r\n",SELECT_TABLE_RESULT);
					USART3_RX_STA=0;

				}
		
				if(t==1000&&seconfary_menu==0)//Á¬Ðø10ÃëÖÓÃ»ÓÐÊÕµ½ÈÎºÎÊý¾Ý,¼ì²éÁ¬½ÓÊÇ²»ÊÇ»¹´æÔÚ.
				{
					constate=atk_8266_consta_check();//µÃµ½Á¬½Ó×´Ì¬
					if(constate=='+')Show_Str(120,300,200,12,"Á¬½Ó³É¹¦",12,0);  //Á¬½Ó×´Ì¬
					else Show_Str(120,300,200,24,"Á¬½ÓÊ§°Ü",12,0); 	  	 
					t=0;
				}

				atk_8266_at_response(1);
				
			}
			atk_8266_msg_show(32,155,0);
			OSTaskResume((OS_TCB*)&interfaceTaskTCB,&err);
			
			wifi_flag=0;
		}
	OSTimeDlyHMSM(0,0,0,10,OS_OPT_TIME_HMSM_STRICT,&err); //ÑÓÊ±200ms
	}
}

extern u8  TIM5CH1_CAPTURE_STA;		//ÊäÈë²¶»ñ×´Ì¬		    				
extern u32	TIM5CH1_CAPTURE_VAL;	//ÊäÈë²¶»ñÖµ 


void ultrasonic_task(void *p_arg)
{
	long long temp=0;
	u8 ult_times0,ult_times1,ult_times2;
	long long ult_temp=0;
	
	OS_ERR err;
	p_arg = p_arg;
	
	while(1)
	{
		if(TIM5CH1_CAPTURE_STA&0X80)        //³É¹¦²¶»ñµ½ÁËÒ»´Î¸ßµçÆ½
		{
			temp=TIM5CH1_CAPTURE_STA&0X3F; 
			temp*=0XFFFFFFFF;		 	    //Òç³öÊ±¼ä×ÜºÍ
			temp+=TIM5CH1_CAPTURE_VAL;      //µÃµ½×ÜµÄ¸ßµçÆ½Ê±¼ä
			temp=temp/58;
			
			printf("HIGH:%lld cm\r\n",temp);//´òÓ¡×ÜµÄ¸ßµãÆ½Ê±¼ä		
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
					TIM_SetTIM3Compare4(399);	//ÐÞ¸Ä±È½ÏÖµ£¬ÐÞ¸ÄÕ¼¿Õ±È×ó
					TIM_SetTIM3Compare3(399);	//ÐÞ¸Ä±È½ÏÖµ£¬ÐÞ¸ÄÕ¼¿Õ±ÈÓÒ	
					OSTimeDlyHMSM(0,0,5,0,OS_OPT_TIME_HMSM_STRICT,&err); //ÑÓÊ±100ms
					Left_BK(0);Right_BK(0);
					Left_FR(0);Right_FR(1);
					Left_BK(1);Right_BK(1);
					ult_detection=1;
					ult_times2=0;
				}
			}
//			printf("music:%d\r\n\r\n",music_play);

			TIM5CH1_CAPTURE_STA=0;          //¿ªÆôÏÂÒ»´Î²¶»ñ
		}
		HAL_GPIO_WritePin(GPIOB,GPIO_PIN_15,GPIO_PIN_SET);
		delay_us(15);
		HAL_GPIO_WritePin(GPIOB,GPIO_PIN_15,GPIO_PIN_RESET);

		
		
		
//		printf("%d\r\n",ult_detection);
		
		OSTimeDlyHMSM(0,0,0,100,OS_OPT_TIME_HMSM_STRICT,&err); //ÑÓÊ±100ms
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
			if(Rx_485_BUF[0]==1)//´Å¹ìµÚÒ»Î»
			{
	//			len=USART_RX_STA&0x3fff;//µÃµ½´Ë´Î½ÓÊÕµ½µÄÊý¾Ý³¤¶È
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

		OSTimeDlyHMSM(0,0,0,5,OS_OPT_TIME_HMSM_STRICT,&err); //ÑÓÊ±100ms
			
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


//motor_driveÈÎÎñº¯Êý
//ÓÐ¹ì¼°³¬Éù²¨±ÜÕÏ
void motor_drive_task(void *p_arg)
{
	u16 AGV_feedback;
	u8 ult_flag=0;//¼ì²âµ½Ç°·½ÓÐÕÏ°­ÎïÊ±£¬¿ØÖÆ½øÈë±ÜÕÏÁ÷³Ì
	u8 direction_flag=0;//Óöµ½ÕÏ°­Ê±0Ñ¡ÔñÍù×ó£¬1Ñ¡ÔñÍùÓÒ
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
				TIM_SetTIM3Compare4(349);	//ÐÞ¸Ä±È½ÏÖµ£¬ÐÞ¸ÄÕ¼¿Õ±È×ó
				TIM_SetTIM3Compare3(349);	//ÐÞ¸Ä±È½ÏÖµ£¬ÐÞ¸ÄÕ¼¿Õ±ÈÓÒ	
			}
					
			if((AGV_feedback<0x0128)&&(AGV_feedback!=0x0108)&&(AGV_feedback!=0x0060)&&AGV_feedback!=0)
			{					
				if(process_control==6)
				{
					
					TIM_SetTIM3Compare4(459);	//ÐÞ¸Ä±È½ÏÖµ£¬ÐÞ¸ÄÕ¼¿Õ±È×ó
					TIM_SetTIM3Compare3(349);	//ÐÞ¸Ä±È½ÏÖµ£¬ÐÞ¸ÄÕ¼¿Õ±ÈÓÒ
					process_control=0;	
					OSTimeDlyHMSM(0,0,3,500,OS_OPT_TIME_HMSM_STRICT,&err); //ÑÓÊ±2s	
				}
				if(process_control==7)
				{
					
					TIM_SetTIM3Compare4(498);	//ÐÞ¸Ä±È½ÏÖµ£¬ÐÞ¸ÄÕ¼¿Õ±È×ó
					TIM_SetTIM3Compare3(349);	//ÐÞ¸Ä±È½ÏÖµ£¬ÐÞ¸ÄÕ¼¿Õ±ÈÓÒ
					process_control=0;	
					OSTimeDlyHMSM(0,0,3,0,OS_OPT_TIME_HMSM_STRICT,&err); //ÑÓÊ±2s	
				}
				else
				{
					printf("aaa\r\n");
					TIM_SetTIM3Compare4(249);	//ÐÞ¸Ä±È½ÏÖµ£¬ÐÞ¸ÄÕ¼¿Õ±È×ó
					TIM_SetTIM3Compare3(429);	//ÐÞ¸Ä±È½ÏÖµ£¬ÐÞ¸ÄÕ¼¿Õ±ÈÓÒ	
				}					
			}
			
			if(AGV_feedback>0x03c8)
			{
				if(process_control==6)
				{
					TIM_SetTIM3Compare4(349);	//ÐÞ¸Ä±È½ÏÖµ£¬ÐÞ¸ÄÕ¼¿Õ±È×ó
					TIM_SetTIM3Compare3(429);	//ÐÞ¸Ä±È½ÏÖµ£¬ÐÞ¸ÄÕ¼¿Õ±ÈÓÒ
					process_control=0;	
					OSTimeDlyHMSM(0,0,2,0,OS_OPT_TIME_HMSM_STRICT,&err); //ÑÓÊ±2s

				}
				if(process_control==7)
				{
					
					TIM_SetTIM3Compare4(349);	//ÐÞ¸Ä±È½ÏÖµ£¬ÐÞ¸ÄÕ¼¿Õ±È×ó
					TIM_SetTIM3Compare3(498);	//ÐÞ¸Ä±È½ÏÖµ£¬ÐÞ¸ÄÕ¼¿Õ±ÈÓÒ
					process_control=0;	
					OSTimeDlyHMSM(0,0,3,0,OS_OPT_TIME_HMSM_STRICT,&err); //ÑÓÊ±2s	
				}
				else
				{
					printf("aaa\r\n");
					TIM_SetTIM3Compare4(429);	//ÐÞ¸Ä±È½ÏÖµ£¬ÐÞ¸ÄÕ¼¿Õ±È×ó
					TIM_SetTIM3Compare3(249);	//ÐÞ¸Ä±È½ÏÖµ£¬ÐÞ¸ÄÕ¼¿Õ±ÈÓÒ
				}
			}
		}
		if(LETS_RUN_FLAG==1)
		{
			if(RFID_RESULT==1||RFID_RESULT==2||RFID_RESULT==3||RFID_RESULT==5)
			{
				if(arrive_process_control==0)
				{
					TIM_SetTIM3Compare4(349);	//ÐÞ¸Ä±È½ÏÖµ£¬ÐÞ¸ÄÕ¼¿Õ±È×ó
					TIM_SetTIM3Compare3(498);	//ÐÞ¸Ä±È½ÏÖµ£¬ÐÞ¸ÄÕ¼¿Õ±ÈÓÒ
					arrive_process_control=1;
					OSTimeDlyHMSM(0,0,4,0,OS_OPT_TIME_HMSM_STRICT,&err); //ÑÓÊ±2s
				}
				if(arrive_process_control==1)
				{
					TIM_SetTIM3Compare4(399);	//ÐÞ¸Ä±È½ÏÖµ£¬ÐÞ¸ÄÕ¼¿Õ±È×ó
					TIM_SetTIM3Compare3(399);	//ÐÞ¸Ä±È½ÏÖµ£¬ÐÞ¸ÄÕ¼¿Õ±ÈÓÒ
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
						TIM_SetTIM3Compare4(399);	//ÐÞ¸Ä±È½ÏÖµ£¬ÐÞ¸ÄÕ¼¿Õ±È×ó
						TIM_SetTIM3Compare3(498);	//ÐÞ¸Ä±È½ÏÖµ£¬ÐÞ¸ÄÕ¼¿Õ±ÈÓÒ
						OSTimeDlyHMSM(0,0,0,500,OS_OPT_TIME_HMSM_STRICT,&err); //ÑÓÊ±2s
						Left_BK(0);Right_BK(0);
						Left_FR(0);Right_FR(1);
						Left_BK(1);Right_BK(1);
						TIM_SetTIM3Compare4(399);	//ÐÞ¸Ä±È½ÏÖµ£¬ÐÞ¸ÄÕ¼¿Õ±È×ó
						TIM_SetTIM3Compare3(399);	//ÐÞ¸Ä±È½ÏÖµ£¬ÐÞ¸ÄÕ¼¿Õ±ÈÓÒ
					}
					if(AGV_feedback>0x8000)
					{
						Left_BK(0);Right_BK(0);
						Left_FR(1);Right_FR(0);
						Left_BK(1);Right_BK(1);
						TIM_SetTIM3Compare4(498);	//ÐÞ¸Ä±È½ÏÖµ£¬ÐÞ¸ÄÕ¼¿Õ±È×ó
						TIM_SetTIM3Compare3(399);	//ÐÞ¸Ä±È½ÏÖµ£¬ÐÞ¸ÄÕ¼¿Õ±ÈÓÒ
						OSTimeDlyHMSM(0,0,0,500,OS_OPT_TIME_HMSM_STRICT,&err); //ÑÓÊ±2s
						Left_BK(0);Right_BK(0);
						Left_FR(0);Right_FR(1);
						Left_BK(1);Right_BK(1);
						TIM_SetTIM3Compare4(399);	//ÐÞ¸Ä±È½ÏÖµ£¬ÐÞ¸ÄÕ¼¿Õ±È×ó
						TIM_SetTIM3Compare3(399);	//ÐÞ¸Ä±È½ÏÖµ£¬ÐÞ¸ÄÕ¼¿Õ±ÈÓÒ
					}
				}
				if(RFID_RESULT==6||RFID_RESULT==7||RFID_RESULT==8||RFID_RESULT==10)
				{
					TIM_SetTIM3Compare4(498);	//ÐÞ¸Ä±È½ÏÖµ£¬ÐÞ¸ÄÕ¼¿Õ±È×ó
					TIM_SetTIM3Compare3(498);	//ÐÞ¸Ä±È½ÏÖµ£¬ÐÞ¸ÄÕ¼¿Õ±ÈÓÒ
					LCD_Clear(WHITE);
					OSTimeDlyHMSM(0,0,0,50,OS_OPT_TIME_HMSM_STRICT,&err); //ÑÓÊ±5s
					OSTaskResume((OS_TCB*)&ov5640TaskTCB,&err);
					qr_show:  if(QR_CODE_RESULT==1) 
					{
						OSTimeDlyHMSM(0,0,2,0,OS_OPT_TIME_HMSM_STRICT,&err); //ÑÓÊ±5s
						QR_CODE_RESULT=0;
					}
					else
					{
						OSTimeDlyHMSM(0,0,0,100,OS_OPT_TIME_HMSM_STRICT,&err); //ÑÓÊ±5s
						goto qr_show;
					}
					OSTaskSuspend((OS_TCB*)&ov5640TaskTCB,&err);
					OSTaskResume((OS_TCB*)&interfaceTaskTCB,&err);
					PCF8574_WriteBit(RS485_RE_IO,0);
//					Left_BK(0);Right_BK(0);
//					Left_FR(1);Right_FR(0);
//					Left_BK(1);Right_BK(1);
//					TIM_SetTIM3Compare4(399);	//ÐÞ¸Ä±È½ÏÖµ£¬ÐÞ¸ÄÕ¼¿Õ±È×ó
//					TIM_SetTIM3Compare3(399);	//ÐÞ¸Ä±È½ÏÖµ£¬ÐÞ¸ÄÕ¼¿Õ±ÈÓÒ
//					OSTimeDlyHMSM(0,0,2,0,OS_OPT_TIME_HMSM_STRICT,&err); //ÑÓÊ±5s
//					Left_BK(0);Right_BK(0);
//					Left_FR(0);Right_FR(1);
//					Left_BK(1);Right_BK(1);
					TIM_SetTIM3Compare4(498);	//ÐÞ¸Ä±È½ÏÖµ£¬ÐÞ¸ÄÕ¼¿Õ±È×ó
					TIM_SetTIM3Compare3(399);	//ÐÞ¸Ä±È½ÏÖµ£¬ÐÞ¸ÄÕ¼¿Õ±ÈÓÒ
					OSTimeDlyHMSM(0,0,11,0,OS_OPT_TIME_HMSM_STRICT,&err); //ÑÓÊ±11s
					TIM_SetTIM3Compare4(399);	//ÐÞ¸Ä±È½ÏÖµ£¬ÐÞ¸ÄÕ¼¿Õ±È×ó
					TIM_SetTIM3Compare3(399);	//ÐÞ¸Ä±È½ÏÖµ£¬ÐÞ¸ÄÕ¼¿Õ±ÈÓÒ
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
		
		if(ult_detection==1&LETS_RUN_FLAG==0) ult_flag=1;//¼ì²âµ½Ç°·½ÓÐÕÏ°­
			
		if((ult_flag==1)&&(direction_flag==0)&&LETS_RUN_FLAG==0)
		{
			if(process_control==0)
			{
				TIM_SetTIM3Compare4(399);	//ÐÞ¸Ä±È½ÏÖµ£¬ÐÞ¸ÄÕ¼¿Õ±È×ó
				TIM_SetTIM3Compare3(498);	//ÐÞ¸Ä±È½ÏÖµ£¬ÐÞ¸ÄÕ¼¿Õ±ÈÓÒ
				process_control=1;
				OSTimeDlyHMSM(0,0,4,0,OS_OPT_TIME_HMSM_STRICT,&err); //ÑÓÊ±2s
		
				printf("a\r\n");
			}
//			printf("ult_detection%d\r\n",ult_detection);
			if((ult_detection==0)&&(direction_flag==0))
			{
				if(process_control==1)
				{
					TIM_SetTIM3Compare4(399);	//ÐÞ¸Ä±È½ÏÖµ£¬ÐÞ¸ÄÕ¼¿Õ±È×ó
					TIM_SetTIM3Compare3(399);	//ÐÞ¸Ä±È½ÏÖµ£¬ÐÞ¸ÄÕ¼¿Õ±ÈÓÒ
					process_control=2;
					OSTimeDlyHMSM(0,0,5,0,OS_OPT_TIME_HMSM_STRICT,&err); //ÑÓÊ±2s
					printf("aa\r\n");
				}
				if(process_control==2)
				{
					TIM_SetTIM3Compare4(498);	//ÐÞ¸Ä±È½ÏÖµ£¬ÐÞ¸ÄÕ¼¿Õ±È×ó
					TIM_SetTIM3Compare3(399);	//ÐÞ¸Ä±È½ÏÖµ£¬ÐÞ¸ÄÕ¼¿Õ±ÈÓ
					process_control=3;
					OSTimeDlyHMSM(0,0,4,0,OS_OPT_TIME_HMSM_STRICT,&err); //ÑÓÊ±2s
					printf("bb\r\n");
				}
				if(process_control==3)
				{
					TIM_SetTIM3Compare4(399);	//ÐÞ¸Ä±È½ÏÖµ£¬ÐÞ¸ÄÕ¼¿Õ±È×ó
					TIM_SetTIM3Compare3(399);	//ÐÞ¸Ä±È½ÏÖµ£¬ÐÞ¸ÄÕ¼¿Õ±ÈÓÒ
					process_control=4;
					OSTimeDlyHMSM(0,0,6,0,OS_OPT_TIME_HMSM_STRICT,&err); //ÑÓÊ±2s
				}
				if(process_control==4)
				{
					TIM_SetTIM3Compare4(498);	//ÐÞ¸Ä±È½ÏÖµ£¬ÐÞ¸ÄÕ¼¿Õ±È×ó
					TIM_SetTIM3Compare3(399);	//ÐÞ¸Ä±È½ÏÖµ£¬ÐÞ¸ÄÕ¼¿Õ±ÈÓÒ
					process_control=5;
					AGV_feedback=0;
					OSTimeDlyHMSM(0,0,4,0,OS_OPT_TIME_HMSM_STRICT,&err); //ÑÓÊ±2s
				}
				if(process_control==5)
				{
					TIM_SetTIM3Compare4(399);	//ÐÞ¸Ä±È½ÏÖµ£¬ÐÞ¸ÄÕ¼¿Õ±È×ó
					TIM_SetTIM3Compare3(399);	//ÐÞ¸Ä±È½ÏÖµ£¬ÐÞ¸ÄÕ¼¿Õ±ÈÓÒ
					process_control=6;

					ult_flag=0;
//					process_control=0;
//					OSTimeDlyHMSM(0,0,2,0,OS_OPT_TIME_HMSM_STRICT,&err); //ÑÓÊ±2s
					printf("cc\r\n");

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
					TIM_SetTIM3Compare4(399);	//ÐÞ¸Ä±È½ÏÖµ£¬ÐÞ¸ÄÕ¼¿Õ±È×ó
					TIM_SetTIM3Compare3(498);	//ÐÞ¸Ä±È½ÏÖµ£¬ÐÞ¸ÄÕ¼¿Õ±ÈÓÒ
					OSTimeDlyHMSM(0,0,4,0,OS_OPT_TIME_HMSM_STRICT,&err); //ÑÓÊ±2s
					Left_BK(0);Right_BK(0);
					Left_FR(0);Right_FR(1);
					Left_BK(1);Right_BK(1);
					process_control=0;
					printf("c\r\n");
				}
				if(process_control==2)
				{
					Left_BK(0);Right_BK(0);
					Left_FR(1);Right_FR(0);
					Left_BK(1);Right_BK(1);
					TIM_SetTIM3Compare4(498);	//ÐÞ¸Ä±È½ÏÖµ£¬ÐÞ¸ÄÕ¼¿Õ±È×ó
					TIM_SetTIM3Compare3(399);	//ÐÞ¸Ä±È½ÏÖµ£¬ÐÞ¸ÄÕ¼¿Õ±ÈÓÒ
					OSTimeDlyHMSM(0,0,4,0,OS_OPT_TIME_HMSM_STRICT,&err); //ÑÓÊ±2s
					TIM_SetTIM3Compare4(399);	//ÐÞ¸Ä±È½ÏÖµ£¬ÐÞ¸ÄÕ¼¿Õ±È×ó
					TIM_SetTIM3Compare3(399);	//ÐÞ¸Ä±È½ÏÖµ£¬ÐÞ¸ÄÕ¼¿Õ±ÈÓÒ
					OSTimeDlyHMSM(0,0,4,0,OS_OPT_TIME_HMSM_STRICT,&err); //ÑÓÊ±2s
					TIM_SetTIM3Compare4(399);	//ÐÞ¸Ä±È½ÏÖµ£¬ÐÞ¸ÄÕ¼¿Õ±È×ó
					TIM_SetTIM3Compare3(498);	//ÐÞ¸Ä±È½ÏÖµ£¬ÐÞ¸ÄÕ¼¿Õ±ÈÓÒ
					OSTimeDlyHMSM(0,0,4,0,OS_OPT_TIME_HMSM_STRICT,&err); //ÑÓÊ±2s
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
				TIM_SetTIM3Compare4(498);	//ÐÞ¸Ä±È½ÏÖµ£¬ÐÞ¸ÄÕ¼¿Õ±È×ó
				TIM_SetTIM3Compare3(399);	//ÐÞ¸Ä±È½ÏÖµ£¬ÐÞ¸ÄÕ¼¿Õ±ÈÓÒ
				OSTimeDlyHMSM(0,0,4,0,OS_OPT_TIME_HMSM_STRICT,&err); //ÑÓÊ±2s
				process_control=1;
				printf("q\r\n");
			}
//			OSTimeDlyHMSM(0,0,1,0,OS_OPT_TIME_HMSM_STRICT,&err); //ÑÓÊ±2s
			if(ult_detection==0)
			{
				if(process_control==1)
				{
					TIM_SetTIM3Compare4(399);	//ÐÞ¸Ä±È½ÏÖµ£¬ÐÞ¸ÄÕ¼¿Õ±È×ó
					TIM_SetTIM3Compare3(399);	//ÐÞ¸Ä±È½ÏÖµ£¬ÐÞ¸ÄÕ¼¿Õ±ÈÓÒ
					process_control=2;
					OSTimeDlyHMSM(0,0,6,0,OS_OPT_TIME_HMSM_STRICT,&err); //ÑÓÊ±2s
					printf("ww\r\n");
				}
				if(process_control==2)
				{
					TIM_SetTIM3Compare4(399);	//ÐÞ¸Ä±È½ÏÖµ£¬ÐÞ¸ÄÕ¼¿Õ±È×ó
					TIM_SetTIM3Compare3(498);	//ÐÞ¸Ä±È½ÏÖµ£¬ÐÞ¸ÄÕ¼¿Õ±ÈÓÒ
					process_control=3;
					OSTimeDlyHMSM(0,0,3,500,OS_OPT_TIME_HMSM_STRICT,&err); //ÑÓÊ±2s
					printf("ee\r\n");
				}
				if(process_control==3)
				{
					TIM_SetTIM3Compare4(399);	//ÐÞ¸Ä±È½ÏÖµ£¬ÐÞ¸ÄÕ¼¿Õ±È×ó
					TIM_SetTIM3Compare3(399);	//ÐÞ¸Ä±È½ÏÖµ£¬ÐÞ¸ÄÕ¼¿Õ±ÈÓÒ
					process_control=4;
					OSTimeDlyHMSM(0,0,6,0,OS_OPT_TIME_HMSM_STRICT,&err); //ÑÓÊ±2s
				}
				if(process_control==4)
				{
					TIM_SetTIM3Compare4(399);	//ÐÞ¸Ä±È½ÏÖµ£¬ÐÞ¸ÄÕ¼¿Õ±È×ó
					TIM_SetTIM3Compare3(498);	//ÐÞ¸Ä±È½ÏÖµ£¬ÐÞ¸ÄÕ¼¿Õ±ÈÓÒ
					process_control=5;
					OSTimeDlyHMSM(0,0,4,0,OS_OPT_TIME_HMSM_STRICT,&err); //ÑÓÊ±2s
				}
				if(process_control==5)
				{
					
					TIM_SetTIM3Compare4(399);	//ÐÞ¸Ä±È½ÏÖµ£¬ÐÞ¸ÄÕ¼¿Õ±È×ó
					TIM_SetTIM3Compare3(399);	//ÐÞ¸Ä±È½ÏÖµ£¬ÐÞ¸ÄÕ¼¿Õ±ÈÓÒ
//					if(AGV_feedback!=0)
//					{
//						printf("AGV_feedbackAGV_feedback:\r\n\r\n%x\r\n\r\n",AGV_feedback);
//						TIM_SetTIM3Compare4(399);	//ÐÞ¸Ä±È½ÏÖµ£¬ÐÞ¸ÄÕ¼¿Õ±È×ó
//						TIM_SetTIM3Compare3(498);	//ÐÞ¸Ä±È½ÏÖµ£¬ÐÞ¸ÄÕ¼¿Õ±ÈÓÒ
						ult_flag=0;
						direction_flag=0;
						process_control=6;
						printf("ff\r\n");
//						OSTimeDlyHMSM(0,0,2,0,OS_OPT_TIME_HMSM_STRICT,&err); //ÑÓÊ±2s
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
					TIM_SetTIM3Compare4(498);	//ÐÞ¸Ä±È½ÏÖµ£¬ÐÞ¸ÄÕ¼¿Õ±È×ó
					TIM_SetTIM3Compare3(399);	//ÐÞ¸Ä±È½ÏÖµ£¬ÐÞ¸ÄÕ¼¿Õ±ÈÓÒ
					OSTimeDlyHMSM(0,0,4,0,OS_OPT_TIME_HMSM_STRICT,&err); //ÑÓÊ±2s
					Left_BK(0);Right_BK(0);
					Left_FR(0);Right_FR(1);
					Left_BK(1);Right_BK(1);
					process_control=0;
					printf("e\r\n");
				}
				if(process_control==3)
				{
					Left_BK(0);Right_BK(0);
					Left_FR(1);Right_FR(0);
					Left_BK(1);Right_BK(1);
					TIM_SetTIM3Compare4(399);	//ÐÞ¸Ä±È½ÏÖµ£¬ÐÞ¸ÄÕ¼¿Õ±È×ó
					TIM_SetTIM3Compare3(498);	//ÐÞ¸Ä±È½ÏÖµ£¬ÐÞ¸ÄÕ¼¿Õ±ÈÓÒ
					OSTimeDlyHMSM(0,0,4,0,OS_OPT_TIME_HMSM_STRICT,&err); //ÑÓÊ±2s
					TIM_SetTIM3Compare4(399);	//ÐÞ¸Ä±È½ÏÖµ£¬ÐÞ¸ÄÕ¼¿Õ±È×ó
					TIM_SetTIM3Compare3(399);	//ÐÞ¸Ä±È½ÏÖµ£¬ÐÞ¸ÄÕ¼¿Õ±ÈÓÒ
					OSTimeDlyHMSM(0,0,4,0,OS_OPT_TIME_HMSM_STRICT,&err); //ÑÓÊ±2s
					TIM_SetTIM3Compare4(498);	//ÐÞ¸Ä±È½ÏÖµ£¬ÐÞ¸ÄÕ¼¿Õ±È×ó
					TIM_SetTIM3Compare3(399);	//ÐÞ¸Ä±È½ÏÖµ£¬ÐÞ¸ÄÕ¼¿Õ±ÈÓÒ
					OSTimeDlyHMSM(0,0,4,0,OS_OPT_TIME_HMSM_STRICT,&err); //ÑÓÊ±2s
					Left_BK(0);Right_BK(0);
					Left_FR(0);Right_FR(1);
					Left_BK(1);Right_BK(1);
					process_control=0;
				}
			}
		}			
//		printf("%d %d %d %d\r\n",ult_detection,direction_flag,ult_flag,LETS_RUN_FLAG);
//		printf("process_control %d\r\n",process_control);
		OSTimeDlyHMSM(0,0,0,10,OS_OPT_TIME_HMSM_STRICT,&err); //ÑÓÊ±10ms
	}
}

//lidarÈÎÎñº¯Êý
void MUSIC_task(void *p_arg)
{
	OS_ERR err;
	p_arg = p_arg;
    
	while(1)
	{
		while(music_play)
		{
			static u16 i=0;
			i++;
			if(i==1)
			{
				W25QXX_Init();				   	//³õÊ¼»¯W25Q256
				W25QXX_Init();				    //³õÊ¼»¯W25Q256
				WM8978_Init();				    //³õÊ¼»¯WM8978
				WM8978_HPvol_Set(40,40);	    //¶ú»úÒôÁ¿ÉèÖÃ
				WM8978_SPKvol_Set(30);		    //À®°ÈÒôÁ¿ÉèÖÃ
				PCF8574_Init();					//³õÊ¼»¯PCF8574
				
			}
			audio_play();
			
			if(i>5) i=2;
			OSTimeDlyHMSM(0,0,0,50,OS_OPT_TIME_HMSM_STRICT,&err); //ÑÓÊ±200ms
		}
		OSTimeDlyHMSM(0,0,0,50,OS_OPT_TIME_HMSM_STRICT,&err); //ÑÓÊ±200ms
	}
}

//ÉãÏñÍ·Êý¾ÝDMA½ÓÊÕÍê³ÉÖÐ¶Ï»Øµ÷º¯Êý
void qr_dcmi_rx_callback(void)
{  

	CPU_SR_ALLOC();
	u32 *pbuf;
	u16 i;
	pbuf=(u32*)(rgb_data_buf+dcmi_curline*qr_image_width);//½«rgb_data_bufµØÖ·Æ«ÒÆ¸³Öµ¸øpbuf
	
	if(DMADMCI_Handler.Instance->CR&(1<<19))//DMAÊ¹ÓÃbuf1,¶ÁÈ¡buf0
	{ 
		for(i=0;i<qr_image_width/2;i++)
		{
			pbuf[i]=dcmi_line_buf[0][i];
		} 
	}else 										//DMAÊ¹ÓÃbuf0,¶ÁÈ¡buf1
	{
		for(i=0;i<qr_image_width/2;i++)
		{
			pbuf[i]=dcmi_line_buf[1][i];
		} 
	} 
	dcmi_curline++;
}

//ÏÔÊ¾Í¼Ïñ
void qr_show_image(u16 xoff,u16 yoff,u16 width,u16 height,u16 *imagebuf)
{

	CPU_SR_ALLOC();
	u16 linecnt=yoff;
	
	if(lcdltdc.pwidth!=0)//RGBÆÁ
	{
		for(linecnt=0;linecnt<height;linecnt++)
		{
			LTDC_Color_Fill(xoff,linecnt+yoff,xoff+width-1,linecnt+yoff,imagebuf+linecnt*width);//RGBÆÁ,DM2DÌî³ä 
		}
		
	}else LCD_Color_Fill(xoff,yoff,xoff+width-1,yoff+height-1,imagebuf);	//MCUÆÁ,Ö±½ÓÏÔÊ¾
}

//imagewidth:<=240;´óÓÚ240Ê±,ÊÇ240µÄÕûÊý±¶
//imagebuf:RGBÍ¼ÏñÊý¾Ý»º³åÇø
void qr_decode(u16 imagewidth,u16 *imagebuf)
{
	OS_ERR err;
	CPU_SR_ALLOC();
	static u8 bartype=0; 
	u8 *bmp;
	u8 *result=NULL;
	u16 Color;
	u16 i,j;	
	u16 qr_img_width=0;						//ÊäÈëÊ¶±ðÆ÷µÄÍ¼Ïñ¿í¶È,×î´ó²»³¬¹ý240!
	u8 qr_img_scale=0;						//Ñ¹Ëõ±ÈÀýÒò×Ó
	
	if(imagewidth>240)
	{
		if(imagewidth%240)return ;	//²»ÊÇ240µÄ±¶Êý,Ö±½ÓÍË³ö
		qr_img_width=240;
		qr_img_scale=imagewidth/qr_img_width;
	}else
	{
		qr_img_width=imagewidth;
		qr_img_scale=1;
	}  
	result=mymalloc(SRAMIN,1536);//ÉêÇëÊ¶±ð½á¹û´æ·ÅÄÚ´æ
	bmp=mymalloc(SRAMDTCM,qr_img_width*qr_img_width);//DTCM¹ÜÀíÄÚ´æÎª120K£¬ÕâÀïÉêÇë240*240=56K 
	mymemset(bmp,0,qr_img_width*qr_img_width);
	if(lcdltdc.pwidth==0)//MCUÆÁ,ÎÞÐè¾µÏñ
	{ 
		for(i=0;i<qr_img_width;i++)		
		{
			for(j=0;j<qr_img_width;j++)		//½«RGB565Í¼Æ¬×ª³É»Ò¶È
			{	
				Color=*(imagebuf+((i*imagewidth)+j)*qr_img_scale); //°´ÕÕqr_img_scaleÑ¹Ëõ³É240*240
				*(bmp+i*qr_img_width+j)=(((Color&0xF800)>> 8)*76+((Color&0x7E0)>>3)*150+((Color&0x001F)<<3)*30)>>8;
			}		
		}
	}else	//RGBÆÁ,ÐèÒª¾µÏñ
	{
		for(i=0;i<qr_img_width;i++)		
		{
			for(j=0;j<qr_img_width;j++)		//½«RGB565Í¼Æ¬×ª³É»Ò¶È
			{	
				Color=*(imagebuf+((i*imagewidth)+qr_img_width-j-1)*qr_img_scale);//°´ÕÕqr_img_scaleÑ¹Ëõ³É240*240
				*(bmp+i*qr_img_width+j)=(((Color&0xF800)>> 8)*76+((Color&0x7E0)>>3)*150+((Color&0x001F)<<3)*30)>>8;
			}		
		}		
	}
	atk_qr_decode(qr_img_width,qr_img_width,bmp,bartype,result);//Ê¶±ð»Ò¶ÈÍ¼Æ¬£¨×¢Òâ£ºµ¥´ÎºÄÊ±Ô¼0.2S£©
	
	if(result[0]==0)//Ã»ÓÐÊ¶±ð³öÀ´
	{
		QR_CODE_RESULT=0;
		bartype++;
		if(bartype>=5)bartype=0; 
	}
	else if(result[0]!=0)//Ê¶±ð³öÀ´ÁË£¬ÏÔÊ¾½á¹û
	{	
		QR_CODE_RESULT=1;
		PCF8574_WriteBit(BEEP_IO,0);//´ò¿ª·äÃùÆ÷
		OSTimeDlyHMSM(0,0,0,100,OS_OPT_TIME_HMSM_STRICT,&err); //ÑÓÊ±200ms
		PCF8574_WriteBit(BEEP_IO,1);
//		POINT_COLOR=BLUE; 
		LCD_Fill(0,(lcddev.height+qr_image_width)/2+20,lcddev.width,lcddev.height,BLACK);
		Show_Str(0,(lcddev.height+qr_image_width)/2+20,lcddev.width,
								(lcddev.height-qr_image_width)/2-20,(u8*)result,16,0							
						);//LCDÏÔÊ¾Ê¶±ð½á¹û
		printf("\r\nresult:\r\n%s\r\n",result);//´®¿Ú´òÓ¡Ê¶±ð½á¹û 		
	}
	myfree(SRAMDTCM,bmp);		//ÊÍ·Å»Ò¶ÈÍ¼bmpÄÚ´æ
	myfree(SRAMIN,result);	//ÊÍ·ÅÊ¶±ð½á¹û	
}  


