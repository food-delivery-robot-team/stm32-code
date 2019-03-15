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
#define interface_STK_SIZE 		1024
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
#define motor_drive_TASK_PRIO		5
//ÈÎÎñ¶ÑÕ»´óÐ¡	
#define motor_drive_STK_SIZE 		256
//ÈÎÎñ¿ØÖÆ¿é
OS_TCB motor_driveTaskTCB;
//ÈÎÎñ¶ÑÕ»	
CPU_STK motor_drive_TASK_STK[motor_drive_STK_SIZE];
void motor_drive_task(void *p_arg);


//ÈÎÎñÓÅÏÈ¼¶
#define ultrasonic_TASK_PRIO		5
//ÈÎÎñ¶ÑÕ»´óÐ¡	
#define ultrasonic_STK_SIZE 		256
//ÈÎÎñ¿ØÖÆ¿é
OS_TCB ultrasonicTaskTCB;
//ÈÎÎñ¶ÑÕ»	
CPU_STK ultrasonic_TASK_STK[ultrasonic_STK_SIZE];
void ultrasonic_task(void *p_arg);

//ÈÎÎñÓÅÏÈ¼¶
#define AGV_guide_TASK_PRIO		4
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
#define lidar_STK_SIZE       256
//ÈÎÎñ¿ØÖÆ¿é
OS_TCB lidarTaskTCB;
//ÈÎÎñ¶ÑÕ»	
CPU_STK lidar_TASK_STK[lidar_STK_SIZE];
//ÈÎÎñº¯Êý
void lidar_task(void *p_arg);

//ÈÎÎñÓÅÏÈ¼¶
#define RFID_TASK_PRIO		4
//ÈÎÎñ¶ÑÕ»´óÐ¡	
#define RFID_STK_SIZE       128
//ÈÎÎñ¿ØÖÆ¿é
OS_TCB RFIDTaskTCB;
//ÈÎÎñ¶ÑÕ»	
CPU_STK RFID_TASK_STK[RFID_STK_SIZE];
//ÈÎÎñº¯Êý
void RFID_task(void *p_arg);

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
	uart_init(115200);		        //´®¿Ú³õÊ¼»¯
	usmart_dev.init(108); 		    //³õÊ¼»¯USMART	
    LED_Init();                     //³õÊ¼»¯LED
	KEY_Init();                     //³õÊ¼»¯°´¼ü
	SDRAM_Init();                   //³õÊ¼»¯SDRAM
	LCD_Init();                     //³õÊ¼»¯LCD
	RS485_Init(115200);		        //³õÊ¼»¯RS485
	ultrasonic_init();              //³õÊ¼»¯³¬Éù²¨´«¸ÐÆ÷
	motor_drive_Init();
	W25QXX_Init();				   	//³õÊ¼»¯W25Q256
	W25QXX_Init();				    //³õÊ¼»¯W25Q256
    WM8978_Init();				    //³õÊ¼»¯WM8978
	WM8978_HPvol_Set(40,40);	    //¶ú»úÒôÁ¿ÉèÖÃ
	WM8978_SPKvol_Set(50);		    //À®°ÈÒôÁ¿ÉèÖÃ
	PCF8574_Init();									//³õÊ¼»¯PCF8574
	OV5640_Init();									//³õÊ¼»¯OV5640
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
	
	OS_ERR err;
	p_arg = p_arg;
	
	OSTaskSuspend((OS_TCB*)&ov5640TaskTCB,&err);
	OSTaskSuspend((OS_TCB*)&RFIDTaskTCB,&err);
	
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
	//					printf("resume\r\n");

				}
				else if(tp_dev.x[0]>20&&tp_dev.y[0]>100&&tp_dev.x[0]<160&&tp_dev.y[0]<180&&seconfary_menu==0)
				{

						OSTaskResume((OS_TCB*)&interfaceTaskTCB,&err);
						OSTaskResume((OS_TCB*)&RFIDTaskTCB,&err);	

				}
				else if(tp_dev.x[0]>20&&tp_dev.y[0]>200&&tp_dev.x[0]<160&&tp_dev.y[0]<280&&seconfary_menu==0)
				{

						OSTaskResume((OS_TCB*)&interfaceTaskTCB,&err);
						OSTaskResume((OS_TCB*)&lidarTaskTCB,&err);	

				}
				else if(tp_dev.x[0]>180&&tp_dev.y[0]>20&&tp_dev.x[0]<280&&tp_dev.y[0]<80)
				{
					
					OSTaskSuspend((OS_TCB*)&ov5640TaskTCB,&err);
					OSTaskSuspend((OS_TCB*)&RFIDTaskTCB,&err);
					OSTaskSuspend((OS_TCB*)&lidarTaskTCB,&err);
//					printf("suspend\r\n");
//					LCD_Clear(WHITE);
					OSTaskResume((OS_TCB*)&interfaceTaskTCB,&err);
					seconfary_menu=0;

				}
				else OSTimeDlyHMSM(0,0,0,10,OS_OPT_TIME_HMSM_STRICT,&err);	//Ã»ÓÐ°´¼ü°´ÏÂµÄÊ±ºò 			   
			}
			
		}else OSTimeDlyHMSM(0,0,0,10,OS_OPT_TIME_HMSM_STRICT,&err);	//Ã»ÓÐ°´¼ü°´ÏÂµÄÊ±ºò 
	}
}

//interfaceÈÎÎñº¯Êý
void interface_task(void *p_arg)
{
	
	OS_ERR err;
	p_arg = p_arg;
	while(1)
	{	
		if(tp_dev.x[0]>20&&tp_dev.y[0]>20&&tp_dev.x[0]<160&&tp_dev.y[0]<80||
		   tp_dev.x[0]>20&&tp_dev.y[0]>100&&tp_dev.x[0]<160&&tp_dev.y[0]<180||
		   tp_dev.x[0]>20&&tp_dev.y[0]<280&&tp_dev.x[0]<160&&tp_dev.y[0]>200
		)
		{
			LCD_Clear(WHITE);//ÇåÆÁ 
			LCD_DrawRectangle(180, 20, 280, 80);
			LCD_ShowString(195,30,200,16,32,(u8*)"EXIT");		
			
		}
		else
		{
			LCD_Clear(WHITE);//ÇåÆÁ 
			LCD_DrawRectangle(20, 20, 160, 80);
			LCD_ShowString(30,35,200,16,32,(u8*)"QR CODE");
			LCD_DrawRectangle(20, 100, 160, 180);
			LCD_ShowString(30,125,200,16,32,(u8*)"RFID");
			LCD_DrawRectangle(20, 200, 160, 280);
			LCD_ShowString(30,220,200,16,32,(u8*)"LIDAR");
		}
		OSTaskSuspend((OS_TCB*)&interfaceTaskTCB,&err);
	}
	
	
}

//ov56400ÈÎÎñº¯Êý
void ov5640_task(void *p_arg)
{
	
	OS_ERR err;
	p_arg = p_arg;
	

//		while(1)
//		{
//			key=KEY_Scan(0);//²»Ö§³ÖÁ¬°´
//			if(key)
//			{ 
//				OV5640_Focus_Single();  //°´KEY0¡¢KEY1¡¢KEYUPÊÖ¶¯µ¥´Î×Ô¶¯¶Ô½¹
//				
//				if(key==KEY2_PRES)break;//°´KEY2½áÊøÊ¶±ð
//			} 
//			if(readok==1)			//²É¼¯µ½ÁËÒ»Ö¡Í¼Ïñ
//			{		
//				readok=0;
//				qr_show_image((lcddev.width-qr_image_width)/2,(lcddev.height-qr_image_width)/2,qr_image_width,qr_image_width,rgb_data_buf);
//				qr_decode(qr_image_width,rgb_data_buf);
//			}
//			i++;
//			if(i==20)//DS0ÉÁË¸.
//			{
//				i=0;
//				LED0_Toggle;
//			}
////			OSTimeDlyHMSM(0,0,0,20,OS_OPT_TIME_HMSM_STRICT,&err); //ÑÓÊ±200ms
//		}
//		atk_qr_destroy();//ÊÍ·ÅËã·¨ÄÚ´æ
//		printf("3SRAM IN:%d\r\n",my_mem_perused(SRAMIN));
//		printf("3SRAM EX:%d\r\n",my_mem_perused(SRAMEX));
//		printf("3SRAM DCTM:%d\r\n",my_mem_perused(SRAMDTCM)); 
		while(1)
		{
			if(tp_dev.sta&TP_PRES_DOWN)			//´¥ÃþÆÁ±»°´ÏÂ
			{
				seconfary_menu=1;
			}
			OSTimeDlyHMSM(0,0,0,200,OS_OPT_TIME_HMSM_STRICT,&err); //ÑÓÊ±200ms
			if(readok==1)			//²É¼¯µ½ÁËÒ»Ö¡Í¼Ïñ
			{		
				readok=0;
				qr_show_image((lcddev.width-qr_image_width)/2,(lcddev.height-qr_image_width)/2,qr_image_width,qr_image_width,rgb_data_buf);
				qr_decode(qr_image_width,rgb_data_buf);
			}
//			printf("1\r\n");
		}

}



//lidarÈÎÎñº¯Êý
void RFID_task(void *p_arg)
{
	u8 len,i;
	long num;
	u8 receive[50];
	OS_ERR err;
	p_arg = p_arg;
	
	while(1)
	{
		if(tp_dev.sta&TP_PRES_DOWN)			//´¥ÃþÆÁ±»°´ÏÂ
		{
			seconfary_menu=1;
		}
		LCD_ShowString(20,160,200,16,32,(u8*)"result:");
		if(USART_RX_STA&0x8000)
		{
			len=USART_RX_STA&0x3fff;//µÃµ½´Ë´Î½ÓÊÕµ½µÄÊý¾Ý³¤¶È
			for(i=0;i<len;i++)
			{
//				num=USART_RX_BUF[i]*pow(10,(len-i));
				receive[i]=USART_RX_BUF[i];
				LCD_ShowChar(20+i*20,200,receive[i],24,0);
			}

			USART_RX_STA=0;
		}else OSTimeDlyHMSM(0,0,0,200,OS_OPT_TIME_HMSM_STRICT,&err); //ÑÓÊ±200ms
		OSTimeDlyHMSM(0,0,0,200,OS_OPT_TIME_HMSM_STRICT,&err); //ÑÓÊ±200ms
	}
}


//lidarÈÎÎñº¯Êý
void lidar_task(void *p_arg)
{
	OS_ERR err;
	p_arg = p_arg;
	
	while(1)
	{
		
		if(tp_dev.sta&TP_PRES_DOWN)			//´¥ÃþÆÁ±»°´ÏÂ
		{
			seconfary_menu=1;
		}
		
		LCD_ShowString(20,160,200,16,32,(u8*)"lidar:");
		
		OSTimeDlyHMSM(0,0,0,200,OS_OPT_TIME_HMSM_STRICT,&err); //ÑÓÊ±200ms
		
	}
}


extern u8  TIM5CH1_CAPTURE_STA;		//ÊäÈë²¶»ñ×´Ì¬		    				
extern u32	TIM5CH1_CAPTURE_VAL;	//ÊäÈë²¶»ñÖµ 
u8 ult_detection;

void ultrasonic_task(void *p_arg)
{
	long long temp=0;
	
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
//			printf("HIGH:%lld cm\r\n",temp/58);//´òÓ¡×ÜµÄ¸ßµãÆ½Ê±¼ä
			TIM5CH1_CAPTURE_STA=0;          //¿ªÆôÏÂÒ»´Î²¶»ñ
		}
		HAL_GPIO_WritePin(GPIOB,GPIO_PIN_15,GPIO_PIN_SET);
		delay_us(15);
		HAL_GPIO_WritePin(GPIOB,GPIO_PIN_15,GPIO_PIN_RESET);

		if(temp>100) ult_detection=0;
		if(temp<100) ult_detection=1;
		if(temp<50) ult_detection=2;
		if(temp<20) 
		{
			ult_detection=3;
			Left_BK(0);Right_BK(0);
			Left_FR(1);Right_FR(0);
			Left_BK(1);Right_BK(1);
			OSTimeDlyHMSM(0,0,2,0,OS_OPT_TIME_HMSM_STRICT,&err); //ÑÓÊ±100ms
			Left_BK(0);Right_BK(0);
			Left_FR(0);Right_FR(1);
			Left_BK(1);Right_BK(1);
		}
		printf("%d\r\n",ult_detection);
		
		OSTimeDlyHMSM(0,0,0,500,OS_OPT_TIME_HMSM_STRICT,&err); //ÑÓÊ±100ms
	}
}

u8 USART_RX_FLAG;
u16 USART_RX_STORAGE;
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
		if(USART_RX_STA&0x8000)
		{					   
			len=USART_RX_STA&0x3fff;//µÃµ½´Ë´Î½ÓÊÕµ½µÄÊý¾Ý³¤¶È

			crc_check=CRC16((uint8_t*)USART_RX_BUF,len-2);
			if((USART_RX_BUF[len-1]==(crc_check>>8))&&(USART_RX_BUF[len-2]==(crc_check&0X00FF)))
			{
				for(i=0;i<len;i++)
				{
					AGV_INF[i]=USART_RX_BUF[i];
				}
			}

			
			USART_RX_STA=0;
		}
		OSTimeDlyHMSM(0,0,0,10,OS_OPT_TIME_HMSM_STRICT,&err); //ÑÓÊ±100ms
			
		if(USART_RX_FLAG==1)
		{
			USART_RX_STORAGE++;
			usart_rx_ss=USART_RX_STORAGE-USART_RX_STA;

			if(usart_rx_ss>2)
			{
				USART_RX_STA|=0x8000;
				USART_RX_STORAGE=0;
				USART_RX_FLAG=0;
			}
			if(USART_RX_STA==8)
			{
				USART_RX_STA|=0x8000;
				USART_RX_STORAGE=0;
				USART_RX_FLAG=0;
			}
		}
		
	}
}


//motor_driveÈÎÎñº¯Êý
void motor_drive_task(void *p_arg)
{
	u16 AGV_feedback;
	u8 ult_flag=0;//¼ì²âµ½Ç°·½ÓÐÕÏ°­ÎïÊ±£¬¿ØÖÆ½øÈë±ÜÕÏÁ÷³Ì
	u8 direction_flag=0;//Óöµ½ÕÏ°­Ê±0Ñ¡ÔñÍù×ó£¬1Ñ¡ÔñÍùÓÒ
	
	OS_ERR err;
	p_arg = p_arg;
	
	while(1)
	{
		AGV_feedback=AGV_INF[4];
		AGV_feedback=(AGV_feedback<<8)+AGV_INF[5];
//		printf("%x",AGV_feedback);
		if(ult_detection==0&&ult_flag==0)
		{
			
			if(((0x0128<AGV_feedback&&AGV_feedback<0x03c8)||(AGV_feedback==0x0108)||(AGV_feedback==0x0060))&&AGV_feedback!=0)
			{
				TIM_SetTIM3Compare4(399);	//ÐÞ¸Ä±È½ÏÖµ£¬ÐÞ¸ÄÕ¼¿Õ±È×ó
				TIM_SetTIM3Compare3(399);	//ÐÞ¸Ä±È½ÏÖµ£¬ÐÞ¸ÄÕ¼¿Õ±ÈÓÒ	
			}
					
			if((AGV_feedback<0x0128)&&(AGV_feedback!=0x0108)&&(AGV_feedback!=0x0060)&&AGV_feedback!=0)
			{					
				TIM_SetTIM3Compare4(299);	//ÐÞ¸Ä±È½ÏÖµ£¬ÐÞ¸ÄÕ¼¿Õ±È×ó
				TIM_SetTIM3Compare3(399);	//ÐÞ¸Ä±È½ÏÖµ£¬ÐÞ¸ÄÕ¼¿Õ±ÈÓÒ		
			}
			
			if(AGV_feedback>0x03c8)
			{
				TIM_SetTIM3Compare4(399);	//ÐÞ¸Ä±È½ÏÖµ£¬ÐÞ¸ÄÕ¼¿Õ±È×ó
				TIM_SetTIM3Compare3(299);	//ÐÞ¸Ä±È½ÏÖµ£¬ÐÞ¸ÄÕ¼¿Õ±ÈÓÒ
			}
			
		}
		if(ult_detection==1) ult_flag=1;//¼ì²âµ½Ç°·½Ò»Ã×ÓÐÕÏ°­
			
		if(ult_flag==1&&direction_flag==0)
		{
			TIM_SetTIM3Compare4(449);	//ÐÞ¸Ä±È½ÏÖµ£¬ÐÞ¸ÄÕ¼¿Õ±È×ó
			TIM_SetTIM3Compare3(498);	//ÐÞ¸Ä±È½ÏÖµ£¬ÐÞ¸ÄÕ¼¿Õ±ÈÓÒ
			OSTimeDlyHMSM(0,0,2,0,OS_OPT_TIME_HMSM_STRICT,&err); //ÑÓÊ±2s
			if(ult_detection==0)
			{
				TIM_SetTIM3Compare4(449);	//ÐÞ¸Ä±È½ÏÖµ£¬ÐÞ¸ÄÕ¼¿Õ±È×ó
				TIM_SetTIM3Compare3(449);	//ÐÞ¸Ä±È½ÏÖµ£¬ÐÞ¸ÄÕ¼¿Õ±ÈÓÒ
				OSTimeDlyHMSM(0,0,2,0,OS_OPT_TIME_HMSM_STRICT,&err); //ÑÓÊ±2s
				TIM_SetTIM3Compare4(498);	//ÐÞ¸Ä±È½ÏÖµ£¬ÐÞ¸ÄÕ¼¿Õ±È×ó
				TIM_SetTIM3Compare3(449);	//ÐÞ¸Ä±È½ÏÖµ£¬ÐÞ¸ÄÕ¼¿Õ±ÈÓ
				OSTimeDlyHMSM(0,0,2,0,OS_OPT_TIME_HMSM_STRICT,&err); //ÑÓÊ±2s
				
			}
			if(ult_detection>1)
			{
				direction_flag=1;
				Left_BK(0);Right_BK(0);
				Left_FR(1);Right_FR(0);
				Left_BK(1);Right_BK(1);
				TIM_SetTIM3Compare4(449);	//ÐÞ¸Ä±È½ÏÖµ£¬ÐÞ¸ÄÕ¼¿Õ±È×ó
				TIM_SetTIM3Compare3(498);	//ÐÞ¸Ä±È½ÏÖµ£¬ÐÞ¸ÄÕ¼¿Õ±ÈÓÒ
				OSTimeDlyHMSM(0,0,2,0,OS_OPT_TIME_HMSM_STRICT,&err); //ÑÓÊ±2s
				Left_BK(0);Right_BK(0);
				Left_FR(0);Right_FR(1);
				Left_BK(1);Right_BK(1);
			}
				
		}
		if(ult_flag==1&&direction_flag==1)
		{
			TIM_SetTIM3Compare4(498);	//ÐÞ¸Ä±È½ÏÖµ£¬ÐÞ¸ÄÕ¼¿Õ±È×ó
			TIM_SetTIM3Compare3(449);	//ÐÞ¸Ä±È½ÏÖµ£¬ÐÞ¸ÄÕ¼¿Õ±ÈÓÒ
			OSTimeDlyHMSM(0,0,2,0,OS_OPT_TIME_HMSM_STRICT,&err); //ÑÓÊ±2s
			if(ult_detection==0)
			{
				TIM_SetTIM3Compare4(449);	//ÐÞ¸Ä±È½ÏÖµ£¬ÐÞ¸ÄÕ¼¿Õ±È×ó
				TIM_SetTIM3Compare3(449);	//ÐÞ¸Ä±È½ÏÖµ£¬ÐÞ¸ÄÕ¼¿Õ±ÈÓÒ
				OSTimeDlyHMSM(0,0,2,0,OS_OPT_TIME_HMSM_STRICT,&err); //ÑÓÊ±2s
				TIM_SetTIM3Compare4(449);	//ÐÞ¸Ä±È½ÏÖµ£¬ÐÞ¸ÄÕ¼¿Õ±È×ó
				TIM_SetTIM3Compare3(498);	//ÐÞ¸Ä±È½ÏÖµ£¬ÐÞ¸ÄÕ¼¿Õ±ÈÓÒ
			}
			if(ult_detection>1)
			{
				direction_flag=1;
				Left_BK(0);Right_BK(0);
				Left_FR(1);Right_FR(0);
				Left_BK(1);Right_BK(1);
				TIM_SetTIM3Compare4(449);	//ÐÞ¸Ä±È½ÏÖµ£¬ÐÞ¸ÄÕ¼¿Õ±È×ó
				TIM_SetTIM3Compare3(498);	//ÐÞ¸Ä±È½ÏÖµ£¬ÐÞ¸ÄÕ¼¿Õ±ÈÓÒ
				OSTimeDlyHMSM(0,0,2,0,OS_OPT_TIME_HMSM_STRICT,&err); //ÑÓÊ±2s
				Left_BK(0);Right_BK(0);
				Left_FR(0);Right_FR(1);
				Left_BK(1);Right_BK(1);
			}
		}			
		
		OSTimeDlyHMSM(0,0,0,10,OS_OPT_TIME_HMSM_STRICT,&err); //ÑÓÊ±10ms
		
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
		bartype++;
		if(bartype>=5)bartype=0; 
	}
	else if(result[0]!=0)//Ê¶±ð³öÀ´ÁË£¬ÏÔÊ¾½á¹û
	{	
		PCF8574_WriteBit(BEEP_IO,0);//´ò¿ª·äÃùÆ÷
		OSTimeDlyHMSM(0,0,0,100,OS_OPT_TIME_HMSM_STRICT,&err); //ÑÓÊ±200ms
		PCF8574_WriteBit(BEEP_IO,1);
		POINT_COLOR=BLUE; 
		LCD_Fill(0,(lcddev.height+qr_image_width)/2+20,lcddev.width,lcddev.height,BLACK);
		Show_Str(0,(lcddev.height+qr_image_width)/2+20,lcddev.width,
								(lcddev.height-qr_image_width)/2-20,(u8*)result,16,0							
						);//LCDÏÔÊ¾Ê¶±ð½á¹û
		printf("\r\nresult:\r\n%s\r\n",result);//´®¿Ú´òÓ¡Ê¶±ð½á¹û 		
	}
	myfree(SRAMDTCM,bmp);		//ÊÍ·Å»Ò¶ÈÍ¼bmpÄÚ´æ
	myfree(SRAMIN,result);	//ÊÍ·ÅÊ¶±ð½á¹û	
}  


