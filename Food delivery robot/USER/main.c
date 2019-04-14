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
 ALIENTEK ������STM32F7������ UCOSIIIʵ��
 ��4-1 UCOSII��ֲʵ��
 
 UCOSIII���������ȼ��û�������ʹ�ã�ALIENTEK
 ����Щ���ȼ��������UCOSIII��5��ϵͳ�ڲ�����
 ���ȼ�0���жϷ������������� OS_IntQTask()
 ���ȼ�1��ʱ�ӽ������� OS_TickTask()
 ���ȼ�2����ʱ���� OS_TmrTask()
 ���ȼ�OS_CFG_PRIO_MAX-2��ͳ������ OS_StatTask()
 ���ȼ�OS_CFG_PRIO_MAX-1���������� OS_IdleTask()
 ����֧�֣�www.openedv.com
 �Ա����̣�http://eboard.taobao.com 
 ��ע΢�Ź���ƽ̨΢�źţ�"����ԭ��"����ѻ�ȡSTM32���ϡ�
 ������������ӿƼ����޹�˾  
 ���ߣ�����ԭ�� @ALIENTEK
************************************************/

//�������ȼ�
#define START_TASK_PRIO		3
//�����ջ��С	
#define START_STK_SIZE 		512
//������ƿ�
OS_TCB StartTaskTCB;
//�����ջ	
CPU_STK START_TASK_STK[START_STK_SIZE];
//������
void start_task(void *p_arg);

//�������ȼ�
#define interface_TASK_PRIO		4
//�����ջ��С	
#define interface_STK_SIZE 		256
//������ƿ�
OS_TCB interfaceTaskTCB;
//�����ջ	
CPU_STK interface_TASK_STK[interface_STK_SIZE];
void interface_task(void *p_arg);

//�������ȼ�
#define ov5640_TASK_PRIO		5
//�����ջ��С	
#define ov5640_STK_SIZE 		1024
//������ƿ�
OS_TCB ov5640TaskTCB;
//�����ջ	
CPU_STK ov5640_TASK_STK[ov5640_STK_SIZE];
void ov5640_task(void *p_arg);

//�������ȼ�
#define motor_drive_TASK_PRIO		4
//�����ջ��С	
#define motor_drive_STK_SIZE 		256
//������ƿ�
OS_TCB motor_driveTaskTCB;
//�����ջ	
CPU_STK motor_drive_TASK_STK[motor_drive_STK_SIZE];
void motor_drive_task(void *p_arg);


//�������ȼ�
#define ultrasonic_TASK_PRIO		4
//�����ջ��С	
#define ultrasonic_STK_SIZE 		256
//������ƿ�
OS_TCB ultrasonicTaskTCB;
//�����ջ	
CPU_STK ultrasonic_TASK_STK[ultrasonic_STK_SIZE];
void ultrasonic_task(void *p_arg);

//�������ȼ�
#define AGV_guide_TASK_PRIO		5
//�����ջ��С	
#define AGV_guide_STK_SIZE 		128
//������ƿ�
OS_TCB AGV_guideTaskTCB;
//�����ջ	
CPU_STK AGV_guide_TASK_STK[AGV_guide_STK_SIZE];
void AGV_guide_task(void *p_arg);

//�������ȼ�
#define lidar_TASK_PRIO		4
//�����ջ��С	
#define lidar_STK_SIZE       512
//������ƿ�
OS_TCB lidarTaskTCB;
//�����ջ	
CPU_STK lidar_TASK_STK[lidar_STK_SIZE];
//������
void lidar_task(void *p_arg);

//�������ȼ�
#define RFID_TASK_PRIO		5
//�����ջ��С	
#define RFID_STK_SIZE       128
//������ƿ�
OS_TCB RFIDTaskTCB;
//�����ջ	
CPU_STK RFID_TASK_STK[RFID_STK_SIZE];
//������
void RFID_task(void *p_arg);

//�������ȼ�
#define MUSIC_TASK_PRIO		5
//�����ջ��С	
#define MUSIC_STK_SIZE       256
//������ƿ�
OS_TCB MUSICTaskTCB;
//�����ջ	
CPU_STK MUSIC_TASK_STK[MUSIC_STK_SIZE];
//������
void MUSIC_task(void *p_arg);

//�������ȼ�
#define SELECT_TABLE_TASK_PRIO		5
//�����ջ��С	
#define SELECT_TABLE_STK_SIZE       256
//������ƿ�
OS_TCB SELECT_TABLETaskTCB;
//�����ջ	
CPU_STK SELECT_TABLE_TASK_STK[SELECT_TABLE_STK_SIZE];
//������
void SELECT_TABLE_task(void *p_arg);

//�������ȼ�
#define LETS_RUN_TASK_PRIO		5
//�����ջ��С	
#define LETS_RUN_STK_SIZE       256
//������ƿ�
OS_TCB LETS_RUNTaskTCB;
//�����ջ	
CPU_STK LETS_RUN_TASK_STK[LETS_RUN_STK_SIZE];
//������
void LETS_RUN_task(void *p_arg);

//�������ȼ�
#define WIFI_TASK_PRIO		4
//�����ջ��С	
#define WIFI_STK_SIZE       256
//������ƿ�
OS_TCB WIFITaskTCB;
//�����ջ	
CPU_STK WIFI_TASK_STK[WIFI_STK_SIZE];
//������
void WIFI_task(void *p_arg);

//�������ȼ�
#define runing_TASK_PRIO		4
//�����ջ��С
#define runing_STK_SIZE		256
//������ƿ�
OS_TCB	runingTaskTCB;
//�����ջ
__align(8) CPU_STK	runing_TASK_STK[runing_STK_SIZE];
//������
void runing_task(void *p_arg);



////////////////////////////////////////////////////////
//OS_TMR 	tmr1;		//��ʱ��1
//OS_TMR	tmr2;		//��ʱ��2
//void tmr1_callback(void *p_tmr, void *p_arg); 	//��ʱ��1�ص�����
//void tmr2_callback(void *p_tmr, void *p_arg);	//��ʱ��2�ص�����
u8 Lidar_Start[2]={0xA5,0x60};
u8 Lidar_Stop[2]={0xA5,0x65};  //��ʼɨ��,ֹͣɨ��


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
u8 LETS_RUN_FLAG;//�Ƿ����Ͳ�����

u8 USART_RX_FLAG;
u16 USART_RX_STORAGE;

u16 qr_image_width;						//����ʶ��ͼ��Ŀ�ȣ�����=��ȣ�
u8 	readok=0;									//�ɼ���һ֡���ݱ�ʶ
u32 *dcmi_line_buf[2];				//����ͷ����һ��һ�ж�ȡ,�����л���  
u16 *rgb_data_buf;						//RGB565֡����buf 
u16 dcmi_curline=0;						//����ͷ�������,��ǰ�б��	

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
	
    Write_Through();                //͸д
    Cache_Enable();                 //��L1-Cache
    HAL_Init();				        //��ʼ��HAL��
    Stm32_Clock_Init(432,25,2,9);   //����ʱ��,216Mhz 
    delay_init(216);                //��ʱ��ʼ��
	uart_init(128000);		        //���ڳ�ʼ��
	usart3_init(115200);  						//��ʼ������3������Ϊ115200
	usmart_dev.init(108); 		    //��ʼ��USMART	
    LED_Init();                     //��ʼ��LED
	KEY_Init();                     //��ʼ������
	SDRAM_Init();                   //��ʼ��SDRAM
	LCD_Init();                     //��ʼ��LCD
	
	ultrasonic_init();              //��ʼ��������������
	motor_drive_Init();
	
//	W25QXX_Init();				   	//��ʼ��W25Q256
//	W25QXX_Init();				    //��ʼ��W25Q256
//    WM8978_Init();				    //��ʼ��WM8978
//	WM8978_HPvol_Set(40,40);	    //������������
//	WM8978_SPKvol_Set(30);		    //������������
	PCF8574_Init();					//��ʼ��PCF8574
	OV5640_Init();					//��ʼ��OV5640
	RS485_Init(9600);		        //��ʼ��RS485
	tp_dev.init();				    //��ʼ��������
	my_mem_init(SRAMIN);            //��ʼ���ڲ��ڴ��
	my_mem_init(SRAMEX);            //��ʼ���ⲿSDRAM�ڴ��
	my_mem_init(SRAMDTCM);          //��ʼ���ڲ�DTCM�ڴ��
	
	exfuns_init();		            //Ϊfatfs��ر��������ڴ�  
    f_mount(fs[0],"0:",1);          //����SD�� 
 	f_mount(fs[1],"1:",1);          //����SPI FLASH.   
	
//	TIM3_Init(9,108-1);             //108M/108=1M�ļ���Ƶ�ʣ��Զ���װ��Ϊ100����ôPWMƵ��Ϊ1M/10=100khz
	TIM5_CH1_Cap_Init(0XFFFFFFFF,108-1); //��1MHZ��Ƶ�ʼ���
	TIM3_PWM_Init(500-1,108-1);     //108M/108=1M�ļ���Ƶ�ʣ��Զ���װ��Ϊ500����ôPWMƵ��Ϊ1M/500=2kHZ
	
	TIM_SetTIM3Compare4(399);	//�޸ıȽ�ֵ���޸�ռ�ձ�
	TIM_SetTIM3Compare3(399);	//�޸ıȽ�ֵ���޸�ռ�ձ�
	
	LCD_Draw_Circle(240,400,2);  //����״�����
	
//	HAL_UART_Transmit_IT(&UART1_Handler,Lidar_Start,2);          //���ڷ�������ָ��
	
	POINT_COLOR=RED; 
	LCD_Clear(BLACK); 	
	while(font_init()) 		//����ֿ�
	{	    
		LCD_ShowString(30,50,200,16,16,(u8*)"Font Error!");
		OSTimeDlyHMSM(0,0,0,200,OS_OPT_TIME_HMSM_STRICT,&err); //��ʱ200ms				  
		LCD_Fill(30,50,240,66,WHITE);//�����ʾ	     
		OSTimeDlyHMSM(0,0,0,200,OS_OPT_TIME_HMSM_STRICT,&err); //��ʱ200ms				  
	}  	 

	while(OV5640_Init())//��ʼ��OV5640
	{
		Show_Str(30,190,240,16,(u8*)"OV5640 ����!",16,0);
		OSTimeDlyHMSM(0,0,0,200,OS_OPT_TIME_HMSM_STRICT,&err); //��ʱ200ms
	    LCD_Fill(30,190,239,206,WHITE);
		OSTimeDlyHMSM(0,0,0,200,OS_OPT_TIME_HMSM_STRICT,&err); //��ʱ200ms
	}	
	//�Զ��Խ���ʼ��
	OV5640_RGB565_Mode();		//RGB565ģʽ 
	OV5640_Focus_Init(); 
	OV5640_Light_Mode(0);		//�Զ�ģʽ
	OV5640_Color_Saturation(3);	//ɫ�ʱ��Ͷ�0
	OV5640_Brightness(4);		//����0
	OV5640_Contrast(3);			//�Աȶ�0
	OV5640_Sharpness(33);		//�Զ����
	OV5640_Focus_Constant();//���������Խ�
	DCMI_Init();						//DCMI���� 

	
	
		
	qr_image_width=lcddev.width;
	if(qr_image_width>480)qr_image_width=480;//����qr_image_width����Ϊ240�ı���
	if(qr_image_width==320)qr_image_width=240;
	Show_Str(0,(lcddev.height+qr_image_width)/2+4,240,16,(u8*)"ʶ������",16,1);
	
	dcmi_line_buf[0]=mymalloc(SRAMIN,qr_image_width*2);						//Ϊ�л�����������ڴ�	
	dcmi_line_buf[1]=mymalloc(SRAMIN,qr_image_width*2);						//Ϊ�л�����������ڴ�
	rgb_data_buf=mymalloc(SRAMEX,qr_image_width*qr_image_width*2);//Ϊrgb֡���������ڴ�
	
	dcmi_rx_callback=qr_dcmi_rx_callback;//DMA���ݽ����жϻص�����
	DCMI_DMA_Init((u32)dcmi_line_buf[0],(u32)dcmi_line_buf[1],qr_image_width/2,DMA_MDATAALIGN_HALFWORD,DMA_MINC_ENABLE);//DCMI DMA����  
	fac=800/qr_image_width;	//�õ���������
	OV5640_OutSize_Set((1280-fac*qr_image_width)/2,(800-fac*qr_image_width)/2,qr_image_width,qr_image_width); 
	DCMI_Start(); 					//��������	 
		
	printf("SRAM IN:%d\r\n",my_mem_perused(SRAMIN));
	printf("SRAM EX:%d\r\n",my_mem_perused(SRAMEX));
	printf("SRAM DCTM:%d\r\n",my_mem_perused(SRAMDTCM)); 
	
	atk_qr_init();//��ʼ��ʶ��⣬Ϊ�㷨�����ڴ�
	
	printf("1SRAM IN:%d\r\n",my_mem_perused(SRAMIN));
	printf("1SRAM EX:%d\r\n",my_mem_perused(SRAMEX));
	printf("1SRAM DCTM:%d\r\n",my_mem_perused(SRAMDTCM));
	
	POINT_COLOR=RED;
	Show_Str_Mid(0,30,"ATK-ESP8266 WIFIģ�����",16,240); 
	while(atk_8266_send_cmd("AT","OK",20))//���WIFIģ���Ƿ�����
	{
		atk_8266_quit_trans();//�˳�͸��
		atk_8266_send_cmd("AT+CIPMODE=0","OK",200);  //�ر�͸��ģʽ	
//		Show_Str(40,55,200,16,"δ��⵽ģ��!!!",16,0);
		OSTimeDlyHMSM(0,0,0,800,OS_OPT_TIME_HMSM_STRICT,&err); //��ʱ200ms
//		LCD_Fill(40,55,200,55+16,WHITE);
//		Show_Str(40,55,200,16,"��������ģ��...",16,0); 
	} 
	while(atk_8266_send_cmd("ATE0","OK",20));//�رջ���
	atk_8266_msg_show(32,155,0);

	OSInit(&err);		            //��ʼ��UCOSIII
	OS_CRITICAL_ENTER();            //�����ٽ���
	//������ʼ����
	OSTaskCreate((OS_TCB 	* )&StartTaskTCB,		//������ƿ�
				 (CPU_CHAR	* )"start task", 		//��������
                 (OS_TASK_PTR )start_task, 			//������
                 (void		* )0,					//���ݸ��������Ĳ���
                 (OS_PRIO	  )START_TASK_PRIO,     //�������ȼ�
                 (CPU_STK   * )&START_TASK_STK[0],	//�����ջ����ַ
                 (CPU_STK_SIZE)START_STK_SIZE/10,	//�����ջ�����λ
                 (CPU_STK_SIZE)START_STK_SIZE,		//�����ջ��С
                 (OS_MSG_QTY  )0,					//�����ڲ���Ϣ�����ܹ����յ������Ϣ��Ŀ,Ϊ0ʱ��ֹ������Ϣ
                 (OS_TICK	  )0,					//��ʹ��ʱ��Ƭ��תʱ��ʱ��Ƭ���ȣ�Ϊ0ʱΪĬ�ϳ��ȣ�
                 (void   	* )0,					//�û�����Ĵ洢��
                 (OS_OPT      )OS_OPT_TASK_STK_CHK|OS_OPT_TASK_STK_CLR|OS_OPT_TASK_SAVE_FP, //����ѡ��,Ϊ�˱���������������񶼱��渡��Ĵ�����ֵ
                 (OS_ERR 	* )&err);				//��Ÿú�������ʱ�ķ���ֵ
	OS_CRITICAL_EXIT();	//�˳��ٽ���	 
	OSStart(&err);      //����UCOSIII
    while(1)
    {
	} 
}

//��ʼ������
void start_task(void *p_arg)
{
	OS_ERR err;
	CPU_SR_ALLOC();
	p_arg = p_arg;

	CPU_Init();
#if OS_CFG_STAT_TASK_EN > 0u
   OSStatTaskCPUUsageInit(&err);  	//ͳ������                
#endif
	
#ifdef CPU_CFG_INT_DIS_MEAS_EN		//���ʹ���˲����жϹر�ʱ��
    CPU_IntDisMeasMaxCurReset();	
#endif

#if	OS_CFG_SCHED_ROUND_ROBIN_EN  //��ʹ��ʱ��Ƭ��ת��ʱ��
	 //ʹ��ʱ��Ƭ��ת���ȹ���,����Ĭ�ϵ�ʱ��Ƭ����s
	OSSchedRoundRobinCfg(DEF_ENABLED,10,&err);  
#endif		
	
	//������ʱ��1
//	OSTmrCreate((OS_TMR		*)&tmr1,		//��ʱ��1
//                (CPU_CHAR	*)"tmr1",		//��ʱ������
//                (OS_TICK	 )20,			//20*10=200ms
//                (OS_TICK	 )100,          //100*10=1000ms
//                (OS_OPT		 )OS_OPT_TMR_PERIODIC, //����ģʽ
//                (OS_TMR_CALLBACK_PTR)tmr1_callback,//��ʱ��1�ص�����
//                (void	    *)0,			//����Ϊ0
//                (OS_ERR	    *)&err);		//���صĴ�����		
				
	
	OS_CRITICAL_ENTER();	//�����ٽ���
	//�������в�������
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
				 
//����interface����
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
				 
	//����ov5640����
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
	
				 
	//����lidar����
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
				 
	//����RFID����
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
	//����MUSIC����
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
//����RFID����
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
//����lidar����
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
//����lidar����
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
//����ultrasonic����
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

//����AGV_guide����
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

//����motor_drive_guide����
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
				 
	OS_CRITICAL_EXIT();	//�����ٽ���				 
	OS_TaskSuspend((OS_TCB*)&StartTaskTCB,&err);		//����ʼ����			 
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
		if(tp_dev.sta&TP_PRES_DOWN)			//������������
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
					HAL_UART_Transmit_IT(&UART1_Handler,Lidar_Start,2);          //���ڷ�������ָ��
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
					HAL_UART_Transmit_IT(&UART1_Handler,Lidar_Stop,2);          //���ڷ�������ָ��
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
							HAL_UART_Transmit_IT(&UART1_Handler,Lidar_Stop,2);          //���ڷ�������ָ��
							OSTaskSuspend((OS_TCB*)&lidarTaskTCB,&err);
		//					printf("suspend\r\n");
		//					LCD_Clear(WHITE);
							OSTaskResume((OS_TCB*)&interfaceTaskTCB,&err);
							
							OSTimeDlyHMSM(0,0,1,0,OS_OPT_TIME_HMSM_STRICT,&err); //��ʱ200ms
							
							seconfary_menu=0;
						}
				else OSTimeDlyHMSM(0,0,0,10,OS_OPT_TIME_HMSM_STRICT,&err);	//û�а������µ�ʱ�� 			   
			}
			
		}else OSTimeDlyHMSM(0,0,0,10,OS_OPT_TIME_HMSM_STRICT,&err);	//û�а������µ�ʱ�� 
		times++;
		if(times==500) 
		{
			OSTaskResume((OS_TCB*)&interfaceTaskTCB,&err);//ÿ���������ʾ
			times=0;
		}
	}
}

//interface������
void interface_task(void *p_arg)
{
	
	OS_ERR err;
	p_arg = p_arg;
	while(1)
	{	
		if(TOUCH_flag==0)
		{
			LCD_Clear(WHITE);//���� 
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
			LCD_Clear(WHITE);//���� 
			LCD_DrawRectangle(180, 20, 280, 80);
			LCD_ShowString(195,30,200,16,32,(u8*)"EXIT");		
			TOUCH_flag=0;
		}
		if(TOUCH_flag==2)
		{
			LCD_Clear(WHITE);//���� 
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
			LCD_Clear(WHITE);//���� 
			LCD_DrawRectangle(180, 20, 260, 80);
			LCD_ShowString(195,30,200,16,32,(u8*)"EXIT");
			LCD_Draw_Circle(240,400,2);  //����״�����
			TOUCH_flag=0;
		}
		
		OSTaskSuspend((OS_TCB*)&interfaceTaskTCB,&err);
	}
	
	
	
}

//ov56400������
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

			OSTimeDlyHMSM(0,0,0,200,OS_OPT_TIME_HMSM_STRICT,&err); //��ʱ200ms
//			if(i>0) 
//			{
////				OV5640_Init();					//��ʼ��OV5640
////				
////				while(OV5640_Init())//��ʼ��OV5640
////				{
////					Show_Str(30,190,240,16,(u8*)"OV5640 ����!",16,0);
////					OSTimeDlyHMSM(0,0,0,200,OS_OPT_TIME_HMSM_STRICT,&err); //��ʱ200ms
////					LCD_Fill(30,190,239,206,WHITE);
////					OSTimeDlyHMSM(0,0,0,200,OS_OPT_TIME_HMSM_STRICT,&err); //��ʱ200ms
////				}	
////				//�Զ��Խ���ʼ��
////				OV5640_RGB565_Mode();		//RGB565ģʽ 
////				OV5640_Focus_Init(); 
////				OV5640_Light_Mode(0);		//�Զ�ģʽ
////				OV5640_Color_Saturation(3);	//ɫ�ʱ��Ͷ�0
////				OV5640_Brightness(4);		//����0
////				OV5640_Contrast(3);			//�Աȶ�0
////				OV5640_Sharpness(33);		//�Զ����
////				OV5640_Focus_Constant();//���������Խ�
////				DCMI_Init();						//DCMI���� 
////				
////				qr_image_width=lcddev.width;
////				if(qr_image_width>480)qr_image_width=480;//����qr_image_width����Ϊ240�ı���
////				if(qr_image_width==320)qr_image_width=240;
////				Show_Str(0,(lcddev.height+qr_image_width)/2+4,240,16,(u8*)"ʶ������",16,1);
////				
////				dcmi_line_buf[0]=mymalloc(SRAMIN,qr_image_width*2);						//Ϊ�л�����������ڴ�	
////				dcmi_line_buf[1]=mymalloc(SRAMIN,qr_image_width*2);						//Ϊ�л�����������ڴ�
////				rgb_data_buf=mymalloc(SRAMEX,qr_image_width*qr_image_width*2);//Ϊrgb֡���������ڴ�
////				
////				dcmi_rx_callback=qr_dcmi_rx_callback;//DMA���ݽ����жϻص�����
////				DCMI_DMA_Init((u32)dcmi_line_buf[0],(u32)dcmi_line_buf[1],qr_image_width/2,DMA_MDATAALIGN_HALFWORD,DMA_MINC_ENABLE);//DCMI DMA����  
////				fac=800/qr_image_width;	//�õ���������
////				OV5640_OutSize_Set((1280-fac*qr_image_width)/2,(800-fac*qr_image_width)/2,qr_image_width,qr_image_width); 
////				DCMI_Start(); 					//��������	 
////					
////				printf("SRAM IN:%d\r\n",my_mem_perused(SRAMIN));
////				printf("SRAM EX:%d\r\n",my_mem_perused(SRAMEX));
////				printf("SRAM DCTM:%d\r\n",my_mem_perused(SRAMDTCM)); 
////				
////				atk_qr_init();//��ʼ��ʶ��⣬Ϊ�㷨�����ڴ�
////				
////				printf("1SRAM IN:%d\r\n",my_mem_perused(SRAMIN));
////				printf("1SRAM EX:%d\r\n",my_mem_perused(SRAMEX));
////				printf("1SRAM DCTM:%d\r\n",my_mem_perused(SRAMDTCM));
//				i=0;
//			}
			if(readok==1)			//�ɼ�����һ֡ͼ��
			{		
				readok=0;
				qr_show_image((lcddev.width-qr_image_width)/2,(lcddev.height-qr_image_width)/2,qr_image_width,qr_image_width,rgb_data_buf);
				qr_decode(qr_image_width,rgb_data_buf);
			}

		}

}


//lidar������
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
		OSTimeDlyHMSM(0,0,0,50,OS_OPT_TIME_HMSM_STRICT,&err); //��ʱ200ms
	}
}


//lidar������
void SELECT_TABLE_task(void *p_arg)
{
	OS_ERR err;
	p_arg = p_arg;
	
	while(1)
	{
//		if(tp_dev.sta&TP_PRES_DOWN)			//������������
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
		
		OSTimeDlyHMSM(0,0,0,100,OS_OPT_TIME_HMSM_STRICT,&err); //��ʱ200ms
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
		
		OSTimeDlyHMSM(0,0,0,100,OS_OPT_TIME_HMSM_STRICT,&err); //��ʱ200ms
	}
}

extern u8 Lidar_receive_flag;

//lidar������
void lidar_task(void *p_arg)
{
	
	
	u32 count;
	extern float AngleFSA,AngleLSA,Anglei[100],Lidar_Distance[100],Lidar_x[100],Lidar_y[100]; //��ʼ��,������,�м��,����
	extern u8 lidar_i,lidar_LSN,lidar_b,lidar_n,lidar_c;
	extern u8 lidar_a,lidar_m,Lidar_Data[150];//��������
	float k;
	u8 key;
	
	OS_ERR err;
	p_arg = p_arg;
	
	while(1)
	{
		
//		if(tp_dev.sta&TP_PRES_DOWN)			//������������
//		{
//			seconfary_menu=1;
//		}
		
		LCD_Draw_Circle(240,400,2);  //����״�����
		
//		if(Lidar_receive_flag==1)
//		{
//			Lidar_receive_flag=0;

//				
//			
//		}
		
		key=KEY_Scan(0);
			switch(key)
			{				 
				
				case  KEY2_PRES:	//LCD�����Ŵ�
							k=k+0.02f;
							break;
				
				case  KEY0_PRES:	//LCD������С
							k=k-0.02f;
							break;
				case  KEY1_PRES:	//LCD�����Ŵ�
							HAL_UART_Transmit_IT(&UART1_Handler,Lidar_Start,2);          //���ڷ�������ָ��
				
							PCF8574_WriteBit(BEEP_IO,0);//�򿪷�����
							OSTimeDlyHMSM(0,0,0,100,OS_OPT_TIME_HMSM_STRICT,&err); //��ʱ200ms
							PCF8574_WriteBit(BEEP_IO,1);
							PCF8574_WriteBit(RS485_RE_IO,0);
							break;
			}
		
		if(k==0)
		{
			k=0.02;
		}
		
//		evade();
			
//		LCD_ShowString(0,80,240,32,32,"FSA=");	  //��ʾ��ʼ��,������
//		LCD_ShowNum(80,80,AngleFSA,3,32);
//		LCD_ShowString(0,120,240,32,32,"LSA=");	
//		LCD_ShowNum(80,120,AngleLSA,3,32);
	
		for(lidar_m=0;lidar_m<=lidar_LSN;lidar_m++)
		{
//			LCD_DrawPoint(240-(Lidar_x[lidar_m]*k),400-(Lidar_y[lidar_m]*k));
//			LCD_ShowNum(200,20*lidar_m,Anglei[lidar_m],6,16);                //LCD��ʾ�Ƕ�
//			LCD_ShowNum(300,20*lidar_m,Distance[lidar_m],6,16);	       //LCD��ʾ����
//			printf("%f,%f        ",Anglei[m],Distance[m]);  //������ʾ
		}
		
		OSTimeDlyHMSM(0,0,0,3,OS_OPT_TIME_HMSM_STRICT,&err); //��ʱ200ms				
		count++;
		if(count==250)
		{
			
			LCD_Clear(WHITE);
			count=0;
		}
		
		
	}
}

extern u8 wifi_flag;
	
//lidar������
void WIFI_task(void *p_arg)
{
	u8 i,key;
	u16 rlen;
	
	u8 netpro=0;	//����ģʽ
	u8 timex=0; 
	u8 ipbuf[16]; 	//IP����
	u8 *p;
	u16 t=999;		//���ٵ�һ�λ�ȡ����״̬
	u8 res=0;
	u8 constate=0;	//����״̬
	
	OS_ERR err;
	p_arg = p_arg;
	
	p=mymalloc(SRAMIN,32);							//����32�ֽ��ڴ�
	USART3_RX_STA=0;
	while(1)
	{
		
		
		if(wifi_flag)
		{
			atk_8266_wifiap_test();	//WIFI AP����

			while(1)
			{
				t++;
				OSTimeDlyHMSM(0,0,0,5,OS_OPT_TIME_HMSM_STRICT,&err); //��ʱ200ms
		  
				if(USART3_RX_STA&0X8000)		//���յ�һ��������
				{ 
					rlen=USART3_RX_STA&0X7FFF;	//�õ����ν��յ������ݳ���
					USART3_RX_BUF[rlen]=0;		//��ӽ����� 
					for(i=0;i<rlen;i++)
					{
						printf("USART3_RX_BUF:%d\r\n",USART3_RX_BUF[i]);	//���͵�����   
					}
					printf("\r\n");	//���͵����� 
					Show_Str(80,340,180,190,USART3_RX_BUF,12,0);//��ʾ���յ�������  
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
					PCF8574_WriteBit(BEEP_IO,0);//�򿪷�����
					OSTimeDlyHMSM(0,0,0,100,OS_OPT_TIME_HMSM_STRICT,&err); //��ʱ200ms
					PCF8574_WriteBit(BEEP_IO,1);
					PCF8574_WriteBit(RS485_RE_IO,0);
					printf("SELECT_TABLE_RESULT %d\r\n",SELECT_TABLE_RESULT);
					USART3_RX_STA=0;

				}
		
				if(t==1000&&seconfary_menu==0)//����10����û���յ��κ�����,��������ǲ��ǻ�����.
				{
					constate=atk_8266_consta_check();//�õ�����״̬
					if(constate=='+')Show_Str(120,300,200,12,"���ӳɹ�",12,0);  //����״̬
					else Show_Str(120,300,200,24,"����ʧ��",12,0); 	  	 
					t=0;
				}

				atk_8266_at_response(1);
				
			}
			atk_8266_msg_show(32,155,0);
			OSTaskResume((OS_TCB*)&interfaceTaskTCB,&err);
			
			wifi_flag=0;
		}
	OSTimeDlyHMSM(0,0,0,10,OS_OPT_TIME_HMSM_STRICT,&err); //��ʱ200ms
	}
}

extern u8  TIM5CH1_CAPTURE_STA;		//���벶��״̬		    				
extern u32	TIM5CH1_CAPTURE_VAL;	//���벶��ֵ 


void ultrasonic_task(void *p_arg)
{
	long long temp=0;
	u8 ult_times0,ult_times1,ult_times2;
	long long ult_temp=0;
	
	OS_ERR err;
	p_arg = p_arg;
	
	while(1)
	{
		if(TIM5CH1_CAPTURE_STA&0X80)        //�ɹ�������һ�θߵ�ƽ
		{
			temp=TIM5CH1_CAPTURE_STA&0X3F; 
			temp*=0XFFFFFFFF;		 	    //���ʱ���ܺ�
			temp+=TIM5CH1_CAPTURE_VAL;      //�õ��ܵĸߵ�ƽʱ��
			temp=temp/58;
			
			printf("HIGH:%lld cm\r\n",temp);//��ӡ�ܵĸߵ�ƽʱ��		
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
					TIM_SetTIM3Compare4(399);	//�޸ıȽ�ֵ���޸�ռ�ձ���
					TIM_SetTIM3Compare3(399);	//�޸ıȽ�ֵ���޸�ռ�ձ���	
					OSTimeDlyHMSM(0,0,5,0,OS_OPT_TIME_HMSM_STRICT,&err); //��ʱ100ms
					Left_BK(0);Right_BK(0);
					Left_FR(0);Right_FR(1);
					Left_BK(1);Right_BK(1);
					ult_detection=1;
					ult_times2=0;
				}
			}
//			printf("music:%d\r\n\r\n",music_play);

			TIM5CH1_CAPTURE_STA=0;          //������һ�β���
		}
		HAL_GPIO_WritePin(GPIOB,GPIO_PIN_15,GPIO_PIN_SET);
		delay_us(15);
		HAL_GPIO_WritePin(GPIOB,GPIO_PIN_15,GPIO_PIN_RESET);

		
		
		
//		printf("%d\r\n",ult_detection);
		
		OSTimeDlyHMSM(0,0,0,100,OS_OPT_TIME_HMSM_STRICT,&err); //��ʱ100ms
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
			if(Rx_485_BUF[0]==1)//�Ź��һλ
			{
	//			len=USART_RX_STA&0x3fff;//�õ��˴ν��յ������ݳ���
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

		OSTimeDlyHMSM(0,0,0,5,OS_OPT_TIME_HMSM_STRICT,&err); //��ʱ100ms
			
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


//motor_drive������
//�й켰����������
void motor_drive_task(void *p_arg)
{
	u16 AGV_feedback;
	u8 ult_flag=0;//��⵽ǰ�����ϰ���ʱ�����ƽ����������
	u8 direction_flag=0;//�����ϰ�ʱ0ѡ������1ѡ������
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
				TIM_SetTIM3Compare4(349);	//�޸ıȽ�ֵ���޸�ռ�ձ���
				TIM_SetTIM3Compare3(349);	//�޸ıȽ�ֵ���޸�ռ�ձ���	
			}
					
			if((AGV_feedback<0x0128)&&(AGV_feedback!=0x0108)&&(AGV_feedback!=0x0060)&&AGV_feedback!=0)
			{					
				if(process_control==6)
				{
					
					TIM_SetTIM3Compare4(459);	//�޸ıȽ�ֵ���޸�ռ�ձ���
					TIM_SetTIM3Compare3(349);	//�޸ıȽ�ֵ���޸�ռ�ձ���
					process_control=0;	
					OSTimeDlyHMSM(0,0,3,500,OS_OPT_TIME_HMSM_STRICT,&err); //��ʱ2s	
				}
				if(process_control==7)
				{
					
					TIM_SetTIM3Compare4(498);	//�޸ıȽ�ֵ���޸�ռ�ձ���
					TIM_SetTIM3Compare3(349);	//�޸ıȽ�ֵ���޸�ռ�ձ���
					process_control=0;	
					OSTimeDlyHMSM(0,0,3,0,OS_OPT_TIME_HMSM_STRICT,&err); //��ʱ2s	
				}
				else
				{
					printf("aaa\r\n");
					TIM_SetTIM3Compare4(249);	//�޸ıȽ�ֵ���޸�ռ�ձ���
					TIM_SetTIM3Compare3(429);	//�޸ıȽ�ֵ���޸�ռ�ձ���	
				}					
			}
			
			if(AGV_feedback>0x03c8)
			{
				if(process_control==6)
				{
					TIM_SetTIM3Compare4(349);	//�޸ıȽ�ֵ���޸�ռ�ձ���
					TIM_SetTIM3Compare3(429);	//�޸ıȽ�ֵ���޸�ռ�ձ���
					process_control=0;	
					OSTimeDlyHMSM(0,0,2,0,OS_OPT_TIME_HMSM_STRICT,&err); //��ʱ2s

				}
				if(process_control==7)
				{
					
					TIM_SetTIM3Compare4(349);	//�޸ıȽ�ֵ���޸�ռ�ձ���
					TIM_SetTIM3Compare3(498);	//�޸ıȽ�ֵ���޸�ռ�ձ���
					process_control=0;	
					OSTimeDlyHMSM(0,0,3,0,OS_OPT_TIME_HMSM_STRICT,&err); //��ʱ2s	
				}
				else
				{
					printf("aaa\r\n");
					TIM_SetTIM3Compare4(429);	//�޸ıȽ�ֵ���޸�ռ�ձ���
					TIM_SetTIM3Compare3(249);	//�޸ıȽ�ֵ���޸�ռ�ձ���
				}
			}
		}
		if(LETS_RUN_FLAG==1)
		{
			if(RFID_RESULT==1||RFID_RESULT==2||RFID_RESULT==3||RFID_RESULT==5)
			{
				if(arrive_process_control==0)
				{
					TIM_SetTIM3Compare4(349);	//�޸ıȽ�ֵ���޸�ռ�ձ���
					TIM_SetTIM3Compare3(498);	//�޸ıȽ�ֵ���޸�ռ�ձ���
					arrive_process_control=1;
					OSTimeDlyHMSM(0,0,4,0,OS_OPT_TIME_HMSM_STRICT,&err); //��ʱ2s
				}
				if(arrive_process_control==1)
				{
					TIM_SetTIM3Compare4(399);	//�޸ıȽ�ֵ���޸�ռ�ձ���
					TIM_SetTIM3Compare3(399);	//�޸ıȽ�ֵ���޸�ռ�ձ���
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
						TIM_SetTIM3Compare4(399);	//�޸ıȽ�ֵ���޸�ռ�ձ���
						TIM_SetTIM3Compare3(498);	//�޸ıȽ�ֵ���޸�ռ�ձ���
						OSTimeDlyHMSM(0,0,0,500,OS_OPT_TIME_HMSM_STRICT,&err); //��ʱ2s
						Left_BK(0);Right_BK(0);
						Left_FR(0);Right_FR(1);
						Left_BK(1);Right_BK(1);
						TIM_SetTIM3Compare4(399);	//�޸ıȽ�ֵ���޸�ռ�ձ���
						TIM_SetTIM3Compare3(399);	//�޸ıȽ�ֵ���޸�ռ�ձ���
					}
					if(AGV_feedback>0x8000)
					{
						Left_BK(0);Right_BK(0);
						Left_FR(1);Right_FR(0);
						Left_BK(1);Right_BK(1);
						TIM_SetTIM3Compare4(498);	//�޸ıȽ�ֵ���޸�ռ�ձ���
						TIM_SetTIM3Compare3(399);	//�޸ıȽ�ֵ���޸�ռ�ձ���
						OSTimeDlyHMSM(0,0,0,500,OS_OPT_TIME_HMSM_STRICT,&err); //��ʱ2s
						Left_BK(0);Right_BK(0);
						Left_FR(0);Right_FR(1);
						Left_BK(1);Right_BK(1);
						TIM_SetTIM3Compare4(399);	//�޸ıȽ�ֵ���޸�ռ�ձ���
						TIM_SetTIM3Compare3(399);	//�޸ıȽ�ֵ���޸�ռ�ձ���
					}
				}
				if(RFID_RESULT==6||RFID_RESULT==7||RFID_RESULT==8||RFID_RESULT==10)
				{
					TIM_SetTIM3Compare4(498);	//�޸ıȽ�ֵ���޸�ռ�ձ���
					TIM_SetTIM3Compare3(498);	//�޸ıȽ�ֵ���޸�ռ�ձ���
					LCD_Clear(WHITE);
					OSTimeDlyHMSM(0,0,0,50,OS_OPT_TIME_HMSM_STRICT,&err); //��ʱ5s
					OSTaskResume((OS_TCB*)&ov5640TaskTCB,&err);
					qr_show:  if(QR_CODE_RESULT==1) 
					{
						OSTimeDlyHMSM(0,0,2,0,OS_OPT_TIME_HMSM_STRICT,&err); //��ʱ5s
						QR_CODE_RESULT=0;
					}
					else
					{
						OSTimeDlyHMSM(0,0,0,100,OS_OPT_TIME_HMSM_STRICT,&err); //��ʱ5s
						goto qr_show;
					}
					OSTaskSuspend((OS_TCB*)&ov5640TaskTCB,&err);
					OSTaskResume((OS_TCB*)&interfaceTaskTCB,&err);
					PCF8574_WriteBit(RS485_RE_IO,0);
//					Left_BK(0);Right_BK(0);
//					Left_FR(1);Right_FR(0);
//					Left_BK(1);Right_BK(1);
//					TIM_SetTIM3Compare4(399);	//�޸ıȽ�ֵ���޸�ռ�ձ���
//					TIM_SetTIM3Compare3(399);	//�޸ıȽ�ֵ���޸�ռ�ձ���
//					OSTimeDlyHMSM(0,0,2,0,OS_OPT_TIME_HMSM_STRICT,&err); //��ʱ5s
//					Left_BK(0);Right_BK(0);
//					Left_FR(0);Right_FR(1);
//					Left_BK(1);Right_BK(1);
					TIM_SetTIM3Compare4(498);	//�޸ıȽ�ֵ���޸�ռ�ձ���
					TIM_SetTIM3Compare3(399);	//�޸ıȽ�ֵ���޸�ռ�ձ���
					OSTimeDlyHMSM(0,0,11,0,OS_OPT_TIME_HMSM_STRICT,&err); //��ʱ11s
					TIM_SetTIM3Compare4(399);	//�޸ıȽ�ֵ���޸�ռ�ձ���
					TIM_SetTIM3Compare3(399);	//�޸ıȽ�ֵ���޸�ռ�ձ���
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
		
		if(ult_detection==1&LETS_RUN_FLAG==0) ult_flag=1;//��⵽ǰ�����ϰ�
			
		if((ult_flag==1)&&(direction_flag==0)&&LETS_RUN_FLAG==0)
		{
			if(process_control==0)
			{
				TIM_SetTIM3Compare4(399);	//�޸ıȽ�ֵ���޸�ռ�ձ���
				TIM_SetTIM3Compare3(498);	//�޸ıȽ�ֵ���޸�ռ�ձ���
				process_control=1;
				OSTimeDlyHMSM(0,0,4,0,OS_OPT_TIME_HMSM_STRICT,&err); //��ʱ2s
		
				printf("a\r\n");
			}
//			printf("ult_detection%d\r\n",ult_detection);
			if((ult_detection==0)&&(direction_flag==0))
			{
				if(process_control==1)
				{
					TIM_SetTIM3Compare4(399);	//�޸ıȽ�ֵ���޸�ռ�ձ���
					TIM_SetTIM3Compare3(399);	//�޸ıȽ�ֵ���޸�ռ�ձ���
					process_control=2;
					OSTimeDlyHMSM(0,0,5,0,OS_OPT_TIME_HMSM_STRICT,&err); //��ʱ2s
					printf("aa\r\n");
				}
				if(process_control==2)
				{
					TIM_SetTIM3Compare4(498);	//�޸ıȽ�ֵ���޸�ռ�ձ���
					TIM_SetTIM3Compare3(399);	//�޸ıȽ�ֵ���޸�ռ�ձ��
					process_control=3;
					OSTimeDlyHMSM(0,0,4,0,OS_OPT_TIME_HMSM_STRICT,&err); //��ʱ2s
					printf("bb\r\n");
				}
				if(process_control==3)
				{
					TIM_SetTIM3Compare4(399);	//�޸ıȽ�ֵ���޸�ռ�ձ���
					TIM_SetTIM3Compare3(399);	//�޸ıȽ�ֵ���޸�ռ�ձ���
					process_control=4;
					OSTimeDlyHMSM(0,0,6,0,OS_OPT_TIME_HMSM_STRICT,&err); //��ʱ2s
				}
				if(process_control==4)
				{
					TIM_SetTIM3Compare4(498);	//�޸ıȽ�ֵ���޸�ռ�ձ���
					TIM_SetTIM3Compare3(399);	//�޸ıȽ�ֵ���޸�ռ�ձ���
					process_control=5;
					AGV_feedback=0;
					OSTimeDlyHMSM(0,0,4,0,OS_OPT_TIME_HMSM_STRICT,&err); //��ʱ2s
				}
				if(process_control==5)
				{
					TIM_SetTIM3Compare4(399);	//�޸ıȽ�ֵ���޸�ռ�ձ���
					TIM_SetTIM3Compare3(399);	//�޸ıȽ�ֵ���޸�ռ�ձ���
					process_control=6;

					ult_flag=0;
//					process_control=0;
//					OSTimeDlyHMSM(0,0,2,0,OS_OPT_TIME_HMSM_STRICT,&err); //��ʱ2s
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
					TIM_SetTIM3Compare4(399);	//�޸ıȽ�ֵ���޸�ռ�ձ���
					TIM_SetTIM3Compare3(498);	//�޸ıȽ�ֵ���޸�ռ�ձ���
					OSTimeDlyHMSM(0,0,4,0,OS_OPT_TIME_HMSM_STRICT,&err); //��ʱ2s
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
					TIM_SetTIM3Compare4(498);	//�޸ıȽ�ֵ���޸�ռ�ձ���
					TIM_SetTIM3Compare3(399);	//�޸ıȽ�ֵ���޸�ռ�ձ���
					OSTimeDlyHMSM(0,0,4,0,OS_OPT_TIME_HMSM_STRICT,&err); //��ʱ2s
					TIM_SetTIM3Compare4(399);	//�޸ıȽ�ֵ���޸�ռ�ձ���
					TIM_SetTIM3Compare3(399);	//�޸ıȽ�ֵ���޸�ռ�ձ���
					OSTimeDlyHMSM(0,0,4,0,OS_OPT_TIME_HMSM_STRICT,&err); //��ʱ2s
					TIM_SetTIM3Compare4(399);	//�޸ıȽ�ֵ���޸�ռ�ձ���
					TIM_SetTIM3Compare3(498);	//�޸ıȽ�ֵ���޸�ռ�ձ���
					OSTimeDlyHMSM(0,0,4,0,OS_OPT_TIME_HMSM_STRICT,&err); //��ʱ2s
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
				TIM_SetTIM3Compare4(498);	//�޸ıȽ�ֵ���޸�ռ�ձ���
				TIM_SetTIM3Compare3(399);	//�޸ıȽ�ֵ���޸�ռ�ձ���
				OSTimeDlyHMSM(0,0,4,0,OS_OPT_TIME_HMSM_STRICT,&err); //��ʱ2s
				process_control=1;
				printf("q\r\n");
			}
//			OSTimeDlyHMSM(0,0,1,0,OS_OPT_TIME_HMSM_STRICT,&err); //��ʱ2s
			if(ult_detection==0)
			{
				if(process_control==1)
				{
					TIM_SetTIM3Compare4(399);	//�޸ıȽ�ֵ���޸�ռ�ձ���
					TIM_SetTIM3Compare3(399);	//�޸ıȽ�ֵ���޸�ռ�ձ���
					process_control=2;
					OSTimeDlyHMSM(0,0,6,0,OS_OPT_TIME_HMSM_STRICT,&err); //��ʱ2s
					printf("ww\r\n");
				}
				if(process_control==2)
				{
					TIM_SetTIM3Compare4(399);	//�޸ıȽ�ֵ���޸�ռ�ձ���
					TIM_SetTIM3Compare3(498);	//�޸ıȽ�ֵ���޸�ռ�ձ���
					process_control=3;
					OSTimeDlyHMSM(0,0,3,500,OS_OPT_TIME_HMSM_STRICT,&err); //��ʱ2s
					printf("ee\r\n");
				}
				if(process_control==3)
				{
					TIM_SetTIM3Compare4(399);	//�޸ıȽ�ֵ���޸�ռ�ձ���
					TIM_SetTIM3Compare3(399);	//�޸ıȽ�ֵ���޸�ռ�ձ���
					process_control=4;
					OSTimeDlyHMSM(0,0,6,0,OS_OPT_TIME_HMSM_STRICT,&err); //��ʱ2s
				}
				if(process_control==4)
				{
					TIM_SetTIM3Compare4(399);	//�޸ıȽ�ֵ���޸�ռ�ձ���
					TIM_SetTIM3Compare3(498);	//�޸ıȽ�ֵ���޸�ռ�ձ���
					process_control=5;
					OSTimeDlyHMSM(0,0,4,0,OS_OPT_TIME_HMSM_STRICT,&err); //��ʱ2s
				}
				if(process_control==5)
				{
					
					TIM_SetTIM3Compare4(399);	//�޸ıȽ�ֵ���޸�ռ�ձ���
					TIM_SetTIM3Compare3(399);	//�޸ıȽ�ֵ���޸�ռ�ձ���
//					if(AGV_feedback!=0)
//					{
//						printf("AGV_feedbackAGV_feedback:\r\n\r\n%x\r\n\r\n",AGV_feedback);
//						TIM_SetTIM3Compare4(399);	//�޸ıȽ�ֵ���޸�ռ�ձ���
//						TIM_SetTIM3Compare3(498);	//�޸ıȽ�ֵ���޸�ռ�ձ���
						ult_flag=0;
						direction_flag=0;
						process_control=6;
						printf("ff\r\n");
//						OSTimeDlyHMSM(0,0,2,0,OS_OPT_TIME_HMSM_STRICT,&err); //��ʱ2s
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
					TIM_SetTIM3Compare4(498);	//�޸ıȽ�ֵ���޸�ռ�ձ���
					TIM_SetTIM3Compare3(399);	//�޸ıȽ�ֵ���޸�ռ�ձ���
					OSTimeDlyHMSM(0,0,4,0,OS_OPT_TIME_HMSM_STRICT,&err); //��ʱ2s
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
					TIM_SetTIM3Compare4(399);	//�޸ıȽ�ֵ���޸�ռ�ձ���
					TIM_SetTIM3Compare3(498);	//�޸ıȽ�ֵ���޸�ռ�ձ���
					OSTimeDlyHMSM(0,0,4,0,OS_OPT_TIME_HMSM_STRICT,&err); //��ʱ2s
					TIM_SetTIM3Compare4(399);	//�޸ıȽ�ֵ���޸�ռ�ձ���
					TIM_SetTIM3Compare3(399);	//�޸ıȽ�ֵ���޸�ռ�ձ���
					OSTimeDlyHMSM(0,0,4,0,OS_OPT_TIME_HMSM_STRICT,&err); //��ʱ2s
					TIM_SetTIM3Compare4(498);	//�޸ıȽ�ֵ���޸�ռ�ձ���
					TIM_SetTIM3Compare3(399);	//�޸ıȽ�ֵ���޸�ռ�ձ���
					OSTimeDlyHMSM(0,0,4,0,OS_OPT_TIME_HMSM_STRICT,&err); //��ʱ2s
					Left_BK(0);Right_BK(0);
					Left_FR(0);Right_FR(1);
					Left_BK(1);Right_BK(1);
					process_control=0;
				}
			}
		}			
//		printf("%d %d %d %d\r\n",ult_detection,direction_flag,ult_flag,LETS_RUN_FLAG);
//		printf("process_control %d\r\n",process_control);
		OSTimeDlyHMSM(0,0,0,10,OS_OPT_TIME_HMSM_STRICT,&err); //��ʱ10ms
	}
}

//lidar������
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
				W25QXX_Init();				   	//��ʼ��W25Q256
				W25QXX_Init();				    //��ʼ��W25Q256
				WM8978_Init();				    //��ʼ��WM8978
				WM8978_HPvol_Set(40,40);	    //������������
				WM8978_SPKvol_Set(30);		    //������������
				PCF8574_Init();					//��ʼ��PCF8574
				
			}
			audio_play();
			
			if(i>5) i=2;
			OSTimeDlyHMSM(0,0,0,50,OS_OPT_TIME_HMSM_STRICT,&err); //��ʱ200ms
		}
		OSTimeDlyHMSM(0,0,0,50,OS_OPT_TIME_HMSM_STRICT,&err); //��ʱ200ms
	}
}

//����ͷ����DMA��������жϻص�����
void qr_dcmi_rx_callback(void)
{  

	CPU_SR_ALLOC();
	u32 *pbuf;
	u16 i;
	pbuf=(u32*)(rgb_data_buf+dcmi_curline*qr_image_width);//��rgb_data_buf��ַƫ�Ƹ�ֵ��pbuf
	
	if(DMADMCI_Handler.Instance->CR&(1<<19))//DMAʹ��buf1,��ȡbuf0
	{ 
		for(i=0;i<qr_image_width/2;i++)
		{
			pbuf[i]=dcmi_line_buf[0][i];
		} 
	}else 										//DMAʹ��buf0,��ȡbuf1
	{
		for(i=0;i<qr_image_width/2;i++)
		{
			pbuf[i]=dcmi_line_buf[1][i];
		} 
	} 
	dcmi_curline++;
}

//��ʾͼ��
void qr_show_image(u16 xoff,u16 yoff,u16 width,u16 height,u16 *imagebuf)
{

	CPU_SR_ALLOC();
	u16 linecnt=yoff;
	
	if(lcdltdc.pwidth!=0)//RGB��
	{
		for(linecnt=0;linecnt<height;linecnt++)
		{
			LTDC_Color_Fill(xoff,linecnt+yoff,xoff+width-1,linecnt+yoff,imagebuf+linecnt*width);//RGB��,DM2D��� 
		}
		
	}else LCD_Color_Fill(xoff,yoff,xoff+width-1,yoff+height-1,imagebuf);	//MCU��,ֱ����ʾ
}

//imagewidth:<=240;����240ʱ,��240��������
//imagebuf:RGBͼ�����ݻ�����
void qr_decode(u16 imagewidth,u16 *imagebuf)
{
	OS_ERR err;
	CPU_SR_ALLOC();
	static u8 bartype=0; 
	u8 *bmp;
	u8 *result=NULL;
	u16 Color;
	u16 i,j;	
	u16 qr_img_width=0;						//����ʶ������ͼ����,��󲻳���240!
	u8 qr_img_scale=0;						//ѹ����������
	
	if(imagewidth>240)
	{
		if(imagewidth%240)return ;	//����240�ı���,ֱ���˳�
		qr_img_width=240;
		qr_img_scale=imagewidth/qr_img_width;
	}else
	{
		qr_img_width=imagewidth;
		qr_img_scale=1;
	}  
	result=mymalloc(SRAMIN,1536);//����ʶ��������ڴ�
	bmp=mymalloc(SRAMDTCM,qr_img_width*qr_img_width);//DTCM�����ڴ�Ϊ120K����������240*240=56K 
	mymemset(bmp,0,qr_img_width*qr_img_width);
	if(lcdltdc.pwidth==0)//MCU��,���辵��
	{ 
		for(i=0;i<qr_img_width;i++)		
		{
			for(j=0;j<qr_img_width;j++)		//��RGB565ͼƬת�ɻҶ�
			{	
				Color=*(imagebuf+((i*imagewidth)+j)*qr_img_scale); //����qr_img_scaleѹ����240*240
				*(bmp+i*qr_img_width+j)=(((Color&0xF800)>> 8)*76+((Color&0x7E0)>>3)*150+((Color&0x001F)<<3)*30)>>8;
			}		
		}
	}else	//RGB��,��Ҫ����
	{
		for(i=0;i<qr_img_width;i++)		
		{
			for(j=0;j<qr_img_width;j++)		//��RGB565ͼƬת�ɻҶ�
			{	
				Color=*(imagebuf+((i*imagewidth)+qr_img_width-j-1)*qr_img_scale);//����qr_img_scaleѹ����240*240
				*(bmp+i*qr_img_width+j)=(((Color&0xF800)>> 8)*76+((Color&0x7E0)>>3)*150+((Color&0x001F)<<3)*30)>>8;
			}		
		}		
	}
	atk_qr_decode(qr_img_width,qr_img_width,bmp,bartype,result);//ʶ��Ҷ�ͼƬ��ע�⣺���κ�ʱԼ0.2S��
	
	if(result[0]==0)//û��ʶ�����
	{
		QR_CODE_RESULT=0;
		bartype++;
		if(bartype>=5)bartype=0; 
	}
	else if(result[0]!=0)//ʶ������ˣ���ʾ���
	{	
		QR_CODE_RESULT=1;
		PCF8574_WriteBit(BEEP_IO,0);//�򿪷�����
		OSTimeDlyHMSM(0,0,0,100,OS_OPT_TIME_HMSM_STRICT,&err); //��ʱ200ms
		PCF8574_WriteBit(BEEP_IO,1);
//		POINT_COLOR=BLUE; 
		LCD_Fill(0,(lcddev.height+qr_image_width)/2+20,lcddev.width,lcddev.height,BLACK);
		Show_Str(0,(lcddev.height+qr_image_width)/2+20,lcddev.width,
								(lcddev.height-qr_image_width)/2-20,(u8*)result,16,0							
						);//LCD��ʾʶ����
		printf("\r\nresult:\r\n%s\r\n",result);//���ڴ�ӡʶ���� 		
	}
	myfree(SRAMDTCM,bmp);		//�ͷŻҶ�ͼbmp�ڴ�
	myfree(SRAMIN,result);	//�ͷ�ʶ����	
}  


