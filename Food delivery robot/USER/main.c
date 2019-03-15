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
#define interface_STK_SIZE 		1024
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
#define motor_drive_TASK_PRIO		5
//�����ջ��С	
#define motor_drive_STK_SIZE 		256
//������ƿ�
OS_TCB motor_driveTaskTCB;
//�����ջ	
CPU_STK motor_drive_TASK_STK[motor_drive_STK_SIZE];
void motor_drive_task(void *p_arg);


//�������ȼ�
#define ultrasonic_TASK_PRIO		5
//�����ջ��С	
#define ultrasonic_STK_SIZE 		256
//������ƿ�
OS_TCB ultrasonicTaskTCB;
//�����ջ	
CPU_STK ultrasonic_TASK_STK[ultrasonic_STK_SIZE];
void ultrasonic_task(void *p_arg);

//�������ȼ�
#define AGV_guide_TASK_PRIO		4
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
#define lidar_STK_SIZE       256
//������ƿ�
OS_TCB lidarTaskTCB;
//�����ջ	
CPU_STK lidar_TASK_STK[lidar_STK_SIZE];
//������
void lidar_task(void *p_arg);

//�������ȼ�
#define RFID_TASK_PRIO		4
//�����ջ��С	
#define RFID_STK_SIZE       128
//������ƿ�
OS_TCB RFIDTaskTCB;
//�����ջ	
CPU_STK RFID_TASK_STK[RFID_STK_SIZE];
//������
void RFID_task(void *p_arg);

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
	uart_init(115200);		        //���ڳ�ʼ��
	usmart_dev.init(108); 		    //��ʼ��USMART	
    LED_Init();                     //��ʼ��LED
	KEY_Init();                     //��ʼ������
	SDRAM_Init();                   //��ʼ��SDRAM
	LCD_Init();                     //��ʼ��LCD
	RS485_Init(115200);		        //��ʼ��RS485
	ultrasonic_init();              //��ʼ��������������
	motor_drive_Init();
	W25QXX_Init();				   				//��ʼ��W25Q256
	PCF8574_Init();									//��ʼ��PCF8574
	OV5640_Init();									//��ʼ��OV5640
	tp_dev.init();				    //��ʼ��������
	my_mem_init(SRAMIN);            //��ʼ���ڲ��ڴ��
	my_mem_init(SRAMEX);            //��ʼ���ⲿSDRAM�ڴ��
	my_mem_init(SRAMDTCM);          //��ʼ���ڲ�DTCM�ڴ��
//	TIM3_Init(9,108-1);             //108M/108=1M�ļ���Ƶ�ʣ��Զ���װ��Ϊ100����ôPWMƵ��Ϊ1M/10=100khz
	TIM5_CH1_Cap_Init(0XFFFFFFFF,108-1); //��1MHZ��Ƶ�ʼ���
	TIM3_PWM_Init(500-1,108-1);     //108M/108=1M�ļ���Ƶ�ʣ��Զ���װ��Ϊ500����ôPWMƵ��Ϊ1M/500=2kHZ
	
	TIM_SetTIM3Compare4(399);	//�޸ıȽ�ֵ���޸�ռ�ձ�
	TIM_SetTIM3Compare3(399);	//�޸ıȽ�ֵ���޸�ռ�ձ�
	
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
	
	OS_ERR err;
	p_arg = p_arg;
	
	OSTaskSuspend((OS_TCB*)&ov5640TaskTCB,&err);
	OSTaskSuspend((OS_TCB*)&RFIDTaskTCB,&err);
	
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
				else OSTimeDlyHMSM(0,0,0,10,OS_OPT_TIME_HMSM_STRICT,&err);	//û�а������µ�ʱ�� 			   
			}
			
		}else OSTimeDlyHMSM(0,0,0,10,OS_OPT_TIME_HMSM_STRICT,&err);	//û�а������µ�ʱ�� 
	}
}

//interface������
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
			LCD_Clear(WHITE);//���� 
			LCD_DrawRectangle(180, 20, 280, 80);
			LCD_ShowString(195,30,200,16,32,(u8*)"EXIT");		
			
		}
		else
		{
			LCD_Clear(WHITE);//���� 
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

//ov56400������
void ov5640_task(void *p_arg)
{
							 
 	u8 key;						   
	u8 i;
	
	OS_ERR err;
	p_arg = p_arg;
	

//		while(1)
//		{
//			key=KEY_Scan(0);//��֧������
//			if(key)
//			{ 
//				OV5640_Focus_Single();  //��KEY0��KEY1��KEYUP�ֶ������Զ��Խ�
//				
//				if(key==KEY2_PRES)break;//��KEY2����ʶ��
//			} 
//			if(readok==1)			//�ɼ�����һ֡ͼ��
//			{		
//				readok=0;
//				qr_show_image((lcddev.width-qr_image_width)/2,(lcddev.height-qr_image_width)/2,qr_image_width,qr_image_width,rgb_data_buf);
//				qr_decode(qr_image_width,rgb_data_buf);
//			}
//			i++;
//			if(i==20)//DS0��˸.
//			{
//				i=0;
//				LED0_Toggle;
//			}
////			OSTimeDlyHMSM(0,0,0,20,OS_OPT_TIME_HMSM_STRICT,&err); //��ʱ200ms
//		}
//		atk_qr_destroy();//�ͷ��㷨�ڴ�
//		printf("3SRAM IN:%d\r\n",my_mem_perused(SRAMIN));
//		printf("3SRAM EX:%d\r\n",my_mem_perused(SRAMEX));
//		printf("3SRAM DCTM:%d\r\n",my_mem_perused(SRAMDTCM)); 
		while(1)
		{
			if(tp_dev.sta&TP_PRES_DOWN)			//������������
			{
				seconfary_menu=1;
			}
			OSTimeDlyHMSM(0,0,0,200,OS_OPT_TIME_HMSM_STRICT,&err); //��ʱ200ms
			if(readok==1)			//�ɼ�����һ֡ͼ��
			{		
				readok=0;
				qr_show_image((lcddev.width-qr_image_width)/2,(lcddev.height-qr_image_width)/2,qr_image_width,qr_image_width,rgb_data_buf);
				qr_decode(qr_image_width,rgb_data_buf);
			}
//			printf("1\r\n");
		}

}



//lidar������
void RFID_task(void *p_arg)
{
	u8 len,i;
	long num;
	u8 receive[50];
	OS_ERR err;
	p_arg = p_arg;
	
	while(1)
	{
		if(tp_dev.sta&TP_PRES_DOWN)			//������������
		{
			seconfary_menu=1;
		}
		LCD_ShowString(20,160,200,16,32,(u8*)"result:");
		if(USART_RX_STA&0x8000)
		{
			len=USART_RX_STA&0x3fff;//�õ��˴ν��յ������ݳ���
			for(i=0;i<len;i++)
			{
//				num=USART_RX_BUF[i]*pow(10,(len-i));
				receive[i]=USART_RX_BUF[i];
				LCD_ShowChar(20+i*20,200,receive[i],24,0);
			}

			USART_RX_STA=0;
		}else OSTimeDlyHMSM(0,0,0,200,OS_OPT_TIME_HMSM_STRICT,&err); //��ʱ200ms
		OSTimeDlyHMSM(0,0,0,200,OS_OPT_TIME_HMSM_STRICT,&err); //��ʱ200ms
	}
}


//lidar������
void lidar_task(void *p_arg)
{
	OS_ERR err;
	p_arg = p_arg;
	
	while(1)
	{
		
		if(tp_dev.sta&TP_PRES_DOWN)			//������������
		{
			seconfary_menu=1;
		}
		
		LCD_ShowString(20,160,200,16,32,(u8*)"lidar:");
		
		OSTimeDlyHMSM(0,0,0,200,OS_OPT_TIME_HMSM_STRICT,&err); //��ʱ200ms
		
	}
}


extern u8  TIM5CH1_CAPTURE_STA;		//���벶��״̬		    				
extern u32	TIM5CH1_CAPTURE_VAL;	//���벶��ֵ 

void ultrasonic_task(void *p_arg)
{
	long long temp=0;
	
	OS_ERR err;
	p_arg = p_arg;
	
	while(1)
	{
		if(TIM5CH1_CAPTURE_STA&0X80)        //�ɹ�������һ�θߵ�ƽ
		{
			temp=TIM5CH1_CAPTURE_STA&0X3F; 
			temp*=0XFFFFFFFF;		 	    //���ʱ���ܺ�
			temp+=TIM5CH1_CAPTURE_VAL;      //�õ��ܵĸߵ�ƽʱ��
//			printf("HIGH:%lld cm\r\n",temp/58);//��ӡ�ܵĸߵ�ƽʱ��
			TIM5CH1_CAPTURE_STA=0;          //������һ�β���
		}
		HAL_GPIO_WritePin(GPIOB,GPIO_PIN_15,GPIO_PIN_SET);
		delay_us(15);
		HAL_GPIO_WritePin(GPIOB,GPIO_PIN_15,GPIO_PIN_RESET);

		OSTimeDlyHMSM(0,0,0,100,OS_OPT_TIME_HMSM_STRICT,&err); //��ʱ100ms
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
			len=USART_RX_STA&0x3fff;//�õ��˴ν��յ������ݳ���
//			HAL_UART_Transmit(&UART1_Handler,(uint8_t*)USART_RX_BUF,len,1000);	//���ͽ��յ�������
//			while(__HAL_UART_GET_FLAG(&UART1_Handler,UART_FLAG_TC)!=SET);		//�ȴ����ͽ���
			crc_check=CRC16((uint8_t*)USART_RX_BUF,len-2);
			if((USART_RX_BUF[len-1]==(crc_check>>8))&&(USART_RX_BUF[len-2]==(crc_check&0X00FF)))
			{
				for(i=0;i<len;i++)
				{
					AGV_INF[i]=USART_RX_BUF[i];
				}
			}
//			printf("%x",crc_check&0x00FF);
//			printf("%x",USART_RX_BUF[len-1]);
//			printf("%x",USART_RX_BUF[len]);
			
			USART_RX_STA=0;
		}
		OSTimeDlyHMSM(0,0,0,10,OS_OPT_TIME_HMSM_STRICT,&err); //��ʱ100ms
			
		if(USART_RX_FLAG==1)
		{
			USART_RX_STORAGE++;
			usart_rx_ss=USART_RX_STORAGE-USART_RX_STA;
//			if(USART_RX_BUF[0]==0x20&&USART_RX_BUF[1]==0x19)
//			{
//				USART_RX_STA|=0x4000;
//				USART_RX_STORAGE=0;
//				USART_RX_FLAG=0;
//			}
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


//motor_drive������
void motor_drive_task(void *p_arg)
{
	u8 len,flag_485;
	u8 send_again,t;
	u16 AGV_feedback;

//	u8 receive_s_reply[8]={0x20,0x19,0x0a,0x00,0x00,0x00,0x19,0x61};
//	u8 receive_e_reply[8]={0x20,0x19,0x0b,0x00,0x00,0x00,0x18,0x9d};
//	u8 transmit_buf[10]={0x20,0x19,0x00,0x00,0x00,0x00,0x1a,0xb9,0x0d,0x0a};
//	u8 RX_485BUF[10]={0x20,0x19,0x00,0x00,0x00,0x00,0x1a,0xb9,0x0d,0x0a};
//	unsigned short crc_check;
	
	OS_ERR err;
	p_arg = p_arg;
	
	
	
	
	while(1)
	{
		Left_FR(0);
		Right_FR(1);
		Left_BK(1);
		Right_BK(1);
//		RS485_Receive_Data(RX_485BUF,&flag_485);
		
//		if(flag_485)//���յ�F1�ظ�
//		{	
////			len=USART_RX_STA&0x3fff;//�õ��˴ν��յ������ݳ���
//			crc_check=CRC16((uint8_t*)RX_485BUF,6);
//			if((RX_485BUF[7]==(crc_check>>8))&&(RX_485BUF[6]==(crc_check&0X00FF)))
//			{
//				if(RX_485BUF[2]==0x0a) send_again=0;
//				if(RX_485BUF[2]==0x0b) send_again=1;
//			}
//			if(send_again==1)
//			{
////				HAL_UART_Transmit(&UART1_Handler,(uint8_t*)transmit_buf,8,1000);	//���ͽ��յ�������
////				while(__HAL_UART_GET_FLAG(&UART1_Handler,UART_FLAG_TC)!=SET);		//�ȴ����ͽ���
//				RS485_Send_Data(transmit_buf,10);//����5���ֽ� 
//				send_again=0;
//			}

//		}
//		
//		if(!flag_485)
//		{
			AGV_feedback=AGV_INF[4];
			AGV_feedback=(AGV_feedback<<8)+AGV_INF[5];
//			printf("%x",AGV_feedback);
			if(((0x0128<AGV_feedback&&AGV_feedback<0x03c8)||(AGV_feedback==0x0108)||(AGV_feedback==0x0060))&&AGV_feedback!=0)
			{
				
				TIM_SetTIM3Compare4(399);	//�޸ıȽ�ֵ���޸�ռ�ձ���
				TIM_SetTIM3Compare3(399);	//�޸ıȽ�ֵ���޸�ռ�ձ���
				
//				transmit_buf[2]=0x00;transmit_buf[5]=0x00;
//				
//				crc_check=CRC16((uint8_t*)transmit_buf,6);
//				transmit_buf[6]=crc_check&0X00FF;
//				transmit_buf[7]=crc_check>>8;
//				
////				HAL_UART_Transmit(&UART1_Handler,(uint8_t*)transmit_buf,8,1000);	//���ͽ��յ�������
////				while(__HAL_UART_GET_FLAG(&UART1_Handler,UART_FLAG_TC)!=SET);		//�ȴ����ͽ���
//				RS485_Send_Data(transmit_buf,10);//����5���ֽ� 
			}
				
			if((AGV_feedback<0x0128)&&(AGV_feedback!=0x0108)&&(AGV_feedback!=0x0060)&&AGV_feedback!=0)
			{
				
				TIM_SetTIM3Compare4(299);	//�޸ıȽ�ֵ���޸�ռ�ձ�
				TIM_SetTIM3Compare3(399);	//�޸ıȽ�ֵ���޸�ռ�ձ�
//				transmit_buf[2]=0x01;transmit_buf[5]=0x01;
//				
//				crc_check=CRC16((uint8_t*)transmit_buf,6);
//				transmit_buf[6]=crc_check&0X00FF;
//				transmit_buf[7]=crc_check>>8;
//				
////				HAL_UART_Transmit(&UART1_Handler,(uint8_t*)transmit_buf,8,1000);	//���ͽ��յ�������
////				while(__HAL_UART_GET_FLAG(&UART1_Handler,UART_FLAG_TC)!=SET);		//�ȴ����ͽ���
//				RS485_Send_Data(transmit_buf,10);//����5���ֽ� 
				
			}
			if(AGV_feedback>0x03c8)
			{
				TIM_SetTIM3Compare4(399);	//�޸ıȽ�ֵ���޸�ռ�ձ�
				TIM_SetTIM3Compare3(299);	//�޸ıȽ�ֵ���޸�ռ�ձ�
//				transmit_buf[2]=0x01;transmit_buf[5]=0x02;
//				
//				crc_check=CRC16((uint8_t*)transmit_buf,6);
//				transmit_buf[6]=crc_check&0X00FF;
//				transmit_buf[7]=crc_check>>8;
//				
////				HAL_UART_Transmit(&UART1_Handler,(uint8_t*)transmit_buf,8,1000);	//���ͽ��յ�������
////				while(__HAL_UART_GET_FLAG(&UART1_Handler,UART_FLAG_TC)!=SET);		//�ȴ����ͽ���
//				RS485_Send_Data(transmit_buf,10);//����5���ֽ� 
			}
//		}
		
		
		OSTimeDlyHMSM(0,0,0,10,OS_OPT_TIME_HMSM_STRICT,&err); //��ʱ10ms
		
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
		bartype++;
		if(bartype>=5)bartype=0; 
	}
	else if(result[0]!=0)//ʶ������ˣ���ʾ���
	{	
		PCF8574_WriteBit(BEEP_IO,0);//�򿪷�����
		OSTimeDlyHMSM(0,0,0,100,OS_OPT_TIME_HMSM_STRICT,&err); //��ʱ200ms
		PCF8574_WriteBit(BEEP_IO,1);
		POINT_COLOR=BLUE; 
		LCD_Fill(0,(lcddev.height+qr_image_width)/2+20,lcddev.width,lcddev.height,BLACK);
		Show_Str(0,(lcddev.height+qr_image_width)/2+20,lcddev.width,
								(lcddev.height-qr_image_width)/2-20,(u8*)result,16,0							
						);//LCD��ʾʶ����
		printf("\r\nresult:\r\n%s\r\n",result);//���ڴ�ӡʶ���� 		
	}
	myfree(SRAMDTCM,bmp);		//�ͷŻҶ�ͼbmp�ڴ�
	myfree(SRAMIN,result);	//�ͷ�ʶ����	
}  


