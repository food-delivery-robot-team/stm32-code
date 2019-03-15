#include "sys.h"
#include "delay.h"
#include "usart.h"
#include "led.h"
#include "timer.h"
#include "motor_drive.h"
#include "crc.h"
 
 
/************************************************
 ALIENTEKս��STM32������ʵ��1
 �����ʵ�� 
 ����֧�֣�www.openedv.com
 �Ա����̣�http://eboard.taobao.com 
 ��ע΢�Ź���ƽ̨΢�źţ�"����ԭ��"����ѻ�ȡSTM32���ϡ�
 ������������ӿƼ����޹�˾  
 ���ߣ�����ԭ�� @ALIENTEK
************************************************/

u8 receive_s_reply[8]={0x20,0x19,0x0a,0x00,0x00,0x00,0x19,0x61};
u8 receive_e_reply[8]={0x20,0x19,0x0b,0x00,0x00,0x00,0x18,0x9d};

 int main(void)
 {	
	u16 t;  
	u16 len;
    u8 wheel_flag;
	unsigned short crc_check;

	delay_init();	    //��ʱ������ʼ��	  
	motor_drive_Init();		  	//��ʼ����LED���ӵ�Ӳ���ӿ�
	uart_init(115200);	 //���ڳ�ʼ��Ϊ115200
	TIM3_PWM_Init(899,39);
	
	while(1)
	{
		TIM_SetCompare3(TIM3,799);
		TIM_SetCompare2(TIM3,799);
		
		Right_BK=1;
		Left_BK=1;

		
		
//		if(USART_RX_STA&0x8000)
//		{					   
//			len=USART_RX_STA&0x3fff;//�õ��˴ν��յ������ݳ���
//			
//			crc_check=CRC16((uint8_t*)USART_RX_BUF,len-2);
//			
//			if((USART_RX_BUF[len-1]==(crc_check>>8))&&(USART_RX_BUF[len-2]==(crc_check&0X00FF)))
//			{
//				for(t=0;t<8;t++)
//				{
//					USART_SendData(USART1, receive_s_reply[t]);//�򴮿�1��������
//					while(USART_GetFlagStatus(USART1,USART_FLAG_TC)!=SET);//�ȴ����ͽ���
//				}
//				
//				if(USART_RX_BUF[5]==0x00) wheel_flag=0;
//				if(USART_RX_BUF[5]==0x01) wheel_flag=1;
//				if(USART_RX_BUF[5]==0x02) wheel_flag=2;
//					
//				if(wheel_flag==0)
//				{
//					switch(USART_RX_BUF[2])
//					{
//						case 0x00:
//							 Left_X1=Left_X2=Left_X3=0;
//							 Right_X1=Right_X2=Right_X3=0;
//							 break;
//						case 0x01:
//							 Left_X1=Left_X2=0;
//							 Left_X3=1;
//							 Right_X1=Right_X2=0;
//							 Right_X3=1;
//							 break;
//						case 0x02:
//							 Left_X1=Left_X3=0;
//							 Left_X2=1;
//							 Right_X1=Right_X3=0;
//							 Right_X2=1;
//							 break;
//						case 0x03:
//							 Left_X2=Left_X3=1;
//							 Left_X1=0;
//							 Right_X2=Right_X3=1;
//							 Right_X2=0;
//							 break;
//						case 0x04:
//							 Left_X2=Left_X3=0;
//							 Left_X1=1;
//							 Right_X2=Right_X3=0;
//							 Right_X2=1;
//							 break;
//						case 0x05:
//							 Left_X1=Left_X3=1;
//							 Left_X1=0;
//							 Right_X1=Right_X3=1;
//							 Right_X2=0;
//							 break;
//						case 0x06:
//							 Left_X1=Left_X2=1;
//							 Left_X3=0;
//							 Right_X1=Right_X2=1;
//							 Right_X3=0;
//							 break;
//						case 0x07:
//							 Left_X1=Left_X2=Left_X3=1;
//							 Right_X1=Right_X2=Right_X3=1;
//							 break;
//					}
////					if(USART_RX_BUF[3]==0x00) Left_FR=0;Right_FR=1;
////					if(USART_RX_BUF[3]==0x01) Left_FR=1;Right_FR=0;
//					
//					if(USART_RX_BUF[4]==0x00) Left_BK=Right_BK=1;
//					if(USART_RX_BUF[4]==0x01) Left_BK=Right_BK=0;
//				}
//				if(wheel_flag==1)
//				{
//					switch(USART_RX_BUF[2])
//					{
//						case 0x00:
//							 Left_X1=Left_X2=Left_X3=0;
//							 break;
//						case 0x01:
//							 Left_X1=Left_X2=0;
//							 Left_X3=1;
//							 break;
//						case 0x02:
//							 Left_X1=Left_X3=0;
//							 Left_X2=1;
//							 break;
//						case 0x03:
//							 Left_X2=Left_X3=1;
//							 Left_X1=0;
//							 break;
//						case 0x04:
//							 Left_X2=Left_X3=0;
//							 Left_X1=1;
//							 break;
//						case 0x05:
//							 Left_X1=Left_X3=1;
//							 Left_X1=0;
//							 break;
//						case 0x06:
//							 Left_X1=Left_X2=1;
//							 Left_X3=0;
//							 break;
//						case 0x07:
//							 Left_X1=Left_X2=Left_X3=1;
//							 break;
//					}
//					if(USART_RX_BUF[3]==0x00) Left_FR=0;
//					if(USART_RX_BUF[3]==0x01) Left_FR=1;
//					
//					if(USART_RX_BUF[4]==0x00) Left_BK=1;
//					if(USART_RX_BUF[4]==0x01) Left_BK=0;
//				}
//				if(wheel_flag==2)
//				{
//					switch(USART_RX_BUF[2])
//					{
//						case 0x00:
//							 Right_X1=Right_X2=Right_X3=0;
//							 break;
//						case 0x01:
//							 Right_X1=Right_X2=0;
//							 Right_X3=1;
//							 break;
//						case 0x02:
//							 Right_X1=Right_X3=0;
//							 Right_X2=1;
//							 break;
//						case 0x03:
//							 Right_X2=Right_X3=1;
//							 Right_X2=0;
//							 break;
//						case 0x04:
//							 Right_X2=Right_X3=0;
//							 Right_X2=1;
//							 break;
//						case 0x05:
//							 Right_X1=Right_X3=1;
//							 Right_X2=0;
//							 break;
//						case 0x06:
//							 Right_X1=Right_X2=1;
//							 Right_X3=0;
//							 break;
//						case 0x07:
//							 Right_X1=Right_X2=Right_X3=1;
//							 break;
//					}
//					if(USART_RX_BUF[3]==0x00) Right_FR=1;
//					if(USART_RX_BUF[3]==0x01) Right_FR=0;
//					
//					if(USART_RX_BUF[4]==0x00) Right_BK=1;
//					if(USART_RX_BUF[4]==0x01) Right_BK=0;
//				}
//				
//			}
//			else
//			{
//				for(t=0;t<8;t++)
//				{
//					USART_SendData(USART1, receive_e_reply[t]);//�򴮿�1��������
//					while(USART_GetFlagStatus(USART1,USART_FLAG_TC)!=SET);//�ȴ����ͽ���
//				}
//			}
//			
//			USART_RX_STA=0;
//		}
	}
 }

