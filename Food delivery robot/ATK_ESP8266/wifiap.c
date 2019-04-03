#include "common.h"
#include "delay.h"
#include "usart.h"
#include "includes.h"
/////////////////////////////////////////////////////////////////////////////////////////////////////////// 
//������ֻ��ѧϰʹ�ã�δ��������ɣ��������������κ���;
//ALIENTEK STM32������
//ATK-ESP8266 WIFIģ�� WIFI AP��������	   
//����ԭ��@ALIENTEK
//������̳:www.openedv.com
//�޸�����:2014/4/3
//�汾��V1.0
//��Ȩ���У�����ؾ���
//Copyright(C) ������������ӿƼ����޹�˾ 2009-2019
//All rights reserved									  
/////////////////////////////////////////////////////////////////////////////////////////////////////////// 


//ATK-ESP8266 WIFI AP����
//���ڲ���TCP/UDP����
//����ֵ:0,����
//    ����,�������
extern u8 led0sta;



u8 atk_8266_wifiap_test(void)
{
	u8 netpro=0;	//����ģʽ
	u8 key,i;
	u8 timex=0; 
	u8 ipbuf[16]; 	//IP����
	u8 *p;
	u16 t=999;		//���ٵ�һ�λ�ȡ����״̬
	u8 res=0;
	u16 rlen=0;
	u8 constate=0;	//����״̬
	

	
	p=mymalloc(SRAMIN,32);							//����32�ֽ��ڴ�
	
	
	
PRESTA:
	netpro=0;
	atk_8266_send_cmd("AT+CIPMUX=1","OK",20);   //0�������ӣ�1��������
	sprintf((char*)p,"AT+CIPSERVER=1,%s",(u8*)portnum);
	atk_8266_send_cmd(p,"OK",20);     //����Serverģʽ���˿ں�Ϊ8086
	atk_8266_get_wanip(ipbuf);//������ģʽ,��ȡWAN IP
	
	USART3_RX_STA=0;
//	while(1)
//	{
//		key=KEY_Scan(0);
//		if(key==KEY0_PRES)
//		{
//			sprintf((char*)p,"%d\r\n",t);//��������
//			atk_8266_send_cmd("AT+CIPSEND=0,25","OK",200);  //����ָ�����ȵ�����
//			delay_ms(200);
//			atk_8266_send_data(p,"OK",100);  //����ָ�����ȵ�����
//		}
//		t++;
//		OSTimeDlyHMSM(0,0,0,5,OS_OPT_TIME_HMSM_STRICT,&err); //��ʱ200ms
//		  
//		if(USART3_RX_STA&0X8000)		//���յ�һ��������
//		{ 
//			rlen=USART3_RX_STA&0X7FFF;	//�õ����ν��յ������ݳ���
//			USART3_RX_BUF[rlen]=0;		//��ӽ����� 
//			for(i=0;i<rlen;i++)
//			{
//				printf("USART3_RX_BUF:%d\r\n",USART3_RX_BUF[i]);	//���͵�����   
//			}
//			printf("\r\n");	//���͵����� 
//			Show_Str(80,340,180,190,USART3_RX_BUF,12,0);//��ʾ���յ�������  
//			if(USART3_RX_BUF[11]==48&&USART3_RX_BUF[12]==49) 
//			{
//				SELECT_TABLE_RESULT=1;
//				SELECT_TABLE_flag=1;
//			}
//			if(USART3_RX_BUF[11]==48&&USART3_RX_BUF[12]==50) 
//			{
//				SELECT_TABLE_RESULT=2;
//				SELECT_TABLE_flag=1;
//			}
//			if(USART3_RX_BUF[11]==48&&USART3_RX_BUF[12]==51) 
//			{
//				SELECT_TABLE_RESULT=3;
//				SELECT_TABLE_flag=1;
//			}
//			if(USART3_RX_BUF[11]==0x0a&&USART3_RX_BUF[12]==0x0b) 
//			{
//				SELECT_TABLE_RESULT=10;
//				SELECT_TABLE_flag=1;
//			}
//			printf("SELECT_TABLE_flag %d SELECT_TABLE_RESULT %d\r\n",SELECT_TABLE_flag,SELECT_TABLE_RESULT);
//			USART3_RX_STA=0;
//			break;
//		}
//		
//		if(t==1000)//����10����û���յ��κ�����,��������ǲ��ǻ�����.
//		{
//			constate=atk_8266_consta_check();//�õ�����״̬
//			if(constate=='+')Show_Str(120,300,200,12,"���ӳɹ�",12,0);  //����״̬
//			else Show_Str(120,300,200,24,"����ʧ��",12,0); 	  	 
//			t=0;
//		}

//		atk_8266_at_response(1);
//	}
//	myfree(SRAMIN,p);		//�ͷ��ڴ� 
//	return res;		
} 







