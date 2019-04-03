#include "common.h"
#include "delay.h"
#include "usart.h"
#include "includes.h"
/////////////////////////////////////////////////////////////////////////////////////////////////////////// 
//本程序只供学习使用，未经作者许可，不得用于其它任何用途
//ALIENTEK STM32开发板
//ATK-ESP8266 WIFI模块 WIFI AP驱动代码	   
//正点原子@ALIENTEK
//技术论坛:www.openedv.com
//修改日期:2014/4/3
//版本：V1.0
//版权所有，盗版必究。
//Copyright(C) 广州市星翼电子科技有限公司 2009-2019
//All rights reserved									  
/////////////////////////////////////////////////////////////////////////////////////////////////////////// 


//ATK-ESP8266 WIFI AP测试
//用于测试TCP/UDP连接
//返回值:0,正常
//    其他,错误代码
extern u8 led0sta;



u8 atk_8266_wifiap_test(void)
{
	u8 netpro=0;	//网络模式
	u8 key,i;
	u8 timex=0; 
	u8 ipbuf[16]; 	//IP缓存
	u8 *p;
	u16 t=999;		//加速第一次获取链接状态
	u8 res=0;
	u16 rlen=0;
	u8 constate=0;	//连接状态
	

	
	p=mymalloc(SRAMIN,32);							//申请32字节内存
	
	
	
PRESTA:
	netpro=0;
	atk_8266_send_cmd("AT+CIPMUX=1","OK",20);   //0：单连接，1：多连接
	sprintf((char*)p,"AT+CIPSERVER=1,%s",(u8*)portnum);
	atk_8266_send_cmd(p,"OK",20);     //开启Server模式，端口号为8086
	atk_8266_get_wanip(ipbuf);//服务器模式,获取WAN IP
	
	USART3_RX_STA=0;
//	while(1)
//	{
//		key=KEY_Scan(0);
//		if(key==KEY0_PRES)
//		{
//			sprintf((char*)p,"%d\r\n",t);//测试数据
//			atk_8266_send_cmd("AT+CIPSEND=0,25","OK",200);  //发送指定长度的数据
//			delay_ms(200);
//			atk_8266_send_data(p,"OK",100);  //发送指定长度的数据
//		}
//		t++;
//		OSTimeDlyHMSM(0,0,0,5,OS_OPT_TIME_HMSM_STRICT,&err); //延时200ms
//		  
//		if(USART3_RX_STA&0X8000)		//接收到一次数据了
//		{ 
//			rlen=USART3_RX_STA&0X7FFF;	//得到本次接收到的数据长度
//			USART3_RX_BUF[rlen]=0;		//添加结束符 
//			for(i=0;i<rlen;i++)
//			{
//				printf("USART3_RX_BUF:%d\r\n",USART3_RX_BUF[i]);	//发送到串口   
//			}
//			printf("\r\n");	//发送到串口 
//			Show_Str(80,340,180,190,USART3_RX_BUF,12,0);//显示接收到的数据  
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
//		if(t==1000)//连续10秒钟没有收到任何数据,检查连接是不是还存在.
//		{
//			constate=atk_8266_consta_check();//得到连接状态
//			if(constate=='+')Show_Str(120,300,200,12,"连接成功",12,0);  //连接状态
//			else Show_Str(120,300,200,24,"连接失败",12,0); 	  	 
//			t=0;
//		}

//		atk_8266_at_response(1);
//	}
//	myfree(SRAMIN,p);		//释放内存 
//	return res;		
} 







