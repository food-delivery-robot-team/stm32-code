#include "usart.h"
#include "lidar.h"
#include "math.h"
#include "lcd.h"
////////////////////////////////////////////////////////////////////////////////// 	 
//如果使用os,则包括下面的头文件即可.
#if SYSTEM_SUPPORT_OS
#include "includes.h"					//os 使用	  
#endif
//////////////////////////////////////////////////////////////////////////////////	 
//本程序只供学习使用，未经作者许可，不得用于其它任何用途
//ALIENTEK STM32F7开发板
//串口1初始化		   
//正点原子@ALIENTEK
//技术论坛:www.openedv.com
//修改日期:2015/6/23
//版本：V1.5
//版权所有，盗版必究。
//Copyright(C) 广州市星翼电子科技有限公司 2009-2019
//All rights reserved
//********************************************************************************
//V1.0修改说明 
////////////////////////////////////////////////////////////////////////////////// 	  
//加入以下代码,支持printf函数,而不需要选择use MicroLIB	  
//#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)	
#if 1
#pragma import(__use_no_semihosting)             
//标准库需要的支持函数                 
struct __FILE 
{ 
	int handle; 
}; 

FILE __stdout;       
//定义_sys_exit()以避免使用半主机模式    
void _sys_exit(int x) 
{ 
	x = x; 
} 
//重定义fputc函数 
int fputc(int ch, FILE *f)
{ 	
	while((USART1->ISR&0X40)==0);//循环发送,直到发送完毕   
	USART1->TDR=(u8)ch;      
	return ch;
}
#endif 

#if EN_USART1_RX   //如果使能了接收
//串口1中断服务程序
//注意,读取USARTx->SR能避免莫名其妙的错误   	
u8 USART_RX_BUF[USART_REC_LEN];     //接收缓冲,最大USART_REC_LEN个字节.
u8 USART3_RX_BUF[USART3_MAX_RECV_LEN]; 				//接收缓冲,最大USART3_MAX_RECV_LEN个字节.

//串口发送缓存区 	
__align(8) u8 USART3_TX_BUF[USART3_MAX_SEND_LEN]; 	//发送缓冲,最大USART3_MAX_SEND_LEN字节  	  

//接收状态
//bit15，	接收完成标志
//bit14，	接收到0x0d
//bit13~0，	接收到的有效字节数目
u16 USART_RX_STA=0;       //接收状态标记	
u16 USART3_RX_STA=0;       //接收状态标记	

u8 aRxBuffer[RXBUFFERSIZE];//HAL库使用的串口接收缓冲
UART_HandleTypeDef UART1_Handler; //UART句柄
UART_HandleTypeDef UART3_Handler; //UART句柄

extern u16 USART_RX_STORAGE;
extern u8 USART_RX_FLAG;

//初始化IO 串口1 
//bound:波特率
void uart_init(u32 bound)
{	
	//UART 初始化设置
	UART1_Handler.Instance=USART1;					    //USART1
	UART1_Handler.Init.BaudRate=bound;				    //波特率
	UART1_Handler.Init.WordLength=UART_WORDLENGTH_8B;   //字长为8位数据格式
	UART1_Handler.Init.StopBits=UART_STOPBITS_1;	    //一个停止位
	UART1_Handler.Init.Parity=UART_PARITY_NONE;		    //无奇偶校验位
	UART1_Handler.Init.HwFlowCtl=UART_HWCONTROL_NONE;   //无硬件流控
	UART1_Handler.Init.Mode=UART_MODE_TX_RX;		    //收发模式
	HAL_UART_Init(&UART1_Handler);					    //HAL_UART_Init()会使能UART1
	
//	HAL_UART_Receive_IT(&UART1_Handler, (u8 *)aRxBuffer, RXBUFFERSIZE);//该函数会开启接收中断：标志位UART_IT_RXNE，并且设置接收缓冲以及接收缓冲接收最大数据量
  
}

 //初始化IO 串口3 
//bound:波特率
void usart3_init(u32 bound)
{	
	//UART 初始化设置
	UART3_Handler.Instance=USART3;					    //USART3
	UART3_Handler.Init.BaudRate=bound;				    //波特率
	UART3_Handler.Init.WordLength=UART_WORDLENGTH_8B;   //字长为8位数据格式
	UART3_Handler.Init.StopBits=UART_STOPBITS_1;	    //一个停止位
	UART3_Handler.Init.Parity=UART_PARITY_NONE;		    //无奇偶校验位
	UART3_Handler.Init.HwFlowCtl=UART_HWCONTROL_NONE;   //无硬件流控
	UART3_Handler.Init.Mode=UART_MODE_TX_RX;		    //收发模式
	HAL_UART_Init(&UART3_Handler);					    //HAL_UART_Init()会使能UART1
}

//串口3,printf 函数
//确保一次发送数据不超过USART3_MAX_SEND_LEN字节
void u3_printf(char* fmt,...)  
{  
	u16 i,j; 
	va_list ap; 
	va_start(ap,fmt);
	vsprintf((char*)USART3_TX_BUF,fmt,ap);
	va_end(ap);
	i=strlen((const char*)USART3_TX_BUF);		//此次发送数据的长度
	for(j=0;j<i;j++)							//循环发送数据
	{
		while((USART3->ISR&0X40)==0);			//循环发送,直到发送完毕   
		USART3->TDR=USART3_TX_BUF[j];  
	} 
}

void USART3_IRQHandler(void)
{
	u8 res;	    
#if SYSTEM_SUPPORT_OS	 	//使用OS
	OSIntEnter();    
#endif	
	if(__HAL_UART_GET_FLAG(&UART3_Handler,UART_FLAG_RXNE)!=RESET)//接收到数据
	{	 
//		HAL_UART_Receive(&UART3_Handler,&res,1,1000);
		res=USART3->RDR; 			 
		if((USART3_RX_STA&(1<<15))==0)//接收完的一批数据,还没有被处理,则不再接收其他数据
		{ 
			if(USART3_RX_STA<USART3_MAX_RECV_LEN)	//还可以接收数据
			{
//				__HAL_TIM_SetCounter(&TIM7_Handler,0);	
				TIM7->CNT=0;         				//计数器清空	
				if(USART3_RX_STA==0) 				//使能定时器7的中断 
				{
//					__HAL_RCC_TIM7_CLK_ENABLE();            //使能TIM7时钟
					TIM7->CR1|=1<<0;     			//使能定时器7
				}
				USART3_RX_BUF[USART3_RX_STA++]=res;	//记录接收到的值	 
			}else 
			{
				USART3_RX_STA|=1<<15;				//强制标记接收完成
			} 
		}
	} 
#if SYSTEM_SUPPORT_OS	 	//使用OS
	OSIntExit();  											 
#endif	
}   

//UART底层初始化，时钟使能，引脚配置，中断配置
//此函数会被HAL_UART_Init()调用
//huart:串口句柄

void HAL_UART_MspInit(UART_HandleTypeDef *huart)
{
    //GPIO端口设置
	GPIO_InitTypeDef GPIO_Initure;
	
	if(huart->Instance==USART1)//如果是串口1，进行串口1 MSP初始化
	{
		__HAL_RCC_GPIOA_CLK_ENABLE();			//使能GPIOA时钟
		__HAL_RCC_USART1_CLK_ENABLE();			//使能USART1时钟
	
		GPIO_Initure.Pin=GPIO_PIN_9;			//PA9
		GPIO_Initure.Mode=GPIO_MODE_AF_PP;		//复用推挽输出
		GPIO_Initure.Pull=GPIO_PULLUP;			//上拉
		GPIO_Initure.Speed=GPIO_SPEED_FAST;		//高速
		GPIO_Initure.Alternate=GPIO_AF7_USART1;	//复用为USART1
		HAL_GPIO_Init(GPIOA,&GPIO_Initure);	   	//初始化PA9

		GPIO_Initure.Pin=GPIO_PIN_10;			//PA10
		HAL_GPIO_Init(GPIOA,&GPIO_Initure);	   	//初始化PA10
		
#if EN_USART1_RX
		__HAL_UART_ENABLE_IT(huart,UART_IT_RXNE);		//开启接收中断
		HAL_NVIC_EnableIRQ(USART1_IRQn);				//使能USART1中断通道
		HAL_NVIC_SetPriority(USART1_IRQn,3,3);			//抢占优先级3，子优先级3
#endif	
	}
	
	if(huart==(&UART3_Handler))
	{
		  //GPIO端口设置
		GPIO_InitTypeDef GPIO_Initure;
	
		__HAL_RCC_GPIOB_CLK_ENABLE();			//使能GPIOB时钟
		__HAL_RCC_USART3_CLK_ENABLE();			//使能USART3时钟
	
		GPIO_Initure.Pin=GPIO_PIN_10;			//PB10
		GPIO_Initure.Mode=GPIO_MODE_AF_PP;		//复用推挽输出
		GPIO_Initure.Pull=GPIO_PULLUP;			//上拉
		GPIO_Initure.Speed=GPIO_SPEED_FAST;		//高速
		GPIO_Initure.Alternate=GPIO_AF7_USART3;	//复用为USART3
		HAL_GPIO_Init(GPIOB,&GPIO_Initure);	   	//初始化PB10

		GPIO_Initure.Pin=GPIO_PIN_11;			//PB11
		HAL_GPIO_Init(GPIOB,&GPIO_Initure);	   	//初始化PB11
	
//		__HAL_UART_DISABLE_IT(huart,UART_IT_TC);
		__HAL_UART_ENABLE_IT(huart,UART_IT_RXNE);		//开启接收中断
		HAL_NVIC_EnableIRQ(USART3_IRQn);				//使能USART3中断
		HAL_NVIC_SetPriority(USART3_IRQn,2,3);			//抢占优先级2，子优先级3	
		TIM7_Int_Init(1000-1,9000-1);		//100ms中断
		USART3_RX_STA=0;		//清零
		TIM7->CR1&=~(1<<0);        //关闭定时器7
	}

}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
//	if(huart->Instance==USART1)//如果是串口1
//	{
//		if((USART_RX_STA&0x8000)==0)//接收未完成
//		{
////			if(USART_RX_STA&0x4000)//接收到了0x0d
////			{
////				if(aRxBuffer[0]!=0x0a)USART_RX_STA=0;//接收错误,重新开始
////				else USART_RX_STA|=0x8000;	//接收完成了 
////			}
////			else //还没收到0X0D
////			{	  
////				if(aRxBuffer[0]==0x0d)USART_RX_STA|=0x4000;
////				else
////				{
//					USART_RX_BUF[USART_RX_STA&0X3FFF]=aRxBuffer[0] ;
//					USART_RX_STA++;
//			        USART_RX_STORAGE=USART_RX_STA;
//					if(USART_RX_STA>(USART_REC_LEN-1))
//					{
//						USART_RX_STA=0;//接收数据错误,重新开始接收	  
//						USART_RX_STORAGE=0;
//						USART_RX_FLAG=0;
//					}
////				}		 
////			}
//		}

//	}
	if(huart->Instance==USART1)//如果是串口1
	{
		if((USART_RX_STA&0x8000)==0)//接收未完成
		{
			if(USART_RX_STA&0x4000)//接收到了0x0d
			{
				if(aRxBuffer[0]!=0x0a)USART_RX_STA=0;//接收错误,重新开始
				else USART_RX_STA|=0x8000;	//接收完成了 
			}
			else //还没收到0X0D
			{	
				if(aRxBuffer[0]==0x0d)USART_RX_STA|=0x4000;
				else
				{
					USART_RX_BUF[USART_RX_STA&0X3FFF]=aRxBuffer[0] ;
					USART_RX_STA++;
					if(USART_RX_STA>(USART_REC_LEN-1))USART_RX_STA=0;//接收数据错误,重新开始接收	  
				}		 
			}
		}

	}
}
 
extern u8 lidar_i,lidar_LSN,lidar_b,lidar_n,lidar_c,lidar_a,lidar_m,Lidar_Data[150];

extern float AngleFSA,AngleLSA,Anglei[100],Lidar_Distance[100],Lidar_x[100],Lidar_y[100]; ; //初始角,结束角,中间角,距离
u8 Lidar_receive_flag;

//串口1中断服务程序
//void USART1_IRQHandler(void)                	
//{ 
//	u32 timeout=0;
//    u32 maxDelay=0x1FFFF;
////#if SYSTEM_SUPPORT_OS	 	//使用OS
////	OSIntEnter();    
////#endif
//	
//	USART_RX_FLAG=1;
//	
//	HAL_UART_IRQHandler(&UART1_Handler);	//调用HAL库中断处理公用函数
//	
//	timeout=0;
//	
//	if((__HAL_UART_GET_FLAG(&UART1_Handler,UART_FLAG_ORE)!=RESET)) //清除ORE
//	{
//		HAL_UART_Receive_IT(&UART1_Handler,(u8 *)aRxBuffer,RXBUFFERSIZE);
//		__HAL_UART_CLEAR_OREFLAG(&UART1_Handler);
//	}
//	
//    while (HAL_UART_GetState(&UART1_Handler)!=HAL_UART_STATE_READY)//等待就绪
//	{
//        timeout++;////超时处理
//        if(timeout>maxDelay) break;		
//	}
//     
//	timeout=0;
//	while(HAL_UART_Receive_IT(&UART1_Handler,(u8 *)aRxBuffer, RXBUFFERSIZE)!=HAL_OK)//一次处理完成之后，重新开启中断并设置RxXferCount为1
//	{
//        timeout++; //超时处理
//        if(timeout>maxDelay) break;	
//	}
//	
////	HAL_UART_Receive(&UART1_Handler,(u8 *)aRxBuffer,RXBUFFERSIZE,10);
//	HAL_UART_Receive_IT(&UART1_Handler,(u8 *)aRxBuffer,RXBUFFERSIZE);

//	if(*aRxBuffer==0xAA)
//	{		
//	   lidar_a=0;
//	   lidar_b=0;
//	   lidar_n=0;
//	   lidar_i=1;
//	   lidar_c=0;
//	}
//	Lidar_Data[lidar_a]=*aRxBuffer;
//	
//	AngleFSA_Cal();
//	AngleLSA_Cal();
//	Anglei_Cal();
//	Distance_Cal();
//	Adumbrate();
//			
////		evade();
//			
//	lidar_a++;			
//			
//	
//	Lidar_receive_flag=1;
//	
////#if SYSTEM_SUPPORT_OS	 	//使用OS
////	OSIntExit();  											 
////#endif
//} 
//#endif	

/*下面代码我们直接把中断控制逻辑写在中断服务函数内部。*/
///*

//串口1中断服务程序
void USART1_IRQHandler(void)                	
{ 
	u8 Res;
//#if SYSTEM_SUPPORT_OS	 	//使用OS
//	OSIntEnter();    
//#endif
	u8 i;
	if((__HAL_UART_GET_FLAG(&UART1_Handler,UART_FLAG_ORE)!=RESET)) //清除ORE
	{
		HAL_UART_Receive(&UART1_Handler,&Res,1,1000); 
		__HAL_UART_CLEAR_OREFLAG(&UART1_Handler);
	}
//	if((__HAL_UART_GET_FLAG(&UART1_Handler,UART_FLAG_PE)!=RESET)) //清除ORE
//	{
//		HAL_UART_Receive(&UART1_Handler,&Res,1,1000); 
//		__HAL_UART_CLEAR_PEFLAG(&UART1_Handler);
//	}
//	if((__HAL_UART_GET_FLAG(&UART1_Handler,UART_FLAG_FE)!=RESET)) //清除ORE
//	{
//		HAL_UART_Receive(&UART1_Handler,&Res,1,1000); 
//		__HAL_UART_CLEAR_FEFLAG(&UART1_Handler);
//	}
	if((__HAL_UART_GET_FLAG(&UART1_Handler,UART_FLAG_RXNE)!=RESET))  //接收中断(接收到的数据必须是0x0d 0x0a结尾)
	{
        HAL_UART_Receive(&UART1_Handler,&Res,1,1000); 
		__HAL_UART_CLEAR_IT(&UART1_Handler,UART_FLAG_RXNE);

		if((USART_RX_STA&0x8000)==0)//接收未完成
		{
				
				if(Res==0xaa)
				{
					USART_RX_STA|=0x4000;
					lidar_a=0;
				    lidar_b=0;
				    lidar_n=0;
					lidar_i=1;
					lidar_c=0;
				}
				else
				{
					USART_RX_BUF[USART_RX_STA&0X3FFF]=Res ;
					USART_RX_STA++;
					if(USART_RX_STA>(USART_REC_LEN-1))USART_RX_STA=0;//接收数据错误,重新开始接收
					
					
				}
			
				Lidar_Data[lidar_a]=Res;
				AngleFSA_Cal();
				AngleLSA_Cal();
				Anglei_Cal();
				Distance_Cal();
				Adumbrate();
			
				lidar_a++;

				
			}
	
	}
	HAL_UART_IRQHandler(&UART1_Handler);	
//#if SYSTEM_SUPPORT_OS	 	//使用OS
//	OSIntExit();  											 
//#endif
} 
#endif	
//*/

 

 




