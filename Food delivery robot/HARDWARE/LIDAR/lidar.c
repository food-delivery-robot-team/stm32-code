#include "lidar.h"
#include "lcd.h"
#include "led.h"
#include "math.h"
//#include "motor.h"
#include "delay.h"


u8 lidar_a,lidar_m,Lidar_Data[150];

float AngleFSA,AngleLSA,Anglei[100],Lidar_Distance[100],Lidar_x[100],Lidar_y[100]; //³õÊ¼½Ç,½áÊø½Ç,ÖÐ¼ä½Ç,¾àÀë

u8 lidar_i=1,lidar_LSN,lidar_b,lidar_n,lidar_c;


void AngleFSA_Cal(void)   //³õÊ¼½Ç¼ÆËã
{
	u32 AngleFSAL,AngleFSAH;
		
	  if(lidar_a==5)
		{
			AngleFSAL=Lidar_Data[4];
			AngleFSAH=Lidar_Data[5];
			AngleFSA=(((AngleFSAH*256)+AngleFSAL)>>1)*0.015625;
		}
}

void AngleLSA_Cal(void)   //½áÊø½Ç¼ÆËã
{
  u32 AngleLSAL,AngleLSAH;
	
	  if(lidar_a==7)
		{
			AngleLSAL=Lidar_Data[6];
			AngleLSAH=Lidar_Data[7];
			AngleLSA=(((AngleLSAH*256)+AngleLSAL)>>1)*0.015625;
		}
}
		
void Distance_Cal(void)   //¾àÀë¼ÆËã
{ 
	  u32 DistanceH,DistanceL;
	  
		if((lidar_a>9)&&(lidar_a%2==1))
		{
			DistanceL=Lidar_Data[lidar_a-1];
			DistanceH=Lidar_Data[lidar_a];
			Lidar_Distance[lidar_b]=((DistanceH*256)+DistanceL)*0.25;
			lidar_b++;
		}
		

}

void Anglei_Cal(void)   //ÖÐ¼ä½Ç¼ÆËã
{ 
	if(lidar_a==3)
	{
		lidar_LSN=Lidar_Data[3];
	}
	
	if(lidar_LSN==1)
	{
	   Anglei[lidar_n]=AngleFSA;
	}
	
	if(lidar_LSN>1)
	{		
			if((lidar_a>7)&&(lidar_i<=lidar_LSN))
			{
				Anglei[lidar_n]=((AngleLSA-AngleFSA)/(lidar_LSN-1))*(lidar_i-1)+AngleFSA;
				lidar_n++;
				lidar_i++;
			}
    }
	
}

void Adumbrate(void)   //»­ÂÖÀª
{
  float val;
	
	val=PI/180;
	Lidar_x[lidar_c]=sin((360-Anglei[lidar_c])*val)*Lidar_Distance[lidar_c];
	Lidar_y[lidar_c]=cos((360-Anglei[lidar_c])*val)*Lidar_Distance[lidar_c];
  lidar_c++;
				
}	

void evade(void)
{
  float Get_Ang,Get_Dis;
	int h;
	
	if(lidar_a==0)
	{
		h=0;
	}
	
//	move_forward();
  if(lidar_a>9)
	{
			Get_Ang=Anglei[h];
			Get_Dis=Lidar_Distance[h];

			
			if((Get_Ang>0)&&(Get_Ang<30)&&(Get_Dis<400)&&(Get_Dis>0))
			{
//			  stop();
//				delay_ms(300);
//				move_backward();
//				delay_ms(300);
//				stop();
//				delay_ms(300);
//			  turn_left();
//				delay_ms(300);
//				move_forward();
			}
			else if((Get_Ang>330)&&(Get_Ang<360)&&(Get_Dis<400)&&(Get_Dis>0))
			{
//			  stop();
//				delay_ms(300);
//				move_backward();
//				delay_ms(300);
//				stop();
//				delay_ms(300);
//			  turn_right();
//				delay_ms(300);
//				move_forward();
			}
			else if((Get_Ang>40)&&(Get_Ang<70)&&(Get_Dis<400)&&(Get_Dis>0))
			{
//				turn_left();
//				delay_ms(300);
//				move_forward();

			}
			
			else if((Get_Ang<320)&&(Get_Ang>290)&&(Get_Dis<400)&&(Get_Dis>0))
			{
//				turn_right();
//				delay_ms(300);
//				move_forward();

			}
			else 
			{
//			  move_forward();	  
			}	
			
			h++;
  }
	
		
//	if((Get_Ang>0)&&(Get_Ang<30)&&(Get_Dis>200))
//	{
//	  move_forward();
//	}
//	if((Get_Ang<360)&&(Get_Ang>330)&&(Get_Dis>200))
//	{
//	  move_forward();
//	}
	
}


