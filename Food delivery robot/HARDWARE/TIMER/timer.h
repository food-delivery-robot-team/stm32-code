#ifndef _TIMER_H
#define _TIMER_H
#include "sys.h"
//////////////////////////////////////////////////////////////////////////////////	 
//������ֻ��ѧϰʹ�ã�δ��������ɣ��������������κ���;
//ALIENTEK STM32F7������
//��ʱ����������	   
//����ԭ��@ALIENTEK
//������̳:www.openedv.com
//��������:2015/11/27
//�汾��V1.0
//��Ȩ���У�����ؾ���
//Copyright(C) ������������ӿƼ����޹�˾ 2014-2024
//All rights reserved									  
////////////////////////////////////////////////////////////////////////////////// 
extern TIM_HandleTypeDef TIM3_Handler;      //��ʱ��3PWM��� 
extern TIM_OC_InitTypeDef TIM3_CH4Handler;  //��ʱ��3ͨ��4���
extern TIM_HandleTypeDef TIM5_Handler;      //��ʱ��5���

void TIM3_Init(u16 arr,u16 psc);    //��ʱ����ʼ��
void TIM3_PWM_Init(u16 arr,u16 psc);
void TIM5_CH1_Cap_Init(u32 arr,u16 psc); 	//��ʼ��TIM5_CH1���벶��

void TIM_SetTIM3Compare4(u32 compare);
void TIM_SetTIM3Compare3(u32 compare);
#endif

