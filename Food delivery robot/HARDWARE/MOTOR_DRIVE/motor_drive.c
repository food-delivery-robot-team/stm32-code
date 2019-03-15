#include "motor_drive.h"

void motor_drive_Init(void)
{
    GPIO_InitTypeDef GPIO_Initure;
    __HAL_RCC_GPIOB_CLK_ENABLE();			//开启GPIOB时钟	
	__HAL_RCC_GPIOE_CLK_ENABLE();			//开启GPIOB时钟	
	
    GPIO_Initure.Pin=GPIO_PIN_13|GPIO_PIN_14; //PB0,1
    GPIO_Initure.Mode=GPIO_MODE_OUTPUT_PP;  //推挽输出
    GPIO_Initure.Pull=GPIO_PULLUP;          //上拉
    GPIO_Initure.Speed=GPIO_SPEED_HIGH;     //高速
    HAL_GPIO_Init(GPIOB,&GPIO_Initure);
	
	GPIO_Initure.Pin=GPIO_PIN_2|GPIO_PIN_4; //PB0,1
    GPIO_Initure.Mode=GPIO_MODE_OUTPUT_PP;  //推挽输出
    GPIO_Initure.Pull=GPIO_PULLUP;          //上拉
    GPIO_Initure.Speed=GPIO_SPEED_HIGH;     //高速
    HAL_GPIO_Init(GPIOE,&GPIO_Initure);
	
//    HAL_GPIO_WritePin(GPIOB,GPIO_PIN_13,GPIO_PIN_SET);	//PB0置0
//	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_14,GPIO_PIN_SET);	//PB1置1 
//	
//	HAL_GPIO_WritePin(GPIOE,GPIO_PIN_2,GPIO_PIN_SET);	//PB0置0
//	HAL_GPIO_WritePin(GPIOE,GPIO_PIN_4,GPIO_PIN_SET);	//PB1置1 
	Left_FR(0);
	Right_FR(1);
	Left_BK(1);
	Right_BK(1);

}

