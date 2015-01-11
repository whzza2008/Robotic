#include "main.h"


void GPIO_Config(void);
void Timer_Config(void);
void A_north_polar(int);
void A_south_polar(int);
void B_north_polar(void);
void B_south_polar(void);
void A_polar_off(void);
void B_polar_off(void);
//void TIM4_Config(void);
void PB_Config(void);
static void Mode_Button(void);
static void Increase_Button(void);
void GPIOC_Config(void);
static void Decrease_Button(void);
int dir_mode = 0;
int mode_button = 0;
int count =0;
int mode_state=1;
int state = 0;
int direct = 0;
int speed = 1310;
int decrease_button=0;
int increase_button=0;
char time[10];

int main(void){

	GPIO_Config();
	GPIOC_Config();
	Timer_Config();
	TIM_ITConfig(TIM4, TIM_IT_CC1, ENABLE);
  TIM_Cmd(TIM4, ENABLE);
	Mode_Button();
	Decrease_Button();
	Increase_Button();
	PB_Config();
	LCD_Init();
  LCD_LayerInit();
  LTDC_Cmd(ENABLE);
  LCD_SetLayer(LCD_FOREGROUND_LAYER);
	LCD_Clear(LCD_COLOR_WHITE);
	
	//STM_EVAL_PBInit(BUTTON_USER, BUTTON_MODE_EXTI);
	//GPIO_ResetBits(GPIOA, GPIO_Pin_5);//PinA-Enable
	GPIO_ResetBits(GPIOA, GPIO_Pin_9);//A 1
	GPIO_ResetBits(GPIOA, GPIO_Pin_10);//~A 2
	//GPIO_ResetBits(GPIOC, GPIO_Pin_3);//PinB-Enable
	GPIO_ResetBits(GPIOC, GPIO_Pin_8);//B 3
	GPIO_ResetBits(GPIOC, GPIO_Pin_11);//~B 4
	
	while(1){
		sprintf(time,"%d",count);
		LCD_DisplayStringLine(LINE(0),  (uint8_t *) time);
		sprintf(time,"%d",state);
		LCD_DisplayStringLine(LINE(1),  (uint8_t *) time);
		sprintf(time,"%d",dir_mode);
		LCD_DisplayStringLine(LINE(2),  (uint8_t *) time);
		sprintf(time,"%d",speed);
		LCD_DisplayStringLine(LINE(3),  (uint8_t *) time);
		//##############
		if(decrease_button==1){
			if(speed<300){speed = 1310;}
			speed = speed -100;
			decrease_button =0;
			LCD_ClearLine(LINE(3));
		}
		if(increase_button==1){
			speed = speed+100;
			increase_button = 0;
		}
		//#################
		if(mode_button ==1){mode_state=1;}
		else if(mode_button >1){mode_state=0;mode_button=0;}
		//##############
		if(dir_mode == 1){direct =1;}
		else if(dir_mode>1){direct = 0;dir_mode =0;}
		//##############
		//----------full step mode---------
		if(mode_state == 0){
			
			if (count>speed){
				count = 0;
				LCD_ClearLine(LINE(0));
				state++;
				if(state>3){state =0;}
				if(state == 0){
					A_north_polar(direct);
					B_north_polar();
				}
				else if(state == 1){
					A_south_polar(direct);
					B_north_polar();
				}
				else if(state == 2){
					A_south_polar(direct);
					B_south_polar();
				}
				else{
					A_north_polar(direct);
					B_south_polar();
				}
			}	
		}
	 //------------full step end---------
	 //------------half step mode-------
		else if(mode_state==1){
			if(count>(speed/2)){
				LCD_ClearLine(LINE(0));
				count = 0;
				state++;
				if(state>7){state = 0;}
				if(state == 0){
					A_north_polar(direct);
					B_north_polar();
				}
				else if(state == 1){
					A_polar_off();
					B_north_polar();
				}
				else if(state == 2){
					A_south_polar(direct);
					B_north_polar();
				}
				else if(state == 3){
					A_south_polar(direct);
					B_polar_off();
				}
				else if(state == 4){
					A_south_polar(direct);
					B_south_polar();
				}
				else if(state ==5){
					A_polar_off();
					B_south_polar();
				}
				else if(state == 6){
					A_north_polar(direct);
					B_south_polar();	
				}
				else if(state == 7){
					A_north_polar(direct);
					B_polar_off();
				}
			}
		}
	//-------------half step end -----------
	};
}
//----------main end--------
//------------------motor function-------
void A_north_polar(int direction){
	if(direction ==0){
		//GPIO_ResetBits(GPIOA, GPIO_Pin_5);
		GPIO_SetBits(GPIOA, GPIO_Pin_9);
		GPIO_ResetBits(GPIOA, GPIO_Pin_10);
		//GPIO_SetBits(GPIOA, GPIO_Pin_5);
	}
	else if(direction ==1){
		//GPIO_ResetBits(GPIOA, GPIO_Pin_5);
		GPIO_ResetBits(GPIOA, GPIO_Pin_9);
		GPIO_SetBits(GPIOA, GPIO_Pin_10);
		//GPIO_SetBits(GPIOA, GPIO_Pin_5);
	}
}

void A_south_polar(int direction){
	if(direction ==0){
		//GPIO_ResetBits(GPIOA, GPIO_Pin_5);
		GPIO_ResetBits(GPIOA, GPIO_Pin_9);
		GPIO_SetBits(GPIOA, GPIO_Pin_10);
		//GPIO_SetBits(GPIOA, GPIO_Pin_5);
	}
	else if(direction ==1){
		//GPIO_ResetBits(GPIOA, GPIO_Pin_5);
		GPIO_SetBits(GPIOA, GPIO_Pin_9);
		GPIO_ResetBits(GPIOA, GPIO_Pin_10);
		//GPIO_SetBits(GPIOA, GPIO_Pin_5);
	}
}
void B_north_polar(void){
		GPIO_SetBits(GPIOC, GPIO_Pin_8);
		GPIO_ResetBits(GPIOC, GPIO_Pin_11);
}
void B_south_polar(void){
	 GPIO_ResetBits(GPIOC, GPIO_Pin_8);
	 GPIO_SetBits(GPIOC, GPIO_Pin_11);
}
void A_polar_off(void){
	//GPIO_ResetBits(GPIOA, GPIO_Pin_5);
	GPIO_ResetBits(GPIOA, GPIO_Pin_9);
	GPIO_ResetBits(GPIOA, GPIO_Pin_10);
	//GPIO_SetBits(GPIOA, GPIO_Pin_5);
	}
void B_polar_off(void){
	//GPIO_ResetBits(GPIOC, GPIO_Pin_3);
	GPIO_ResetBits(GPIOC, GPIO_Pin_8);
	GPIO_ResetBits(GPIOC, GPIO_Pin_11);
	//GPIO_SetBits(GPIOC, GPIO_Pin_3);
}
//--------------motor function end------------
/**
 * Configure the GPIO for output to the motor.
 */
void GPIO_Config(void){
	
	GPIO_InitTypeDef GPIO_InitStructure;
	

  /* TIM4 clock enable */
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);

  /* GPIOD clock enable */
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
	//RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
  
  /* GPIOD Configuration: TIM4 CH1 (PD12), TIM4 CH2 (PD13), TIM4 CH3 (PD14) and TIM4 CH4 (PD15) */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9 | GPIO_Pin_10 ;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;
  GPIO_Init(GPIOA, &GPIO_InitStructure); 
  /* Connect TIM4 pins to AF2 */  
}
void GPIOC_Config(void){
	GPIO_InitTypeDef GPIO_InitStructure2;
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource7, GPIO_AF_TIM4);
	//-------------------------------------------
	GPIO_InitStructure2.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_11 ;
  GPIO_InitStructure2.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure2.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_InitStructure2.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure2.GPIO_PuPd = GPIO_PuPd_NOPULL ;
  GPIO_Init(GPIOC, &GPIO_InitStructure2); 
  /* Connect TIM4 pins to AF2 */  
  GPIO_PinAFConfig(GPIOC, GPIO_PinSource7, GPIO_AF_TIM4);
	
	
}


/**
 * Configure the TIM4 in output compare mode.
 */
void Timer_Config(void){
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	TIM_OCInitTypeDef TIM_OCInitStructure;
	
	//since TIMER 3 is on APB1 bus, need to enale APB1 bus clock first
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4,ENABLE);
	
	//====================================================
	//Enable TIM4 global interrupt ====does this part need to be done before TIM_BaseStructure set up?
	NVIC_InitStructure.NVIC_IRQChannel=TIM4_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0X00;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority=0x01;
	NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE;
	NVIC_Init(&NVIC_InitStructure);	
	//================================================
	
	TIM_TimeBaseStructure.TIM_Period=65535; // need to be larger than CCR1_VAL, has no effect on the Output compare event.
	TIM_TimeBaseStructure.TIM_Prescaler=1800;//(uint16_t) ((SystemCoreClock / 2) / 500000) - 1;    //why all the example make this one equal 0, and then use 
					//function TIM_PrescalerConfig() to re-assign the prescaller value?
	TIM_TimeBaseStructure.TIM_ClockDivision=0;
	TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up;	
	 
	TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);
	/////////////////////OCConfig//////////////////
	
	TIM_OCInitStructure.TIM_OCMode=TIM_OCMode_Timing;
	TIM_OCInitStructure.TIM_OutputState=TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_Pulse=50;
	TIM_OCInitStructure.TIM_OCPolarity=TIM_OCPolarity_High;
	
	TIM_OC1Init(TIM4, &TIM_OCInitStructure);
	
	TIM_OC1PreloadConfig(TIM4, TIM_OCPreload_Disable); 
	
}
/*void TIM4_Config(void){
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4,ENABLE);
	TIM_TimeBaseStructure.TIM_Period=65535; 
	TIM_TimeBaseStructure.TIM_Prescaler=18*2;
	TIM_TimeBaseStructure.TIM_ClockDivision=0;
	TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up;	 
	TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);
	TIM_Cmd(TIM4, ENABLE);
}*/
void PB_Config(void)
{
/* Initialize User_Button on STM32F4-Discovery
   * Normally one would need to initialize the EXTI interrupt
   * to handle the 'User' button, however the function already
   * does this.
   */
  STM_EVAL_PBInit(BUTTON_USER, BUTTON_MODE_EXTI);
}
static void Mode_Button(void){//PB4
	EXTI_InitTypeDef   EXTI_InitStructure;
  GPIO_InitTypeDef   GPIO_InitStructure;
  NVIC_InitTypeDef   NVIC_InitStructure;

  /* Enable GPIOC clock */
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
  
  /* Enable SYSCFG clock */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);
  
  /* Configure PC13 pin as input floating */
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	//GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_Init(GPIOB, &GPIO_InitStructure);

  /* Connect EXTI Line15 to PC13 pin */
  SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOB, EXTI_PinSource4);

  /* Configure EXTI Line13 */
  EXTI_InitStructure.EXTI_Line = EXTI_Line4;
  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
  EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  EXTI_Init(&EXTI_InitStructure);
	//NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);
  /* Enable and set EXTI15_10 Interrupt to the lowest priority */
	NVIC_EnableIRQ(EXTI4_IRQn);
  NVIC_InitStructure.NVIC_IRQChannel = EXTI4_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x0F;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x0F;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;

  NVIC_Init(&NVIC_InitStructure);
	EXTI_ClearITPendingBit(EXTI_Line4);
}
static void Decrease_Button(void){//PD2
	EXTI_InitTypeDef   EXTI_InitStructure;
  GPIO_InitTypeDef   GPIO_InitStructure;
  NVIC_InitTypeDef   NVIC_InitStructure;

 
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
  

  RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);
  

  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	//GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_Init(GPIOD, &GPIO_InitStructure);

  
  SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOD, EXTI_PinSource2);


  EXTI_InitStructure.EXTI_Line = EXTI_Line2;
  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
  EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  EXTI_Init(&EXTI_InitStructure);
	//NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);
  
	//NVIC_EnableIRQ(EXTI4_IRQn);
  NVIC_InitStructure.NVIC_IRQChannel = EXTI2_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x0F;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x0F;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;

  NVIC_Init(&NVIC_InitStructure);
	EXTI_ClearITPendingBit(EXTI_Line2);
}
static void Increase_Button(void){//PC3
	EXTI_InitTypeDef   EXTI_InitStructure;
  GPIO_InitTypeDef   GPIO_InitStructure;
  NVIC_InitTypeDef   NVIC_InitStructure;

 
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
  

  RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);
  

  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	//GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_Init(GPIOC, &GPIO_InitStructure);

  
  SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOC, EXTI_PinSource3);


  EXTI_InitStructure.EXTI_Line = EXTI_Line3;
  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
  EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  EXTI_Init(&EXTI_InitStructure);
	//NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);
  
	//NVIC_EnableIRQ(EXTI4_IRQn);
  NVIC_InitStructure.NVIC_IRQChannel = EXTI3_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x0F;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x0F;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;

  NVIC_Init(&NVIC_InitStructure);
	EXTI_ClearITPendingBit(EXTI_Line3);
}

