#include "main.h"

#define ADC3_DR_ADDRESS    ((uint32_t)0x4001224C)


//the following two addresses are useful when using the ADC and DAC in DMA mode

#define ADC_CDR_ADDR 				((uint32_t)(ADC_BASE + 0x08)) //ADC_CDR address as described in section 10.13.17 of reference manual
//ADC CDR is the common regular (not injected) data register for dual and triple modes
//also defined in stm32f4xx_adc.c as 
//#define CDR_ADDRESS 			((uint32_t)0x40012308) 
//which results in the same address

void ADC_Config(void);
void PWM_Config(void);
void PWM_SetUp(int CCR );
void TIM4_Config(void);
void TIM4_OCConfig(void);
void GPIO_Config(void);
void PWM_Tim(void);
extern uint32_t time;
static void Decrease_Button(void);
static void Increase_Button(void);
__IO uint16_t ADC3ConvertedValue = 0;
__IO uint16_t uhADC3ConvertedValue = 0;
__IO uint32_t uwADC3ConvertedVoltage = 0;
TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
TIM_OCInitTypeDef  TIM_OCInitStructure;

/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
uint16_t CCR1_Val = 350;
uint16_t CCR2_Val = 262;
uint16_t CCR3_Val = 175;
uint16_t CCR4_Val = 87;
uint16_t PrescalerValue = 0;
uint16_t temp_value;
extern uint16_t increase;
extern uint16_t decrease;
extern int count;
uint32_t uwVoltage;
uint32_t temp2;
char temper[10];
char Set_Show[10];
int factor;
int NOV = 0;
uint16_t Set_Point=20;
int main(void){
	GPIO_Config();
	
	 /* LCD initiatization */
  LCD_Init();
  
  /* LCD Layer initiatization */
  LCD_LayerInit();
    
  /* Enable the LTDC */
  LTDC_Cmd(ENABLE);
  
  /* Set LCD foreground layer */
  LCD_SetLayer(LCD_FOREGROUND_LAYER);
	
	//configure push button
	STM_EVAL_PBInit(BUTTON_USER, BUTTON_MODE_EXTI);
	
	
	/***********************Implement the following functions*****************/
	//configure ADC device
	ADC_Config();
	//configure PWM output
	PWM_Config();
	TIM4_Config();
	TIM4_OCConfig();
	/* TIM Interrupts enable */
  TIM_ITConfig(TIM4, TIM_IT_CC1, ENABLE);

  TIM_Cmd(TIM4, ENABLE);
	Decrease_Button();
	Increase_Button();

	
	ADC_SoftwareStartConv(ADC3);
	
	LCD_Clear(LCD_COLOR_WHITE);
	
	//Display a string in one line, on the first line (line=0)
	LCD_DisplayStringLine(LINE(0),  (uint8_t *) "PWM Fan control");
	
	while(1){
		//--------------------------------increase button------------//increase button
		while (GPIO_ReadInputDataBit( GPIOB,GPIO_Pin_4) != Bit_SET){
			
			if(count ==1){time++;count = 0;}
		
			if((time)==5){increase++;time=0;}
			
//if(Set_Point<=10){LCD_ClearLine(LINE(2));}
			
		}
		//-------------------------------end------------------------
		//----------------------------decrease button---------------//decrease button
		while (GPIO_ReadInputDataBit( GPIOD,GPIO_Pin_2) != Bit_SET){
			if(count ==1){time++;count = 0;}//if 10 HZ interupt
			if((time)==5){decrease++;time =0;}
			
			//if(Set_Point<=10){LCD_ClearLine(LINE(2));}
			//GPIO_ReadInputDataBit( GPIOB,GPIO_Pin_4)
		}
		//---------------------------------end----------------------
		
		
		uwADC3ConvertedVoltage = uhADC3ConvertedValue *3000/0xFFF;
		uwVoltage = (uwADC3ConvertedVoltage)/30;
		temp_value = uwVoltage;
		factor = 349/(40-Set_Point);
		//temp2 =uwADC3ConvertedVoltage;
		sprintf(temper,"%d",temp_value);
		sprintf(Set_Show,"%d",Set_Point);
		if(temp_value==9){LCD_ClearLine(LINE(1));}
		//if(Set_Point==9){LCD_ClearLine(LINE(2));}
		//if(factor==9){LCD_ClearLine(LINE(3));}
		
		LCD_DisplayStringLine(LINE(1),  (uint8_t *) temper);
		LCD_DisplayStringLine(LINE(2),  (uint8_t *) Set_Show);
		sprintf(Set_Show,"%d",factor);
		LCD_DisplayStringLine(LINE(3),  (uint8_t *) Set_Show);
	
		//PWM_SetUp(350);
		//sprintf(temper,"%d",temp2);
		//LCD_DisplayStringLine(LINE(2),  (uint8_t *) temper);
		if(NOV != temp_value){//according to seting point ,to have different speed of PWM
			if(temp_value>=Set_Point){PWM_SetUp(CCR1_Val+(temp_value - Set_Point)*factor);}
			else{PWM_SetUp(0);}
			NOV = temp_value; 
		}
		if(increase >0){//value change
				if(Set_Point>=10){LCD_ClearLine(LINE(2));}
				if(factor>=10){LCD_ClearLine(LINE(3));}
				if(Set_Point>=40){Set_Point=40;}
				else{Set_Point+=increase;}
				increase =0;}
		if(decrease >0){//value change
				if(Set_Point>=10){LCD_ClearLine(LINE(2));}
				if(factor>=10){LCD_ClearLine(LINE(3));}
				if(Set_Point>40||Set_Point==0){Set_Point = 0;}	
				else{Set_Point -=decrease;}
				decrease =0;}
		
		
		
	};
}

/**
 * Use this function to configure the ADC input.
 */
void ADC_Config(void){
	ADC_InitTypeDef       ADC_InitStructure;
  ADC_CommonInitTypeDef ADC_CommonInitStructure;
  DMA_InitTypeDef       DMA_InitStructure;
  GPIO_InitTypeDef      GPIO_InitStructure;

  /* Enable ADC3, DMA2 and GPIO clocks ****************************************/
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2 | RCC_AHB1Periph_GPIOC, ENABLE);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC3, ENABLE);

  /* DMA2 Stream0 channel2 configuration **************************************/
  DMA_InitStructure.DMA_Channel = DMA_Channel_2;  
  DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)ADC3_DR_ADDRESS;
  DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)&uhADC3ConvertedValue;
  DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;
  DMA_InitStructure.DMA_BufferSize = 1;
  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Disable;
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
  DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
  DMA_InitStructure.DMA_Priority = DMA_Priority_High;
  DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;         
  DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_HalfFull;
  DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
  DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
  DMA_Init(DMA2_Stream0, &DMA_InitStructure);
  DMA_Cmd(DMA2_Stream0, ENABLE);

  /* Configure ADC3 Channel13 pin as analog input ******************************/
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOC, &GPIO_InitStructure);

  /* ADC Common Init **********************************************************/
  ADC_CommonInitStructure.ADC_Mode = ADC_Mode_Independent;
  ADC_CommonInitStructure.ADC_Prescaler = ADC_Prescaler_Div2;
  ADC_CommonInitStructure.ADC_DMAAccessMode = ADC_DMAAccessMode_Disabled;
  ADC_CommonInitStructure.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_5Cycles;
  ADC_CommonInit(&ADC_CommonInitStructure);

  /* ADC3 Init ****************************************************************/
  ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;
  ADC_InitStructure.ADC_ScanConvMode = DISABLE;
  ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;
  ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;
  ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_T1_CC1;	
  ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
  ADC_InitStructure.ADC_NbrOfConversion = 1;
  ADC_Init(ADC3, &ADC_InitStructure);

  /* ADC3 regular channel13 configuration *************************************/
  ADC_RegularChannelConfig(ADC3, ADC_Channel_13, 1, ADC_SampleTime_3Cycles);

 /* Enable DMA request after last transfer (Single-ADC mode) */
  ADC_DMARequestAfterLastTransferCmd(ADC3, ENABLE);

  /* Enable ADC3 DMA */
  ADC_DMACmd(ADC3, ENABLE);

  /* Enable ADC3 */
  ADC_Cmd(ADC3, ENABLE);
	
}

/**
 * Use this function to configure the PWM output.
 */

	
	
/*	
void PWM_1(void){
	TIM_OC2PreloadConfig(TIM4, TIM_OCPreload_Disable );
	TIM_OC3PreloadConfig(TIM4, TIM_OCPreload_Disable );
	 TIM_OC4PreloadConfig(TIM4, TIM_OCPreload_Disable) ;

	
  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_Pulse = CCR1_Val;
  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;

  TIM_OC1Init(TIM4, &TIM_OCInitStructure);

  TIM_OC1PreloadConfig(TIM4, TIM_OCPreload_Enable);
	
	}
void PWM_2(void){
	TIM_OC1PreloadConfig(TIM4, TIM_OCPreload_Disable );
	TIM_OC3PreloadConfig(TIM4, TIM_OCPreload_Disable );
	TIM_OC4PreloadConfig(TIM4, TIM_OCPreload_Disable) ;

	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_Pulse = CCR2_Val;

  TIM_OC2Init(TIM4, &TIM_OCInitStructure);

  TIM_OC2PreloadConfig(TIM4, TIM_OCPreload_Enable);

}
void PWM_3(void){
	TIM_OC1PreloadConfig(TIM4, TIM_OCPreload_Disable );
	TIM_OC2PreloadConfig(TIM4, TIM_OCPreload_Disable );
	TIM_OC4PreloadConfig(TIM4, TIM_OCPreload_Disable) ;

	
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_Pulse = CCR3_Val;

  TIM_OC3Init(TIM4, &TIM_OCInitStructure);

  TIM_OC3PreloadConfig(TIM4, TIM_OCPreload_Enable);

}*/

void PWM_SetUp(int CCR){
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_Pulse = CCR;
  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;

  TIM_OC2Init(TIM3, &TIM_OCInitStructure);

  TIM_OC2PreloadConfig(TIM3, TIM_OCPreload_Enable);
	
	/*
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_Pulse = CCR;
	TIM_OC2Init(TIM3, &TIM_OCInitStructure);
  TIM_OC2PreloadConfig(TIM3, TIM_OCPreload_Enable);*/
	
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource7, GPIO_AF_TIM3);
	}

void PWM_Config(void){
	RCC_TIMCLKPresConfig(RCC_TIMPrescActivated);

  /* Compute the prescaler value */
  PrescalerValue = (uint16_t) (SystemCoreClock / 21000000) - 1;

  /* Time base configuration */
  TIM_TimeBaseStructure.TIM_Period = 699;
  TIM_TimeBaseStructure.TIM_Prescaler = PrescalerValue;
  TIM_TimeBaseStructure.TIM_ClockDivision = 0;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;

  TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);

  /*  
  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_Pulse = CCR1_Val;
  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;

  TIM_OC1Init(TIM4, &TIM_OCInitStructure);

  TIM_OC1PreloadConfig(TIM4, TIM_OCPreload_Enable);

  
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_Pulse = CCR2_Val;

  TIM_OC2Init(TIM4, &TIM_OCInitStructure);

  TIM_OC2PreloadConfig(TIM4, TIM_OCPreload_Enable);

  
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_Pulse = CCR3_Val;

  TIM_OC3Init(TIM4, &TIM_OCInitStructure);

  TIM_OC3PreloadConfig(TIM4, TIM_OCPreload_Enable);

 
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_Pulse = CCR4_Val;*/

  TIM_OC2Init(TIM3, &TIM_OCInitStructure);

  TIM_OC2PreloadConfig(TIM3, TIM_OCPreload_Enable);

  TIM_ARRPreloadConfig(TIM3, ENABLE);

   
  TIM_Cmd(TIM3, ENABLE);
	
}
void GPIO_Config(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;

  /* TIM4 clock enable */
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);

  /* GPIOD clock enable */
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
  
  /* GPIOD Configuration: TIM4 CH1 (PD12), TIM4 CH2 (PD13), TIM4 CH3 (PD14) and TIM4 CH4 (PD15) */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP ;
  GPIO_Init(GPIOA, &GPIO_InitStructure); 

  /* Connect TIM4 pins to AF2 */  
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource7, GPIO_AF_TIM3);
  
}
static void Decrease_Button(void){
	EXTI_InitTypeDef   EXTI_InitStructure;
  GPIO_InitTypeDef   GPIO_InitStructure;
  NVIC_InitTypeDef   NVIC_InitStructure;

  /* Enable GPIOC clock */
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
  
  /* Enable SYSCFG clock */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);
  
  /* Configure PC13 pin as input floating */
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
	//GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	//GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_Init(GPIOD, &GPIO_InitStructure);

  /* Connect EXTI Line15 to PC13 pin */
  SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOD, EXTI_PinSource2);

  /* Configure EXTI Line13 */
  EXTI_InitStructure.EXTI_Line = EXTI_Line2;
  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
  EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  EXTI_Init(&EXTI_InitStructure);
	//NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);
  /* Enable and set EXTI15_10 Interrupt to the lowest priority */
	//NVIC_EnableIRQ(EXTI4_IRQn);
  NVIC_InitStructure.NVIC_IRQChannel = EXTI2_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x0F;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x0F;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;

  NVIC_Init(&NVIC_InitStructure);
	EXTI_ClearITPendingBit(EXTI_Line2);
}
static void Increase_Button(void){
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
	//GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
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

void TIM4_Config(void){
TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
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
	TIM_TimeBaseStructure.TIM_Prescaler=18000*2;//(uint16_t) ((SystemCoreClock / 2) / 500000) - 1;    //why all the example make this one equal 0, and then use 
					//function TIM_PrescalerConfig() to re-assign the prescaller value?
	TIM_TimeBaseStructure.TIM_ClockDivision=0;
	TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up;	
	 
	TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);}

	void TIM4_OCConfig(void) {
	TIM_OCInitTypeDef TIM_OCInitStructure;
	
	TIM_OCInitStructure.TIM_OCMode=TIM_OCMode_Timing;
	TIM_OCInitStructure.TIM_OutputState=TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_Pulse=500;
	TIM_OCInitStructure.TIM_OCPolarity=TIM_OCPolarity_High;
	
	TIM_OC1Init(TIM4, &TIM_OCInitStructure);
	
	TIM_OC1PreloadConfig(TIM4, TIM_OCPreload_Disable); //if disabled, 
	//the TIMx_CCRx register can be updated at any time by software to control the output
	//waveform---from the reference manual
}	
	
