#include "main.h"
#include "stm32f4xx_rtc.h"
#include "stdint.h"


//memory location to write to in the device
__IO uint16_t memLocation = 0x000A; //pick any location within range

uint8_t Tx1_Buffer ;
uint8_t Rx1_Buffer;
uint16_t NumDataRead = 1;
__IO uint8_t UBPressed = 0;
__IO uint8_t EBPressed = 0;
__IO uint8_t CBPressed = 0;
uint16_t line;
__IO uint32_t LsiFreq = 0;
__IO uint32_t CaptureNumber = 0, PeriodValue = 0;


/* Private function prototypes -----------------------------------------------*/
void RTC_Config(void);
static void EXTILine14_Config(void);
static void EXTILine4_Config(void);
void store(uint8_t num,__IO uint16_t location);
void memorysetting(void);
uint32_t GetLSIFrequency(void);
RTC_InitTypeDef   RTC_InitStructure;
RTC_TimeTypeDef 	RTC_TimeStructure;
RTC_DateTypeDef 	RTC_DateStructure;
char Time[10];
char ReadTime[10];
uint8_t h1,H1;
uint8_t h2,H2;
uint8_t m1,M1;
uint8_t m2,M2;
uint8_t s1,S1;
uint8_t s2,S2;
extern int count;
extern int state;
int hh = 8;
int mm = 0;
int ss = 0;
int YY = 0;
int MM = 0;
int DD = 0;
int main(void){
	
	//configure push-button interrupts
	PB_Config();
	
	 /* LCD initiatization */
  LCD_Init();
  
  /* LCD Layer initiatization */
  LCD_LayerInit();
    
  /* Enable the LTDC */
  LTDC_Cmd(ENABLE);
  
  /* Set LCD foreground layer */
  LCD_SetLayer(LCD_FOREGROUND_LAYER);

	
	
	//======You need to develop the following functions======
	//Note: these are just placeholders; function definitions are at bottom of this file
	//configure real-time clock
	RTC_Config();
	
	//configure external push-buttons and interrupts
	ExtPB_Config();
	
	
	//main program
	
	LCD_Clear(LCD_COLOR_WHITE);
		
	line=0;
	//Display a string in one line, on the first line (line=0)
	//LCD_DisplayStringLine(LINE(line),  (uint8_t *) "Init EEPROM...");
	//line++;
	
	//i2c_init(); //initialize the i2c chip
	sEE_Init();  

  
  
	
	LCD_DisplayStringLine(LINE(0),  (uint8_t *) "1"); 
	
	LCD_DisplayStringLine(LINE(line),  (uint8_t *) "done..."); 
	line++;
	
	LCD_DisplayStringLine(LINE(line),  (uint8_t *) "Writing...");
	line++;
	
	
	//LCD_DisplayStringLine(LINE(0),  (uint8_t *) "1");
	
	/* First write in the memory followed by a read of the written data --------*/
  /* Write on I2C EEPROM from memLocation */
  sEE_WriteBuffer(&Tx1_Buffer, memLocation,1); 

  /* Wait for EEPROM standby state */
  sEE_WaitEepromStandbyState();  
 
  
	LCD_DisplayStringLine(LINE(line),  (uint8_t *) "Reading...");
  /* Read from I2C EEPROM from memLocation */
  sEE_ReadBuffer(&Rx1_Buffer, memLocation, (uint16_t *)(&NumDataRead)); 
	line++;
	
	
	LCD_DisplayStringLine(LINE(line),  (uint8_t *) "Comparing...");  
	line++;
//	LCD_DisplayStringLine(LINE(8),  (uint8_t *) &Rx1_Buffer);
	EXTILine14_Config();
	EXTILine4_Config();
	if(Tx1_Buffer== Rx1_Buffer){
		LCD_DisplayStringLine(LINE(line),  (uint8_t *) "Success!");  
	}else{
		LCD_DisplayStringLine(LINE(line),  (uint8_t *) "Mismatch!"); 
	}
	//RTC_TimeStructInit(&RTC_TimeStructure);
	//main loop
	memorysetting();
	while(1){
		//RTC_SetDate(RTC_Format_BIN, &RTC_DateStructure);
	  RTC_GetTime(RTC_Format_BIN,&RTC_TimeStructure);
		hh = RTC_TimeStructure.RTC_Hours;
		mm = RTC_TimeStructure.RTC_Minutes;
		ss = RTC_TimeStructure.RTC_Seconds;
		sprintf(Time,"%0.2d:%0.2d:%0.2d",hh,mm,ss);
		
		LCD_DisplayStringLine(LINE(5),  (uint8_t *) Time); 
		if(UBPressed ==1){//store memory
			store(H1,memLocation+6);
			store(H2,memLocation+7);
			store(M1,memLocation+8);
			store(M2,memLocation+9);
			store(S1,memLocation+10);
			store(S2,memLocation+11);
			H1 = Time[0];
			H2 = Time[1];
			M1 = Time[3];
			M2 = Time[4];
			S1 = Time[6];
			S2 = Time[7];
			store(H1,memLocation);
			store(H2,memLocation+1);
			store(M1,memLocation+2);
			store(M2,memLocation+3);
			store(S1,memLocation+4);
			store(S2,memLocation+5);
			UBPressed =0 ;
			
		}
		if((CBPressed == 1)&&(state ==0)){//read data and display
				sEE_ReadBuffer(&h1, memLocation, (uint16_t *)(&NumDataRead)); 
				sEE_ReadBuffer(&h2, memLocation+1, (uint16_t *)(&NumDataRead));
				sEE_ReadBuffer(&m1, memLocation+2, (uint16_t *)(&NumDataRead));
				sEE_ReadBuffer(&m2, memLocation+3, (uint16_t *)(&NumDataRead));
				sEE_ReadBuffer(&s1, memLocation+4, (uint16_t *)(&NumDataRead));
				sEE_ReadBuffer(&s2, memLocation+5, (uint16_t *)(&NumDataRead));
				ReadTime[0]=h1;
				ReadTime[1]=h2;
				ReadTime[2]=':';
				ReadTime[3]=m1;
				ReadTime[4]=m2;
				ReadTime[5]=':';
				ReadTime[6]=s1;
				ReadTime[7]=s2;
				LCD_DisplayStringLine(LINE(10),  (uint8_t *) "display time:");
				LCD_DisplayStringLine(LINE(11),  (uint8_t *) ReadTime);
				sEE_ReadBuffer(&h1, memLocation+6, (uint16_t *)(&NumDataRead)); 
				sEE_ReadBuffer(&h2, memLocation+7, (uint16_t *)(&NumDataRead));
				sEE_ReadBuffer(&m1, memLocation+8, (uint16_t *)(&NumDataRead));
				sEE_ReadBuffer(&m2, memLocation+9, (uint16_t *)(&NumDataRead));
				sEE_ReadBuffer(&s1, memLocation+10, (uint16_t *)(&NumDataRead));
				sEE_ReadBuffer(&s2, memLocation+11, (uint16_t *)(&NumDataRead));
				ReadTime[0]=h1;
				ReadTime[1]=h2;
				ReadTime[2]=':';
				ReadTime[3]=m1;
				ReadTime[4]=m2;
				ReadTime[5]=':';
				ReadTime[6]=s1;
				ReadTime[7]=s2;
				LCD_DisplayStringLine(LINE(12),  (uint8_t *) ReadTime);
				CBPressed =0;
		}
		
		
	}
}



void PB_Config(void)
{
/* Initialize User_Button on STM32F4-Discovery
   * Normally one would need to initialize the EXTI interrupt
   * to handle the 'User' button, however the function already
   * does this.
   */
  STM_EVAL_PBInit(BUTTON_USER, BUTTON_MODE_EXTI);
}


void store(uint8_t num,__IO uint16_t location){//function of store in ROM
	sEE_WriteBuffer(&num, location,1); 
	sEE_WaitEepromStandbyState();
	}
void memorysetting(void){//empty all time
			store('n',memLocation);
			store('o',memLocation+1);
			store('t',memLocation+2);
			store('i',memLocation+3);
			store('m',memLocation+4);
			store('e',memLocation+5);
			store('n',memLocation+6);
			store('o',memLocation+7);
			store('t',memLocation+8);
			store('i',memLocation+9);
			store('m',memLocation+10);
			store('e',memLocation+11);
		
		}

/**
 * Use this function to configure the GPIO to handle input from
 * external pushbuttons and configure them so that you will handle
 * them through external interrupts.
 */

void ExtPB_Config(void){
	
}

/**
 * Configure the RTC to operate based on the LSI (Internal Low Speed oscillator)
 * and configure one of the alarms (A or B) to trigger an external interrupt every second
 * (e.g. EXTI line 17 for alarm A).
 */
void RTC_Config(void){
	NVIC_InitTypeDef NVIC_InitStructure; 
  EXTI_InitTypeDef EXTI_InitStructure;

  /* Enable the PWR clock */
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR, ENABLE);

  /* Allow access to RTC */
  PWR_BackupAccessCmd(ENABLE);

/* LSI used as RTC source clock */
/* The RTC Clock may varies due to LSI frequency dispersion. */   
  /* Enable the LSI OSC */ 
  RCC_LSICmd(ENABLE);
 while(RCC_GetFlagStatus(RCC_FLAG_LSIRDY) == RESET)
  {
  }

  /* Select the RTC Clock Source */
  RCC_RTCCLKConfig(RCC_RTCCLKSource_LSI);
   
  /* Enable the RTC Clock */
  RCC_RTCCLKCmd(ENABLE);

  /* Wait for RTC APB registers synchronisation */
  RTC_WaitForSynchro();

  /* Calendar Configuration with LSI supposed at 32KHz */
  RTC_InitStructure.RTC_AsynchPrediv = 0x7F;
  RTC_InitStructure.RTC_SynchPrediv	=  0xFF; /* (32KHz / 128) - 1 = 0xFF*/
  RTC_InitStructure.RTC_HourFormat = RTC_HourFormat_24;
  RTC_Init(&RTC_InitStructure); 
	/* Set the Time */
	RTC_TimeStructure.RTC_Hours = hh;
	RTC_TimeStructure.RTC_Minutes = mm;
	RTC_TimeStructure.RTC_Seconds = ss;
	RTC_SetTime(RTC_Format_BCD, &RTC_TimeStructure);
	/* Set the Date */
	RTC_DateStructure.RTC_Month = 0x10;
	RTC_DateStructure.RTC_WeekDay = RTC_Weekday_Monday;
	RTC_DateStructure.RTC_Date = 0x23;
	RTC_DateStructure.RTC_Year = 0x14;
	RTC_SetDate(RTC_Format_BCD, &RTC_DateStructure);
/* EXTI configuration *******************************************************/
  EXTI_ClearITPendingBit(EXTI_Line22);
  EXTI_InitStructure.EXTI_Line = EXTI_Line22;
  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
  EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  EXTI_Init(&EXTI_InitStructure);
  
  /* Enable the RTC Wakeup Interrupt */
  NVIC_InitStructure.NVIC_IRQChannel = RTC_WKUP_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);  

  /* Configure the RTC WakeUp Clock source: CK_SPRE (1Hz) */
  RTC_WakeUpClockConfig(RTC_WakeUpClock_CK_SPRE_16bits);
  RTC_SetWakeUpCounter(0x0);

  /* Enable the RTC Wakeup Interrupt */
  RTC_ITConfig(RTC_IT_WUT, ENABLE);
  
  /* Enable Wakeup Counter */
  RTC_WakeUpCmd(ENABLE);	
	
}
uint32_t GetLSIFrequency(void)
{
  NVIC_InitTypeDef   NVIC_InitStructure;
  TIM_ICInitTypeDef  TIM_ICInitStructure;
  RCC_ClocksTypeDef  RCC_ClockFreq;

  /* Enable the LSI oscillator ************************************************/
  RCC_LSICmd(ENABLE);
  
  /* Wait till LSI is ready */
  while (RCC_GetFlagStatus(RCC_FLAG_LSIRDY) == RESET)
  {}

  /* TIM5 configuration *******************************************************/ 
  /* Enable TIM5 clock */
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5, ENABLE);
  
  /* Connect internally the TIM5_CH4 Input Capture to the LSI clock output */
  TIM_RemapConfig(TIM5, TIM5_LSI);

  /* Configure TIM5 presclaer */
  TIM_PrescalerConfig(TIM5, 0, TIM_PSCReloadMode_Immediate);
  
  /* TIM5 configuration: Input Capture mode ---------------------
     The LSI oscillator is connected to TIM5 CH4
     The Rising edge is used as active edge,
     The TIM5 CCR4 is used to compute the frequency value 
  ------------------------------------------------------------ */
  TIM_ICInitStructure.TIM_Channel = TIM_Channel_4;
  TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;
  TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
  TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV8;
  TIM_ICInitStructure.TIM_ICFilter = 0;
  TIM_ICInit(TIM5, &TIM_ICInitStructure);
  
  /* Enable TIM5 Interrupt channel */
  NVIC_InitStructure.NVIC_IRQChannel = TIM5_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);

  /* Enable TIM5 counter */
  TIM_Cmd(TIM5, ENABLE);

  /* Reset the flags */
  TIM5->SR = 0;
    
  /* Enable the CC4 Interrupt Request */  
  TIM_ITConfig(TIM5, TIM_IT_CC4, ENABLE);


  /* Wait until the TIM5 get 2 LSI edges (refer to TIM5_IRQHandler() in 
    stm32f4xx_it.c file) ******************************************************/
  while(CaptureNumber != 2)
  {
  }
  /* Deinitialize the TIM5 peripheral registers to their default reset values */
  TIM_DeInit(TIM5);


  /* Compute the LSI frequency, depending on TIM5 input clock frequency (PCLK1)*/
  /* Get SYSCLK, HCLK and PCLKx frequency */
  RCC_GetClocksFreq(&RCC_ClockFreq);

  /* Get PCLK1 prescaler */
  if ((RCC->CFGR & RCC_CFGR_PPRE1) == 0)
  { 
    /* PCLK1 prescaler equal to 1 => TIMCLK = PCLK1 */
    return ((RCC_ClockFreq.PCLK1_Frequency / PeriodValue) * 8);
  }
  else
  { /* PCLK1 prescaler different from 1 => TIMCLK = 2 * PCLK1 */
    return (((2 * RCC_ClockFreq.PCLK1_Frequency) / PeriodValue) * 8) ;
  }
}
static void EXTILine14_Config(void)
{
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

static void EXTILine4_Config(void)
{
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
	//NVIC_EnableIRQ(EXTI4_IRQn);
  NVIC_InitStructure.NVIC_IRQChannel = EXTI4_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x0F;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x0F;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;

  NVIC_Init(&NVIC_InitStructure);
	EXTI_ClearITPendingBit(EXTI_Line4);
}

