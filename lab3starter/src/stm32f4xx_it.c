/**
  ******************************************************************************
  * @file    TIM_TimeBase/stm32f4xx_it.c 
  * @author  MCD Application Team
  * @version V1.0.0
  * @date    19-September-2011
  * @brief   Main Interrupt Service Routines.
  *          This file provides template for all exceptions handler and peripherals
  *          interrupt service routine.
  ******************************************************************************
  * @attention
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; COPYRIGHT 2011 STMicroelectronics</center></h2>
  ******************************************************************************
  */ 

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_it.h"
#include "stm32f429i_discovery.h"
#include "main.h"
extern __IO uint8_t UBPressed;
extern __IO uint8_t EBPressed;
extern __IO uint8_t CBPressed;
int state=0;
int count =0;
extern int hh;
extern int mm;
extern int ss;
extern char Time[10];
extern int YY;
extern int MM;
extern int DD;
extern char ReadTime[10];
extern uint8_t h1;
extern uint8_t h2;
extern uint8_t m1;
extern uint8_t m2;
extern uint8_t s1;
extern uint8_t s2;
extern uint16_t NumDataRead;
char date[10];

/**
  * @brief  Converts a 2 digit decimal to BCD format.
  * @param  Value: Byte to be converted.
  * @retval Converted byte
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/


/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/******************************************************************************/
/*            Cortex-M4 Processor Exceptions Handlers                         */
/******************************************************************************/

/**
  * @brief  This function handles NMI exception.
  * @param  None
  * @retval None
  */
void NMI_Handler(void)
{
}

/**
  * @brief  This function handles Hard Fault exception.
  * @param  None
  * @retval None
  */
void HardFault_Handler(void)
{
  /* Go to infinite loop when Hard Fault exception occurs */
  while (1)
  {}
}

/**
  * @brief  This function handles Memory Manage exception.
  * @param  None
  * @retval None
  */
void MemManage_Handler(void)
{
  /* Go to infinite loop when Memory Manage exception occurs */
  while (1)
  {}
}

/**
  * @brief  This function handles Bus Fault exception.
  * @param  None
  * @retval None
  */
void BusFault_Handler(void)
{
  /* Go to infinite loop when Bus Fault exception occurs */
  while (1)
  {}
}

/**
  * @brief  This function handles Usage Fault exception.
  * @param  None
  * @retval None
  */
void UsageFault_Handler(void)
{
  /* Go to infinite loop when Usage Fault exception occurs */
  while (1)
  {}
}

/**
  * @brief  This function handles Debug Monitor exception.
  * @param  None
  * @retval None
  */
void DebugMon_Handler(void)
{}

/**
  * @brief  This function handles SVCall exception.
  * @param  None
  * @retval None
  */
void SVC_Handler(void)
{}

/**
  * @brief  This function handles PendSV_Handler exception.
  * @param  None
  * @retval None
  */
void PendSV_Handler(void)
{}

/**
  * @brief  This function handles SysTick Handler.
  * @param  None
  * @retval None
  */
void SysTick_Handler(void){
	//uncomment me if you want to handle systicks
	//TimingDelay_Decrement();
}

/******************************************************************************/
/*            STM32F4xx Peripherals Interrupt Handlers                        */
/******************************************************************************/

/**
  * @brief  This function handles TIM3 global interrupt request.
  * @param  None
  * @retval None
  */
void TIM3_IRQHandler(void)
{
  
}

/**
 * External interrupt channel 0 Interrupt Handler. This handles
 * the user button.
 */
void EXTI0_IRQHandler(void){//user button
	//UBPressed =1;
	while (STM_EVAL_PBGetState(BUTTON_USER) == Bit_SET){//if user button hold counting
		count ++;
		if(count >4000000){//if counting long display date
			RTC_GetDate(RTC_Format_BIN, &RTC_DateStructure);
			YY = RTC_DateStructure.RTC_Year;
			MM = RTC_DateStructure.RTC_Month;
			DD = RTC_DateStructure.RTC_Date;
			sprintf(date,"%0.2d/%0.2d/%0.2d",YY,MM,DD);
			LCD_DisplayStringLine(LINE(9),  (uint8_t *) date); 
				
				}
		//else{UBPressed =1;}
		}
	if((count<4000000)&&(state ==0)){UBPressed =1;}//if button press short UB = 1
	count = 0;
	
	LCD_ClearLine(LINE(9));
	
	
	//clear the pending bit otherwise the handler will fire continually
	EXTI_ClearITPendingBit(USER_BUTTON_EXTI_LINE);
	//this is configured to handle the push-button
}
void EXTI2_IRQHandler(void){//external button
	//change state
	
		
		
			state++;
			if(state ==1){LCD_DisplayStringLine(LINE(7),  (uint8_t *) "changing HH");  }
			else if(state ==2){LCD_DisplayStringLine(LINE(7),  (uint8_t *) "changing MM");}
			else if(state>=3){state =0;
							LCD_DisplayStringLine(LINE(7),  (uint8_t *) "state=0        ");
	}

	if(EXTI_GetITStatus(EXTI_Line2)!= RESET){
		
		
	/* add user-button handling code here */

	/* don't execute the ISR until the button is released */
		//while (STM_EVAL_PBGetState(BUTTON_USER) == Bit_SET){count ++;}
	
	}
		
					
				  	
						EXTI_ClearITPendingBit(EXTI_Line2);

		}
			/*STM_EVAL_LEDToggle(LED4);
			STM_EVAL_LEDToggle(LED3);}*/
	
	
	
	//EXTI_ClearITPendingBit(EXTI_Line4);



/**
 * External interrupt channel 1 for external button interrupts.
 * Think about using this put the program into 'set mode'.
 */
void EXTI4_IRQHandler(void){//according to state then have different condition
	CBPressed=1;
	//LCD_DisplayStringLine(LINE(8),  (uint8_t *) "skjdajsd");
	
	if(state ==1){//if state =1 chang hour
					RTC_GetTime(RTC_Format_BIN,&RTC_TimeStructure);
					RTC_TimeStructure.RTC_Hours++;
					if(RTC_TimeStructure.RTC_Hours>=24){RTC_TimeStructure.RTC_Hours=0;}
					RTC_SetTime(RTC_Format_BIN,&RTC_TimeStructure);
					RTC_GetTime(RTC_Format_BIN,&RTC_TimeStructure);
					hh = RTC_TimeStructure.RTC_Hours;
					mm = RTC_TimeStructure.RTC_Minutes;
					ss = RTC_TimeStructure.RTC_Seconds;
					
					sprintf(Time,"%0.2d:%0.2d:%0.2d",hh,mm,ss);
					LCD_DisplayStringLine(LINE(5),  (uint8_t *) Time); 
					
		}
				else if(state ==2){//if state =2, change minuet
					RTC_GetTime(RTC_Format_BIN,&RTC_TimeStructure);
					
					RTC_TimeStructure.RTC_Minutes++;
					if(RTC_TimeStructure.RTC_Minutes>=60){RTC_TimeStructure.RTC_Minutes=0;}
					RTC_SetTime(RTC_Format_BIN,&RTC_TimeStructure);
					RTC_GetTime(RTC_Format_BIN,&RTC_TimeStructure);
					hh = RTC_TimeStructure.RTC_Hours;
					mm = RTC_TimeStructure.RTC_Minutes;
					ss = RTC_TimeStructure.RTC_Seconds;
					
					sprintf(Time,"%0.2d:%0.2d:%0.2d",hh,mm,ss);
					LCD_DisplayStringLine(LINE(5),  (uint8_t *) Time);
					 
				}
				
				
				
	
	//clear pending bit
	EXTI_ClearITPendingBit(EXTI_Line4);
}

/**
 * External interrupt channel 3 for external button interrupts.
 * Think about using this to change the time segment when in 'set mode' and otherwise
 * to write to the EEPROM when in 'display mode'.
 */
void EXTI3_IRQHandler(void){
	//your code here
	//don't forget to clear the pending bit
}

/*
 * This can be used to handle the RTC alarm interrupts. If you are
 * using alarm A or B you can configure the alarms to trigger every second.
 */
void RTC_Alarm_IRQHandler(void){
	
}

/******************************************************************************/
/*                 STM32F4xx Peripherals Interrupt Handlers                   */
/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (startup_stm32f4xx.s).                                               */
/******************************************************************************/

/**
  * @brief  This function handles PPP interrupt request.
  * @param  None
  * @retval None
  */
/*void PPP_IRQHandler(void)
{
}*/

/**
  * @}
  */ 

/**
  * @}
  */ 

/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/
