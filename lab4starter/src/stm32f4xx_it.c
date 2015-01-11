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

/** @addtogroup STM32F4_Discovery_Peripheral_Examples
  * @{
  */

/** @addtogroup TIM_TimeBase
  * @{
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
	
uint32_t time;
uint16_t increase;
uint16_t decrease;
int count =0;
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
	
}

/******************************************************************************/
/*            STM32F4xx Peripherals Interrupt Handlers                        */
/******************************************************************************/

/**
  * @brief  This function handles TIM3 global interrupt request.
  * @param  None
  * @retval None
  */
void TIM4_IRQHandler(void)
{
	if (TIM_GetITStatus(TIM4, TIM_IT_CC1) != RESET){
	}
	TIM_ClearITPendingBit(TIM4, TIM_IT_CC1);
		count = 1;
		
		TIM_SetCounter(TIM4, 0x0000);
}

/**
 * Sets the system in set mode or display mode.
 */
void EXTI0_IRQHandler(void){
	/*while (STM_EVAL_PBGetState(BUTTON_USER) == Bit_SET){
			//time++;
			if(count ==1){time++;count = 0;}
		//time = TIM_GetCounter(TIM4);
			if(time == 5){increase =1;}
			
		}*/
	time =0;
	/* Clear the Right Button EXTI line pending bit */
  EXTI_ClearITPendingBit(USER_BUTTON_EXTI_LINE);
}

/**
 * ADC Interrupt handler
 */
void ADC_IRQHandler(void){

}
void EXTI4_IRQHandler(void){	
	//LCD_DisplayStringLine(LINE(5),  (uint8_t *) "triger");
		//TIM_SetCounter(TIM4, 0x0000); 
	
	
/*	while (GPIO_ReadInputDataBit( GPIOB,GPIO_Pin_4) == Bit_SET){
			time++;
			if(count ==1){time++;count = 0;}
		//time = TIM_GetCounter(TIM4);
			if(time == 40000){increase =1;}
			//GPIO_ReadInputDataBit( GPIOB,GPIO_Pin_4)
		}*/
	time =0;
	if(EXTI_GetITStatus(EXTI_Line4)!= RESET){
	
	}
	
	EXTI_ClearITPendingBit(EXTI_Line4);

}
void EXTI2_IRQHandler(void){//external button
	
	/*while (GPIO_ReadInputDataBit( GPIOD,GPIO_Pin_2) == Bit_SET){
			if(count ==1){time++;count = 0;}
			if(time == 5){decrease =1;}
			//GPIO_ReadInputDataBit( GPIOB,GPIO_Pin_4)
		}*/
	time =0;
		
		
			
	

	if(EXTI_GetITStatus(EXTI_Line2)!= RESET){
	}
		
					
				  	
						EXTI_ClearITPendingBit(EXTI_Line2);

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
