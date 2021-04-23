/** 
  ******************************************************************************
  * @file    Examples_LL/USART/USART_Communication_Tx_IT/Src/stm32f1xx_it.c
  * @author  MCD Application Team
  * @brief   Main Interrupt Service Routines.
  *          This file provides template for all exceptions handler and
  *          peripherals interrupt service routine.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2016 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_it.h"
#include "uart.h"
#include "motor.h"

/** @addtogroup STM32F1xx_LL_Examples
  * @{
  */

/** @addtogroup USART_Communication_Tx_IT
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/******************************************************************************/
/*            Cortex-M3 Processor Exceptions Handlers                         */
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
  {
  }
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
  {
  }
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
  {
  }
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
  {
  }
}

/**
  * @brief  This function handles SVCall exception.
  * @param  None
  * @retval None
  */
void SVC_Handler(void)
{
}

/**
  * @brief  This function handles Debug Monitor exception.
  * @param  None
  * @retval None
  */
void DebugMon_Handler(void)
{
}

/**
  * @brief  This function handles PendSVC exception.
  * @param  None
  * @retval None
  */
void PendSV_Handler(void)
{
}

/**
  * @brief  This function handles SysTick Handler.
  * @param  None
  * @retval None
  */
void SysTick_Handler(void)
{
}

/******************************************************************************/
/*                 STM32F1xx Peripherals Interrupt Handlers                   */
/*  Add here the Interrupt Handler for the used peripheral(s), for the        */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (startup_stm32f1xx.s).                                               */
/******************************************************************************/

/**
  * @brief  This function handles external lines 10 to 15 interrupt request.
  * @param  None
  * @retval None
  */
void USER_BUTTON_IRQHANDLER(void)
{
  /* Manage Flags */
  if(LL_EXTI_IsActiveFlag_0_31(USER_BUTTON_EXTI_LINE) != RESET)
  {
    /* Clear EXTI flag */
    LL_EXTI_ClearFlag_0_31(USER_BUTTON_EXTI_LINE);

    /* Handle user button press in dedicated function */
    UserButton_Callback(); 
  }
}

/**
  * Brief   This function handles USARTx Instance interrupt request.
  * Param   None
  * Retval  None
  */
void USART2_IRQHandler(void)
{
  if(LL_USART_IsEnabledIT_TXE(USART2) && LL_USART_IsActiveFlag_TXE(USART2))
  {
    /* TXE flag will be automatically cleared when writing new data in DR register */

    /* Call function in charge of handling empty DR => will lead to transmission of next character */
    USART2_TXEmpty_Callback();
  }

  if(LL_USART_IsEnabledIT_TC(USART2) && LL_USART_IsActiveFlag_TC(USART2))
  {
    /* Clear TC flag */
    LL_USART_ClearFlag_TC(USART2);
    /* Call function in charge of handling end of transmission of sent character
       and prepare next charcater transmission */
    USART2_CharTransmitComplete_Callback();
  }

  /* Check RXNE flag value in SR register */
  if(LL_USART_IsActiveFlag_RXNE(USART2) && LL_USART_IsEnabledIT_RXNE(USART2))
  {
    /* RXNE flag will be cleared by reading of DR register (done in call) */
    /* Call function in charge of handling Character reception */
    USART2_CharReception_Callback();
  }

  if(LL_USART_IsEnabledIT_ERROR(USART2) && LL_USART_IsActiveFlag_NE(USART2))
  {
    /* Call Error function */
    USART2_Error_Callback();
  }
	
}

void USART3_IRQHandler(void)
{
  if(LL_USART_IsEnabledIT_TXE(USART3) && LL_USART_IsActiveFlag_TXE(USART3))
  {
    /* TXE flag will be automatically cleared when writing new data in DR register */
    /* Call function in charge of handling empty DR => will lead to transmission of next character */
    //USART3_TXEmpty_Callback();
  }

  if(LL_USART_IsEnabledIT_TC(USART3) && LL_USART_IsActiveFlag_TC(USART3))
  {
    /* Clear TC flag */
    LL_USART_ClearFlag_TC(USART3);
    /* Call function in charge of handling end of transmission of sent character
       and prepare next charcater transmission */
    //USART3_CharTransmitComplete_Callback();
  }

  /* Check RXNE flag value in SR register */
  if(LL_USART_IsActiveFlag_RXNE(USART3) && LL_USART_IsEnabledIT_RXNE(USART3))
  {
    /* RXNE flag will be cleared by reading of DR register (done in call) */
    /* Call function in charge of handling Character reception */
    USART3_CharReception_Callback();
  }

  if(LL_USART_IsEnabledIT_ERROR(USART3) && LL_USART_IsActiveFlag_NE(USART3))
  {
    /* Call Error function */
    USART3_Error_Callback();
  }
	
}

/**
* @brief  This function handles TIM4 capture/compare interrupt.
* @param  None
* @retval None
*/
void TIM4_IRQHandler(void)
{
  /* Check whether CC1 interrupt is pending */
  if(MOTOR_TIMER_IsActive_LEFT() == 1)
  {
    /* Clear the update interrupt flag*/
    //LL_TIM_ClearFlag_CC1(TIM4);
    MOTOR_TIMER_CLEAR_FLAG_LEFT();

    /* TIM4 capture/compare interrupt processing(function defined in main.c) */
    //TimerCaptureCompare_Left();
    TimerCaptureCompareLeft();
  }
  /* Check whether CC2 interrupt is pending */
  if(MOTOR_TIMER_IsActive_RIGHT() == 1)
  {
    /* Clear the update interrupt flag*/
    //LL_TIM_ClearFlag_CC2(TIM4);
    MOTOR_TIMER_CLEAR_FLAG_RIGHT();

    /* TIM4 capture/compare interrupt processing(function defined in main.c) */
    //TimerCaptureCompare_Right();
    TimerCaptureCompareRight();
  }

}

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
