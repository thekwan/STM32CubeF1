/**
  ******************************************************************************
  * @file    Examples_LL/USART/USART_Communication_Tx_IT/Src/main.c
  * @author  MCD Application Team
  * @brief   This example describes how to send bytes over USART IP using
  *          the STM32F1xx USART LL API.
  *          Peripheral initialization done using LL unitary services functions.
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
#include <stdio.h>
#include <string.h>
#include "main.h"
#include "uart.h"
#include "led.h"
#include "motor.h"
#include "battery.h"

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
__IO uint8_t ubButtonPress = 0;


/* Private function prototypes -----------------------------------------------*/
void     SystemClock_Config(void);
void     Configure_USART(void);

/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Main program
  * @param  None
  * @retval None
  */
int main(void)
{
  /* Configure the system clock to 72 MHz */
  SystemClock_Config();

  /* Initialize LED2 */
  LED_Init();

  /* Configure USARTx (USART IP configuration and related GPIO initialization) */
  Configure_USART();

  /* Motor driver controller logic init. */
  Motor_Init();

  Battery_Measure_Init();


  /* Set LED2 Off */
  LED_Off();
  LED_On();

#if 0
  /* TEST_CODE */
  while(1) {
      printf_uart("Motor control test!\n");
      LL_mDelay(1000);
  }
#endif
  

  //UserButton_Callback();
  printf_uart("Hello!! ATANK MCU FW is successfully initialized.\n");
  
  //LED_Blinking(LED_BLINK_FAST);

  /* Infinite loop */
  while (1) {
      char buf[256];
      if(scanf_uart(buf, 256) > 0) {
          if(strncmp(buf, "lt", 256) == 0) {
              Motor_Left_Turn();
              printf_uart("Motor left turn!\n");
          }
          else if(strncmp(buf, "rt", 256) == 0) {
              Motor_Right_Turn();
              printf_uart("Motor right turn!\n");
          }
          else if(strncmp(buf, "rf", 256) == 0) {
              Motor_Run_Forward();
              printf_uart("Motor run forward!\n");
          }
          else if(strncmp(buf, "rb", 256) == 0) {
              Motor_Run_Backward();
              printf_uart("Motor run backward!\n");
          }
          else if(strncmp(buf, "st", 256) == 0) {
              Motor_All_Stop();
              printf_uart("Motor all stop!\n");
          }
          else if(strncmp(buf, "bv", 256) == 0) {
              uint16_t volt_list[16];
              // volt_list is only for DEBUG
              uint16_t volt = Get_Battery_Voltage(volt_list);
              uint16_t v1, v2;
              v1 = volt/1000;
              v2 = volt - (v1*1000);
              sprintf(buf, "Main battery voltage = %d.%d (V)\n", v1, v2);
              printf_uart(buf);
#if 0
              // DEBUG
              int i;
              printf_uart("volt_list\n");
              for(i=0; i < 16; i++) {
                sprintf(buf, "%d\n", volt_list[i]);
                printf_uart(buf);
              }
#endif
          }
          else {
              sprintf(buf, "Unkown command...\n");
              printf_uart(buf);
          }
      }
  }
}


/**
  * @brief  System Clock Configuration
  *         The system Clock is configured as follow :
  *            System Clock source            = PLL (HSE)
  *            SYSCLK(Hz)                     = 72000000
  *            HCLK(Hz)                       = 72000000
  *            AHB Prescaler                  = 1
  *            APB1 Prescaler                 = 2
  *            APB2 Prescaler                 = 1
  *            HSE Frequency(Hz)              = 8000000
  *            PLLMUL                         = 9
  *            Flash Latency(WS)              = 2
  * @param  None
  * @retval None
  */
void SystemClock_Config(void)
{
  /* Set FLASH latency */
  LL_FLASH_SetLatency(LL_FLASH_LATENCY_2);

  /* Enable HSE oscillator */
  //LL_RCC_HSE_EnableBypass();
  LL_RCC_HSE_DisableBypass();
  LL_RCC_HSE_Enable();
  while(LL_RCC_HSE_IsReady() != 1)
  {
  };

  /* Main PLL configuration and activation */
  LL_RCC_PLL_ConfigDomain_SYS(LL_RCC_PLLSOURCE_HSE_DIV_1, LL_RCC_PLL_MUL_9);

  LL_RCC_PLL_Enable();
  while(LL_RCC_PLL_IsReady() != 1)
  {
  };

  /* Sysclk activation on the main PLL */
  LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);
  LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_PLL);
  while(LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_PLL)
  {
  };

  /* Set APB1 & APB2 prescaler*/
  LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_2);
  LL_RCC_SetAPB2Prescaler(LL_RCC_APB2_DIV_1);

  /* Set systick to 1ms in using frequency set to 72MHz */
  LL_Init1msTick(72000000);

  /* Update CMSIS variable (which can be updated also through SystemCoreClockUpdate function) */
  LL_SetSystemCoreClock(72000000);
}

/******************************************************************************/
/*   IRQ HANDLER TREATMENT Functions                                          */
/******************************************************************************/
/**
  * @brief  Function to manage Button push
  * @param  None
  * @retval None
  */
void UserButton_Callback(void)
{
  /* Start transfer only if not already ongoing */
  if (ubSend == 0)
  {
    /* Start USART transmission : Will initiate TXE interrupt after DR register is empty */
    //LL_USART_TransmitData8(USARTx_INSTANCE, aStringToSend[ubSend++]); 

    /* Enable TXE interrupt */
    //LL_USART_EnableIT_TXE(USARTx_INSTANCE); 
  }
}

/**
  * @brief  Function called in case of error detected in USART IT Handler
  * @param  None
  * @retval None
  */
void Error_Callback(void)
{
  __IO uint32_t sr_reg;

  /* Disable USARTx_IRQn */
  NVIC_DisableIRQ(USARTx_IRQn);
  
  /* Error handling example :
    - Read USART SR register to identify flag that leads to IT raising
    - Perform corresponding error handling treatment according to flag
  */
  sr_reg = LL_USART_ReadReg(USARTx_INSTANCE, SR);
  if (sr_reg & LL_USART_SR_NE)
  {
    /* case Noise Error flag is raised : ... */
    LED_Blinking(LED_BLINK_FAST);
  }
  else
  {
    /* Unexpected IT source : Set LED to Blinking mode to indicate error occurs */
    LED_Blinking(LED_BLINK_ERROR);
  }
}

#ifdef  USE_FULL_ASSERT

/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d", file, line) */

  /* Infinite loop */
  while (1)
  {
  }
}
#endif

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
