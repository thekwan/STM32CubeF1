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
#include <stdlib.h>
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
int OutputMode = OUTPUT_MODE_LOG;

/* Private function prototypes -----------------------------------------------*/
void     SystemClock_Config(void);

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
  //Configure_DMA();
  //Configure_SPI();

  /* Motor driver controller logic init. */
  Motor_Init();

  Battery_Measure_Init();


  /* Set LED2 Off */
  LED_Off();
  LED_On();

  const char *version = "v1.5a(OutputModeTest)\n";


  //printf_uart("Hello!! ATANK MCU FW is successfully initialized.\n");
  printf_uart((char *)version);
  
  //LED_Blinking(LED_BLINK_FAST);

  /* Infinite loop */
  while (1) {
      int32_t speed, speed_result;
      char buf[256];
      if(scanf_uart(buf, 256) > 0) {
          if(strncmp(buf, "lt", 256) == 0) {
              Motor_Set_DIR_Left_Turn();
              printf_uart("Motor left turn!\n");
          }
          else if(strncmp(buf, "rt", 256) == 0) {
              Motor_Set_DIR_Right_Turn();
              printf_uart("Motor right turn!\n");
          }
          else if(strncmp(buf, "rf", 256) == 0) {
              Motor_Set_DIR_Forward();
              printf_uart("Motor run forward!\n");
          }
          else if(strncmp(buf, "rb", 256) == 0) {
              Motor_Set_DIR_Backward();
              printf_uart("Motor run backward!\n");
          }
          else if(strncmp(buf, "st", 256) == 0) {
              Motor_All_Stop();
              printf_uart("Motor all stop!\n");
          }
          else if(strncmp(buf, "reset", 256) == 0) {
              NVIC_SystemReset();
              printf_uart("FW program reset!\n");
          }
          else if(strncmp(buf, "OutputModeStop", 256) == 0) {
              OutputMode = OUTPUT_MODE_STOP;
              while (!isTxBufferFlushed());
          }
          else if(strncmp(buf, "OutputModeLog", 256) == 0) {
              while (!isTxBufferFlushed());
              OutputMode = OUTPUT_MODE_LOG;
          }
          else if(strncmp(buf, "OutputModeData", 256) == 0) {
              while (!isTxBufferFlushed());
              OutputMode = OUTPUT_MODE_DATA;
          }
#if 0
          else if(strncmp(buf, "SwitchOutput", 256) == 0) {
              if (OutputMode == OUTPUT_MODE_LOG) {
                  printf_uart("Switch output mode to 'DATA'\n");
                  while (!isTxBufferFlushed());
                  OutputMode = OUTPUT_MODE_DATA;
              } else {
                  while (!isTxBufferFlushed());
                  OutputMode = OUTPUT_MODE_LOG;
                  printf_uart("Switch output mode to 'LOG'\n");
              }
          }
          else if(strncmp(buf, "lsu", 256) == 0) {
              speed = Motor_Left_Speed_Up();
              sprintf(buf, "Motor left speed = %ld\n", speed);
              printf_uart(buf);
          }
          else if(strncmp(buf, "lsd", 256) == 0) {
              speed = Motor_Left_Speed_Down();
              sprintf(buf, "Motor left speed = %ld\n", speed);
              printf_uart(buf);
          }
          else if(strncmp(buf, "rsu", 256) == 0) {
              speed = Motor_Right_Speed_Up();
              sprintf(buf, "Motor right speed = %ld\n", speed);
              printf_uart(buf);
          }
          else if(strncmp(buf, "bsu", 256) == 0) {
              int32_t rspeed = Motor_Right_Speed_Up();
              int32_t lspeed = Motor_Left_Speed_Up();
              sprintf(buf, "Motor left/right speed = %ld / %ld\n", lspeed, rspeed);
              printf_uart(buf);
          }
          else if(strncmp(buf, "rsd", 256) == 0) {
              speed = Motor_Right_Speed_Down();
              sprintf(buf, "Motor right speed = %ld\n", speed);
              printf_uart(buf);
          }
#endif
          else if(strncmp(buf, "left_motor_speed_", 17) == 0) {
              speed = atoi(buf+17);
              speed_result = Motor_Left_Speed_Set(speed);
              sprintf(buf, "Motor left speed = %ld (%ld)\n", speed_result, speed);
              printf_uart(buf);
          }
          else if(strncmp(buf, "right_motor_speed_", 18) == 0) {
              speed = atoi(buf+18);
              speed_result = Motor_Right_Speed_Set(speed);
              sprintf(buf, "Motor right speed = %ld, (%ld)\n", speed_result, speed);
              printf_uart(buf);
          }
          else if(strncmp(buf, "set_speed_", 10) == 0) {
              speed = atoi(buf+10);
              int32_t lspeed = Motor_Left_Speed_Set(speed);
              int32_t rspeed = Motor_Right_Speed_Set(speed);
              sprintf(buf, "Motor speed(L,R) = %ld, %ld (%ld)\n", lspeed, rspeed, speed);
              printf_uart(buf);
          }
          /*
          else if(strncmp(buf, "left_speed_iir", 255) == 0) {
              uint32_t hsens_speed_iir = Motor_Get_Hsens_Speed_Left();
              sprintf(buf, "left_hsens_speed_iir  = %lu\n", hsens_speed_iir);
              printf_uart(buf);
          }
          else if(strncmp(buf, "right_speed_iir", 255) == 0) {
              uint32_t hsens_speed_iir = Motor_Get_Hsens_Speed_Right();
              sprintf(buf, "right_hsens_speed_iir = %lu\n", hsens_speed_iir);
              printf_uart(buf);
          }
          */
          else if(strncmp(buf, "version", 255) == 0) {
              printf_uart((char *)version);
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
}

/**
  * @brief  Function called in case of error detected in USART IT Handler
  * @param  None
  * @retval None
  */
void USART2_Error_Callback(void)
{
  __IO uint32_t sr_reg;

  /* Disable USARTx_IRQn */
  NVIC_DisableIRQ(USART2_IRQn);
  
  /* Error handling example :
    - Read USART SR register to identify flag that leads to IT raising
    - Perform corresponding error handling treatment according to flag
  */
  sr_reg = LL_USART_ReadReg(USART2, SR);
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

void USART3_Error_Callback(void)
{
  __IO uint32_t sr_reg;

  /* Disable USARTx_IRQn */
  NVIC_DisableIRQ(USART3_IRQn);
  
  /* Error handling example :
    - Read USART SR register to identify flag that leads to IT raising
    - Perform corresponding error handling treatment according to flag
  */
  sr_reg = LL_USART_ReadReg(USART3, SR);
  if (sr_reg & LL_USART_SR_NE)
  {
    /* case Noise Error flag is raised : ... */
    LED_Blinking(LED_BLINK_SLOW);
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
