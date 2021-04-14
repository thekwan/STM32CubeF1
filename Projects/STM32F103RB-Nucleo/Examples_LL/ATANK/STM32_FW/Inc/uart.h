/**
  ******************************************************************************
  * @file    uart.h
  * @author  Deokhwan, Kim
  * @brief   Header for uart.c module
  ******************************************************************************
  * @attention
  *
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __UART_H__
#define __UART_H__

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_ll_bus.h"
#include "stm32f1xx_ll_rcc.h"
#include "stm32f1xx_ll_system.h"
#include "stm32f1xx_ll_utils.h"
#include "stm32f1xx_ll_gpio.h"
#include "stm32f1xx_ll_exti.h"
#include "stm32f1xx_ll_usart.h"
#include "stm32f1xx_ll_pwr.h"
#if defined(USE_FULL_ASSERT)
#include "stm32_assert.h"
#endif /* USE_FULL_ASSERT */

/* Exported types ------------------------------------------------------------*/

/* Exported constants --------------------------------------------------------*/

/* Define used to enable Virtual Com Port use : 
     USE_VCP_CONNECTION == 0
       USART1 instance is used. (TX on PA.09, RX on PA.10)
       (requires wiring USART1 TX/Rx Pins to PC connection (could be achieved thanks to a USB to UART adapter)
     USE_VCP_CONNECTION == 1
       USART2 instance is used. (TX on PA.02, RX on PA.03)
       (please ensure that USART communication between the target MCU and ST-LINK MCU is properly enabled 
       on HW board in order to support Virtual Com Port)
*/
#define USE_VCP_CONNECTION       1

/* Private definitions covering GPIO clock and USART pins 
   depending on selected USART instance. */
#if (USE_VCP_CONNECTION == 0) 

/* USART1 instance is used. (TX on PA.09, RX on PA.10)
   (requires wiring USART1 TX/Rx Pins to USB to UART adapter) */
#define USARTx_INSTANCE               USART1
#define USARTx_CLK_ENABLE()           LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_USART1)
#define USARTx_IRQn                   USART1_IRQn
#define USARTx_IRQHandler             USART1_IRQHandler

#define USARTx_GPIO_CLK_ENABLE()      LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_GPIOA)   /* Enable the peripheral clock of GPIOA */
#define USARTx_TX_PIN                 LL_GPIO_PIN_9
#define USARTx_TX_GPIO_PORT           GPIOA
#define USARTx_RX_PIN                 LL_GPIO_PIN_10
#define USARTx_RX_GPIO_PORT           GPIOA
#define APB_Div 1

#else
    
/* USART2 instance is used. (TX on PA.02, RX on PA.03)
   (please ensure that USART communication between the target MCU and ST-LINK MCU is properly enabled 
    on HW board in order to support Virtual Com Port) */
#define USARTx_INSTANCE               USART2
#define USARTx_CLK_ENABLE()           LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_USART2)
#define USARTx_CLK_SOURCE()           LL_RCC_SetUSARTClockSource(LL_RCC_USART2_CLKSOURCE_PCLK1)
#define USARTx_IRQn                   USART2_IRQn
#define USARTx_IRQHandler             USART2_IRQHandler

#define USARTx_GPIO_CLK_ENABLE()      LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_GPIOA)   /* Enable the peripheral clock of GPIOA */
#define USARTx_TX_PIN                 LL_GPIO_PIN_2
#define USARTx_TX_GPIO_PORT           GPIOA
#define USARTx_SET_TX_GPIO_AF()       LL_GPIO_SetAFPin_0_7(GPIOA, LL_GPIO_PIN_2, LL_GPIO_AF_7)
#define USARTx_RX_PIN                 LL_GPIO_PIN_3
#define USARTx_RX_GPIO_PORT           GPIOA
#define APB_Div 2

#endif /* (USE_VCP_CONNECTION == 0) */


/* Exported macro ------------------------------------------------------------*/

/* Exported functions ------------------------------------------------------- */
/* IRQ Handler treatment functions */
void Configure_USART(void);
void USART_TXEmpty_Callback(void); 
void USART_CharTransmitComplete_Callback(void); 
void USART_CharReception_Callback(void);


/* Public API --------------------------------------------------------------- */
int printf_uart(char *string);
int scanf_uart(char *buf, int buf_size);

#endif /* __UART_H__ */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
