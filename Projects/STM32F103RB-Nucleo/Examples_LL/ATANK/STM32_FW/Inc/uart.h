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

   
/* USART2 instance(cUart) is used as Command line. (TX on PA.02, RX on PA.03)
 */
#define cUSARTx_INSTANCE               USART2
#define cUSARTx_CLK_ENABLE()           LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_USART2)
#define cUSARTx_CLK_SOURCE()           LL_RCC_SetUSARTClockSource(LL_RCC_USART2_CLKSOURCE_PCLK1)
#define cUSARTx_IRQn                   USART2_IRQn
#define cUSARTx_IRQHandler             USART2_IRQHandler

#define cUSARTx_GPIO_CLK_ENABLE()      LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_GPIOA)   /* Enable the peripheral clock of GPIOA */
#define cUSARTx_TX_PIN                 LL_GPIO_PIN_2
#define cUSARTx_TX_GPIO_PORT           GPIOA
#define cUSARTx_SET_TX_GPIO_AF()       LL_GPIO_SetAFPin_0_7(GPIOA, LL_GPIO_PIN_2, LL_GPIO_AF_7)
#define cUSARTx_RX_PIN                 LL_GPIO_PIN_3
#define cUSARTx_RX_GPIO_PORT           GPIOA


/* USART3 instance(lUart) is used as Lidar comm.   (TX not used, RX on PB.11)
 */
#define lUSARTx_INSTANCE               USART3
#define lUSARTx_CLK_ENABLE()           LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_USART3)
#define lUSARTx_CLK_SOURCE()           LL_RCC_SetUSARTClockSource(LL_RCC_USART3_CLKSOURCE_PCLK1)
#define lUSARTx_IRQn                   USART3_IRQn
#define lUSARTx_IRQHandler             USART3_IRQHandler
#define lUSARTx_GPIO_CLK_ENABLE()      LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_GPIOB)   /* Enable the peripheral clock of GPIOA */
//#define lUSARTx_TX_PIN                 LL_GPIO_PIN_x
//#define lUSARTx_TX_GPIO_PORT           GPIOA
//#define lUSARTx_SET_TX_GPIO_AF()       LL_GPIO_SetAFPin_0_7(GPIOA, LL_GPIO_PIN_2, LL_GPIO_AF_7)
#define lUSARTx_RX_PIN                 LL_GPIO_PIN_11
#define lUSARTx_RX_GPIO_PORT           GPIOB


#define APB_Div 2



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
