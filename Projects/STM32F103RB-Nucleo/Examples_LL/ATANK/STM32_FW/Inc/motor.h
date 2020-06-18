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
#ifndef __MOTOR_H__
#define __MOTOR_H__

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_ll_bus.h"
#include "stm32f1xx_ll_rcc.h"
#include "stm32f1xx_ll_system.h"
#include "stm32f1xx_ll_utils.h"
#include "stm32f1xx_ll_gpio.h"
#include "stm32f1xx_ll_exti.h"
#include "stm32f1xx_ll_pwr.h"
#include "stm32f1xx_ll_tim.h"
#if defined(USE_FULL_ASSERT)
#include "stm32_assert.h"
#endif /* USE_FULL_ASSERT */

/* Exported types ------------------------------------------------------------*/
typedef struct _hsensor_list {
    uint32_t meas[16];
    uint32_t diff[16];
    int32_t index;
} hsens_list;

/* Exported constants --------------------------------------------------------*/

/**
  * @brief  TIM3_ARR register maximum value                                            
  */
#define TIM4_ARR_MAX (uint32_t)0xFFFF


/**
  * @brief MOTOR_LEFT_CONTROL_PINS
  */

#define MOTOR_LEFT_HSENS               LL_GPIO_PIN_6
#define MOTOR_LEFT_PWM                 LL_GPIO_PIN_8
#define MOTOR_LEFT_PIN_0               LL_GPIO_PIN_12
#define MOTOR_LEFT_PIN_1               LL_GPIO_PIN_13
#define MOTOR_LEFT_PORT                GPIOB
#define MOTOR_LEFT_CLK_ENABLE()        LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_GPIOB)


/**
  * @brief MOTOR_RIGHT_CONTROL_PINS
  */

#define MOTOR_RIGHT_HSENS              LL_GPIO_PIN_7
#define MOTOR_RIGHT_PWM                LL_GPIO_PIN_9
#define MOTOR_RIGHT_PIN_0              LL_GPIO_PIN_14
#define MOTOR_RIGHT_PIN_1              LL_GPIO_PIN_15
#define MOTOR_RIGHT_PORT               GPIOB
#define MOTOR_RIGHT_CLK_ENABLE()       LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_GPIOB)

/**
  * @brief Toggle periods for various blinking modes
  */

//#define LED_BLINK_FAST  200
//#define LED_BLINK_SLOW  500
//#define LED_BLINK_ERROR 1000


/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */
/* IRQ Handler treatment functions */
/* Public API --------------------------------------------------------------- */

void     Motor_Init(void);
void     Motor_Left_Turn(void);
void     Motor_Right_Turn(void);
void     Motor_Run_Forward(void);
void     Motor_Run_Backward(void);
void     Motor_Left_Stop(void);
void     Motor_Right_Stop(void);
void     Motor_All_Stop(void);
uint32_t Motor_Left_Speed_Up(void);
uint32_t Motor_Left_Speed_Down(void);
uint32_t Motor_Right_Speed_Up(void);
uint32_t Motor_Right_Speed_Down(void);
void     Motor_Timer_Init(void);
void     getTimerCaptureLeft(hsens_list **ptr);
void     getTimerCaptureRight(hsens_list **ptr);
void     TimerCaptureCompare_Left(void);
void     TimerCaptureCompare_Right(void);

#endif /* __MOTOR_H__ */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
