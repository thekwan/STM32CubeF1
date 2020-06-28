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
typedef struct _motor_info{
    uint32_t   id;
     int32_t   speed_rate;
    uint32_t   hsens_timer_prev;
    uint32_t   hsens_period_iir;
} MotorInfo;

/* Exported constants --------------------------------------------------------*/

/**
  * @brief  TIM3_ARR register maximum value                                            
  */
#define TIM4_ARR_MAX (uint32_t)0xFFFF


/**
  * MOTOR ID
  */

#define MOTOR_LEFT_ID                  0
#define MOTOR_RIGHT_ID                 1


/**
  * @brief MOTOR_CONTROL_PINS
  */

#define MOTOR_LEFT_PWM_PIN             LL_GPIO_PIN_8
#define MOTOR_LEFT_DIR_PIN_0           LL_GPIO_PIN_12
#define MOTOR_LEFT_DIR_PIN_1           LL_GPIO_PIN_13
#define MOTOR_LEFT_PORT                GPIOB
#define MOTOR_LEFT_PIN_CLK_ENABLE()    LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_GPIOB)


/**
  * @brief MOTOR_RIGHT_CONTROL_PINS
  */

#define MOTOR_RIGHT_PWM_PIN            LL_GPIO_PIN_9
#define MOTOR_RIGHT_DIR_PIN_0          LL_GPIO_PIN_14
#define MOTOR_RIGHT_DIR_PIN_1          LL_GPIO_PIN_15
#define MOTOR_RIGHT_PORT               GPIOB
#define MOTOR_RIGHT_PIN_CLK_ENABLE()   LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_GPIOB)


/*
 * MOTOR PWM/HoleSensor Timer
 */
#define MOTOR_TIMER                    TIM4
#define MOTOR_TIMER_ARR_MAX            TIM4_ARR_MAX
#define MOTOR_TIMER_IRQn               TIM4_IRQn
#define MOTOR_TIMER_CLK_ENABLE()       LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM4); 
#define MOTOR_TIMER_CH_LEFT_HSENS      LL_TIM_CHANNEL_CH1
#define MOTOR_TIMER_CH_RIGHT_HSENS     LL_TIM_CHANNEL_CH2
#define MOTOR_TIMER_CH_LEFT_PWM        LL_TIM_CHANNEL_CH3
#define MOTOR_TIMER_CH_RIGHT_PWM       LL_TIM_CHANNEL_CH4
#define MOTOR_TIMER_CAPTURE_LEFT()     LL_TIM_IC_GetCaptureCH1(MOTOR_TIMER)
#define MOTOR_TIMER_CAPTURE_RIGHT()    LL_TIM_IC_GetCaptureCH2(MOTOR_TIMER)
#define MOTOR_TIMER_OC_SetCompLEFT     LL_TIM_OC_SetCompareCH3
#define MOTOR_TIMER_OC_SetCompRIGHT    LL_TIM_OC_SetCompareCH4

#define MOTOR_TIMER_CLEAR_FLAG_LEFT()  LL_TIM_ClearFlag_CC1(MOTOR_TIMER)
#define MOTOR_TIMER_CLEAR_FLAG_RIGHT() LL_TIM_ClearFlag_CC2(MOTOR_TIMER)
#define MOTOR_TIMER_IsActive_LEFT()    LL_TIM_IsActiveFlag_CC1(MOTOR_TIMER)
#define MOTOR_TIMER_IsActive_RIGHT()   LL_TIM_IsActiveFlag_CC2(MOTOR_TIMER)

/*
 * MOTOR Hole-sensor PINS
 */
#define MOTOR_LEFT_HSENS_PIN           LL_GPIO_PIN_6
#define MOTOR_RIGHT_HSENS_PIN          LL_GPIO_PIN_7

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
void     Motor_Set_DIR_Left_Turn(void);
void     Motor_Set_DIR_Right_Turn(void);
void     Motor_Set_DIR_Forward(void);
void     Motor_Set_DIR_Backward(void);
void     Motor_Left_Stop(void);
void     Motor_Right_Stop(void);
void     Motor_All_Stop(void);

int32_t Motor_Left_Speed_Up(void);
int32_t Motor_Right_Speed_Up(void);
int32_t Motor_Left_Speed_Down(void);
int32_t Motor_Right_Speed_Down(void);

uint32_t Motor_Get_Hsens_Speed_Left(void);
uint32_t Motor_Get_Hsens_Speed_Right(void);

void     Motor_Timer_Init(void);
void     TimerCaptureCompareLeft(void);
void     TimerCaptureCompareRight(void);

#endif /* __MOTOR_H__ */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
