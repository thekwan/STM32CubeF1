/**
  ******************************************************************************
  * @file    uart.c
  * @author  Deokhwan, Kim
  * @brief   UART HL API.
  ******************************************************************************
  * @attention
  *
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "motor.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
uint32_t motor_speed_rate_left = 0;
uint32_t motor_speed_rate_right = 0;
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Initialize Motor driver
  * @param  None
  * @retval None
  */
void Motor_Init(void)
{
  /* Enable the LED2 Clock */
  MOTOR_LEFT_CLK_ENABLE();
  MOTOR_RIGHT_CLK_ENABLE();

  /* Configure IO in output push-pull mode to drive external LED2 */
  LL_GPIO_SetPinMode(MOTOR_LEFT_PORT , MOTOR_LEFT_PWM   , LL_GPIO_MODE_ALTERNATE);
  LL_GPIO_SetPinMode(MOTOR_LEFT_PORT , MOTOR_LEFT_PIN_0 , LL_GPIO_MODE_OUTPUT);
  LL_GPIO_SetPinMode(MOTOR_LEFT_PORT , MOTOR_LEFT_PIN_1 , LL_GPIO_MODE_OUTPUT);
  LL_GPIO_SetPinMode(MOTOR_RIGHT_PORT, MOTOR_RIGHT_PWM  , LL_GPIO_MODE_ALTERNATE);
  LL_GPIO_SetPinMode(MOTOR_RIGHT_PORT, MOTOR_RIGHT_PIN_0, LL_GPIO_MODE_OUTPUT);
  LL_GPIO_SetPinMode(MOTOR_RIGHT_PORT, MOTOR_RIGHT_PIN_1, LL_GPIO_MODE_OUTPUT);
  /* Reset value is LL_GPIO_OUTPUT_PUSHPULL */
  LL_GPIO_SetPinOutputType(MOTOR_LEFT_PORT , MOTOR_LEFT_PWM   , LL_GPIO_PULL_DOWN      );
  LL_GPIO_SetPinOutputType(MOTOR_LEFT_PORT , MOTOR_LEFT_PIN_0 , LL_GPIO_OUTPUT_PUSHPULL);
  LL_GPIO_SetPinOutputType(MOTOR_LEFT_PORT , MOTOR_LEFT_PIN_1 , LL_GPIO_OUTPUT_PUSHPULL);
  LL_GPIO_SetPinOutputType(MOTOR_RIGHT_PORT, MOTOR_RIGHT_PWM  , LL_GPIO_PULL_DOWN      );
  LL_GPIO_SetPinOutputType(MOTOR_RIGHT_PORT, MOTOR_RIGHT_PIN_0, LL_GPIO_OUTPUT_PUSHPULL);
  LL_GPIO_SetPinOutputType(MOTOR_RIGHT_PORT, MOTOR_RIGHT_PIN_1, LL_GPIO_OUTPUT_PUSHPULL);
  /* Reset value is LL_GPIO_SPEED_FREQ_LOW */
  LL_GPIO_SetPinSpeed(MOTOR_LEFT_PORT , MOTOR_LEFT_PWM   , LL_GPIO_SPEED_FREQ_HIGH);
  LL_GPIO_SetPinSpeed(MOTOR_LEFT_PORT , MOTOR_LEFT_PIN_0 , LL_GPIO_SPEED_FREQ_LOW);
  LL_GPIO_SetPinSpeed(MOTOR_LEFT_PORT , MOTOR_LEFT_PIN_1 , LL_GPIO_SPEED_FREQ_LOW);
  LL_GPIO_SetPinSpeed(MOTOR_RIGHT_PORT, MOTOR_RIGHT_PWM  , LL_GPIO_SPEED_FREQ_HIGH);
  LL_GPIO_SetPinSpeed(MOTOR_RIGHT_PORT, MOTOR_RIGHT_PIN_0, LL_GPIO_SPEED_FREQ_LOW);
  LL_GPIO_SetPinSpeed(MOTOR_RIGHT_PORT, MOTOR_RIGHT_PIN_1, LL_GPIO_SPEED_FREQ_LOW);
  /* Reset value is LL_GPIO_PULL_NO */
  //LL_GPIO_SetPinPull(LED2_GPIO_PORT, LED2_PIN, LL_GPIO_PULL_NO);
  
  /***********************************************/
  /* Configure the NVIC to handle TIM4 interrupt */
  /***********************************************/
  //NVIC_SetPriority(TIM4_IRQn, 0);
  //NVIC_EnableIRQ(TIM4_IRQn);

  Motor_PWM_Timer_Init();
}

void Motor_PWM_Timer_Init(void) {
  /******************************/
  /* Peripheral clocks enabling */
  /******************************/
  /* Enable the timer peripheral clock */
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM4); 
  
  /***************************/
  /* Time base configuration */
  /***************************/
  /* Set counter mode */
  /* Reset value is LL_TIM_COUNTERMODE_UP */
  //LL_TIM_SetCounterMode(TIM4, LL_TIM_COUNTERMODE_UP);
  
  /* Set the pre-scaler value to have TIM4 counter clock equal to 10 kHz */
  LL_TIM_SetPrescaler(TIM4, __LL_TIM_CALC_PSC(SystemCoreClock, 10000));
  
  /* Enable TIM2_ARR register preload. Writing to or reading from the         */
  /* auto-reload register accesses the preload register. The content of the   */
  /* preload register are transferred into the shadow register at each update */
  /* event (UEV).                                                             */  
  LL_TIM_EnableARRPreload(TIM4);
  
  /* Set the auto-reload value to have a counter frequency of 100 Hz */
  /* TIM2CLK = SystemCoreClock / (APB prescaler & multiplier)               */
  uint32_t TimOutClock = SystemCoreClock/1;
  LL_TIM_SetAutoReload(TIM4, __LL_TIM_CALC_ARR(TimOutClock, LL_TIM_GetPrescaler(TIM4), 100));
  
  /*********************************/
  /* Output waveform configuration */
  /*********************************/
  /* Set output mode */
  /* Reset value is LL_TIM_OCMODE_FROZEN */
  LL_TIM_OC_SetMode(TIM4, LL_TIM_CHANNEL_CH3, LL_TIM_OCMODE_PWM1);
  LL_TIM_OC_SetMode(TIM4, LL_TIM_CHANNEL_CH4, LL_TIM_OCMODE_PWM1);
  
  /* Set output channel polarity */
  /* Reset value is LL_TIM_OCPOLARITY_HIGH */
  //LL_TIM_OC_SetPolarity(TIM4, LL_TIM_CHANNEL_CH1, LL_TIM_OCPOLARITY_HIGH);
  
  /* Set compare value to half of the counter period (50% duty cycle ) */
  LL_TIM_OC_SetCompareCH3(TIM4, ( (LL_TIM_GetAutoReload(TIM4) + 1 ) / 3));
  LL_TIM_OC_SetCompareCH4(TIM4, ( (LL_TIM_GetAutoReload(TIM4) + 1 ) / 3));
  
  /* Enable TIM2_CCR1 register preload. Read/Write operations access the      */
  /* preload register. TIM2_CCR1 preload value is loaded in the active        */
  /* at each update event.                                                    */
  LL_TIM_OC_EnablePreload(TIM4, LL_TIM_CHANNEL_CH3);
  LL_TIM_OC_EnablePreload(TIM4, LL_TIM_CHANNEL_CH4);
  
  /**************************/
  /* TIM4 interrupts set-up */
  /**************************/
  /* Enable the capture/compare interrupt for channel 1*/
  //LL_TIM_EnableIT_CC1(TIM4);
  
  /**********************************/
  /* Start output signal generation */
  /**********************************/
  /* Enable output channel 1 */
  LL_TIM_CC_EnableChannel(TIM4, LL_TIM_CHANNEL_CH3);
  LL_TIM_CC_EnableChannel(TIM4, LL_TIM_CHANNEL_CH4);
  
  /* Enable counter */
  LL_TIM_EnableCounter(TIM4);
  
  /* Force update generation */
  LL_TIM_GenerateEvent_UPDATE(TIM4);

}

/**
  * @brief  Changes the duty cycle of the PWM signal.
  *         D = (T/P)*100
  *           where T is the pulse duration and P is the PWM signal period
  * @param  D Duty cycle
  * @retval None
  */
__STATIC_INLINE void Configure_DutyCycle_LEFT(uint32_t D)
{
  uint32_t P;    /* Pulse duration */
  uint32_t T;    /* PWM signal period */
  
  /* PWM signal period is determined by the value of the auto-reload register */
  T = LL_TIM_GetAutoReload(TIM4) + 1;
  
  /* Pulse duration is determined by the value of the compare register.       */
  /* Its value is calculated in order to match the requested duty cycle.      */
  P = (D*T)/100;
  LL_TIM_OC_SetCompareCH3(TIM4, P);
}

__STATIC_INLINE void Configure_DutyCycle_RIGHT(uint32_t D)
{
  uint32_t P;    /* Pulse duration */
  uint32_t T;    /* PWM signal period */
  
  /* PWM signal period is determined by the value of the auto-reload register */
  T = LL_TIM_GetAutoReload(TIM4) + 1;
  
  /* Pulse duration is determined by the value of the compare register.       */
  /* Its value is calculated in order to match the requested duty cycle.      */
  P = (D*T)/100;
  LL_TIM_OC_SetCompareCH4(TIM4, P);
}


/**
  * @brief  
  * @param  None
  * @retval None
  */
void Motor_Left_Turn(void)
{
  /*  */
  LL_GPIO_ResetOutputPin(MOTOR_LEFT_PORT, MOTOR_LEFT_PIN_0);
  LL_GPIO_SetOutputPin  (MOTOR_LEFT_PORT, MOTOR_LEFT_PIN_1);

  /*  */
  LL_GPIO_SetOutputPin  (MOTOR_RIGHT_PORT, MOTOR_RIGHT_PIN_0);
  LL_GPIO_ResetOutputPin(MOTOR_RIGHT_PORT, MOTOR_RIGHT_PIN_1);
}

/**
  * @brief  
  * @param  None
  * @retval None
  */
void Motor_Right_Turn(void)
{
  /*  */
  LL_GPIO_SetOutputPin  (MOTOR_LEFT_PORT, MOTOR_LEFT_PIN_0);
  LL_GPIO_ResetOutputPin(MOTOR_LEFT_PORT, MOTOR_LEFT_PIN_1);

  /*  */
  LL_GPIO_ResetOutputPin(MOTOR_RIGHT_PORT, MOTOR_RIGHT_PIN_0);
  LL_GPIO_SetOutputPin  (MOTOR_RIGHT_PORT, MOTOR_RIGHT_PIN_1);
}

/**
  * @brief  
  * @param  None
  * @retval None
  */
void Motor_Run_Forward(void)
{
  /*  */
  LL_GPIO_ResetOutputPin(MOTOR_LEFT_PORT, MOTOR_LEFT_PIN_0);
  LL_GPIO_SetOutputPin  (MOTOR_LEFT_PORT, MOTOR_LEFT_PIN_1);

  /*  */
  LL_GPIO_ResetOutputPin(MOTOR_RIGHT_PORT, MOTOR_RIGHT_PIN_0);
  LL_GPIO_SetOutputPin  (MOTOR_RIGHT_PORT, MOTOR_RIGHT_PIN_1);
}

/**
  * @brief  
  * @param  None
  * @retval None
  */
void Motor_Run_Backward(void)
{
  /*  */
  LL_GPIO_SetOutputPin  (MOTOR_LEFT_PORT, MOTOR_LEFT_PIN_0);
  LL_GPIO_ResetOutputPin(MOTOR_LEFT_PORT, MOTOR_LEFT_PIN_1);

  /*  */
  LL_GPIO_SetOutputPin  (MOTOR_RIGHT_PORT, MOTOR_RIGHT_PIN_0);
  LL_GPIO_ResetOutputPin(MOTOR_RIGHT_PORT, MOTOR_RIGHT_PIN_1);
}

/**
  * @brief  
  * @param  None
  * @retval None
  */
void Motor_Left_Stop(void)
{
  /*  */
  LL_GPIO_ResetOutputPin(MOTOR_LEFT_PORT, MOTOR_LEFT_PIN_0);
  LL_GPIO_ResetOutputPin(MOTOR_LEFT_PORT, MOTOR_LEFT_PIN_1);
}

/**
  * @brief  
  * @param  None
  * @retval None
  */
void Motor_Right_Stop(void)
{
  /*  */
  LL_GPIO_ResetOutputPin(MOTOR_RIGHT_PORT, MOTOR_RIGHT_PIN_0);
  LL_GPIO_ResetOutputPin(MOTOR_RIGHT_PORT, MOTOR_RIGHT_PIN_1);
}

/**
  * @brief  
  * @param  None
  * @retval None
  */
void Motor_All_Stop(void)
{
    Motor_Left_Stop();
    Motor_Right_Stop();
}

uint32_t Motor_Left_Speed_Up(void) {
    if(motor_speed_rate_left < 100) {
        motor_speed_rate_left += 10;
    }
    Configure_DutyCycle_LEFT(motor_speed_rate_left);

    return motor_speed_rate_left;
}

uint32_t Motor_Left_Speed_Down(void) {
    if(motor_speed_rate_left > 0) {
        motor_speed_rate_left -= 10;
    }
    Configure_DutyCycle_LEFT(motor_speed_rate_left);

    return motor_speed_rate_left;
}

uint32_t Motor_Right_Speed_Up(void) {
    if(motor_speed_rate_right < 100) {
        motor_speed_rate_right += 10;
    }
    Configure_DutyCycle_RIGHT(motor_speed_rate_right);

    return motor_speed_rate_right;
}

uint32_t Motor_Right_Speed_Down(void) {
    if(motor_speed_rate_right > 0) {
        motor_speed_rate_right -= 10;
    }
    Configure_DutyCycle_RIGHT(motor_speed_rate_right);

    return motor_speed_rate_right;
}


/**
  * @}
  */

/**
  * @}
  */
/************************ (C) Deokhwan, Kim *****END OF FILE****/
