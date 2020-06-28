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
#include "led.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
uint32_t motor_speed_rate_left = 0;
uint32_t motor_speed_rate_right = 0;
uint32_t motor_speed_rate_max  =  100;
uint32_t motor_speed_rate_min  = -100;
uint32_t motor_speed_rate_thr  =  30;
uint32_t motor_speed_rate_step =  10;

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
  MOTOR_LEFT_PIN_CLK_ENABLE();
  MOTOR_RIGHT_PIN_CLK_ENABLE();

  /* Configure IO in output push-pull mode to drive external LED2 */
  LL_GPIO_SetPinMode(MOTOR_LEFT_PORT , MOTOR_LEFT_HSENS_PIN , LL_GPIO_MODE_INPUT);
  LL_GPIO_SetPinMode(MOTOR_LEFT_PORT , MOTOR_LEFT_PWM_PIN   , LL_GPIO_MODE_ALTERNATE);
  LL_GPIO_SetPinMode(MOTOR_LEFT_PORT , MOTOR_LEFT_DIR_PIN_0 , LL_GPIO_MODE_OUTPUT);
  LL_GPIO_SetPinMode(MOTOR_LEFT_PORT , MOTOR_LEFT_DIR_PIN_1 , LL_GPIO_MODE_OUTPUT);
  LL_GPIO_SetPinMode(MOTOR_RIGHT_PORT, MOTOR_RIGHT_HSENS_PIN, LL_GPIO_MODE_INPUT);
  LL_GPIO_SetPinMode(MOTOR_RIGHT_PORT, MOTOR_RIGHT_PWM_PIN  , LL_GPIO_MODE_ALTERNATE);
  LL_GPIO_SetPinMode(MOTOR_RIGHT_PORT, MOTOR_RIGHT_DIR_PIN_0, LL_GPIO_MODE_OUTPUT);
  LL_GPIO_SetPinMode(MOTOR_RIGHT_PORT, MOTOR_RIGHT_DIR_PIN_1, LL_GPIO_MODE_OUTPUT);
  /* Reset value is LL_GPIO_OUTPUT_PUSHPULL */
  LL_GPIO_SetPinOutputType(MOTOR_LEFT_PORT , MOTOR_LEFT_HSENS_PIN , LL_GPIO_PULL_DOWN      );
  LL_GPIO_SetPinOutputType(MOTOR_LEFT_PORT , MOTOR_LEFT_PWM_PIN   , LL_GPIO_PULL_DOWN      );
  LL_GPIO_SetPinOutputType(MOTOR_LEFT_PORT , MOTOR_LEFT_DIR_PIN_0 , LL_GPIO_OUTPUT_PUSHPULL);
  LL_GPIO_SetPinOutputType(MOTOR_LEFT_PORT , MOTOR_LEFT_DIR_PIN_1 , LL_GPIO_OUTPUT_PUSHPULL);
  LL_GPIO_SetPinOutputType(MOTOR_RIGHT_PORT, MOTOR_RIGHT_HSENS_PIN, LL_GPIO_PULL_DOWN      );
  LL_GPIO_SetPinOutputType(MOTOR_RIGHT_PORT, MOTOR_RIGHT_PWM_PIN  , LL_GPIO_PULL_DOWN      );
  LL_GPIO_SetPinOutputType(MOTOR_RIGHT_PORT, MOTOR_RIGHT_DIR_PIN_0, LL_GPIO_OUTPUT_PUSHPULL);
  LL_GPIO_SetPinOutputType(MOTOR_RIGHT_PORT, MOTOR_RIGHT_DIR_PIN_1, LL_GPIO_OUTPUT_PUSHPULL);
  /* Reset value is LL_GPIO_SPEED_FREQ_LOW */
  LL_GPIO_SetPinSpeed(MOTOR_LEFT_PORT , MOTOR_LEFT_PWM_PIN   , LL_GPIO_SPEED_FREQ_HIGH);
  LL_GPIO_SetPinSpeed(MOTOR_LEFT_PORT , MOTOR_LEFT_DIR_PIN_0 , LL_GPIO_SPEED_FREQ_LOW);
  LL_GPIO_SetPinSpeed(MOTOR_LEFT_PORT , MOTOR_LEFT_DIR_PIN_1 , LL_GPIO_SPEED_FREQ_LOW);
  LL_GPIO_SetPinSpeed(MOTOR_RIGHT_PORT, MOTOR_RIGHT_PWM_PIN  , LL_GPIO_SPEED_FREQ_HIGH);
  LL_GPIO_SetPinSpeed(MOTOR_RIGHT_PORT, MOTOR_RIGHT_DIR_PIN_0, LL_GPIO_SPEED_FREQ_LOW);
  LL_GPIO_SetPinSpeed(MOTOR_RIGHT_PORT, MOTOR_RIGHT_DIR_PIN_1, LL_GPIO_SPEED_FREQ_LOW);
  /* Reset value is LL_GPIO_PULL_NO */
  //LL_GPIO_SetPinPull(LED2_GPIO_PORT, LED2_PIN, LL_GPIO_PULL_NO);
  
  /***********************************************/
  /* Configure the NVIC to handle TIM4 interrupt */
  /***********************************************/
  NVIC_SetPriority(MOTOR_TIMER_IRQn, 0);
  NVIC_EnableIRQ(MOTOR_TIMER_IRQn);

  Motor_Timer_Init();
}

void Motor_Timer_Init(void) {
  /******************************/
  /* Peripheral clocks enabling */
  /******************************/
  /* Enable the timer peripheral clock */
  MOTOR_TIMER_CLK_ENABLE();
  
  /***************************/
  /* Time base configuration */
  /***************************/
  /* Set counter mode */
  /* Reset value is LL_TIM_COUNTERMODE_UP */
  LL_TIM_SetCounterMode(MOTOR_TIMER, LL_TIM_COUNTERMODE_UP);
  
  /* Set the pre-scaler value to have TIM4 counter clock equal to 10 kHz */
  //LL_TIM_SetPrescaler(TIM4, __LL_TIM_CALC_PSC(SystemCoreClock, 10000));
  LL_TIM_SetPrescaler(MOTOR_TIMER, (110-1U));
  
  /* Enable TIM2_ARR register preload. Writing to or reading from the         */
  /* auto-reload register accesses the preload register. The content of the   */
  /* preload register are transferred into the shadow register at each update */
  /* event (UEV).                                                             */  
  LL_TIM_EnableARRPreload(MOTOR_TIMER);
  
  /* Set the auto-reload value to have a counter frequency of 100 Hz */
  /* TIM2CLK = SystemCoreClock / (APB prescaler & multiplier)               */
  //uint32_t TimOutClock = SystemCoreClock/1;
  //LL_TIM_SetAutoReload(TIM4, __LL_TIM_CALC_ARR(TimOutClock, LL_TIM_GetPrescaler(TIM4), 100));
  LL_TIM_SetAutoReload(MOTOR_TIMER, MOTOR_TIMER_ARR_MAX);


  /************************************/
  /* Input capture mode configuration */
  /************************************/
  /* Select the active input: IC1 = TI1FP1 */
  LL_TIM_IC_SetActiveInput(MOTOR_TIMER, LL_TIM_CHANNEL_CH1, LL_TIM_ACTIVEINPUT_DIRECTTI);
  LL_TIM_IC_SetActiveInput(MOTOR_TIMER, LL_TIM_CHANNEL_CH2, LL_TIM_ACTIVEINPUT_DIRECTTI);
  
  /* Configure the input filter duration: no filter needed */
  LL_TIM_IC_SetFilter(MOTOR_TIMER, LL_TIM_CHANNEL_CH1, LL_TIM_IC_FILTER_FDIV1_N2);
  LL_TIM_IC_SetFilter(MOTOR_TIMER, LL_TIM_CHANNEL_CH2, LL_TIM_IC_FILTER_FDIV1_N2);

  /* Set input prescaler: prescaler is disabled */
  LL_TIM_IC_SetPrescaler(MOTOR_TIMER, LL_TIM_CHANNEL_CH1, LL_TIM_ICPSC_DIV1);
  LL_TIM_IC_SetPrescaler(MOTOR_TIMER, LL_TIM_CHANNEL_CH2, LL_TIM_ICPSC_DIV1);

  /* Select the edge of the active transition on the TI1 channel: falling edge */
  LL_TIM_IC_SetPolarity(MOTOR_TIMER, LL_TIM_CHANNEL_CH1, LL_TIM_IC_POLARITY_FALLING);
  LL_TIM_IC_SetPolarity(MOTOR_TIMER, LL_TIM_CHANNEL_CH2, LL_TIM_IC_POLARITY_FALLING);
  
  
  /*********************************/
  /* Output waveform configuration */
  /*********************************/
  /* Set output mode */
  /* Reset value is LL_TIM_OCMODE_FROZEN */
  LL_TIM_OC_SetMode(MOTOR_TIMER, LL_TIM_CHANNEL_CH3, LL_TIM_OCMODE_PWM1);
  LL_TIM_OC_SetMode(MOTOR_TIMER, LL_TIM_CHANNEL_CH4, LL_TIM_OCMODE_PWM1);
  
  /* Set output channel polarity */
  /* Reset value is LL_TIM_OCPOLARITY_HIGH */
  //LL_TIM_OC_SetPolarity(MOTOR_TIMER, LL_TIM_CHANNEL_CH1, LL_TIM_OCPOLARITY_HIGH);
  
  /* Set compare value to half of the counter period (50% duty cycle ) */
  //LL_TIM_OC_SetCompareCH3(MOTOR_TIMER, ( (LL_TIM_GetAutoReload(MOTOR_TIMER) + 1 ) / 3));
  //LL_TIM_OC_SetCompareCH4(MOTOR_TIMER, ( (LL_TIM_GetAutoReload(MOTOR_TIMER) + 1 ) / 3));
  LL_TIM_OC_SetCompareCH3(MOTOR_TIMER, 0);     // initial state is 'stop'.
  LL_TIM_OC_SetCompareCH4(MOTOR_TIMER, 0);
  
  /* Enable TIM2_CCR1 register preload. Read/Write operations access the      */
  /* preload register. TIM2_CCR1 preload value is loaded in the active        */
  /* at each update event.                                                    */
  LL_TIM_OC_EnablePreload(MOTOR_TIMER, MOTOR_TIMER_CH_LEFT_PWM);
  LL_TIM_OC_EnablePreload(MOTOR_TIMER, MOTOR_TIMER_CH_RIGHT_PWM);
  

  /**************************/
  /* MOTOR_TIMER interrupts set-up */
  /**************************/
  /* Enable the capture/compare interrupt for channel 1*/
  LL_TIM_EnableIT_CC1(MOTOR_TIMER);
  LL_TIM_EnableIT_CC2(MOTOR_TIMER);
  

  /**********************************/
  /* Start output signal generation */
  /**********************************/
  /* Enable output channel 3,4 */
  LL_TIM_CC_EnableChannel(MOTOR_TIMER, MOTOR_TIMER_CH_LEFT_PWM);
  LL_TIM_CC_EnableChannel(MOTOR_TIMER, MOTOR_TIMER_CH_RIGHT_PWM);
  
  /***********************/
  /* Start input capture */
  /***********************/
  /* Enable input channel 1,2 */
  LL_TIM_CC_EnableChannel(MOTOR_TIMER, MOTOR_TIMER_CH_LEFT_HSENS);
  LL_TIM_CC_EnableChannel(MOTOR_TIMER, MOTOR_TIMER_CH_RIGHT_HSENS);

 
  /* Enable counter */
  LL_TIM_EnableCounter(MOTOR_TIMER);
  
  /* Force update generation */
  LL_TIM_GenerateEvent_UPDATE(MOTOR_TIMER);

}

/**
  * @brief  Timer capture/compare interrupt processing
  * @note TIM3 input capture module is used to capture the value of the counter
  *       after a transition is detected by the corresponding input channel.
  * @param  None
  * @retval None
  */
MotorInfo motor_left  = {MOTOR_LEFT_ID , 0, 0, 0};
MotorInfo motor_right = {MOTOR_RIGHT_ID, 0, 0, 0};

uint32_t  hsens_period_iir_alpha = 65536;
uint32_t  hsens_period_iir_beta  = 65536 >> 4;

void TimerCaptureCompare(MotorInfo *minfo)
{
    uint32_t curr_timer_val = 0;
    if(minfo->id == MOTOR_LEFT_ID) {
        curr_timer_val = MOTOR_TIMER_CAPTURE_LEFT();
    }
    else {
        curr_timer_val = MOTOR_TIMER_CAPTURE_RIGHT();
    }

    if(minfo->hsens_period_iir < 0) {
        minfo->hsens_period_iir = curr_timer_val;
    }
    else {
        uint32_t diff_timer_val;
        if(minfo->hsens_timer_prev > curr_timer_val) {
            uint32_t T = LL_TIM_GetAutoReload(MOTOR_TIMER);
            diff_timer_val = (T+1-minfo->hsens_timer_prev) + curr_timer_val;
        } else {
            diff_timer_val = curr_timer_val - minfo->hsens_timer_prev;
        }

        uint32_t r = minfo->hsens_period_iir;
        uint32_t i = diff_timer_val;
        uint32_t a = hsens_period_iir_alpha - hsens_period_iir_beta;
        uint32_t b = hsens_period_iir_beta;

        if(a < 0) {
            LED_Blinking(LED_BLINK_ERROR);
        }

        minfo->hsens_period_iir = ((r * a) + (i * b)) >> 16;
    }

    minfo->hsens_timer_prev = curr_timer_val;

    return;
}


/**
  * @brief  Changes the duty cycle of the PWM signal.
  *         D = (T/P)*100
  *           where T is the pulse duration and P is the PWM signal period
  * @param  D Duty cycle
  * @retval None
  */
__STATIC_INLINE void Configure_DutyCycle(MotorInfo *minfo)
{
  uint32_t P;    /* Pulse duration */
  uint32_t T;    /* PWM signal period */
  
  if(minfo->id == MOTOR_LEFT_ID) {
      /* PWM signal period is determined by the value of the auto-reload register */
      //T = LL_TIM_GetAutoReload(MOTOR_TIMER) + 1;
      T = LL_TIM_GetAutoReload(MOTOR_TIMER);
      
      /* Pulse duration is determined by the value of the compare register.       */
      /* Its value is calculated in order to match the requested duty cycle.      */
      P = (minfo->speed_rate*T)/100;
      MOTOR_TIMER_OC_SetCompLEFT(TIM4, P);
  }
  else if(minfo->id == MOTOR_RIGHT_ID) {
      /* PWM signal period is determined by the value of the auto-reload register */
      //T = LL_TIM_GetAutoReload(MOTOR_TIMER) + 1;
      T = LL_TIM_GetAutoReload(MOTOR_TIMER);
      
      /* Pulse duration is determined by the value of the compare register.       */
      /* Its value is calculated in order to match the requested duty cycle.      */
      P = (minfo->speed_rate*T)/100;
      MOTOR_TIMER_OC_SetCompRIGHT(TIM4, P);
  }
  else {
      /* Exception case. */
      LED_Blinking(LED_BLINK_ERROR);
  }
}


/**
  * @brief  
  * @param  None
  * @retval None
  */
void Motor_Set_DIR_Left_Turn(void)
{
  /*  */
  LL_GPIO_ResetOutputPin(MOTOR_LEFT_PORT, MOTOR_LEFT_DIR_PIN_0);
  LL_GPIO_SetOutputPin  (MOTOR_LEFT_PORT, MOTOR_LEFT_DIR_PIN_1);

  /*  */
  LL_GPIO_SetOutputPin  (MOTOR_RIGHT_PORT, MOTOR_RIGHT_DIR_PIN_0);
  LL_GPIO_ResetOutputPin(MOTOR_RIGHT_PORT, MOTOR_RIGHT_DIR_PIN_1);
}

/**
  * @brief  
  * @param  None
  * @retval None
  */
void Motor_Set_DIR_Right_Turn(void)
{
  /*  */
  LL_GPIO_SetOutputPin  (MOTOR_LEFT_PORT, MOTOR_LEFT_DIR_PIN_0);
  LL_GPIO_ResetOutputPin(MOTOR_LEFT_PORT, MOTOR_LEFT_DIR_PIN_1);

  /*  */
  LL_GPIO_ResetOutputPin(MOTOR_RIGHT_PORT, MOTOR_RIGHT_DIR_PIN_0);
  LL_GPIO_SetOutputPin  (MOTOR_RIGHT_PORT, MOTOR_RIGHT_DIR_PIN_1);
}

/**
  * @brief  
  * @param  None
  * @retval None
  */
void Motor_Set_DIR_Forward(void)
{
  /*  */
  LL_GPIO_ResetOutputPin(MOTOR_LEFT_PORT, MOTOR_LEFT_DIR_PIN_0);
  LL_GPIO_SetOutputPin  (MOTOR_LEFT_PORT, MOTOR_LEFT_DIR_PIN_1);

  /*  */
  LL_GPIO_ResetOutputPin(MOTOR_RIGHT_PORT, MOTOR_RIGHT_DIR_PIN_0);
  LL_GPIO_SetOutputPin  (MOTOR_RIGHT_PORT, MOTOR_RIGHT_DIR_PIN_1);
}

/**
  * @brief  
  * @param  None
  * @retval None
  */
void Motor_Set_DIR_Backward(void)
{
  /*  */
  LL_GPIO_SetOutputPin  (MOTOR_LEFT_PORT, MOTOR_LEFT_DIR_PIN_0);
  LL_GPIO_ResetOutputPin(MOTOR_LEFT_PORT, MOTOR_LEFT_DIR_PIN_1);

  /*  */
  LL_GPIO_SetOutputPin  (MOTOR_RIGHT_PORT, MOTOR_RIGHT_DIR_PIN_0);
  LL_GPIO_ResetOutputPin(MOTOR_RIGHT_PORT, MOTOR_RIGHT_DIR_PIN_1);
}

/**
  * @brief  
  * @param  None
  * @retval None
  */
void Motor_Left_Stop(void)
{
  /*  */
  LL_GPIO_ResetOutputPin(MOTOR_LEFT_PORT, MOTOR_LEFT_DIR_PIN_0);
  LL_GPIO_ResetOutputPin(MOTOR_LEFT_PORT, MOTOR_LEFT_DIR_PIN_1);
}

/**
  * @brief  
  * @param  None
  * @retval None
  */
void Motor_Right_Stop(void)
{
  /*  */
  LL_GPIO_ResetOutputPin(MOTOR_RIGHT_PORT, MOTOR_RIGHT_DIR_PIN_0);
  LL_GPIO_ResetOutputPin(MOTOR_RIGHT_PORT, MOTOR_RIGHT_DIR_PIN_1);
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

    motor_left.speed_rate = 0;
    motor_right.speed_rate = 0;
}

uint32_t Motor_Speed_Up(MotorInfo *minfo) {
    if(minfo->speed_rate >= 0) {
        if(minfo->speed_rate < motor_speed_rate_thr) {
            minfo->speed_rate = motor_speed_rate_thr;
        }
        else if(minfo->speed_rate < motor_speed_rate_max) {
            minfo->speed_rate += motor_speed_rate_step;
        }
    }
#if 1
    if(minfo->speed_rate < 0) {
        minfo->speed_rate = 0;
    }
#else
    else if(minfo->speed_rate < 0) {
        if(minfo->speed_rate == -motor_speed_rate_thr) {
            minfo->speed_rate = 0;
        }
        else {
            minfo->speed_rate += motor_speed_rate_step;
        }
    }
#endif

    Configure_DutyCycle(minfo);

    return minfo->speed_rate;
}

uint32_t Motor_Speed_Down(MotorInfo *minfo) {
    if(minfo->speed_rate > 0) {
        if(minfo->speed_rate == motor_speed_rate_thr) {
            minfo->speed_rate = 0;
        }
        else {
            minfo->speed_rate -= motor_speed_rate_step;
        }
    }
#if 1
    if(minfo->speed_rate < 0) {
        minfo->speed_rate = 0;
    }
#else
    else if(minfo->speed_rate <= 0) {
        if(minfo->speed_rate == 0) {
            minfo->speed_rate = -motor_speed_rate_thr;
        }
        else if(minfo->speed_rate > motor_speed_rate_min) {
            minfo->speed_rate -= motor_speed_rate_step;
        }
    }
#endif

    Configure_DutyCycle(minfo);

    return minfo->speed_rate;
}

int32_t Motor_Left_Speed_Up(void) {
    return Motor_Speed_Up(&motor_left);
}

int32_t Motor_Right_Speed_Up(void) {
    return Motor_Speed_Up(&motor_right);
}

int32_t Motor_Left_Speed_Down(void) {
    return Motor_Speed_Down(&motor_left);
}

int32_t Motor_Right_Speed_Down(void) {
    return Motor_Speed_Down(&motor_right);
}

uint32_t Motor_Get_Hsens_Speed_Left(void) {
    return motor_left.hsens_period_iir;
}

uint32_t Motor_Get_Hsens_Speed_Right(void) {
    return motor_right.hsens_period_iir;
}

void TimerCaptureCompareLeft(void) {
    TimerCaptureCompare(&motor_left);
}

void TimerCaptureCompareRight(void) {
    TimerCaptureCompare(&motor_right);
}

/**
  * @}
  */

/**
  * @}
  */
/************************ (C) Deokhwan, Kim *****END OF FILE****/
