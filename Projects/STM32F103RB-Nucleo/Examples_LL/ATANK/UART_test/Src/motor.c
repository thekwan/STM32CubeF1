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
  LL_GPIO_SetPinMode(MOTOR_LEFT_PORT, MOTOR_LEFT_PIN_0, LL_GPIO_MODE_OUTPUT);
  LL_GPIO_SetPinMode(MOTOR_LEFT_PORT, MOTOR_LEFT_PIN_1, LL_GPIO_MODE_OUTPUT);
  LL_GPIO_SetPinMode(MOTOR_RIGHT_PORT, MOTOR_RIGHT_PIN_0, LL_GPIO_MODE_OUTPUT);
  LL_GPIO_SetPinMode(MOTOR_RIGHT_PORT, MOTOR_RIGHT_PIN_1, LL_GPIO_MODE_OUTPUT);
  /* Reset value is LL_GPIO_OUTPUT_PUSHPULL */
  LL_GPIO_SetPinOutputType(MOTOR_LEFT_PORT, MOTOR_LEFT_PIN_0, LL_GPIO_OUTPUT_PUSHPULL);
  LL_GPIO_SetPinOutputType(MOTOR_LEFT_PORT, MOTOR_LEFT_PIN_1, LL_GPIO_OUTPUT_PUSHPULL);
  LL_GPIO_SetPinOutputType(MOTOR_RIGHT_PORT, MOTOR_RIGHT_PIN_0, LL_GPIO_OUTPUT_PUSHPULL);
  LL_GPIO_SetPinOutputType(MOTOR_RIGHT_PORT, MOTOR_RIGHT_PIN_1, LL_GPIO_OUTPUT_PUSHPULL);
  /* Reset value is LL_GPIO_SPEED_FREQ_LOW */
  LL_GPIO_SetPinSpeed(MOTOR_LEFT_PORT, MOTOR_LEFT_PIN_0, LL_GPIO_SPEED_FREQ_LOW);
  LL_GPIO_SetPinSpeed(MOTOR_LEFT_PORT, MOTOR_LEFT_PIN_1, LL_GPIO_SPEED_FREQ_LOW);
  LL_GPIO_SetPinSpeed(MOTOR_RIGHT_PORT, MOTOR_RIGHT_PIN_0, LL_GPIO_SPEED_FREQ_LOW);
  LL_GPIO_SetPinSpeed(MOTOR_RIGHT_PORT, MOTOR_RIGHT_PIN_1, LL_GPIO_SPEED_FREQ_LOW);
  /* Reset value is LL_GPIO_PULL_NO */
  //LL_GPIO_SetPinPull(LED2_GPIO_PORT, LED2_PIN, LL_GPIO_PULL_NO);
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


/**
  * @}
  */

/**
  * @}
  */
/************************ (C) Deokhwan, Kim *****END OF FILE****/
