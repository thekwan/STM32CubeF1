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
#ifndef __BATTERY_H__
#define __BATTERY_H__

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_ll_adc.h"
#include "stm32f1xx_ll_bus.h"
#include "stm32f1xx_ll_rcc.h"
#include "stm32f1xx_ll_system.h"
#include "stm32f1xx_ll_cortex.h"
#include "stm32f1xx_ll_utils.h"
#include "stm32f1xx_ll_gpio.h"
#include "stm32f1xx_ll_exti.h"
#include "stm32f1xx_ll_pwr.h"
#if defined(USE_FULL_ASSERT)
#include "stm32_assert.h"
#endif /* USE_FULL_ASSERT */

/* Exported types ------------------------------------------------------------*/

/* Exported constants --------------------------------------------------------*/

/**
  * @brief BATTERY2 
  */

//#define BATTERY_PIN                           LL_GPIO_PIN_4
//#define BATTERY_GPIO_PORT                     GPIOA
//#define BATTERY_GPIO_CLK_ENABLE()             LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_GPIOA)
#define BATTERY_PIN                           LL_GPIO_PIN_0
#define BATTERY_GPIO_PORT                     GPIOB
#define BATTERY_GPIO_CLK_ENABLE()               LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_GPIOB)

#define ADC_CLK_ENABLE()                      LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_ADC1);
//#define ADC_CHANNEL                           LL_ADC_CHANNEL_4
#define ADC_CHANNEL                           LL_ADC_CHANNEL_8

//#define USE_TIMEOUT     0   // don't use time-out
#define USE_TIMEOUT     1   // use time-out


/* Definitions of ADC hardware constraints delays */
/* Note: Only ADC IP HW delays are defined in ADC LL driver driver,           */
/*       not timeout values:                                                  */
/*       Timeout values for ADC operations are dependent to device clock      */
/*       configuration (system clock versus ADC clock),                       */
/*       and therefore must be defined in user application.                   */
/*       Refer to @ref ADC_LL_EC_HW_DELAYS for description of ADC timeout     */
/*       values definition.                                                   */

  /* Timeout values for ADC operations. */
  /* (enable settling time, disable settling time, ...)                       */
  /* Values defined to be higher than worst cases: low clock frequency,       */
  /* maximum prescalers.                                                      */
  /* Example of profile very low frequency : ADC clock frequency 12MHz        */
  /* prescaler 6, sampling time 1.5 ADC clock cycles, resolution 12 bits.     */
  /*  - ADC enable time: maximum delay is 1 us                                */
  /*    (refer to device datasheet, parameter "tSTAB")                        */
  /*  - ADC disable time: maximum delay should be a few ADC clock cycles      */
  /*  - ADC stop conversion time: maximum delay should be a few ADC clock     */
  /*    cycles                                                                */
  /*  - ADC conversion time: with this hypothesis of clock settings, maximum  */
  /*    delay will be 7us.                                                   */
  /*    (refer to device reference manual, section "Timing")                  */
  /* Unit: ms                                                                 */
  #define ADC_CALIBRATION_TIMEOUT_MS       ((uint32_t)   1)
  #define ADC_ENABLE_TIMEOUT_MS            ((uint32_t)   1)
  #define ADC_DISABLE_TIMEOUT_MS           ((uint32_t)   1)
  #define ADC_STOP_CONVERSION_TIMEOUT_MS   ((uint32_t)   1)
  #define ADC_CONVERSION_TIMEOUT_MS        ((uint32_t)   2)
  
  /* Delay between ADC enable and ADC end of calibration.                     */
  /* Delay estimation in CPU cycles: Case of ADC calibration done             */
  /* immediately after ADC enable, ADC clock setting slow                     */
  /* (LL_ADC_CLOCK_ASYNC_DIV32). Use a higher delay if ratio                  */
  /* (CPU clock / ADC clock) is above 32.                                     */
  #define ADC_DELAY_ENABLE_CALIB_CPU_CYCLES  (LL_ADC_DELAY_ENABLE_CALIB_ADC_CYCLES * 32)
  

/* Definitions of environment analog values */
  /* Value of analog reference voltage (Vref+), connected to analog voltage   */
  /* supply Vdda (unit: mV).                                                  */
  #define VDDA_APPLI                       ((uint32_t)3300)

/* Definitions of data related to this example */
  /* ADC unitary conversion timeout */
  /* Considering ADC settings, duration of 1 ADC conversion should always    */
  /* be lower than 1ms.                                                      */
  #define ADC_UNITARY_CONVERSION_TIMEOUT_MS ((uint32_t)   1)

  /* Init variable out of expected ADC conversion data range */
  #define VAR_CONVERTED_DATA_INIT_VALUE    (__LL_ADC_DIGITAL_SCALE(LL_ADC_RESOLUTION_12B) + 1)


/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */
/* IRQ Handler treatment functions */
/* Public API --------------------------------------------------------------- */

void Battery_Measure_Init(void);
uint16_t Get_Battery_Voltage(uint16_t meas_list[16]);

/* ADC APIs */
void Configure_ADC(void);
void Activate_ADC(void);
void ConversionStartPoll_ADC_GrpRegular(void);
//void     BATTERY_On(void);
//void     BATTERY_Off(void);
//void     BATTERY_Blinking(uint32_t Period);

#endif /* __BATTERY_H__ */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
