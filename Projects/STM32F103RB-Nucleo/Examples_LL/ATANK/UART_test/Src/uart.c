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
#include "uart.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
__IO uint8_t ubSend = 0;
const uint8_t uartInfoMessage[] = "STM32F1xx USART LL API is used.\r\n";
uint8_t ubSizeToSendInfoMessage = sizeof(uartInfoMessage);

#define TX_BUFFER_SIZE    256
#define RX_BUFFER_SIZE    256

#define UPGRADE_API

#define WRAP_BUFF_ADDR(ADDR,MAX_SIZE)   \
    do{ if(ADDR >= MAX_SIZE) {  \
            ADDR-= MAX_SIZE;     \
    } }while(0)

/* Tx buffer defines */
int uartTxBufferReadPtr  = 0;
int uartTxBufferWritePtr = 0;
uint8_t uartTxBuffer[TX_BUFFER_SIZE];
int uartTxBufferOverflowFlag = 0;

/* Rx buffer defines */
int uartRxBufferReadPtr  = 0;
int uartRxBufferWritePtr = 0;
uint8_t uartRxBuffer[RX_BUFFER_SIZE];
int uartRxBufferOverflowFlag = 0;

/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/*
 * printf_uart
 */
int printf_uart(char *string)
{
    int i;

    /* checks residual buffer size of Tx buffer.
     */
    WRAP_BUFF_ADDR(uartTxBufferWritePtr, TX_BUFFER_SIZE);
    WRAP_BUFF_ADDR(uartTxBufferReadPtr,  TX_BUFFER_SIZE);
    int bufferElemSize = (uartRxBufferWritePtr - uartRxBufferReadPtr);

    /* checks the target string length.
     */
    int strLength = 0;
    char *str_copy = string;
    for(i = 0; i < TX_BUFFER_SIZE; i++) {
        if(*str_copy++ == 0x0) {
            break;
        }
        strLength++;
    }
    if(strLength < 1)   // if there is no string, return.
        return 0;

    strLength++;    // for NULL char

    /* checks the length is more than residual Tx buffer.
     */
    int residual_buff_size = TX_BUFFER_SIZE - bufferElemSize;
    if(strLength > residual_buff_size) {
        uartTxBufferOverflowFlag = 1;
        return -1;
    }

    for(i = 0; i < strLength; i++) {
        uartTxBuffer[uartTxBufferWritePtr++] = (uint8_t)*string++;
        WRAP_BUFF_ADDR(uartTxBufferWritePtr, TX_BUFFER_SIZE);
    }

    /* Enable TXE interrupt */
    LL_USART_EnableIT_TXE(USARTx_INSTANCE);
    /* Enable TC interrupt */
    //LL_USART_DisableIT_TC(USARTx_INSTANCE);
    /* Fill DR with a new char */
    //while(LL_USART_IsActiveFlag_TXE(USARTx_INSTANCE) == 0);
    //LL_USART_TransmitData8(USARTx_INSTANCE, uartTxBuffer[uartTxBufferReadPtr++]);
    //WRAP_BUFF_ADDR(uartTxBufferReadPtr, TX_BUFFER_SIZE);

  return 0;
}

/*
 * scanf_uart
 */
int scanf_uart(char *buf, int buf_size) 
{
    /* Get the total number of received characters.
     */
    WRAP_BUFF_ADDR(uartTxBufferWritePtr, TX_BUFFER_SIZE);
    WRAP_BUFF_ADDR(uartTxBufferReadPtr,  TX_BUFFER_SIZE);
    int bufferElemSize = (uartRxBufferWritePtr - uartRxBufferReadPtr);

    if(bufferElemSize < 1) {
        buf = 0;
        return 0;
    }

    /* check that there is a string at least.
     */
    int i;
    int found_string = 0;
    int index = uartRxBufferReadPtr;
    for(i = 0; i < bufferElemSize; i++) {
        if(uartRxBuffer[index++] == 0x0) {
        //if(uartRxBuffer[index++] == '\r') {
            found_string = 1;
            break;
        }
        WRAP_BUFF_ADDR(index, RX_BUFFER_SIZE);
    }

    /* if there is no complete string yet, return NULL.
     */
    if(found_string == 0) {
        buf = 0;
        return 0;
    }

    /* if there is a complete string, check it can be copy once again.
     */
    if(i >= buf_size) { // string has more size than 'buf'.
        buf = 0;
        return -1;
    }

    /* if there is a complete string, copy them into 'buf'.
     */
    int copy_size = i;
    for(; i >= 0; i--) {
        *buf++ = uartRxBuffer[uartRxBufferReadPtr++];
        WRAP_BUFF_ADDR(uartRxBufferReadPtr, RX_BUFFER_SIZE);
    }

    return copy_size;
}

/**
  * @brief  This function configures USARTx Instance.
  * @note   This function is used to :
  *         -1- Enable GPIO clock, USART clock and configures the USART pins.
  *         -2- NVIC Configuration for USART interrupts.
  *         -3- Configure USART functional parameters.
  *         -4- Enable USART.
  * @note   Peripheral configuration is minimal configuration from reset values.
  *         Thus, some useless LL unitary functions calls below are provided as
  *         commented examples - setting is default configuration from reset.
  * @param  None
  * @retval None
  */
void Configure_USART(void)
{

  /* (1) Enable GPIO clock and configures the USART pins *********************/

  /* Enable the peripheral clock of GPIO Port */
  USARTx_GPIO_CLK_ENABLE();

  /* Enable USART peripheral clock *******************************************/
  USARTx_CLK_ENABLE();

  /* Configure Tx Pin as : Alternate function, High Speed, Push pull, Pull up */
  LL_GPIO_SetPinMode(        USARTx_TX_GPIO_PORT, USARTx_TX_PIN, LL_GPIO_MODE_ALTERNATE    );
  LL_GPIO_SetPinSpeed(       USARTx_TX_GPIO_PORT, USARTx_TX_PIN, LL_GPIO_SPEED_FREQ_HIGH   );
  LL_GPIO_SetPinOutputType(  USARTx_TX_GPIO_PORT, USARTx_TX_PIN, LL_GPIO_OUTPUT_PUSHPULL   );
  LL_GPIO_SetPinPull(        USARTx_TX_GPIO_PORT, USARTx_TX_PIN, LL_GPIO_PULL_UP           );

  /* Configure Rx Pin as : Input Floating function, High Speed, Pull up */
  LL_GPIO_SetPinMode(        USARTx_RX_GPIO_PORT, USARTx_RX_PIN, LL_GPIO_MODE_FLOATING     );
  LL_GPIO_SetPinSpeed(       USARTx_RX_GPIO_PORT, USARTx_RX_PIN, LL_GPIO_SPEED_FREQ_HIGH   );
  LL_GPIO_SetPinPull(        USARTx_RX_GPIO_PORT, USARTx_RX_PIN, LL_GPIO_PULL_UP           );

  /* (2) NVIC Configuration for USART interrupts */
  /*  - Set priority for USARTx_IRQn */
  /*  - Enable USARTx_IRQn */
  NVIC_SetPriority(USARTx_IRQn, 0);  
  NVIC_EnableIRQ(USARTx_IRQn);

  /* (3) Configure USART functional parameters ********************************/

  /* Disable USART prior modifying configuration registers */
  /* Note: Commented as corresponding to Reset value */
  // LL_USART_Disable(USARTx_INSTANCE);

  /* TX/RX direction */
  LL_USART_SetTransferDirection(USARTx_INSTANCE, LL_USART_DIRECTION_TX_RX);

  /* 8 data bit, 1 start bit, 1 stop bit, no parity */
  LL_USART_ConfigCharacter(USARTx_INSTANCE, LL_USART_DATAWIDTH_8B, LL_USART_PARITY_NONE, LL_USART_STOPBITS_1);

  /* No Hardware Flow control */
  /* Reset value is LL_USART_HWCONTROL_NONE */
  // LL_USART_SetHWFlowCtrl(USARTx_INSTANCE, LL_USART_HWCONTROL_NONE);

  /* Set Baudrate to 115200 using APB frequency set to 72000000/APB_Div Hz */
  /* Frequency available for USART peripheral can also be calculated through LL RCC macro */
  /* Ex :
      Periphclk = LL_RCC_GetUSARTClockFreq(Instance); or LL_RCC_GetUARTClockFreq(Instance); depending on USART/UART instance
  
      In this example, Peripheral Clock is expected to be equal to 72000000/APB_Div Hz => equal to SystemCoreClock/APB_Div
  */
  //LL_USART_SetBaudRate(USARTx_INSTANCE, SystemCoreClock/APB_Div, 115200); 
  LL_USART_SetBaudRate(USARTx_INSTANCE, SystemCoreClock/APB_Div, 9600); 

  /* (4) Enable USART *********************************************************/
  LL_USART_Enable(USARTx_INSTANCE);

  /* Enable RXNE and Error interrupts */
  LL_USART_EnableIT_RXNE(USARTx_INSTANCE);
  LL_USART_EnableIT_ERROR(USARTx_INSTANCE);
}


/******************************************************************************/
/*   IRQ HANDLER TREATMENT Functions                                          */
/******************************************************************************/

/**
  * @brief  Function called for achieving next TX Byte sending
  * @param  None
  * @retval None
  */
void USART_TXEmpty_Callback(void)
{
#if defined(UPGRADE_API)

    WRAP_BUFF_ADDR(uartTxBufferWritePtr, TX_BUFFER_SIZE);
    WRAP_BUFF_ADDR(uartTxBufferReadPtr, TX_BUFFER_SIZE);
    int bufferElemSize = (uartTxBufferWritePtr - uartTxBufferReadPtr);

    if(bufferElemSize == 1) {
        /* Disable TXE interrupt */
        LL_USART_DisableIT_TXE(USARTx_INSTANCE);
        /* Enable TC interrupt */
        LL_USART_EnableIT_TC(USARTx_INSTANCE);
    }

    /* Fill DR with a new char */
    while(LL_USART_IsActiveFlag_TXE(USARTx_INSTANCE) == 0);
    LL_USART_TransmitData8(USARTx_INSTANCE, uartTxBuffer[uartTxBufferReadPtr++]);
    WRAP_BUFF_ADDR(uartTxBufferReadPtr, TX_BUFFER_SIZE);

#else
  if(ubSend == (ubSizeToSendInfoMessage - 1))
  {
    /* Disable TXE interrupt */
    LL_USART_DisableIT_TXE(USARTx_INSTANCE);
    
    /* Enable TC interrupt */
    LL_USART_EnableIT_TC(USARTx_INSTANCE);
  }

  /* Fill DR with a new char */
  LL_USART_TransmitData8(USARTx_INSTANCE, uartInfoMessage[ubSend++]);
#endif  /* UPGRADE_API */
}

/**
  * @brief  Function called at completion of last byte transmission
  * @param  None
  * @retval None
  */
void USART_CharTransmitComplete_Callback(void)
{
#if defined(UPGRADE_API)
    WRAP_BUFF_ADDR(uartTxBufferWritePtr, TX_BUFFER_SIZE);
    WRAP_BUFF_ADDR(uartTxBufferReadPtr, TX_BUFFER_SIZE);
    int bufferElemSize = (uartTxBufferWritePtr - uartTxBufferReadPtr);

    if(bufferElemSize == 0) {
        /* Disable TXE interrupt */
        //LL_USART_DisableIT_TXE(USARTx_INSTANCE);
        /* Disable TC interrupt */
        LL_USART_DisableIT_TC(USARTx_INSTANCE);
    }
#else
  if(ubSend == sizeof(uartInfoMessage))
  {
    ubSend = 0;
    
    /* Disable TC interrupt */
    LL_USART_DisableIT_TC(USARTx_INSTANCE);
    
    /* Turn LED2 On at end of transfer : Tx sequence completed successfully */
    //LED_On();
  }
#endif  /* UPGRADE_API */
}

/**
  * @brief  Function called from USART IRQ Handler when RXNE flag is set
  *         Function is in charge of reading character received on USART RX line.
  * @param  None
  * @retval None
  */
void USART_CharReception_Callback(void)
{
#if defined(UPGRADE_API)
    __IO uint32_t received_char; 
    
    /* Read Received character. RXNE flag is cleared by reading of DR register */
    received_char = LL_USART_ReceiveData8(USARTx_INSTANCE);

    WRAP_BUFF_ADDR(uartRxBufferWritePtr, RX_BUFFER_SIZE);
    WRAP_BUFF_ADDR(uartRxBufferReadPtr, RX_BUFFER_SIZE);
    int bufferElemSize = (uartRxBufferWritePtr - uartRxBufferReadPtr);
    if(bufferElemSize >= RX_BUFFER_SIZE) {
        uartRxBufferOverflowFlag = 1;
    }
    else {
        uartRxBuffer[uartRxBufferWritePtr++] = received_char;
    }

#else
__IO uint32_t received_char;

  /* Read Received character. RXNE flag is cleared by reading of DR register */
  received_char = LL_USART_ReceiveData8(USARTx_INSTANCE);

  /* Check if received value is corresponding to specific one : S or s */
  if ((received_char == 'S') || (received_char == 's'))
  {
    /* Turn LED2 On : Expected character has been received */
    ubSend = 1;
    LL_USART_TransmitData8(USARTx_INSTANCE, received_char);
  }

  /* Check if received value is corresponding to specific one : T or t */
  if ((received_char == 'T') || (received_char == 't'))
  {
    ubSend = 0;

    /* Start USART transmission : Will initiate TXE interrupt after DR register is empty */
    LL_USART_TransmitData8(USARTx_INSTANCE, uartInfoMessage[ubSend++]); 

    /* Enable TXE interrupt */
    LL_USART_EnableIT_TXE(USARTx_INSTANCE); 
  }
#endif
}


/**
  * @}
  */

/**
  * @}
  */
/************************ (C) Deokhwan, Kim *****END OF FILE****/
