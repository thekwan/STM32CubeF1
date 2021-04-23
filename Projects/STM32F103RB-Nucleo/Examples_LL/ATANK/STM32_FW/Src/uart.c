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
#define TX_BUFFER_SIZE    128
#define RX_BUFFER_SIZE    128

#define WRAP_BUFF_ADDR(ADDR,MAX_SIZE)   \
    do{ if(ADDR >= MAX_SIZE) {  \
            ADDR-= MAX_SIZE;     \
    } }while(0)

/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/* Lidar UART Rx buffer defines */
int lidarUartRxBufferWritePtr = 0;
uint8_t lidarUartRxBuffer[2][RX_BUFFER_SIZE];
//int lidarUartRxBufferOverflowFlag = 0;
int lidarUartRxBufferSel = 0;

/* Command UART Tx buffer defines */
int cmdUartTxBufferReadPtr  = 0;
int cmdUartTxBufferWritePtr = 0;
uint8_t cmdUartTxBuffer[TX_BUFFER_SIZE];
int cmdUartTxBufferOverflowFlag = 0;

/* Command UART Rx buffer defines */
int cmdUartRxBufferReadPtr  = 0;
int cmdUartRxBufferWritePtr = 0;
uint8_t cmdUartRxBuffer[RX_BUFFER_SIZE];
int cmdUartRxBufferOverflowFlag = 0;

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
    WRAP_BUFF_ADDR(cmdUartTxBufferWritePtr, TX_BUFFER_SIZE);
    WRAP_BUFF_ADDR(cmdUartTxBufferReadPtr,  TX_BUFFER_SIZE);
    int bufferElemSize = (cmdUartTxBufferWritePtr - cmdUartTxBufferReadPtr);
    if(bufferElemSize < 0) {
        bufferElemSize += TX_BUFFER_SIZE;
    }


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

    /* checks the length is more than residual Tx buffer.
     */
    int residual_buff_size = TX_BUFFER_SIZE - bufferElemSize - 1;
    if(strLength > residual_buff_size) {
        cmdUartTxBufferOverflowFlag = 1;
        return -1;
    }

    for(i = 0; i < strLength; i++) {
        cmdUartTxBuffer[cmdUartTxBufferWritePtr++] = (uint8_t)*string++;
        WRAP_BUFF_ADDR(cmdUartTxBufferWritePtr, TX_BUFFER_SIZE);
    }

    /* Enable TXE interrupt */
    LL_USART_EnableIT_TXE(USART2);
    /* Enable TC interrupt */
    //LL_USART_DisableIT_TC(USARTx_INSTANCE);
    /* Fill DR with a new char */
    //while(LL_USART_IsActiveFlag_TXE(USARTx_INSTANCE) == 0);
    //LL_USART_TransmitData8(USARTx_INSTANCE, cmdUartTxBuffer[cmdUartTxBufferReadPtr++]);
    //WRAP_BUFF_ADDR(cmdUartTxBufferReadPtr, TX_BUFFER_SIZE);

  return 0;
}

/*
 * scanf_uart
 */
int scanf_uart(char *buf, int buf_size) 
{
    /* Get the total number of received characters.
     */
    WRAP_BUFF_ADDR(cmdUartRxBufferWritePtr, RX_BUFFER_SIZE);
    WRAP_BUFF_ADDR(cmdUartRxBufferReadPtr,  RX_BUFFER_SIZE);
    int bufferElemSize = (cmdUartRxBufferWritePtr - cmdUartRxBufferReadPtr);

    if(bufferElemSize == 0) {
        *buf = 0;
        return 0;
    }
    else if(bufferElemSize < 0) {
        bufferElemSize += RX_BUFFER_SIZE;
    }

    /* check that there is a string at least.
     */
    int i;
    int found_string = 0;
    int index = cmdUartRxBufferReadPtr;
    for(i = 0; i < bufferElemSize; i++) {
        if(cmdUartRxBuffer[index++] == 0x0) {
        //if(cmdUartRxBuffer[index++] == '\r') {
            found_string = 1;
            break;
        }
        WRAP_BUFF_ADDR(index, RX_BUFFER_SIZE);
    }

    /* if there is no complete string yet, return NULL.
     */
    if(found_string == 0) {
        *buf = 0;
        return 0;
    }

    /* if there is a complete string, check it can be copy once again.
     */
    if(i >= buf_size) { // string has more size than 'buf'.
        *buf = 0;
        return -1;
    }

    /* if there is a complete string, copy them into 'buf'.
     */
    int copy_size = i;
    for(; i >= 0; i--) {
        *buf++ = cmdUartRxBuffer[cmdUartRxBufferReadPtr++];
        WRAP_BUFF_ADDR(cmdUartRxBufferReadPtr, RX_BUFFER_SIZE);
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
  USART2_GPIO_CLK_ENABLE();
  USART3_GPIO_CLK_ENABLE();

  /* Enable USART peripheral clock *******************************************/
  USART2_CLK_ENABLE();
  USART3_CLK_ENABLE();

  /* Configure (Command) Tx Pin as : Alternate function, High Speed, Push pull, Pull up */
  LL_GPIO_SetPinMode(        USART2_TX_GPIO_PORT, USART2_TX_PIN, LL_GPIO_MODE_ALTERNATE    );
  LL_GPIO_SetPinSpeed(       USART2_TX_GPIO_PORT, USART2_TX_PIN, LL_GPIO_SPEED_FREQ_HIGH   );
  LL_GPIO_SetPinOutputType(  USART2_TX_GPIO_PORT, USART2_TX_PIN, LL_GPIO_OUTPUT_PUSHPULL   );
  LL_GPIO_SetPinPull(        USART2_TX_GPIO_PORT, USART2_TX_PIN, LL_GPIO_PULL_UP           );

  /* Configure (Command) Rx Pin as : Input Floating function, High Speed, Pull up */
  LL_GPIO_SetPinMode(        USART2_RX_GPIO_PORT, USART2_RX_PIN, LL_GPIO_MODE_FLOATING     );
  LL_GPIO_SetPinSpeed(       USART2_RX_GPIO_PORT, USART2_RX_PIN, LL_GPIO_SPEED_FREQ_HIGH   );
  LL_GPIO_SetPinPull(        USART2_RX_GPIO_PORT, USART2_RX_PIN, LL_GPIO_PULL_UP           );

  /* Configure (Lidar) Rx Pin as : Input Floating function, High Speed, Pull up */
  LL_GPIO_SetPinMode(        USART3_RX_GPIO_PORT, USART3_RX_PIN, LL_GPIO_MODE_FLOATING     );
  LL_GPIO_SetPinSpeed(       USART3_RX_GPIO_PORT, USART3_RX_PIN, LL_GPIO_SPEED_FREQ_HIGH   );
  LL_GPIO_SetPinPull(        USART3_RX_GPIO_PORT, USART3_RX_PIN, LL_GPIO_PULL_UP           );


  /* (2) NVIC Configuration for USART interrupts */
  /*  - Set priority for USARTx_IRQn */
  /*  - Enable USARTx_IRQn */
  NVIC_SetPriority(USART2_IRQn, 0);  
  NVIC_EnableIRQ(USART2_IRQn);
  NVIC_SetPriority(USART3_IRQn, 0);  
  NVIC_EnableIRQ(USART3_IRQn);

  /* (3) Configure USART functional parameters ********************************/

  /* Disable USART prior modifying configuration registers */
  /* Note: Commented as corresponding to Reset value */
  // LL_USART_Disable(USARTx_INSTANCE);

  /* TX/RX direction */
  LL_USART_SetTransferDirection(USART2, LL_USART_DIRECTION_TX_RX);
  LL_USART_SetTransferDirection(USART3, LL_USART_DIRECTION_RX);

  /* 8 data bit, 1 start bit, 1 stop bit, no parity */
  LL_USART_ConfigCharacter(USART2, LL_USART_DATAWIDTH_8B, LL_USART_PARITY_NONE, LL_USART_STOPBITS_1);
  LL_USART_ConfigCharacter(USART3, LL_USART_DATAWIDTH_8B, LL_USART_PARITY_NONE, LL_USART_STOPBITS_1);

  /* No Hardware Flow control */
  /* Reset value is LL_USART_HWCONTROL_NONE */
  // LL_USART_SetHWFlowCtrl(USARTx_INSTANCE, LL_USART_HWCONTROL_NONE);

  /* Set Baudrate to 115200 using APB frequency set to 72000000/APB_Div Hz */
  /* Frequency available for USART peripheral can also be calculated through LL RCC macro */
  /* Ex :
      Periphclk = LL_RCC_GetUSARTClockFreq(Instance); or LL_RCC_GetUARTClockFreq(Instance); depending on USART/UART instance
  
      In this example, Peripheral Clock is expected to be equal to 72000000/APB_Div Hz => equal to SystemCoreClock/APB_Div
  */
  LL_USART_SetBaudRate(USART2, SystemCoreClock/APB_Div, 115200); 
  LL_USART_SetBaudRate(USART3, SystemCoreClock/APB_Div, 115200); 
  //LL_USART_SetBaudRate(USARTx_INSTANCE, SystemCoreClock/APB_Div, 9600); 
  //LL_USART_SetBaudRate(USARTx_INSTANCE, SystemCoreClock/APB_Div, 230400); 

  /* (4) Enable USART *********************************************************/
  LL_USART_Enable(USART2);
  LL_USART_Enable(USART3);

  /* Enable RXNE and Error interrupts */
  LL_USART_EnableIT_RXNE(USART2);
  LL_USART_EnableIT_ERROR(USART2);
  LL_USART_EnableIT_RXNE(USART3);
  LL_USART_EnableIT_ERROR(USART3);
}


/**
  * @brief  This function configures the DMA Channels for
  *         i) USART3(for Lidar) RX reception (UART3 DataReg->Mem)
  *         ii) SPI(to RPi) send Lidar data (Mem->SPI DataReg)
  * @note   This function is used to :
  *         -1- Enable DMA1 clock
  *         -2- Configure NVIC for DMA transfer complete/error interrupts 
  *         -3- Configure DMA TX channel functional parameters
  *         -4- Configure DMA RX channel functional parameters
  *         -5- Enable transfer complete/error interrupts
  * @param  None
  * @retval None
  */
void Configure_DMA(void)
{
  /* DMA1 used for USART2 Transmission and Reception
   */
  /* (1) Enable the clock of DMA1 */
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_DMA1);

  /* (2) Configure NVIC for DMA transfer complete/error interrupts */
  // DMA1, Ch3(SPI1_TX)
  NVIC_SetPriority(DMA1_Channel3_IRQn, 0);
  NVIC_EnableIRQ(DMA1_Channel3_IRQn);

  /* (3) Configure the DMA functional parameters for transmission */
  LL_DMA_ConfigTransfer(DMA1, LL_DMA_CHANNEL_3, 
                        LL_DMA_DIRECTION_MEMORY_TO_PERIPH | 
                        LL_DMA_PRIORITY_HIGH              | 
                        LL_DMA_MODE_NORMAL                | 
                        LL_DMA_PERIPH_NOINCREMENT         | 
                        LL_DMA_MEMORY_INCREMENT           | 
                        LL_DMA_PDATAALIGN_BYTE            | 
                        LL_DMA_MDATAALIGN_BYTE);
  LL_DMA_ConfigAddresses(DMA1, LL_DMA_CHANNEL_3,
                         (uint32_t)lidarUartRxBuffer[0],
                         LL_SPI_DMA_GetRegAddr(SPI1),
                         LL_DMA_GetDataTransferDirection(DMA1, LL_DMA_CHANNEL_3));
  LL_DMA_SetDataLength(DMA1, LL_DMA_CHANNEL_3, RX_BUFFER_SIZE);

  /* (5) Enable DMA transfer complete/error interrupts  */
  LL_DMA_EnableIT_TC(DMA1, LL_DMA_CHANNEL_3);
  LL_DMA_EnableIT_TE(DMA1, LL_DMA_CHANNEL_3);
}


/******************************************************************************/
/*   IRQ HANDLER TREATMENT Functions                                          */
/******************************************************************************/

/**
  * @brief  Function called for achieving next TX Byte sending
  * @param  None
  * @retval None
  */
void USART2_TXEmpty_Callback(void)
{
    WRAP_BUFF_ADDR(cmdUartTxBufferWritePtr, TX_BUFFER_SIZE);
    WRAP_BUFF_ADDR(cmdUartTxBufferReadPtr, TX_BUFFER_SIZE);
    int bufferElemSize = (cmdUartTxBufferWritePtr - cmdUartTxBufferReadPtr);
    if(bufferElemSize < 0) {
        bufferElemSize += TX_BUFFER_SIZE;
    }


    if(bufferElemSize == 1) {
        /* Disable TXE interrupt */
        LL_USART_DisableIT_TXE(USART2);
        /* Enable TC interrupt */
        LL_USART_EnableIT_TC(USART2);
    }

    /* Fill DR with a new char */
    while(LL_USART_IsActiveFlag_TXE(USART2) == 0);
    LL_USART_TransmitData8(USART2, cmdUartTxBuffer[cmdUartTxBufferReadPtr++]);
    WRAP_BUFF_ADDR(cmdUartTxBufferReadPtr, TX_BUFFER_SIZE);
}

/**
  * @brief  Function called at completion of last byte transmission
  * @param  None
  * @retval None
  */
void USART2_CharTransmitComplete_Callback(void)
{
    WRAP_BUFF_ADDR(cmdUartTxBufferWritePtr, TX_BUFFER_SIZE);
    WRAP_BUFF_ADDR(cmdUartTxBufferReadPtr, TX_BUFFER_SIZE);
    int bufferElemSize = (cmdUartTxBufferWritePtr - cmdUartTxBufferReadPtr);
    if(bufferElemSize < 0) {
        bufferElemSize += TX_BUFFER_SIZE;
    }

    if(bufferElemSize == 0) {
        /* Disable TXE interrupt */
        //LL_USART_DisableIT_TXE(USARTx_INSTANCE);
        /* Disable TC interrupt */
        LL_USART_DisableIT_TC(USART2);
    }
}

/**
  * @brief  Function called from USART IRQ Handler when RXNE flag is set
  *         Function is in charge of reading character received on USART RX line.
  * @param  None
  * @retval None
  */
void USART2_CharReception_Callback(void)
{
    __IO uint32_t received_char; 
    
    /* Read Received character. RXNE flag is cleared by reading of DR register */
    received_char = LL_USART_ReceiveData8(USART2);

    WRAP_BUFF_ADDR(cmdUartRxBufferWritePtr, RX_BUFFER_SIZE);
    WRAP_BUFF_ADDR(cmdUartRxBufferReadPtr, RX_BUFFER_SIZE);
    int bufferElemSize = (cmdUartRxBufferWritePtr - cmdUartRxBufferReadPtr);
    if(bufferElemSize < 0) {
        bufferElemSize += RX_BUFFER_SIZE;
    }

    if(bufferElemSize >= RX_BUFFER_SIZE) {
        cmdUartRxBufferOverflowFlag = 1;
    }
    else {
        cmdUartRxBuffer[cmdUartRxBufferWritePtr++] = received_char;
    }
}

/**
  * @brief  Function called from USART IRQ Handler when RXNE flag is set
  *         Function is in charge of reading character received on USART RX line.
  * @param  None
  * @retval None
  */
void USART3_CharReception_Callback(void)
{
    __IO uint32_t received_char; 
    
    /* Read Received character. RXNE flag is cleared by reading of DR register */
    received_char = LL_USART_ReceiveData8(USART3);

    lidarUartRxBuffer[lidarUartRxBufferSel][lidarUartRxBufferWritePtr++] = received_char;

    if (lidarUartRxBufferWritePtr >= RX_BUFFER_SIZE) {
        lidarUartRxBufferSel = lidarUartRxBufferSel ^ 0x1;
        lidarUartRxBufferWritePtr = 0;

        // call API to start SPI transmission.
    }
}


/**
  * @}
  */

/**
  * @}
  */
/************************ (C) Deokhwan, Kim *****END OF FILE****/
