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
/* Lidar frame data structure:
 * (4 bytes) 0x55, 0xAA, 0x03, 0x08 (frame-sync, skipped)
 * (2 bytes) <speed(low)> <speed(high)>
 * (2 bytes) <start-angle(low)> <start-angle(high)>
 * (3 bytes) <dist0(low)> <dist0(high)> <quality0>
 * (3 bytes) <dist1(low)> <dist1(high)> <quality1>
 * (3 bytes) <dist2(low)> <dist2(high)> <quality2>
 * (3 bytes) <dist3(low)> <dist3(high)> <quality3>
 * (3 bytes) <dist4(low)> <dist4(high)> <quality4>
 * (3 bytes) <dist5(low)> <dist5(high)> <quality5>
 * (3 bytes) <dist6(low)> <dist6(high)> <quality6>
 * (3 bytes) <dist7(low)> <dist7(high)> <quality7>
 * (2 bytes) <end-angle(low)> <end-angle(high)>
 *
 * (2 bytes) <unknown> <unknown> (could be CRC, skipped)
 */
#define LIDAR_BUFFER_NUM  2
#define LIDAR_FRAME_BSIZE 34

uint8_t  lidar_data[LIDAR_BUFFER_NUM][LIDAR_FRAME_BSIZE];
uint32_t lidar_data_index = 0;
uint32_t lidar_buf_index = 0;

uint8_t SPI1TransmissionComplete = 0;
uint8_t SPI1TransmissionError = 0;
//uint32_t lidar_frame_data_index = 0;
//uint8_t lidar_frame_data[34];
//uint8_t rxdata_prev;

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
  /* UART2: command/log communication channel.
   * UART3: Lidar data reception channel. (received data is sent by SPI.)
   */

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
                         (uint32_t)lidar_data[0],
                         LL_SPI_DMA_GetRegAddr(SPI1),
                         LL_DMA_GetDataTransferDirection(DMA1, LL_DMA_CHANNEL_3));
  LL_DMA_SetDataLength(DMA1, LL_DMA_CHANNEL_3, LIDAR_FRAME_BSIZE);

  /* (5) Enable DMA transfer complete/error interrupts  */
  LL_DMA_EnableIT_TC(DMA1, LL_DMA_CHANNEL_3);
  LL_DMA_EnableIT_TE(DMA1, LL_DMA_CHANNEL_3);
}


/**
  * @brief This function configures SPI1.
  * @note  This function is used to :
  *        -1- Enables GPIO clock and configures the SPI1 pins.
  *        -2- Configure SPI1 functional parameters.
  * @note   Peripheral configuration is minimal configuration from reset values.
  *         Thus, some useless LL unitary functions calls below are provided as
  *         commented examples - setting is default configuration from reset.
  * @param  None
  * @retval None
  */
void Configure_SPI(void)
{
  /* (1) Enables GPIO clock and configures the SPI1 pins ********************/
  /* Enable the peripheral clock of GPIOB */
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_GPIOB);
  /* Remap SPI1 pins and disable JTAG */
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_AFIO);
  LL_GPIO_AF_Remap_SWJ_NOJTAG();
  while((AFIO->MAPR & AFIO_MAPR_SWJ_CFG_JTAGDISABLE) != AFIO_MAPR_SWJ_CFG_JTAGDISABLE);
  LL_GPIO_AF_EnableRemap_SPI1();

  /* Configure SCK Pin connected to pin 30 of CN10 connector */
  LL_GPIO_SetPinMode(GPIOB, LL_GPIO_PIN_3, LL_GPIO_MODE_ALTERNATE);
  LL_GPIO_SetPinSpeed(GPIOB, LL_GPIO_PIN_3, LL_GPIO_SPEED_FREQ_LOW);
  LL_GPIO_SetPinPull(GPIOB, LL_GPIO_PIN_3, LL_GPIO_PULL_DOWN);

  /* Configure MISO Pin connected to pin 28 of CN10 connector */
  //LL_GPIO_SetPinMode(GPIOB, LL_GPIO_PIN_4, LL_GPIO_MODE_ALTERNATE);
  //LL_GPIO_SetPinSpeed(GPIOB, LL_GPIO_PIN_4, LL_GPIO_SPEED_FREQ_LOW);
  //LL_GPIO_SetPinPull(GPIOB, LL_GPIO_PIN_4, LL_GPIO_PULL_DOWN);

  /* Configure MOSI Pin connected to pin 26 of CN10 connector */
  LL_GPIO_SetPinMode(GPIOB, LL_GPIO_PIN_5, LL_GPIO_MODE_ALTERNATE);
  LL_GPIO_SetPinSpeed(GPIOB, LL_GPIO_PIN_5, LL_GPIO_SPEED_FREQ_LOW);
  LL_GPIO_SetPinPull(GPIOB, LL_GPIO_PIN_5, LL_GPIO_PULL_DOWN);

  /* (2) Configure SPI1 functional parameters ********************************/
  /* Enable the peripheral clock of GPIOB */
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_SPI1);

  /* Configure SPI1 communication */
  LL_SPI_SetBaudRatePrescaler(SPI1, LL_SPI_BAUDRATEPRESCALER_DIV128);
  LL_SPI_SetTransferDirection(SPI1,LL_SPI_HALF_DUPLEX_TX);
  LL_SPI_SetClockPhase(SPI1, LL_SPI_PHASE_2EDGE);
  LL_SPI_SetClockPolarity(SPI1, LL_SPI_POLARITY_HIGH);
  /* Reset value is LL_SPI_MSB_FIRST */
  //LL_SPI_SetTransferBitOrder(SPI1, LL_SPI_MSB_FIRST);
  LL_SPI_SetDataWidth(SPI1, LL_SPI_DATAWIDTH_8BIT);
  LL_SPI_SetNSSMode(SPI1, LL_SPI_NSS_SOFT);
//#ifdef MASTER_BOARD
  LL_SPI_SetMode(SPI1, LL_SPI_MODE_MASTER);
//#else
  /* Reset value is LL_SPI_MODE_SLAVE */
  //LL_SPI_SetMode(SPI1, LL_SPI_MODE_SLAVE);
//#endif /* MASTER_BOARD */

  /* Configure SPI1 DMA transfer interrupts */
  /* Enable DMA RX Interrupt */
  //LL_SPI_EnableDMAReq_RX(SPI1);
  /* Enable DMA TX Interrupt */
  LL_SPI_EnableDMAReq_TX(SPI1);
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
    __IO uint8_t received_char; 
    
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
    const int frame_sync[4] = {0x55, 0xAA, 0x03, 0x08};
    __IO uint8_t rxdata; 

    /* Read Received character. RXNE flag is cleared by reading of DR register */
    rxdata = LL_USART_ReceiveData8(USART3);

    lidar_data[lidar_buf_index][lidar_data_index] = rxdata;

    
    /* Find the frame-sync (continuous 4 bytes)
     * if fail to found the sync, then reset the buffer index to 0.
     */
    if (lidar_data_index < 4) {
        if (rxdata == frame_sync[lidar_data_index]) {
            // Success to find sync-byte.
            lidar_data_index++;
        }
        else {
            // Fail to find frame-sync.
            // so try to find the first byte of frame sync
            lidar_data_index = 0;
        }
    }
    else {
        lidar_data_index++;
    }


    /* check the full of frame buffer, 
     * if it is full, activate SPI transmission logic.
     */
    if (lidar_data_index >= LIDAR_FRAME_BSIZE) {
        /* Activate SPI transmission.
         */
        LL_DMA_ConfigAddresses(
            DMA1, LL_DMA_CHANNEL_3,
            (uint32_t)lidar_data[lidar_buf_index],
            LL_SPI_DMA_GetRegAddr(SPI1),
            LL_DMA_GetDataTransferDirection(DMA1, LL_DMA_CHANNEL_3));
        // Enable SPI1
        LL_SPI_Enable(SPI1);
        // Enable DMA Channels
        LL_DMA_EnableChannel(DMA1, LL_DMA_CHANNEL_3);


        /* Lidar buffer switching. (for double-buffering)
         */
        lidar_data_index = 0;
        lidar_buf_index++;
        if (lidar_buf_index >= LIDAR_BUFFER_NUM) {
            lidar_buf_index = 0;
        }
    }
}

/**
  * @brief  Function called from DMA1 IRQ Handler when Tx transfer is completed
  * @param  None
  * @retval None
  */
void SPI1_TransmitComplete_Callback(void)
{
    /* DMA Tx transfer completed */
    // TODO: for debugging, instead of setting the flag, 
    // increases the flag to how many frame sync is detected!
    //SPI1TransmissionComplete = 1; // correct code.
    SPI1TransmissionComplete++;     // debug code (temp)

    /* Disable DMA1 Tx Channel */
    LL_DMA_DisableChannel(DMA1, LL_DMA_CHANNEL_3);
}

void SPI1_TransmitError_Callback(void)
{
    /* DMA Tx transfer completed */
    // TODO: for debugging, instead of setting the flag, 
    // increases the flag to how many frame sync is detected!
    //SPI1TransmissionError = 1; // correct code.
    SPI1TransmissionError++;     // debug code (temp)

    /* Disable DMA1 Tx Channel */
    LL_DMA_DisableChannel(DMA1, LL_DMA_CHANNEL_3);
}

/************************ (C) Deokhwan, Kim *****END OF FILE****/
