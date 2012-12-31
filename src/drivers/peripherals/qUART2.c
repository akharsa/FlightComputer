// API Interface
#include "qUART.h"

// Board specific
#include "lpc17xx_uart.h"
#include "lpc17xx_pinsel.h"
#include "lpc17xx_gpdma.h"

status_t qUARTStatus[qUART_TOTAL] = {0}; /* DEVICE_NOT_READY */
static LPC_UART_TypeDef * uarts[] = {qUART_0, qUART_1, qUART_2};

#define RX_BUF_SIZE 32

volatile uint8_t rx_buf[RX_BUF_SIZE];

ret_t qUART_Init(uint8_t id, uint32_t BaudRate, uint8_t DataBits, qUART_Parity_t Parity, uint8_t StopBits){
	GPDMA_Channel_CFG_Type GPDMACfg;
	PINSEL_CFG_Type PinCfg;
	UART_CFG_Type UARTConfigStruct;
	UART_FIFO_CFG_Type UARTFIFOConfigStruct;

	uint8_t rxPin,txPin,rxPort,txPort,pinFunc;

	// Check if the device wasn't initialized first
	if (qUARTStatus[id]==DEVICE_READY){
		return RET_ALREADY_INIT;
	}

	// Config pins
	if (uarts[id]==LPC_UART0){
		rxPin = PINSEL_PIN_2;
		rxPort = PINSEL_PORT_0;
		txPin = PINSEL_PIN_3;
		txPort = PINSEL_PORT_0;
		pinFunc = 1;
	}else if (uarts[id]==LPC_UART2){
		rxPin = PINSEL_PIN_10;
		rxPort = PINSEL_PORT_0;
		txPin = PINSEL_PIN_11;
		txPort = PINSEL_PORT_0;
		pinFunc = 1;
	}else if (uarts[id]==LPC_UART3){
		rxPin = PINSEL_PIN_0;
		rxPort = PINSEL_PORT_0;
		txPin = PINSEL_PIN_1;
		txPort = PINSEL_PORT_0;
		pinFunc = 2;
	}else{
		return RET_ERROR;
	}

	PinCfg.Funcnum = pinFunc;
	PinCfg.OpenDrain = PINSEL_PINMODE_NORMAL;
	PinCfg.Pinmode = PINSEL_PINMODE_TRISTATE;
	PinCfg.Pinnum = rxPin;
	PinCfg.Portnum = rxPort;
	PINSEL_ConfigPin(&PinCfg);
	PinCfg.Pinnum = txPin;
	PinCfg.Portnum = txPort;
	PINSEL_ConfigPin(&PinCfg);

	UARTConfigStruct.Baud_rate = BaudRate;
	UARTConfigStruct.Databits = UART_DATABIT_8; 	//FIXME: remove hardcode
	UARTConfigStruct.Parity = UART_PARITY_NONE;		//FIXME: remove hardcode
	UARTConfigStruct.Stopbits = UART_STOPBIT_1;		//FIXME: remove hardcode

	UART_Init(uarts[id], &UARTConfigStruct);

	// -------------------------------------------------------
	// UART FIFOS

	UART_FIFOConfigStructInit(&UARTFIFOConfigStruct);
	UARTFIFOConfigStruct.FIFO_DMAMode = ENABLE;
	//UARTFIFOConfigStruct.FIFO_Level = UART_FIFO_TRGLEV3;
	UART_FIFOConfig(uarts[id], &UARTFIFOConfigStruct);

	UART_TxCmd(uarts[id], ENABLE);
	UART_IntConfig(uarts[id], UART_INTCFG_RBR, ENABLE);
	UART_IntConfig(uarts[id], UART_INTCFG_RLS, ENABLE);
	UART_IntConfig(uarts[id], UART_INTCFG_THRE, ENABLE);

	if (uarts[id]==LPC_UART0){
		NVIC_EnableIRQ (UART0_IRQn);
	}else if (uarts[id]==LPC_UART2){
		NVIC_EnableIRQ (UART2_IRQn);
	}else if (uarts[id]==LPC_UART3){
		NVIC_EnableIRQ (UART3_IRQn);
	}else{
		return RET_ERROR;
	}

	// -------------------------------------------------------
	// GPDMA Interrupt configuration section

    /* Initialize GPDMA controller */
	GPDMA_Init();

	/* Setting GPDMA interrupt */
    // Disable interrupt for DMA
    NVIC_DisableIRQ (DMA_IRQn);
    /* preemption = 1, sub-priority = 1 */
    NVIC_SetPriority(DMA_IRQn, ((0x01<<3)|0x01));

    // Setup GPDMA channel --------------------------------
    // channel 1
    GPDMACfg.ChannelNum = 0;
    // Source memory - don't care
    GPDMACfg.SrcMemAddr = 0;
    // Destination memory
    GPDMACfg.DstMemAddr = (uint32_t) &rx_buf;
    // Transfer size
    GPDMACfg.TransferSize = sizeof(rx_buf);
    // Transfer width - don't care
    GPDMACfg.TransferWidth = 0;
    // Transfer type
    GPDMACfg.TransferType = GPDMA_TRANSFERTYPE_P2M;
    // Source connection
    GPDMACfg.SrcConn = GPDMA_CONN_UART0_Rx;
    // Destination connection - don't care
    GPDMACfg.DstConn = 0;
    // Linker List Item - unused
    GPDMACfg.DMALLI = 0;

    GPDMA_Setup(&GPDMACfg);

    // Enable interrupt for DMA
    NVIC_EnableIRQ (DMA_IRQn);

    // Enable GPDMA channel 0
    GPDMA_ChannelCmd(0, ENABLE);


	// -------------------------------------------------------

	qUARTStatus[id] = DEVICE_READY;
	return RET_OK;
}

uint32_t qUART_Send(uint8_t id, uint8_t * buff, size_t size){
	return RET_OK;
}

ret_t qUART_SendByte(uint8_t id, uint8_t ch){
	return RET_OK;
}
ret_t qUART_Register_RBR_Callback(uint8_t id, void (*pf)(uint8_t *, size_t sz)){
	return RET_OK;
}

ret_t qUART_DeInit(uint8_t id){
	if (qUARTStatus[id] == DEVICE_READY){
		UART_DeInit(uarts[id]);
		qUARTStatus[id] = DEVICE_NOT_READY;
		return RET_OK;
	}else{
		return RET_ERROR;
	}
}

void UART0_IRQHandler(void)
{
	UARTx_IRQHandler(0);
}
void UART2_IRQHandler(void)
{
	UARTx_IRQHandler(1);
}
void UART3_IRQHandler(void)
{
	UARTx_IRQHandler(2);
}


void UARTx_IRQHandler(uint8_t id){
	uint32_t intsrc, tmp,tmp1;

	/* Determine the interrupt source */
	intsrc = UART_GetIntId(uarts[id]);
	tmp = intsrc & UART_IIR_INTID_MASK;

	switch (tmp){
	case UART_IIR_INTID_RLS:
		tmp1 = UART_GetLineStatus(uarts[id]);
		// Mask out the Receive Ready and Transmit Holding empty status
		tmp1 &= (UART_LSR_OE | UART_LSR_PE | UART_LSR_FE \
				| UART_LSR_BI | UART_LSR_RXFE);
		// If any error exist
		if (tmp1) {
			UART_IntErr(id,tmp1);
		}
		break;
	case UART_IIR_INTID_RDA:
		break;
	case UART_IIR_INTID_CTI:
		while(1);
		break;
	case UART_IIR_INTID_THRE:
		/*
		while (!(RingBuffer_IsEmpty(&UART_Out_Buffer[id]))) {

		  	  if ((uarts[id]->LSR & UART_LSR_THRE)){
		  		  UART_SendByte(uarts[id], RingBuffer_Remove(&UART_Out_Buffer[id]));
		  	  }else{
		  		  break;
		  	  }
		}
		*/
		break;
	}
}


void DMA_IRQHandler (void)
{
	uint32_t tmp;
	// Scan interrupt pending
	for (tmp = 0; tmp <= 0; tmp++) {
		if (GPDMA_IntGetStatus(GPDMA_STAT_INT, tmp)){
			// Check counter terminal status
			if(GPDMA_IntGetStatus(GPDMA_STAT_INTTC, tmp)){
				// Clear terminate counter Interrupt pending
				GPDMA_ClearIntPending (GPDMA_STATCLR_INTTC, tmp);
			}

			if (GPDMA_IntGetStatus(GPDMA_STAT_INTERR, tmp)){
				// Clear error counter Interrupt pending
				GPDMA_ClearIntPending (GPDMA_STATCLR_INTERR, tmp);
			}
		}
	}
}


void UART_IntErr(uint8_t id, uint8_t bLSErrType)
{
	uint8_t test;
	while (1){
		test = bLSErrType;
	}
}
