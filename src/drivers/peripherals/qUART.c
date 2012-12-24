/*
 * qUART.c
 *
 *  Created on: Feb 1, 2012
 *      Author: Alan
 */

// API Interface
#include "qUART.h"

// Board specific
#include "lpc17xx_uart.h"
#include "lpc17xx_pinsel.h"
#include "lpc17xx_gpdma.h"

#include "LightweightRingBuff.h"

//===========================================================
// Defines
//===========================================================
#define FIFO_TRIGGER_LEVEL 14

//===========================================================
// Variables
//===========================================================
static LPC_UART_TypeDef * uarts[] = {qUART_0, qUART_1, qUART_2};
static uint8_t RxBuff[qUART_TOTAL][FIFO_TRIGGER_LEVEL];
void (*RBR_Handler[qUART_TOTAL])(uint8_t *,size_t sz) = {NULL};

static RingBuff_t UART_Out_Buffer[qUART_TOTAL];

//===========================================================
// Prototypes
//===========================================================

void UART_IntErr(uint8_t id, uint8_t bLSErrType);
void UARTx_IRQHandler(uint8_t id);

//===========================================================
// Functions
//===========================================================

ret_t qUART_Init(uint8_t id, uint32_t BaudRate, uint8_t DataBits, qUART_Parity_t Parity, uint8_t StopBits){

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
	UARTFIFOConfigStruct.FIFO_DMAMode = DISABLE;
	UARTFIFOConfigStruct.FIFO_Level = UART_FIFO_TRGLEV3;
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

	RingBuffer_InitBuffer(&UART_Out_Buffer[id]);

	// -------------------------------------------------------

	qUARTStatus[id] = DEVICE_READY;
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
#if 0
uint32_t qUART_Send(uint8_t id, uint8_t * buff, size_t size){
	if (qUARTStatus[id] == DEVICE_NOT_READY){
		return RET_ERROR;
	}
	return UART_Send(uarts[id],buff,size,BLOCKING);;
}

ret_t qUART_SendByte(uint8_t id, uint8_t ch){
	if (qUARTStatus[id] == DEVICE_NOT_READY){
		return RET_ERROR;
	}

	UART_SendByte(uarts[id],ch);

	return RET_OK;
}
#endif

uint32_t qUART_Send(uint8_t id, uint8_t * buff, size_t size){
	int bLeft = size;

	if (qUARTStatus[id] == DEVICE_NOT_READY){
		return RET_ERROR;
	}

	while (bLeft>0){
		qUART_SendByte(id,*(buff+size-bLeft));
		bLeft--;
	}
	return RET_OK;
}

ret_t qUART_SendByte(uint8_t id, uint8_t ch){

	if (qUARTStatus[id] == DEVICE_NOT_READY){
		return RET_ERROR;
	}

	UART_IntConfig(uarts[id], UART_INTCFG_THRE, DISABLE);

	// Check for the FIFO Empty and the reingBuffer Empty
	 if ( (uarts[id]->LSR & UART_LSR_THRE) && RingBuffer_IsEmpty(&UART_Out_Buffer[id]) ){
		 UART_SendByte(uarts[id],ch);
	 }else{

		 UART_IntConfig(uarts[id], UART_INTCFG_THRE, ENABLE);
		 while(RingBuffer_IsFull(&UART_Out_Buffer[id]));
		 UART_IntConfig(uarts[id], UART_INTCFG_THRE, DISABLE);

		 if (!(RingBuffer_IsFull(&UART_Out_Buffer[id]))) {
			RingBuffer_Insert(&UART_Out_Buffer[id],ch);
		 }else{
			 while(1);
		 }
	 }

	UART_IntConfig(uarts[id], UART_INTCFG_THRE, ENABLE);

	return RET_OK;
}

ret_t qUART_Register_RBR_Callback(uint8_t id, void (*pf)(uint8_t *, size_t sz)){
	if (pf == NULL){
		return RET_ERROR;
	}
	RBR_Handler[id] = pf;

	return RET_OK;
}


//===========================================================
// Handlers
//===========================================================

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
	uint32_t intsrc, tmp, tmp1;
	uint32_t rLen;

	/* Determine the interrupt source */
	intsrc = UART_GetIntId(uarts[id]);
	tmp = intsrc & UART_IIR_INTID_MASK;

	/* Receive Line Status */
	if (tmp == UART_IIR_INTID_RLS){
		// Check line status
		tmp1 = UART_GetLineStatus(uarts[id]);
		// Mask out the Receive Ready and Transmit Holding empty status
		tmp1 &= (UART_LSR_OE | UART_LSR_PE | UART_LSR_FE \
				| UART_LSR_BI | UART_LSR_RXFE);
		// If any error exist
		if (tmp1) {
			UART_IntErr(id,tmp1);
		}
	}

	/* Receive Data Available or Character time-out */
	if ((tmp == UART_IIR_INTID_RDA) || (tmp == UART_IIR_INTID_CTI)){
		/*
		 * The FIFO Triggered so read the buffer.
		 * NON BLOCKING IS FUNDAMENTAL. If the IRQ was caused by RDA then, RLen must
		 * be FIFO_TRIGGER_LEVEL. If the IRQ was caused by CTI, then rLen is important
		 */

		rLen = UART_Receive(uarts[id], (uint8_t *)&RxBuff[0], FIFO_TRIGGER_LEVEL, NONE_BLOCKING);

		//FIXME: Hardcoding!!
		//XXX: Maybe an intermediate buffer is needed.
		if (RBR_Handler[id]!=NULL){
			(*RBR_Handler[id])((uint8_t *)RxBuff,(size_t)rLen);
		}

	}

	/* Transmit Holding Empty */
	// XXX: http://www.embeddedrelated.com/groups/lpc2000/show/46607.php
	if (tmp == UART_IIR_INTID_THRE){
		while (!(RingBuffer_IsEmpty(&UART_Out_Buffer[id]))) {

		  	  if ((uarts[id]->LSR & UART_LSR_THRE)){
		  		  UART_SendByte(uarts[id], RingBuffer_Remove(&UART_Out_Buffer[id]));
		  	  }else{
		  		  break;
		  	  }
		}
	}
}

/*********************************************************************//**
 * @brief		UART Line Status Error
 * @param[in]	bLSErrType	UART Line Status Error Type
 * @return		None
 **********************************************************************/
void UART_IntErr(uint8_t id, uint8_t bLSErrType)
{
	uint8_t test;
	/* Loop forever */
	while (1){
		/* For testing purpose */
		//\TODO: Handle the errors. For example Overrun.
		test = bLSErrType;
	}
}
