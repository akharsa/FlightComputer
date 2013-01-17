// API Interface
#include "qUART.h"

// Board specific
#include "lpc17xx_uart.h"
#include "lpc17xx_pinsel.h"
#include "lpc17xx_gpdma.h"
#include "lpc17xx_timer.h"

#include "LightweightRingBuff.h"
#include "leds.h"
#include "board.h"
status_t qUARTStatus[qUART_TOTAL] = {0}; /* DEVICE_NOT_READY */

#include "FreeRTOS.h"
#include "task.h"

#include "string.h"

#define DMA_CHANNEL_RX	1
#define DMA_CHANNEL_TX	0

#define BUFF_SIZE	300
#define MAX_BUFFERS 2
__IO uint8_t rxBuff[MAX_BUFFERS][BUFF_SIZE];
__IO uint8_t txBuff[MAX_BUFFERS][BUFF_SIZE];
volatile uint32_t txBufferCount = 0;
volatile uint8_t timerRunning = 0;

void (*RBR_Handler[qUART_TOTAL])(uint8_t *,size_t sz) = {NULL};

__IO uint32_t Channel0_TC;
__IO uint32_t Channel0_Err;
__IO uint32_t Channel1_TC;
__IO uint32_t Channel1_Err;

__IO uint8_t selectedRxBuff = 0;
__IO uint8_t selectedTxBuff = 0;

GPDMA_Channel_CFG_Type GPDMACfg_rx;
GPDMA_Channel_CFG_Type GPDMACfg_tx;

//===========================================================
// Variables
//===========================================================
static LPC_UART_TypeDef * uarts[] = {qUART_0, qUART_1, qUART_2};

ret_t qUART_Init(uint8_t id, uint32_t BaudRate, uint8_t DataBits, qUART_Parity_t Parity, uint8_t StopBits){

	PINSEL_CFG_Type PinCfg;
	UART_CFG_Type UARTConfigStruct;
	UART_FIFO_CFG_Type UARTFIFOConfigStruct;
	TIM_TIMERCFG_Type TIM_ConfigStruct;
	TIM_MATCHCFG_Type TIM_MatchConfigStruct ;

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

	// Agregue las interucopiones
	UART_TxCmd(uarts[id], ENABLE);
	UART_IntConfig(uarts[id], UART_INTCFG_RBR, ENABLE);
	//UART_IntConfig(uarts[id], UART_INTCFG_RLS, ENABLE);
	//UART_IntConfig(uarts[id], UART_INTCFG_THRE, ENABLE);


	if (uarts[id]==LPC_UART0){
		NVIC_SetPriority(UART0_IRQn, 6);
		NVIC_EnableIRQ (UART0_IRQn);

	}else if (uarts[id]==LPC_UART2){
		NVIC_SetPriority(UART2_IRQn, 6);
		NVIC_EnableIRQ (UART2_IRQn);

	}else if (uarts[id]==LPC_UART3){
		NVIC_SetPriority(UART3_IRQn, 6);
		NVIC_EnableIRQ (UART3_IRQn);

	}else{
		return RET_ERROR;
	}

	// -------------------------------------------------------
	// UART FIFOS

	UART_FIFOConfigStructInit(&UARTFIFOConfigStruct);
	UARTFIFOConfigStruct.FIFO_Level = UART_FIFO_TRGLEV3;
	UARTFIFOConfigStruct.FIFO_DMAMode = ENABLE;
	UART_FIFOConfig(uarts[id], &UARTFIFOConfigStruct);
	UART_TxCmd(uarts[id], ENABLE);

	GPDMA_Init();

    // Disable interrupt for DMA
    //NVIC_DisableIRQ (DMA_IRQn);
    /* preemption = 1, sub-priority = 1 */
    //NVIC_SetPriority(DMA_IRQn, 1);


    // Estrucutura de configuración para PING PONG
       //-----------------------------------------------------------------------
   	GPDMACfg_tx.ChannelNum = DMA_CHANNEL_TX;
   	GPDMACfg_tx.SrcMemAddr = (uint32_t) txBuff[selectedTxBuff];
   	GPDMACfg_tx.DstMemAddr = 0;
   	GPDMACfg_tx.TransferSize = BUFF_SIZE;
   	GPDMACfg_tx.TransferWidth = 0;
   	GPDMACfg_tx.TransferType = GPDMA_TRANSFERTYPE_M2P;
   	GPDMACfg_tx.SrcConn = 0;
   	GPDMACfg_tx.DstConn = GPDMA_CONN_UART0_Tx; //FIXME: Hardcoded
   	GPDMACfg_tx.DMALLI = 0;

   	GPDMACfg_rx.ChannelNum = DMA_CHANNEL_RX;
   	GPDMACfg_rx.SrcMemAddr = 0;
   	GPDMACfg_rx.DstMemAddr = (uint32_t) rxBuff[selectedRxBuff];
   	GPDMACfg_rx.TransferSize = BUFF_SIZE;
   	GPDMACfg_rx.TransferWidth = 0;
   	GPDMACfg_rx.TransferType = GPDMA_TRANSFERTYPE_P2M;
   	GPDMACfg_rx.SrcConn = GPDMA_CONN_UART0_Rx;//FIXME: Hardcoded
   	GPDMACfg_rx.DstConn = 0;
   	GPDMACfg_rx.DMALLI = 0;

   	//-----------------------------------------------------------------------

	Channel0_TC = 0;
	Channel0_Err = 0;
	Channel1_TC = 0;
	Channel1_Err = 0;

    // Enable interrupt for DMA
    //NVIC_EnableIRQ (DMA_IRQn);

	// Selecciono los buffers
	GPDMA_Setup(&GPDMACfg_rx);
	//GPDMA_Setup(&GPDMACfg_tx);

    // Enable GPDMA channel 0 para transmisión (todavía no hay nada)
//	GPDMA_ChannelCmd(DMA_CHANNEL_TX, DISABLE);
	// Enable GPDMA channel 1 para recepción
	GPDMA_ChannelCmd(DMA_CHANNEL_RX, ENABLE);

	selectedRxBuff = 0;
	selectedTxBuff = 0;


	// Initialize timer 0, prescale count time of 100uS
	TIM_ConfigStruct.PrescaleOption = TIM_PRESCALE_USVAL;
	TIM_ConfigStruct.PrescaleValue	= 1;

	// use channel 0, MR0
	TIM_MatchConfigStruct.MatchChannel = 0;
	TIM_MatchConfigStruct.IntOnMatch   = TRUE;
	TIM_MatchConfigStruct.ResetOnMatch = TRUE;
	TIM_MatchConfigStruct.StopOnMatch  = TRUE;
	TIM_MatchConfigStruct.ExtMatchOutputType =TIM_EXTMATCH_NOTHING;

	TIM_MatchConfigStruct.MatchValue   = 10009;

	// Set configuration for Tim_config and Tim_MatchConfig
	TIM_Init(LPC_TIM0, TIM_TIMER_MODE,&TIM_ConfigStruct);
	TIM_ConfigMatch(LPC_TIM0,&TIM_MatchConfigStruct);

	/* preemption = 1, sub-priority = 1 */
	NVIC_SetPriority(TIMER0_IRQn, 6);

	/* Enable interrupt for timer 0 */
	NVIC_EnableIRQ(TIMER0_IRQn);



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

ret_t qUART_Register_RBR_Callback(uint8_t id, void (*pf)(uint8_t *, size_t sz)){
	if (pf == NULL){
		return RET_ERROR;
	}
	RBR_Handler[id] = pf;

	return RET_OK;
}


void flushBuffer(){

	GPDMACfg_tx.SrcMemAddr = (uint32_t) &txBuff[selectedTxBuff];
	GPDMACfg_tx.TransferSize = txBufferCount;
	GPDMA_Setup(&GPDMACfg_tx);
	GPDMA_ChannelCmd(DMA_CHANNEL_TX, ENABLE);
	//XXX: NO hay chequeo de over run aca
	if (selectedTxBuff==1){
		selectedTxBuff = 0;
	}else{
		selectedTxBuff = 1;
	}

}

uint32_t qUART_Send(uint8_t id, uint8_t * buff, size_t size){

	TIM_Cmd(LPC_TIM0,DISABLE);

	// Chequeo si hay lugar en el buffer de transmision
	if ((txBufferCount+size)<BUFF_SIZE){
		// Si hay lugar lo meto adentro
		memcpy(&(txBuff[selectedTxBuff][txBufferCount]),buff,size);
		txBufferCount += size;
	}else{
		// Si no hay lugar, mando el buffer viejo swapeo y lo meto en el nuevo
		flushBuffer();
		txBufferCount = 0;
		memcpy(&(txBuff[selectedTxBuff][txBufferCount]),buff,size);
		txBufferCount += size;
	}

	if (txBufferCount == (BUFF_SIZE-1)){
		flushBuffer();
		txBufferCount = 0;
	}else{
		if (timerRunning == 1){
			TIM_ResetCounter(LPC_TIM0);
		}else{
			timerRunning = 1;
			TIM_ResetCounter(LPC_TIM0);
			TIM_Cmd(LPC_TIM0,ENABLE);
		}
	}



	return RET_OK;
}

ret_t qUART_SendByte(uint8_t id, uint8_t ch){
	qUART_Send(id,&ch,1);
}

ret_t qUART_ReadByte(uint8_t id, uint8_t * buffer){
}




void DMA_IRQHandler (void)
{
	uint32_t tmp;
	// Scan interrupt pending
	for (tmp = 0; tmp <= 7; tmp++) {
		if (GPDMA_IntGetStatus(GPDMA_STAT_INT, tmp)){
			// Check counter terminal status
			if(GPDMA_IntGetStatus(GPDMA_STAT_INTTC, tmp)){
				// Clear terminate counter Interrupt pending
				GPDMA_ClearIntPending (GPDMA_STATCLR_INTTC, tmp);
				switch (tmp){
					case DMA_CHANNEL_TX:
						Channel0_TC++;
						GPDMA_ChannelCmd(0, DISABLE);
						break;
					case DMA_CHANNEL_RX:
						Channel1_TC++;
						GPDMA_ChannelCmd(1, DISABLE);
						break;
					default:
						break;
				}

			}
				// Check error terminal status
			if (GPDMA_IntGetStatus(GPDMA_STAT_INTERR, tmp)){
				// Clear error counter Interrupt pending
				GPDMA_ClearIntPending (GPDMA_STATCLR_INTERR, tmp);
				switch (tmp){
					case 0:
						Channel0_Err++;
						GPDMA_ChannelCmd(0, DISABLE);
						break;
					case 1:
						Channel1_Err++;
						GPDMA_ChannelCmd(1, DISABLE);
						break;
					default:
						break;
				}
			}
		}
	}
}


//===========================================================
// Handlers
//===========================================================

void UARTx_IRQHandler(uint8_t id);
void UART_IntErr(uint8_t id, uint8_t bLSErrType);

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
	uint32_t buffsize;
	uint32_t i=0;

	for (i=0;i<256;i++){
		// ESTE delay hace que todo funcione,
		// sino parece que no terminaba el transer y le
		// cambiaba el banco antes de terminar y se rompia todo
		// 1000 no es mucho?
	}

	GPDMA_ChannelCmd(DMA_CHANNEL_RX, DISABLE);
	// Desde aca hay solo una FIFO (16 bytes) de tiempo para encender el otro buffer

	buffsize = (LPC_GPDMACH1->DMACCDestAddr) -  (uint32_t)&(rxBuff[selectedRxBuff]);
	if (selectedRxBuff==0){
		selectedRxBuff = 1;
	}else{
		selectedRxBuff = 0;
	}

	GPDMACfg_rx.DstMemAddr = (uint32_t) &rxBuff[selectedRxBuff];
	GPDMA_Setup(&GPDMACfg_rx);

	// Hasta aca!
	GPDMA_ChannelCmd(DMA_CHANNEL_RX, ENABLE);

	if (RBR_Handler[id]!=NULL){
		if (selectedRxBuff==0){
			(*RBR_Handler[id])((uint8_t *)&rxBuff[1],(size_t)buffsize);
		}else{
			(*RBR_Handler[id])((uint8_t *)&rxBuff[0],(size_t)buffsize);
		}
	}

#if 0
	uint32_t intsrc, tmp, tmp1;

	/* Determine the interrupt source */
	intsrc = UART_GetIntId(uarts[id]);
	tmp = intsrc & UART_IIR_INTID_MASK;

	/* Receive Line Status: error checking register */
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
		qLed_TurnOn(STATUS_LED);
	}

	/* Transmit Holding Empty */
	// XXX: http://www.embeddedrelated.com/groups/lpc2000/show/46607.php
	if (tmp == UART_IIR_INTID_THRE){
		qLed_TurnOn(STATUS_LED);
	}
#endif
}


void TIMER0_IRQHandler(void)
{
	if (TIM_GetIntStatus(LPC_TIM0, TIM_MR0_INT)== SET)
	{
		// ESTA PARADO SOLO POR QUE ASI ESTA CONFIGURADO
		flushBuffer();
		timerRunning = 0;
	}
	TIM_ClearIntPending(LPC_TIM0, TIM_MR0_INT);
}

/*********************************************************************//**
 * @brief		UART Line Status Error
 * @param[in]	bLSErrType	UART Line Status Error Type
 * @return		None
 **********************************************************************/
void UART_IntErr(uint8_t id, uint8_t bLSErrType)
{
//	qLed_TurnOn(FRONT_LEFT_LED);
//	while (1){
		//TODO: Handle the errors. For example Overrun.
//	}

}


