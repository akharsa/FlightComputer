// API Interface
#include "qUART.h"

// Board specific
#include "lpc17xx_uart.h"
#include "lpc17xx_pinsel.h"
#include "lpc17xx_gpdma.h"

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
uint8_t rxBuff[MAX_BUFFERS][BUFF_SIZE];
uint8_t txBuff[MAX_BUFFERS][BUFF_SIZE];

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
	//UARTFIFOConfigStruct.FIFO_Level = UART_FIFO_TRGLEV3;
	UARTFIFOConfigStruct.FIFO_DMAMode = ENABLE;
	UART_FIFOConfig(uarts[id], &UARTFIFOConfigStruct);
	UART_TxCmd(uarts[id], ENABLE);

	GPDMA_Init();
    // Disable interrupt for DMA
    NVIC_DisableIRQ (DMA_IRQn);
    /* preemption = 1, sub-priority = 1 */
    NVIC_SetPriority(DMA_IRQn, ((0x01<<3)|0x01));


    // Estrucutura de configuración para PING PONG
       //-----------------------------------------------------------------------
   	GPDMACfg_tx.ChannelNum = DMA_CHANNEL_TX;
   	GPDMACfg_tx.SrcMemAddr = (uint32_t) txBuff[0];
   	GPDMACfg_tx.DstMemAddr = 0;
   	GPDMACfg_tx.TransferSize = BUFF_SIZE;
   	GPDMACfg_tx.TransferWidth = 0;
   	GPDMACfg_tx.TransferType = GPDMA_TRANSFERTYPE_M2P;
   	GPDMACfg_tx.SrcConn = 0;
   	GPDMACfg_tx.DstConn = GPDMA_CONN_UART0_Tx; //FIXME: Hardcoded
   	GPDMACfg_tx.DMALLI = 0;

   	GPDMACfg_rx.ChannelNum = DMA_CHANNEL_RX;
   	GPDMACfg_rx.SrcMemAddr = 0;
   	GPDMACfg_rx.DstMemAddr = (uint32_t) rxBuff[0];
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
    NVIC_EnableIRQ (DMA_IRQn);

	// Selecciono los buffers
	GPDMA_Setup(&GPDMACfg_rx);
	GPDMA_Setup(&GPDMACfg_tx);

    // Enable GPDMA channel 0 para transmisión (todavía no hay nada)
	GPDMA_ChannelCmd(0, DISABLE);
	// Enable GPDMA channel 1 para recepción
	GPDMA_ChannelCmd(1, ENABLE);

	selectedRxBuff = 0;
	selectedTxBuff = 0;

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


uint32_t qUART_Send(uint8_t id, uint8_t * buff, size_t size){

	memcpy(&(txBuff[selectedTxBuff][0]),buff,size);

	GPDMACfg_tx.SrcMemAddr = (uint32_t) &txBuff[selectedTxBuff];

	GPDMA_Setup(&GPDMACfg_tx);

	GPDMA_ChannelCmd(DMA_CHANNEL_TX, ENABLE);

	//XXX: NO hay chequeo de over run aca
	if (selectedTxBuff<(MAX_BUFFERS-1)){
		selectedTxBuff++;
	}else{
		selectedTxBuff = 0;
	}

	return RET_OK;
}

ret_t qUART_SendByte(uint8_t id, uint8_t ch){
	qUART_Send(id,&ch,1);
}

ret_t qUART_ReadByte(uint8_t id, uint8_t * buffer){
}


// =========================================================================
// IRQ Handlers
// =========================================================================

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

