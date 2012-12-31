/***********************************************************************//**
 * @file		qUART.h
 * @brief		Contains all macro definitions and function prototypes
 * 				for the UART driver.
 * @version
 * @date
 * @author
 *************************************************************************/
/** @ingroup BSP */
/** @addtogroup qUART */
/*@{*/

#ifndef qUART_H_
#define qUART_H_

#include"types.h"

#define qUART_TOTAL			3
#define qUART_0				LPC_UART0
#define qUART_1				LPC_UART2
#define qUART_2				LPC_UART3

//===========================================================
// Types
//===========================================================
/** Enum type for parity configuration */
typedef enum{
	QUART_PARITY_NONE=0,	/**< No parity is used */
	QUART_PARITY_ODD,		/**< Adds a 1 if the number of ones is not odd */
	QUART_PARITY_EVEN,		/**< Adds a 1 if the number of ones is not even */
}qUART_Parity_t;

//===========================================================
// Externs
//===========================================================
/** Stores the status of the different UARTS */
/* XXX: Can this be private to the implementation */
extern status_t qUARTStatus[qUART_TOTAL];

//===========================================================
// Prototypes
//===========================================================

/********************************************************************//**
 * @brief 		Configures all the hardware and software needed for getting the UART functional.
 * @param[in]	id UART number
 * @param[in]	BaudRate BaudRate
 * @param[in]	DataBits DataBits
 * @param[in]	Parity Parity
 * @param[in]	StopBits StopBits
 * @return 		RET_OK If the device was initialized
 * @return 		RET_ERROR If the device couldn't be initialized
 * @return 		RET_DEVICE_ALREADY_INIT If the device was already Initialized. Call qUART_DeInit first.
 *********************************************************************/
ret_t qUART_Init(uint8_t id, uint32_t BaudRate, uint8_t DataBits, qUART_Parity_t Parity, uint8_t StopBits);

/********************************************************************//**
 * @brief 		Deinits all the UART hardware.
 * @param[in]	id UART number (defined in BSP_config.h)
 * @return 		RET_OK
 * @return 		RET_ERROR
 *********************************************************************/
ret_t qUART_DeInit(uint8_t qUART_ID);

/********************************************************************//**
 * @brief 		Callback register function.
 * 	This function is called every time a new chunk of data is called. Then buffer pointer and size is passed.
 * 	WARNING: This callback is anidated to the ISR so keep ir short and if possible, decouple to another task to allow the system to return from ISR.
 * @param[in]	id UART number
 * @param[in]	pf Pointer to function that handles new data
 * @return 		RET_OK, RET_ERROR
 *********************************************************************/
ret_t qUART_Register_RBR_Callback(uint8_t id, void (*pf)(uint8_t *, size_t sz));

/********************************************************************//**
 * @brief 		Sends a buffer of data pointed by 'buff' of size 'size'
 * @param[in]	id UART number
 * @param[in]	buff Pointer to data buffer
 * @param[in]	size Buffer length
 * @return 		Number of bytes sent
 *********************************************************************/
uint32_t qUART_Send(uint8_t qUART_ID, uint8_t * buff, size_t size);

/********************************************************************//**
 * @brief 		Sends only one byte
 * @param[in]	id UART number
 * @param[in]	Byte Data to be sent
 * @return 		RET_OK
 * @return 		RET_ERROR
 *********************************************************************/
ret_t qUART_SendByte(uint8_t qUART_ID, uint8_t ch);

/********************************************************************//**
 * @brief 		Sends only one byte
 * @param[in]	id UART number
 * @param[in]	Byte Data to be sent
 * @return 		RET_OK
 * @return 		RET_ERROR
 *********************************************************************/
uint32_t qUART_Read(uint8_t qUART_ID, uint8_t *  ucBuffer, uint32_t len);

#endif /* qUART_H_ */

/*@}*/
/*@}*/
