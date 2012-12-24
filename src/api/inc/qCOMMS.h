/***********************************************************************//**
 * @file		qCOMMS.h
 * @brief		Contains all macro definitions and function prototypes
 * 				for the qCOMMS Comunitacion API. It implements the standar specified at:
 * 				http://quadlse.wikinet.org/wiki/COM/Protocol
 * @version		1.0
 *************************************************************************/

/** @ingroup API */
/** @addtogroup qCOMMS */
/*@{*/

#ifndef COMMS_H_
#define COMMS_H_

#ifndef MY_ADDRESS
	#error "A device address must be specified!"
#endif

#include "types.h"

//===========================================================
// Defines
//===========================================================
/**	Message header symbols */
#define COMMS_HEADER 0xEA

//===========================================================
// Types
//===========================================================

/** Enum type for different message types */
typedef enum {
	MSG_TYPE_SYSTEM=0,		/**< System message */
	MSG_TYPE_CONTROL,		/**< Control message */
	MSG_TYPE_DEBUG,			/**< Debug message */
	MSG_TYPE_TELEMETRY		/**< Telemetry message */
} DataType_t;

/** SYSTEM Msg definitios */
typedef enum {
	SYSTEM_MSG_DEBUG_ON=0,		/**< */
	SYSTEM_MSG_DEBUG_OFF,		/**< */
	SYSTEM_MSG_RESET_IDLE,		/**< */
	SYSTEM_MSG_TLM_ON,			/**< */
	SYSTEM_MSG_TLM_OFF,			/**< */
	SYSTEM_MSG_CHANGE_MODE,		/**< */
} System_Msg_t;

/** Message Struct */
typedef struct{
	uint8_t 		SourceAddress;		/**< Source address of the message. If sending a message, this field is autocompleted by the API. If was a received message, then the source address of the sender. */
	uint8_t 		DestAddress;		/**< Destination address for the message. If the message is received then this is checheck with MY_ADDRESS macro definition. If sending a message, this is the destination address */
	uint8_t 		TimeStamp;			/**< Time stamp of the message. User defined */
	DataType_t		Type;				/**< Message data type */
	uint8_t			Length;				/**< Message length */
	uint8_t  *		Payload;			/**< Pointer to the message buffer. The user must provide the buffer with enough space. */
	uint8_t			Checksum;			/**< Checksum for the whole package. If sending a message, it's autocompleted by the API. If received, it's checked and verified. */
}Msg_t;


//===========================================================
// Prototypes
//===========================================================

/********************************************************************//**
 * @brief 		Create package with payload as data and send it to ground using a preconfigured and running qUART module.
 * The source address of the message is defined by the MY_ADDRESS macro definition.
 * @param[in]	qUART_id qUART ID from a qUART module.
 * @param[in]	dest Destination address
 * @param[in]	type Message data type
 * @param[in]	size Message data size
 * @param[in]	payload	Pointer to a buffer of size "size" containing the data to be sent
 * @return 		RET_OK The packacge was send correctly to the qUART module
 * @return 		RET_OK The packacge wasn't send correctly to the qUART module
 *********************************************************************/
ret_t qComms_SendMsg(uint8_t qUART_id, uint8_t dest, DataType_t type, uint8_t size, const uint8_t * payload);


/********************************************************************//**
 * @brief 		Create package with payload as data and send it to ground using a preconfigured and running qUART module.
 * The source address of the message is defined by the MY_ADDRESS macro definition.
 * @param[in]	qUART_id qUART ID from a qUART module.
 * @param[in]	dest Destination address
 * @param[in]	type Message data type
 * @param[in]	size Message data size
 * @param[in]	payload	Pointer to a buffer of size "size" containing the data to be sent
 * @return 		RET_OK The packacge was send correctly to the qUART module
 * @return 		RET_OK The packacge wasn't send correctly to the qUART module
 *********************************************************************/
ret_t qComms_SendCompoundMsg(uint8_t qUART_id, uint8_t dest, DataType_t type, uint8_t size[], const uint8_t * payload[], int totalPayloads);


/********************************************************************//**
 * @brief 		ParseByte
 * @param[in]	buffer Pointer to a Msg_t object to store the received and processesed byte.
 * This function must be called several times with the same Msg_t Object to decode a full stream of data as a valid message.
 * @param[in]	b New byte.
 * @return		RET_MSG_BYTES_REMAINING If the byte was decoded but the package is not ready yet.
 * @return		RET_MSG_ERROR If the message contains error. Maybe because of an invalid checksum
 * @return		RET_MSG_OK If a new message is completed. The data is available at the buffer object.
 *********************************************************************/
ret_t qComms_ParseByte(Msg_t * buffer, uint8_t b);

#endif /* COMMS_H_ */

/*@}*/
/*@}*/
