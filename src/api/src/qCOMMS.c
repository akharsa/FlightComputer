/*
 * comms.c
 *
 *  Created on: Jan 13, 2012
 *      Author: Alan
 */

#include <stdint.h>
#include "types.h"
#include "qCOMMS.h"
#include "qUART.h"

//===========================================================
// Prototypes
//===========================================================

uint32_t _qComms_getCurrentTimeStamp();
void _qComms_CreateMsg (uint8_t dest, DataType_t type, size_t size, const uint8_t * payload, Msg_t * m);


uint32_t _qComms_getCurrentTimeStamp(){
	return 0x55;
}


//===========================================================
// Types
//===========================================================

typedef enum {
	MSG_STATE_HEADER,
	MSG_STATE_SOURCE,
	MSG_STATE_DEST,
	MSG_STATE_TIME,
	MSG_STATE_TYPE,
	MSG_STATE_LENGTH,
	MSG_STATE_PAYLOAD,
	MSG_STATE_CHECKSUM,
} Msg_Parser_State_t;


//===========================================================
// Functions
//===========================================================
void _qComms_CreateMsg (uint8_t dest, DataType_t type, size_t size, const uint8_t * payload, Msg_t * m){

	m->SourceAddress = MY_ADDRESS;
	m->DestAddress = dest;
	m->TimeStamp = _qComms_getCurrentTimeStamp();
	m->Type = type;
	m->Length = size;
	m->Payload = payload;
	m->Checksum = (uint8_t)	(m->SourceAddress +
					m->DestAddress +
					m->TimeStamp +
					m->Length);
}

ret_t qComms_SendMsg(uint8_t qUART_id, uint8_t dest, DataType_t type, uint8_t size, const uint8_t * payload){
	Msg_t MsgBuffer;

	_qComms_CreateMsg(dest,type,size,payload,&MsgBuffer);

	uint8_t buffer[6+size+1];

	buffer[0] = COMMS_HEADER;
	buffer[1] = MsgBuffer.SourceAddress;
	buffer[2] = MsgBuffer.DestAddress;
	buffer[3] = MsgBuffer.TimeStamp;
	buffer[4] = MsgBuffer.Type;
	buffer[5] = MsgBuffer.Length;

	memcpy(&buffer[6],MsgBuffer.Payload,size);

	buffer[6+size] = MsgBuffer.Checksum;

	qUART_Send(qUART_id,(uint8_t *)buffer,sizeof(buffer));

	/*qUART_SendByte(qUART_id,);
	qUART_SendByte(qUART_id,);
	qUART_SendByte(qUART_id,);
	qUART_SendByte(qUART_id,);
	qUART_SendByte(qUART_id,);
	qUART_SendByte(qUART_id,);*/
	//qUART_Send(qUART_id,(uint8_t *)buffer,sizeof(buffer));
	//qUART_Send(qUART_id,(uint8_t *)MsgBuffer.Payload,MsgBuffer.Length);
	//qUART_SendByte(qUART_id,MsgBuffer.Checksum);

	return RET_OK;
}

ret_t qComms_SendCompoundMsg(uint8_t qUART_id, uint8_t dest, DataType_t type, uint8_t size[], const uint8_t * payload[], int totalPayloads){
	Msg_t MsgBuffer;

	unsigned int msgSize = 0;
	unsigned int i;

	for (i=0;i<totalPayloads;i++){
		msgSize += size[i];
	}

	_qComms_CreateMsg(dest,type,msgSize,payload,&MsgBuffer);

	qUART_SendByte(qUART_id,COMMS_HEADER);
	qUART_SendByte(qUART_id,MsgBuffer.SourceAddress);
	qUART_SendByte(qUART_id,MsgBuffer.DestAddress);
	qUART_SendByte(qUART_id,MsgBuffer.TimeStamp);
	qUART_SendByte(qUART_id,MsgBuffer.Type);
	qUART_SendByte(qUART_id,msgSize);
	for (i=0;i<totalPayloads;i++){
		qUART_Send(qUART_id,payload[i],size[i]);
	}
	qUART_SendByte(qUART_id,MsgBuffer.Checksum);

	return RET_OK;
}
ret_t qComms_ParseByte(Msg_t * msg, uint8_t b){

	static Msg_Parser_State_t state = MSG_STATE_HEADER;
	static unsigned int bytesReaded = 0;

	if (msg->Payload == 0x00){
		return RET_ERROR;
	}

	switch (state){
		case MSG_STATE_HEADER:
		//	if (b==COMMS_HEADER){
				bytesReaded = 0;
				state = MSG_STATE_SOURCE;
				return RET_MSG_BYTES_REMAINING;
			//}else{
				//return RET_MSG_BYTES_REMAINING;
		//	}
			break;
		case MSG_STATE_SOURCE:
			msg->SourceAddress = b;
			state = MSG_STATE_DEST;
			return RET_MSG_BYTES_REMAINING;
			break;
		case MSG_STATE_DEST:
			//TODO: Is the package for me?
			msg->DestAddress = b;
			state = MSG_STATE_TIME;
			return RET_MSG_BYTES_REMAINING;
			break;
		case MSG_STATE_TIME:
			//TODO: Is the correct time or seq?
			msg->TimeStamp = b;
			state = MSG_STATE_TYPE;
			return RET_MSG_BYTES_REMAINING;
			break;
		case MSG_STATE_TYPE:
			msg->Type = b;
			state = MSG_STATE_LENGTH;
			return RET_MSG_BYTES_REMAINING;
			break;
		case MSG_STATE_LENGTH:
			msg->Length = b;
			state = MSG_STATE_PAYLOAD;
			return RET_MSG_BYTES_REMAINING;
			break;
		case MSG_STATE_PAYLOAD:
			*(msg->Payload+bytesReaded) = b;
			if ((++bytesReaded)==msg->Length){
				state = MSG_STATE_CHECKSUM;
				return RET_MSG_BYTES_REMAINING;
			}
			break;
		case MSG_STATE_CHECKSUM:
			if ((uint8_t)  (msg->SourceAddress +
							msg->DestAddress +
							msg->TimeStamp +
							msg->Length) == b ){
				//Valid msg
				state = MSG_STATE_HEADER;
				return RET_MSG_OK;
			}else{
				// Invalid msg
				state = MSG_STATE_HEADER;
				return RET_MSG_ERROR;
			}
			break;
	}

	// The state machine always return.
	// This is an error
	return RET_ERROR;
}
