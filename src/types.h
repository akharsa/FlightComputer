/*
 * types.h
 *
 *  Created on: 26/10/2012
 *      Author: alan
 */

#ifndef TYPES_H_
#define TYPES_H_

#include "lpc_types.h"
#include <stdint.h>
typedef Bool bool;
typedef enum{
	RET_OK=0,
	RET_ERROR,
	RET_ALREADY_INIT,
	RET_MSG_OK,
	RET_MSG_ERROR,
	RET_MSG_BYTES_REMAINING
}ret_t;

typedef enum{
	DEVICE_NOT_READY = 0,
	DEVICE_READY
}status_t;

#ifndef __SIZE_T_DEFINED
#define __SIZE_T_DEFINED
#if __SIZEOF_INT < __SIZEOF_VOID_P
typedef unsigned long size_t;
#else
typedef unsigned int size_t;
#endif
#endif



#endif /* TYPES_H_ */
