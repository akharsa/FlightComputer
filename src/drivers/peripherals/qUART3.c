/*
 * qUART3.c
 *
 *  Created on: 31/12/2012
 *      Author: alan
 */

#include "qUART.h"
#include "FreeRTOS.h"
#include "FreeRTOS_IO.h"

static Peripheral_Descriptor_t xOpenPort;

Peripheral_Descriptor_t xOpenAndConfigureUARTPort( void )
{
	/* The Peripheral_Descriptor_t type is the FreeRTOS+IO equivalent of a descriptor. */
	Peripheral_Descriptor_t xUARTPort;

    /* Open the UART0 port, storing the returned value as the port's descriptor.
    The peripherals that are actually available to be opened depends on the board
    support package being used.  The second parameter is not currently used and can
    be set to anything, although, for future compatibility, it is recommended that
    it is set to NULL.  By default, the port is opened with its transfer mode set
    to polling. */
	xUARTPort = FreeRTOS_open( "/UART0/", NULL );

    /* Configure the port for zero copy Tx.  The third parameter is not used in
    this case. */
//    FreeRTOS_ioctl( xUARTPort, ioctlUSE_ZERO_COPY_TX, NULL );
    FreeRTOS_ioctl( xUARTPort, ioctlUSE_POLLED_TX, NULL );

    /* Configure the same port for circular buffer Rx.  This time the third
    parameter is used, and defines the buffer size.*/
    FreeRTOS_ioctl( xUARTPort, ioctlUSE_CIRCULAR_BUFFER_RX, ( void * ) 100 );


    /* Set the read timeout to 200ms.  This is the maximum time a FreeRTOS_read()
    call will wait for the requested amount of data to be available.  As the port
    is configured to use interrupts, the task performing the read is in the
    Blocked state while the operation is in progress, so not consuming any CPU time.
    An interrupt driven zero copy write does not require a timeout to be set. */
    FreeRTOS_ioctl( xUARTPort, ioctlSET_RX_TIMEOUT, ( void * )  100 );

    return xUARTPort;
}


ret_t qUART_Init(uint8_t id, uint32_t BaudRate, uint8_t DataBits, qUART_Parity_t Parity, uint8_t StopBits){

	portBASE_TYPE xReturn;


	xOpenPort = xOpenAndConfigureUARTPort();

	if( xReturn != pdFAIL ){
		RET_OK;
	}else{
		RET_ERROR;
	}

}

uint32_t qUART_Send(uint8_t qUART_ID, uint8_t * buff, size_t size){
	size_t xBytesTransferred;
	portBASE_TYPE xReturn;

	/* This port is configured to use the zero copy Tx transfer mode, so the
	    		    write mutex must be obtained before starting a new write operation.  Wait
	    		    a maximum of 200ms for the mutex - this task will not consume any CPU time
	    		    while it is waiting. */
	//xReturn = FreeRTOS_ioctl( xOpenPort, ioctlOBTAIN_WRITE_MUTEX, ( void * ) ( 200 / portTICK_RATE_MS ) );


	if (xReturn!=pdFAIL){
	/* The write mutex was obtained, so it is safe to perform a write.  This
				writes BUFFER_SIZE bytes from cBuffer to the peripheral. */

	xBytesTransferred = FreeRTOS_write( xOpenPort, buff, size );

	/* The actual peripheral transmission is performed by an interrupt, so,
				in the particular case of using a zero copy transfer, xBytesTransferred
				will be either 0, if the transfer could not be started, or equal to
				BUFFER_SIZE.  Note however, that the interrupt may still be in the process
				of actually transmitting the data, even though the function has returned.
				The actual transmission of data will have completed when the mutex is
				available again. */


	}else{
		xBytesTransferred = 0;
	}
	return xBytesTransferred;
}

ret_t qUART_SendByte(uint8_t qUART_ID, uint8_t ch){
	qUART_Send(qUART_ID,&ch,1);
	return RET_OK;
}

uint32_t qUART_Read(uint8_t qUART_ID, uint8_t *  ucBuffer, uint32_t len){
    return FreeRTOS_read( xOpenPort, ucBuffer, len);
}
