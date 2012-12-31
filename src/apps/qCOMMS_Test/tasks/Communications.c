/*
 * Comms.c
 *
 *  Created on: Feb 6, 2012
 *      Author: Alan
 *
 */
#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"
#include "semphr.h"

#include "qUART.h"
#include "qCOMMS.h"

#include "string.h"
#include "DebugConsole.h"
#include "board.h"

#include "qESC.h"
#include "leds.h"

/* FreeRTOS+IO includes. */
#include "FreeRTOS_IO.h"


Peripheral_Descriptor_t xOpenAndConfigureUARTPort( void )
{
	/* The Peripheral_Descriptor_t type is the FreeRTOS+IO equivalent of a descriptor. */
	Peripheral_Descriptor_t xUARTPort;

    /* Open the I2C0 port, storing the returned value as the port's descriptor.
    The peripherals that are actually available to be opened depends on the board
    support package being used.  The second parameter is not currently used and can
    be set to anything, although, for future compatibility, it is recommended that
    it is set to NULL.  By default, the port is opened with its transfer mode set
    to polling. */
	xUARTPort = FreeRTOS_open( "/UART0/", NULL );

    /* FreeRTOS_open() returns NULL when the open operation cannot complete.  Check
    the return value is not NULL. */
    configASSERT( xUARTPort );

    /* Configure the port for zero copy Tx.  The third parameter is not used in
    this case. */
    FreeRTOS_ioctl( xUARTPort, ioctlUSE_ZERO_COPY_TX, NULL );

    /* Configure the same port for circular buffer Rx.  This time the third
    parameter is used, and defines the buffer size.*/
    FreeRTOS_ioctl( xUARTPort, ioctlUSE_CIRCULAR_BUFFER_RX, ( void * ) 100 );


    /* Set the read timeout to 200ms.  This is the maximum time a FreeRTOS_read()
    call will wait for the requested amount of data to be available.  As the port
    is configured to use interrupts, the task performing the read is in the
    Blocked state while the operation is in progress, so not consuming any CPU time.
    An interrupt driven zero copy write does not require a timeout to be set. */
    FreeRTOS_ioctl( xUARTPort, ioctlSET_RX_TIMEOUT, ( void * ) ( 200 / portTICK_RATE_MS ) );

    /* Set the I2C clock frequency to 400000. */
   // FreeRTOS_ioctl( xUARTPort, iocltSET_SPEED, ( void * ) 400000 );

    /* Set the I2C slave address to 50. */
    //FreeRTOS_ioctl( xUARTPort, iocltSET_I2C_SLAVE_ADDRESS, ( void * ) 50 );

    /* Return a handle to the open port, which can now be used in FreeRTOS_read()
    and FreeRTOS_write() calls. */
    return xUARTPort;
}

#define BUFFER_SIZE		200
const int8_t cBuffer[200] = { "Hello world!"};


void Communications(void * pvParameters){

	qLed_Init(FRONT_LEFT_LED);

	size_t xBytesTransferred;
	portBASE_TYPE xReturn;

	Peripheral_Descriptor_t xOpenPort;

	xOpenPort = xOpenAndConfigureUARTPort();

	UART_Send(LPC_UART0,cBuffer,BUFFER_SIZE,BLOCKING);

    if( xReturn != pdFAIL )
    {
    	for(;;){

    		/* This port is configured to use the zero copy Tx transfer mode, so the
    		    write mutex must be obtained before starting a new write operation.  Wait
    		    a maximum of 200ms for the mutex - this task will not consume any CPU time
    		    while it is waiting. */
    		xReturn = FreeRTOS_ioctl( xOpenPort, ioctlOBTAIN_WRITE_MUTEX, ( void * ) ( 200 / portTICK_RATE_MS ) );


			/* The write mutex was obtained, so it is safe to perform a write.  This
			writes BUFFER_SIZE bytes from cBuffer to the peripheral. */

			xBytesTransferred = FreeRTOS_write( xOpenPort, cBuffer, BUFFER_SIZE );

			/* The actual peripheral transmission is performed by an interrupt, so,
			in the particular case of using a zero copy transfer, xBytesTransferred
			will be either 0, if the transfer could not be started, or equal to
			BUFFER_SIZE.  Note however, that the interrupt may still be in the process
			of actually transmitting the data, even though the function has returned.
			The actual transmission of data will have completed when the mutex is
			available again. */

			vTaskDelay(500/portTICK_RATE_MS);
    	}
    }

	//vTaskDelay(3000/portTICK_RATE_MS);
	//XXX: Should this be done in the comms api?
/*
	if (qUART_Init(UART_GROUNDCOMM,57600,8,QUART_PARITY_NONE,1)==RET_ERROR){
		while(1);
	}
	qUART_Register_RBR_Callback(UART_GROUNDCOMM, UART_Rx_Handler);

	msg.Payload = msgBuff;

	ConsolePuts_("======================================\r\n",WHITE);
	ConsolePuts_("Autopilot @ FLC_v2p0 running...\r\n\r\n",WHITE);

	ControlQueue = xQueueCreate(4,sizeof(uint8_t)*255);
	xTaskCreate( ControlDataHandle, ( signed char * ) "COMMS/CONTROL", configMINIMAL_STACK_SIZE, ( void * ) NULL, 1, NULL );
	vTaskDelete(NULL);
	*/
}

