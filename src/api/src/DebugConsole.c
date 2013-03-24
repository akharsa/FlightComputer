#include "DebugConsole.h"
#include "types.h"

#include <string.h>
#include "board.h"
#include "qCOMMS.h"


static const char * colorArray[]={COLOR_BLACK,COLOR_RED,COLOR_GREEN,COLOR_YELLOW,COLOR_BLUE,COLOR_MAGENTA,COLOR_MAGENTA,COLOR_CYAN,COLOR_WHITE};
static uint8_t buffer[20];

void intToString(int value, uint8_t* pBuf, uint32_t len, uint8_t base)
{
    static const char* pAscii = "0123456789abcdefghijklmnopqrstuvwxyz";
    int pos = 0;
    int tmpValue = value;

    // the buffer must not be null and at least have a length of 2 to handle one
    // digit and null-terminator
    if (pBuf == 0 || len < 2)
    {
        return;
    }

    // a valid base cannot be less than 2 or larger than 36
    // a base value of 2 means binary representation. A value of 1 would mean only zeros
    // a base larger than 36 can only be used if a larger alphabet were used.
    if (base < 2 || base > 36)
    {
        return;
    }

    // negative value
    if (value < 0)
    {
        tmpValue = -tmpValue;
        value    = -value;
        pBuf[pos++] = '-';
    }

    // calculate the required length of the buffer
    do {
        pos++;
        tmpValue /= base;
    } while(tmpValue > 0);


    if (pos > len)
    {
        // the len parameter is invalid.
        return;
    }

    pBuf[pos] = '\0';

    do {
        pBuf[--pos] = pAscii[value % base];
        value /= base;
    } while(value > 0);

    return;
}


void ConsoleInit(){
	debug_frmwrk_init();
}

void ConsolePuts(char * buff){
	ConsolePuts_(buff,BLACK);
}

uint8_t msgBuffer[300];

void ConsolePuts_(char * buff,int color ){
	uint8_t * msgs[] = {(uint8_t *)(colorArray[color]),(uint8_t *)buff};
	uint32_t pos = 0;
	memcpy(&msgBuffer[pos],msgs[0],strlen((char*)msgs[0]));
	pos += strlen((char*)msgs[0]);
	memcpy(&msgBuffer[pos],msgs[1],strlen((char*)msgs[1]));
	pos += strlen((char*)msgs[1]);

	//qComms_SendMsg(UART_GROUNDCOMM,0xBB,MSG_TYPE_DEBUG,(uint8_t)strlen((char*)msgs[0]),msgs[0]);
	qComms_SendMsg(UART_GROUNDCOMM,0xBB,MSG_TYPE_DEBUG,pos,msgBuffer);

}

void ConsolePutNumber(int value, uint8_t base){
	ConsolePutNumber_(value,base,BLACK);
}

void ConsolePutNumber_(int value, uint8_t base, int color){
	intToString(value, buffer, sizeof(buffer), base);
	ConsolePuts_((char*)buffer,color);
}
