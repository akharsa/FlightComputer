#include "DebugConsole.h"
#include "debug_frmwrk.h"
#include <string.h>

static const char * colorArray[]={COLOR_BLACK,COLOR_RED,COLOR_GREEN,COLOR_YELLOW,COLOR_BLUE,COLOR_MAGENTA,COLOR_MAGENTA,COLOR_CYAN,COLOR_WHITE};

void ConsoleInit(){
	debug_frmwrk_init();
}

void ConsolePuts(char * buff,int color ){

	uint8_t * msgs[] = {(uint8_t *)(colorArray[color]),(uint8_t *)buff};
	uint8_t sz[] = {strlen((char*)colorArray[color]),strlen((char*)buff)};

	UARTPuts(CONSOLE,msgs[0]);
	UARTPuts(CONSOLE,msgs[1]);

}
