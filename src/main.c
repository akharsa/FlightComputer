#ifdef __USE_CMSIS
#include "LPC17xx.h"
#endif

#include "DebugConsole.h"

int main(void) {
	ConsoleInit();
	ConsolePuts("FLC V2.0 Initialized...\r\n",GREEN);

	return 0;
}


void SysTick_Handler(void)
{

}
