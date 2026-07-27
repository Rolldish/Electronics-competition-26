#include "Uart.h"
void uart0_send_char(char ch)
{
	while( DL_UART_isBusy(UART_JY61P_INST) == true );
	DL_UART_Main_transmitData(UART_JY61P_INST, ch);

}
void uart0_send_string(char* str)
{
	while(*str!=0&&str!=0)
	{
		uart0_send_char(*str++);
	}
}