#ifndef __UART2_H__
#define __UART2_H__

#include <stdint.h>

#include "stm32f10x_usart.h"

#include "uart.h"

void UART2_Configuration (int BaudRate=9600);
inline void UART2_SetBaudrate(int BaudRate=9600) { UART_ConfigUSART(USART2, BaudRate); }

int  inline UART2_TxDone(void)    { return USART_GetFlagStatus(USART2, USART_FLAG_TC)   != RESET; }
int  inline UART2_TxEmpty(void)   { return USART_GetFlagStatus(USART2, USART_FLAG_TXE)  != RESET; }
int  inline UART2_RxReady(void)   { return USART_GetFlagStatus(USART2, USART_FLAG_RXNE) != RESET; }

void inline UART2_TxChar(char ch) { USART_SendData(USART2, ch); }
char inline UART2_RxChar(void)    { return (uint8_t)USART_ReceiveData(USART2); }

inline void UART2_TxKick(void) { USART_ITConfig(USART2, USART_IT_TXE, ENABLE); }

int  UART2_Read(uint8_t &Byte);
void UART2_Write(char Byte);

#endif // __UART2_H__
