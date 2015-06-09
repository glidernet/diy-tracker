#ifndef __UART1_H__
#define __UART1_H__

#include <stdint.h>

#include "stm32f10x_usart.h"

void UART1_Configuration (int BaudRate=115200);

int  inline UART1_TxDone(void)    { return USART_GetFlagStatus(USART1, USART_FLAG_TC)   != RESET; }
int  inline UART1_TxEmpty(void)   { return USART_GetFlagStatus(USART1, USART_FLAG_TXE)  != RESET; }
int  inline UART1_RxReady(void)   { return USART_GetFlagStatus(USART1, USART_FLAG_RXNE) != RESET; }
// int inline UART1_TxEmpty(void)    { return USART1->SR & USART_FLAG_TXE; }

void inline UART1_TxChar(char ch) { USART_SendData(USART1, ch); }
char inline UART1_RxChar(void)    { return (uint8_t)USART_ReceiveData(USART1); }

int UART1_Read(uint8_t &Byte);
void UART1_Write(char Byte);
void inline UART1_TxKick(void) { USART_ITConfig(USART1, USART_IT_TXE, ENABLE); }

#endif // __UART1_H__
