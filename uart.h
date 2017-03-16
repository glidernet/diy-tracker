#ifndef __UART_H__
#define __UART_H__

#include "stm32f10x.h"
#include "stm32f10x_usart.h"
#include "stm32f10x_gpio.h"
#include "misc.h"

void UART_ConfigNVIC(uint8_t IRQ, uint8_t Priority, uint8_t SubPriority);
void UART_ConfigGPIO(GPIO_TypeDef* GPIO, uint16_t InpPin, uint16_t OutPin);
void UART_ConfigUSART(USART_TypeDef* USART, int BaudRate);

#endif // __UART_H__
