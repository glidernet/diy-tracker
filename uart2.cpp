
#include <FreeRTOS.h>
#include <task.h>

#include "stm32f10x.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_usart.h"
#include "misc.h"

#include "uart2.h"

#include "fifo.h"

VolatileFIFO<uint8_t, UART2_RxFIFO_Size> UART2_RxFIFO;
VolatileFIFO<uint8_t, UART2_TxFIFO_Size> UART2_TxFIFO;

// UART2 pins:
// PA4 	USART2_CK
///PA0 	USART2_CTS
// PA1 	USART2_RTS
// PA2 	USART2_TX
// PA3 	USART2_RX

void UART2_Configuration (int BaudRate)
{
  UART_ConfigNVIC(USART2_IRQn, 0, 0);                   // COnfigure and enable the USART2 Interrupt

  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2,ENABLE);

  UART_ConfigGPIO(GPIOA, GPIO_Pin_3, GPIO_Pin_2);
  UART_ConfigUSART(USART2, BaudRate);

  UART2_RxFIFO.Clear(); UART2_TxFIFO.Clear();
  USART_Cmd(USART2, ENABLE);                            // Enable USART2
  USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);
  // NVIC_EnableIRQ(USART2_IRQn);
}

#ifdef __cplusplus
  extern "C"
#endif
void USART2_IRQHandler(void)
{ if(USART_GetITStatus(USART2, USART_IT_RXNE) != RESET)
   while(UART2_RxReady()) { uint8_t Byte=UART2_RxChar(); UART2_RxFIFO.Write(Byte); } // write received bytes to the RxFIFO
  if(USART_GetITStatus(USART2, USART_IT_TXE) != RESET)
   while(UART2_TxEmpty())
  { uint8_t Byte;
    if(UART2_TxFIFO.Read(Byte)<=0) { USART_ITConfig(USART2, USART_IT_TXE, DISABLE); break; }
    UART2_TxChar(Byte); }
  // USART_ClearITPendingBit(USART2, USART_IT_TC);
}

int UART2_Read(uint8_t &Byte) { return UART2_RxFIFO.Read(Byte); }

void UART2_Write(char Byte)
{ if(UART2_TxFIFO.isEmpty()) { UART2_TxFIFO.Write(Byte); UART2_TxKick(); return; }
  if(UART2_TxFIFO.Write(Byte)>0) return;
  UART2_TxKick();
  while(UART2_TxFIFO.Write(Byte)<=0) taskYIELD();
  return; }

// Note: UARTx_Write() can only be used after the RTOS is started as they use taskYIELD()


