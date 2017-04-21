
#include <FreeRTOS.h>
#include <task.h>

#include "stm32f10x.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_usart.h"
#include "misc.h"

#include "fifo.h"

#include "uart1.h"

VolatileFIFO<uint8_t, UART1_RxFIFO_Size> UART1_RxFIFO;
VolatileFIFO<uint8_t, UART1_TxFIFO_Size> UART1_TxFIFO; // large buffer for the console output

// UART1 pins:
// PA8 	USART1_CK
// PA11 USART1_CTS
// PA12 USART1_RTS
// PA9 	USART1_TX
// PA10 USART1_RX

void UART1_Configuration (int BaudRate)
{
  UART_ConfigNVIC(USART1_IRQn, 0, 0);                   // COnfigure and enable the USART1 Interrupt

  RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1 | RCC_APB2Periph_GPIOA, ENABLE);

  UART_ConfigGPIO(GPIOA, GPIO_Pin_10, GPIO_Pin_9);      // Configure USART1 Rx (PA10) as input, and USART1 Tx (PA9) as output
  UART_ConfigUSART(USART1, BaudRate);

  UART1_RxFIFO.Clear(); UART1_TxFIFO.Clear();
  USART_Cmd(USART1, ENABLE);                            // Enable USART1
  USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);        // Enable Rx-not-empty interrupt
  // NVIC_EnableIRQ(USART1_IRQn);
}

#ifdef __cplusplus
  extern "C"
#endif
void USART1_IRQHandler(void)
{ if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)
   while(UART1_RxReady()) { uint8_t Byte=UART1_RxChar(); UART1_RxFIFO.Write(Byte); } // write received bytes to the RxFIFO
  if(USART_GetITStatus(USART1, USART_IT_TXE) != RESET)
   while(UART1_TxEmpty())
  { uint8_t Byte;
    if(UART1_TxFIFO.Read(Byte)<=0) { USART_ITConfig(USART1, USART_IT_TXE, DISABLE); break; }
    UART1_TxChar(Byte); }
  // USART_ClearITPendingBit(USART1,USART_IT_RXNE);
  // if other UART1 interrupt sources ...
  // USART_ClearITPendingBit(USART1, USART_IT_TXE);
}

int UART1_Read(uint8_t &Byte) { return UART1_RxFIFO.Read(Byte); } // return number of bytes read (0 or 1)

void UART1_Write(char Byte)
{ if(UART1_TxFIFO.isEmpty()) { UART1_TxFIFO.Write(Byte); UART1_TxKick(); return; }
  if(UART1_TxFIFO.Write(Byte)>0) return;
  UART1_TxKick();
  while(UART1_TxFIFO.Write(Byte)<=0) taskYIELD();
  return; }


