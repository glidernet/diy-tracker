
#include <FreeRTOS.h>
#include <task.h>

#include "stm32f10x.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_usart.h"
#include "misc.h"

#include "uart2.h"

#include "fifo.h"


VolatileFIFO<uint8_t, 32> UART2_RxFIFO;
VolatileFIFO<uint8_t, 32> UART2_TxFIFO;

// UART2 pins:
// PA4 	USART2_CK
///PA0 	USART2_CTS
// PA1 	USART2_RTS
// PA2 	USART2_TX
// PA3 	USART2_RX

void UART2_Configuration (int BaudRate)
{
  NVIC_InitTypeDef NVIC_InitStructure;

  NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;                  // Enable the USART2 Interrupt
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);

  GPIO_InitTypeDef  GPIO_InitStructure;
  USART_InitTypeDef USART_InitStructure;
  USART_ClockInitTypeDef USART_ClockInitStructure;

  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2,ENABLE);

  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;             // Configure USART2 Tx (PA.2) as alternate function push-pull
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
 
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;               // Configure USART2 Rx (PA.3) as input floating
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_Init(GPIOA, &GPIO_InitStructure);

  USART_InitStructure.USART_BaudRate            = BaudRate; // UART2 at 9600bps (GPS)
  USART_InitStructure.USART_WordLength          = USART_WordLength_8b;
  USART_InitStructure.USART_StopBits            = USART_StopBits_1;
  USART_InitStructure.USART_Parity              = USART_Parity_No;
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  USART_InitStructure.USART_Mode                = USART_Mode_Rx | USART_Mode_Tx;
  USART_ClockInitStructure.USART_Clock          = USART_Clock_Disable;
  USART_ClockInitStructure.USART_CPOL           = USART_CPOL_Low;
  USART_ClockInitStructure.USART_CPHA           = USART_CPHA_2Edge;
  USART_ClockInitStructure.USART_LastBit        = USART_LastBit_Disable;
  USART_Init     (USART2, &USART_InitStructure);
  USART_ClockInit(USART2, &USART_ClockInitStructure);   // write parameters
  UART2_RxFIFO.Clear(); UART2_TxFIFO.Clear();
  USART_Cmd(USART2, ENABLE);                            // Enable USART2
  USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);
  NVIC_EnableIRQ(USART2_IRQn);
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


