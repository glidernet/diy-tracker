
#include <FreeRTOS.h>
#include <task.h>

#include "stm32f10x.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_usart.h"
#include "misc.h"

#include "fifo.h"

#include "uart1.h"

SemaphoreHandle_t UART1_Mutex;            // Console port Mutex

VolatileFIFO<uint8_t, 32> UART1_RxFIFO;
VolatileFIFO<uint8_t, 64> UART1_TxFIFO;

// UART1 pins:
// PA8 	USART1_CK
// PA11 USART1_CTS
// PA12 USART1_RTS
// PA9 	USART1_TX
// PA10 USART1_RX

void UART1_Configuration (int BaudRate)
{
  NVIC_InitTypeDef NVIC_InitStructure;

  NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;                  // Enable the USART1 Interrupt
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);

  GPIO_InitTypeDef  GPIO_InitStructure;
  USART_InitTypeDef USART_InitStructure;
  USART_ClockInitTypeDef USART_ClockInitStructure;

  RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1 | RCC_APB2Periph_GPIOA, ENABLE);

  GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_10;          // Configure USART1 Rx (PA10) as input floating
  GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_IN_FLOATING;
  GPIO_Init(GPIOA, &GPIO_InitStructure);

  GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_9;           // Configure USART1 Tx (PA9) as alternate function push-pull
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF_PP;
  GPIO_Init(GPIOA, &GPIO_InitStructure);

  USART_InitStructure.USART_BaudRate            = BaudRate; // UART1 at 115200 bps (console/debug/data exchange)
  USART_InitStructure.USART_WordLength          = USART_WordLength_8b;
  USART_InitStructure.USART_StopBits            = USART_StopBits_1;
  USART_InitStructure.USART_Parity              = USART_Parity_No ;
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  USART_InitStructure.USART_Mode                = USART_Mode_Rx | USART_Mode_Tx;
  USART_ClockInitStructure.USART_Clock          = USART_Clock_Disable;
  USART_ClockInitStructure.USART_CPOL           = USART_CPOL_Low;
  USART_ClockInitStructure.USART_CPHA           = USART_CPHA_2Edge;
  USART_ClockInitStructure.USART_LastBit        = USART_LastBit_Disable;
  USART_Init     (USART1, &USART_InitStructure);
  USART_ClockInit(USART1, &USART_ClockInitStructure);   // write parameters
  UART1_RxFIFO.Clear(); UART1_TxFIFO.Clear();
  USART_Cmd(USART1, ENABLE);                            // Enable USART1
  USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);        // Enable Rx-not-empty interrupt
  NVIC_EnableIRQ(USART1_IRQn);

  UART1_Mutex = xSemaphoreCreateMutex();
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


