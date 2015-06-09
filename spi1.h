#ifndef __SPI1_H__
#define __SPI1_H__

#include <stdint.h>

#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_spi.h"

#ifdef SPEEDUP_STM_LIB
inline void SPI1_Select  (void) { GPIOA->BRR  = GPIO_Pin_4; }
inline void SPI1_Deselect(void) { GPIOA->BSRR = GPIO_Pin_4; }
#else
inline void SPI1_Select  (void) { GPIO_WriteBit(GPIOA, GPIO_Pin_4, Bit_RESET); }
inline void SPI1_Deselect(void) { GPIO_WriteBit(GPIOA, GPIO_Pin_4, Bit_SET  ); }
#endif

inline void SPI1_Select(uint8_t Select)
{ if(Select) SPI1_Select();
        else SPI1_Deselect();
}

void SPI1_Configuration(void);

uint8_t SPI1_TransferByte(uint8_t Byte);

#endif // __SPI1_H__
