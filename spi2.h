#ifndef __SPI2_H__
#define __SPI2_H__

#include <stdint.h>

#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_spi.h"

#ifdef SPEEDUP_STM_LIB
inline void SPI2_Select  (void) { GPIOB->BRR  = GPIO_Pin_12; }
inline void SPI2_Deselect(void) { GPIOB->BSRR = GPIO_Pin_12; }
#else
inline void SPI2_Select  (void) { GPIO_WriteBit(GPIOB, GPIO_Pin_12, Bit_RESET); }
inline void SPI2_Deselect(void) { GPIO_WriteBit(GPIOB, GPIO_Pin_12, Bit_SET  ); }
#endif

void SPI2_Configuration(void);

uint8_t SPI2_TransferByte(uint8_t Byte);

void SPI2_Fast(void);
void SPI2_Slow(void);

#endif // __SPI2_H__
