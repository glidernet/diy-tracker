
#include "spi2.h"

void SPI2_Configuration(void)
{ SPI_InitTypeDef SPI_InitStructure;
  GPIO_InitTypeDef GPIO_InitStructure;

  RCC_APB2PeriphClockCmd (RCC_APB2Periph_GPIOB | RCC_APB2Periph_AFIO, ENABLE);
  RCC_APB1PeriphClockCmd (RCC_APB1Periph_SPI2 ,ENABLE);

  GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_12;                 // SS: output
  GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_Out_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOB, &GPIO_InitStructure);
  SPI2_Deselect();

  GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_13 | GPIO_Pin_15;    // CLK, MOSI: output
  GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOB, &GPIO_InitStructure);

  GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_14;                // MISO: input
  GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_IPU;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOB, &GPIO_InitStructure);

  SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
  SPI_InitStructure.SPI_Mode     = SPI_Mode_Master;
  SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
  SPI_InitStructure.SPI_CPOL     = SPI_CPOL_Low;
  SPI_InitStructure.SPI_CPHA     = SPI_CPHA_1Edge;
  SPI_InitStructure.SPI_NSS      = SPI_NSS_Soft;
  SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_4; // 60MHz/2 = 30MHz SPI bitrate
  SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
  SPI_InitStructure.SPI_CRCPolynomial = 7;
  SPI_Init(SPI2, &SPI_InitStructure);
  // SPI_RxFIFOThresholdConfig(SPI2, SPI_RxFIFOThreshold_QF);
  SPI_CalculateCRC(SPI2, DISABLE);
  SPI_Cmd(SPI2, ENABLE);
}

#ifdef SPEEDUP_STM_LIB
uint8_t SPI2_TransferByte(uint8_t Byte)
{ SPI2->DR = Byte;
  while (!(SPI2->SR & SPI_I2S_FLAG_RXNE));
  return SPI2->DR; }
#else
uint8_t SPI2_TransferByte(uint8_t Byte)
{ SPI_I2S_SendData(SPI2, Byte);
  while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_RXNE) == RESET);
  return SPI_I2S_ReceiveData(SPI2); }
#endif

#ifdef SPEEDUP_STM_LIB
void SPI2_Fast(void)
{ SPI2->CR1 &= 0XFFC7;    // disable (?) the SPI and clear prescaler
  SPI2->CR1 |= SPI_BaudRatePrescaler_2;
  SPI2->CR1 |= 0x0040; }  // re-enable the SPI

void SPI2_Slow(void)
{ SPI2->CR1 &= 0XFFC7;
  SPI2->CR1 |= SPI_BaudRatePrescaler_128;
  SPI2->CR1 |= 0x0040; }
#else

#endif
