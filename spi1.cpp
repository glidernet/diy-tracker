
#include "spi1.h"

void SPI1_Configuration(void)
{ SPI_InitTypeDef SPI_InitStructure;
  GPIO_InitTypeDef GPIO_InitStructure;

  RCC_APB2PeriphClockCmd (RCC_APB2Periph_SPI1, ENABLE);

  GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_4;                 // SS: output
  GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_Out_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
  SPI1_Deselect();

  GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_5 | GPIO_Pin_7;    // CLK, MOSI: output
  GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
  GPIO_Init(GPIOA, &GPIO_InitStructure);

  GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_6;                // MISO: input
  GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_IPU;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
  GPIO_Init(GPIOA, &GPIO_InitStructure);

  SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
  SPI_InitStructure.SPI_Mode     = SPI_Mode_Master;
  SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
  SPI_InitStructure.SPI_CPOL     = SPI_CPOL_Low;
  SPI_InitStructure.SPI_CPHA     = SPI_CPHA_1Edge;
  SPI_InitStructure.SPI_NSS      = SPI_NSS_Soft;
  SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_8; // 60MHz/8 = 7.5MHz SPI bitrate
  SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
  SPI_InitStructure.SPI_CRCPolynomial = 7;
  SPI_Init(SPI1, &SPI_InitStructure);
  // SPI_RxFIFOThresholdConfig(SPI1, SPI_RxFIFOThreshold_QF);
  SPI_CalculateCRC(SPI1, DISABLE);
  SPI_Cmd(SPI1, ENABLE);
}

#ifdef SPEEDUP_STM_LIB
uint8_t SPI1_TransferByte(uint8_t Byte)
{ SPI1->DR = Byte;
  while (!(SPI1->SR & SPI_I2S_FLAG_RXNE));
  return SPI1->DR; }
#else
uint8_t SPI1_TransferByte(uint8_t Byte)
{ SPI_I2S_SendData(SPI1, Byte);
  while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE) == RESET);
  return SPI_I2S_ReceiveData(SPI1); }
#endif


