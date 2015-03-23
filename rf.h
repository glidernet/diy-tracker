#include <stdlib.h>

#define SPEEDUP_STM_LIB

void Debug_Output(char Byte) { while(!UART1_TxEmpty()) taskYIELD(); UART1_TxChar(Byte); }

// ================================================================================

#ifdef SPEEDUP_STM_LIB
inline void SPI1_Select  (void) { GPIOA->BRR  = GPIO_Pin_4; }
inline void SPI1_Deselect(void) { GPIOA->BSRR = GPIO_Pin_4; }
#else
inline void SPI1_Select  (void) { GPIO_WriteBit(GPIOA, GPIO_Pin_4, Bit_RESET); }
inline void SPI1_Deselect(void) { GPIO_WriteBit(GPIOA, GPIO_Pin_4, Bit_SET  ); }
#endif

void SPI1_Configuration(void)
{ SPI_InitTypeDef SPI_InitStructure;
  GPIO_InitTypeDef GPIO_InitStructure;

  RCC_APB2PeriphClockCmd (RCC_APB2Periph_SPI1, ENABLE);

  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;                 // SS: output
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
  SPI1_Deselect();

  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5 | GPIO_Pin_7;    // CLK, MOSI: output
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOA, &GPIO_InitStructure);

  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;                // MISO: input
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOA, &GPIO_InitStructure);

  SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
  SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
  SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
  SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;
  SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;
  SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
  SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_8;
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

// ================================================================================

#define RFM69_FIFO       0x00  // Rx and Tx FIFO: no auto-increment when reading or writing
#define RFM69_OPMODE     0x01  // operating mode
#define RFM69_DATAMODUL  0x02  // processing mode, modulation
#define RFM69_BITRATEMSB 0x03  // bit rate MSB (or rather bit period)
#define RFM69_BITRATELSB 0x04  // bit rate LSB [Xtal period] 0x140 => 100kbp (Xtal=32MHz)
#define RFM69_FDEVMSB    0x05  // frequency deviation MSB
#define RFM69_FDEVLSB    0x06  // frequency deviation LSB [Xtal/0x80000 = 61Hz @ Xtal=32MHz]
#define RFM69_FRFMSB     0x07  // carrier frequency MSB 0xD90000 => 868.000MHz
#define RFM69_FRFMID     0x08  // carrier frequency MID
#define RFM69_FRFLSB     0x09  // carrier frequency LSB [Xtal/0x80000 = 61Hz @ Xtal=32MHz]
#define RFM69_OSC1       0x0A  // RC oscillator calibration trigger
#define RFM69_AFCCTRL    0x0B  // AFC method
#define RFM69_LOWBAT     0x0C  // reserved or low-battery
#define RFM69_LISTEN1    0x0D  //
#define RFM69_LISTEN2    0x0E
#define RFM69_LISTEN3    0x0F
#define RFM69_VERSION    0x10  // chip version/revision
#define RFM69_PALEVEL    0x11  // enable power amplifier #0
#define RFM69_PARAMP     0x12  // power amplifier rise/fall time
#define RFM69_OCP        0x13  // current limit = 45 + 5 * reg.value
#define RFM69_AGCREF     0x14  //
#define RFM69_AGCTHRESH1 0x15
#define RFM69_AGCTHRESH2 0x16
#define RFM69_AGCTHRESH3 0x17
#define RFM69_LNA        0x18  // LNA input impedance and gain
#define RFM69_RXBW       0x19  // Rx bandwidth
#define RFM69_AFCBW      0x1A  // Rx AFC bandwidth
#define RFM69_OOKPEAK    0x1B  //
#define RFM69_OOKAVG     0x1C
#define RFM69_OOKFIX     0x1D
#define RFM69_AFCFEI     0x1E
#define RFM69_AFCMSB     0x1F
#define RFM69_AFCLSB     0x20
#define RFM69_FEIMSB     0x21
#define RFM69_FEILSB     0x22
#define RFM69_RSSICONFIG 0x23  // RSSI measurement control
#define RFM69_RSSIVALUE  0x24  // RSSI measured value
#define RFM69_DIOMAPPING1 0x25  // I/O mapping
#define RFM69_DIOMAPPING2 0x26  // and clock mapping
#define RFM69_IRQFLAGS1   0x27  // IRQ flags: mode
#define RFM69_IRQFLAGS2   0x28  // IRQ flags: FIFO, packet, CRC
#define RFM69_RSSITHRESH  0x29  // RSSI trigger level
#define RFM69_RXTIMEOUT1  0x2A  //
#define RFM69_RXTIMEOUT2  0x2B
#define RFM69_PREAMBLEMSB 0x2C  // preamble length MSB
#define RFM69_PREAMBLELSB 0x2D  // preamble length LSB
#define RFM69_SYNCCONFIG  0x2E  // SYNC size/tolerance/...
#define RFM69_SYNCVALUE1  0x2F  // SYNC bytes
#define RFM69_SYNCVALUE2  0x30
#define RFM69_SYNCVALUE3  0x31
#define RFM69_SYNCVALUE4  0x32
#define RFM69_SYNCVALUE5  0x33
#define RFM69_SYNCVALUE6  0x34
#define RFM69_SYNCVALUE7  0x35
#define RFM69_SYNCVALUE8  0x36
#define RFM69_PACKETCONFIG1 0x37  // packet config
#define RFM69_PAYLOADLENGTH 0x38  // packet size [bytes]
#define RFM69_NODEADRS      0x39  // node address
#define RFM69_BROADCASTADRS 0x3A  // broadcast address
#define RFM69_AUTOMODES     0x3B  //
#define RFM69_FIFOTHRESH    0x3C  // FIFO thresholds and TxStartCondition (on first byte or on full packet)
#define RFM69_PACKETCONFIG2 0x3D  //
#define RFM69_AESKEY1     0x3E  // AES key
#define RFM69_AESKEY2     0x3F
#define RFM69_AESKEY3     0x40
#define RFM69_AESKEY4     0x41
#define RFM69_AESKEY5     0x42
#define RFM69_AESKEY6     0x43
#define RFM69_AESKEY7     0x44
#define RFM69_AESKEY8     0x45
#define RFM69_AESKEY9     0x46
#define RFM69_AESKEY10    0x47
#define RFM69_AESKEY11    0x48
#define RFM69_AESKEY12    0x49
#define RFM69_AESKEY13    0x4A
#define RFM69_AESKEY14    0x4B
#define RFM69_AESKEY15    0x4C
#define RFM69_AESKEY16    0x4D
#define RFM69_TEMP1       0x4E  // temperature measurement control
#define RFM69_TEMP2       0x4F  // temperature measured value
#define RFM69_TESTLNA     0x58  // Sensitivity boost ?
#define RFM69_TESTPA1     0x5A  // only present on RFM69HW/SX1231H
#define RFM69_TESTPA2     0x5C  // only present on RFM69HW/SX1231H
#define RFM69_TESTDAGC    0x6F  // Fading margin improvement ?
#define RFM69_TESTAFC     0x71

#define RFM69_OPMODE_SLEEP   0x00
#define RFM69_OPMODE_STDBY   0x04
#define RFM69_OPMODE_SYNTH   0x08
#define RFM69_OPMODE_TX      0x0C
#define RFM69_OPMODE_RX      0x10
#define RFM69_OPMODE_MASK    0x1C

// bits in IrqFlags1 and IrfFlags2
#define RFM69_IRQ_ModeReady     0x8000 // mode change done (between some modes)
#define RFM69_IRQ_RxReady       0x4000
#define RFM69_IRQ_TxReady       0x2000 // 
#define RFM69_IRQ_PllLock       0x1000 // 
#define RFM69_IRQ_Rssi          0x0800
#define RFM69_IRQ_Timeout       0x0400
#define RFM69_IRQ_AutoMode      0x0200
#define RFM69_IRQ_SyncAddrMatch 0x0100

#define RFM69_IRQ_FifoFull      0x0080 // 
#define RFM69_IRQ_FifoNotEmpty  0x0040 // at least one byte in the FIFO
#define RFM69_IRQ_FifoLevel     0x0020 // more bytes than FifoThreshold
#define RFM69_IRQ_FifoOverrun   0x0010 // write this bit to clear the FIFO
#define RFM69_IRQ_PacketSent    0x0008 // packet transmission was completed
#define RFM69_IRQ_PayloadReady  0x0004
#define RFM69_IRQ_CrcOk         0x0002
#define RFM69_IRQ_Unused        0x0001

// ==================================================================

const uint8_t ManchesterEncode[0x10] =  // lookup table for 4-bit nibbles for quick Manchester encoding
{
   0xAA, // hex: 0, bin: 0000, manch: 10101010
   0xA9, // hex: 1, bin: 0001, manch: 10101001
   0xA6, // hex: 2, bin: 0010, manch: 10100110
   0xA5, // hex: 3, bin: 0011, manch: 10100101
   0x9A, // hex: 4, bin: 0100, manch: 10011010
   0x99, // hex: 5, bin: 0101, manch: 10011001
   0x96, // hex: 6, bin: 0110, manch: 10010110
   0x95, // hex: 7, bin: 0111, manch: 10010101
   0x6A, // hex: 8, bin: 1000, manch: 01101010
   0x69, // hex: 9, bin: 1001, manch: 01101001
   0x66, // hex: A, bin: 1010, manch: 01100110
   0x65, // hex: B, bin: 1011, manch: 01100101
   0x5A, // hex: C, bin: 1100, manch: 01011010
   0x59, // hex: D, bin: 1101, manch: 01011001
   0x56, // hex: E, bin: 1110, manch: 01010110
   0x55  // hex: F, bin: 1111, manch: 01010101
};

const uint8_t ManchesterDecode[256] =  // lookup table for quick Manchester decoding
{                                      // lower nibble has the data bits and the upper nibble the error pattern
  0xF0, 0xE1, 0xE0, 0xF1, 0xD2, 0xC3, 0xC2, 0xD3, 0xD0, 0xC1, 0xC0, 0xD1, 0xF2, 0xE3, 0xE2, 0xF3,
  0xB4, 0xA5, 0xA4, 0xB5, 0x96, 0x87, 0x86, 0x97, 0x94, 0x85, 0x84, 0x95, 0xB6, 0xA7, 0xA6, 0xB7,
  0xB0, 0xA1, 0xA0, 0xB1, 0x92, 0x83, 0x82, 0x93, 0x90, 0x81, 0x80, 0x91, 0xB2, 0xA3, 0xA2, 0xB3,
  0xF4, 0xE5, 0xE4, 0xF5, 0xD6, 0xC7, 0xC6, 0xD7, 0xD4, 0xC5, 0xC4, 0xD5, 0xF6, 0xE7, 0xE6, 0xF7,
  0x78, 0x69, 0x68, 0x79, 0x5A, 0x4B, 0x4A, 0x5B, 0x58, 0x49, 0x48, 0x59, 0x7A, 0x6B, 0x6A, 0x7B,
  0x3C, 0x2D, 0x2C, 0x3D, 0x1E, 0x0F, 0x0E, 0x1F, 0x1C, 0x0D, 0x0C, 0x1D, 0x3E, 0x2F, 0x2E, 0x3F,
  0x38, 0x29, 0x28, 0x39, 0x1A, 0x0B, 0x0A, 0x1B, 0x18, 0x09, 0x08, 0x19, 0x3A, 0x2B, 0x2A, 0x3B,
  0x7C, 0x6D, 0x6C, 0x7D, 0x5E, 0x4F, 0x4E, 0x5F, 0x5C, 0x4D, 0x4C, 0x5D, 0x7E, 0x6F, 0x6E, 0x7F,
  0x70, 0x61, 0x60, 0x71, 0x52, 0x43, 0x42, 0x53, 0x50, 0x41, 0x40, 0x51, 0x72, 0x63, 0x62, 0x73,
  0x34, 0x25, 0x24, 0x35, 0x16, 0x07, 0x06, 0x17, 0x14, 0x05, 0x04, 0x15, 0x36, 0x27, 0x26, 0x37,
  0x30, 0x21, 0x20, 0x31, 0x12, 0x03, 0x02, 0x13, 0x10, 0x01, 0x00, 0x11, 0x32, 0x23, 0x22, 0x33,
  0x74, 0x65, 0x64, 0x75, 0x56, 0x47, 0x46, 0x57, 0x54, 0x45, 0x44, 0x55, 0x76, 0x67, 0x66, 0x77,
  0xF8, 0xE9, 0xE8, 0xF9, 0xDA, 0xCB, 0xCA, 0xDB, 0xD8, 0xC9, 0xC8, 0xD9, 0xFA, 0xEB, 0xEA, 0xFB,
  0xBC, 0xAD, 0xAC, 0xBD, 0x9E, 0x8F, 0x8E, 0x9F, 0x9C, 0x8D, 0x8C, 0x9D, 0xBE, 0xAF, 0xAE, 0xBF,
  0xB8, 0xA9, 0xA8, 0xB9, 0x9A, 0x8B, 0x8A, 0x9B, 0x98, 0x89, 0x88, 0x99, 0xBA, 0xAB, 0xAA, 0xBB,
  0xFC, 0xED, 0xEC, 0xFD, 0xDE, 0xCF, 0xCE, 0xDF, 0xDC, 0xCD, 0xCC, 0xDD, 0xFE, 0xEF, 0xEE, 0xFF
} ;

// ==================================================================

#ifdef SPEEDUP_STM_LIB
inline void RFM69_RESET_On  (void) { GPIOB->BSRR = GPIO_Pin_5; }
inline void RFM69_RESET_Off (void) { GPIOB->BRR  = GPIO_Pin_5; }
#else
inline void RFM69_RESET_On  (void) { GPIO_SetBits  (GPIOB, GPIO_Pin_5); }
inline void RFM69_RESET_Off (void) { GPIO_ResetBits(GPIOB, GPIO_Pin_5); }
#endif

#ifdef SPEEDUP_STM_LIB
inline bool RFM69_DIO0_isOn(void)   { return (GPIOB->IDR & GPIO_Pin_4) != 0; }
inline bool RFM69_DIO4_isOn(void)   { return (GPIOB->IDR & GPIO_Pin_3) != 0; }
#else
inline bool RFM69_DIO0_isOn(void)   { return GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_4) != Bit_RESET; }
inline bool RFM69_DIO4_isOn(void)   { return GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_3) != Bit_RESET; }
#endif

void RFM69_GPIO_Configuration(void)
{ GPIO_InitTypeDef  GPIO_InitStructure;

  RCC_APB2PeriphClockCmd (RCC_APB2Periph_GPIOB /* | RCC_APB2Periph_AFIO */, ENABLE);

  GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_3 | GPIO_Pin_4;    // PB4 = DIO0 and PB3 = DIO4 of RFM69
  GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_IN_FLOATING;
  GPIO_Init(GPIOB, &GPIO_InitStructure);

  GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_5;                // PB5 = RESET (active high)
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_Out_PP;
  GPIO_Init(GPIOB, &GPIO_InitStructure);
}

// ==================================================================

uint8_t RFM69_WriteByte(uint8_t Byte, uint8_t Addr=0)    // write Byte
{ SPI1_Select();
  SPI1_TransferByte(Addr | 0x80);
  uint8_t Old=SPI1_TransferByte(Byte);
  SPI1_Deselect();
  return Old; }

uint16_t RFM69_WriteWord(uint16_t Word, uint8_t Addr=0)  // write Word => two bytes
{ SPI1_Select();
  SPI1_TransferByte(Addr | 0x80);
  uint16_t Old=SPI1_TransferByte(Word>>8);             // upper byte first
  Old = (Old<<8) | SPI1_TransferByte(Word&0xFF);       // lower byte second
  SPI1_Deselect();
  return Old; }

void RFM69_WriteBytes(const uint8_t *Data, uint8_t Len, uint8_t Addr=0)
{ SPI1_Select();
  SPI1_TransferByte(Addr | 0x80);
  for(uint8_t Idx=0; Idx<Len; Idx++)
  { SPI1_TransferByte(Data[Idx]); }
  SPI1_Deselect(); }

uint8_t RFM69_ReadByte (uint8_t Addr=0)
{ SPI1_Select();
  SPI1_TransferByte(Addr);
  uint8_t Byte=SPI1_TransferByte(0);
  SPI1_Deselect();
  return Byte; }

uint16_t RFM69_ReadWord (uint8_t Addr=0)
{ SPI1_Select();
  SPI1_TransferByte(Addr);
  uint16_t Word=SPI1_TransferByte(0);
  Word = (Word<<8) | SPI1_TransferByte(0);
  SPI1_Deselect();
  return Word; }

const uint32_t LowFreq = 14224589; // floor(868.2e6/(32e6/(1<<19))+0.5)
const uint32_t UppFreq = 14227866; // floor(868.4e6/(32e6/(1<<19))+0.5)

uint32_t RFM69_WriteFreq(uint32_t Freq=LowFreq)  // [32MHz/2^19] Set center frequency
{ const uint8_t Addr = RFM69_FRFMSB;
  SPI1_Select();
  SPI1_TransferByte(Addr | 0x80);
  uint32_t Old  =  SPI1_TransferByte(Freq>>16);
  Old = (Old<<8) | SPI1_TransferByte(Freq>>8);
  Old = (Old<<8) | SPI1_TransferByte(Freq);
  SPI1_Deselect();
  return Old; }

void RFM69_WritePacket(const uint8_t *Data, uint8_t Len=26)
{ const uint8_t Addr=RFM69_FIFO;
  SPI1_Select();
  SPI1_TransferByte(Addr | 0x80);
  for(uint8_t Idx=0; Idx<Len; Idx++)
  { uint8_t Byte=Data[Idx];
    SPI1_TransferByte(ManchesterEncode[Byte>>4]);
    SPI1_TransferByte(ManchesterEncode[Byte&0x0F]);
  }
  SPI1_Deselect();
}

void RFM69_ReadPacket(uint8_t *Data, uint8_t *Err, uint8_t Len=26)
{ const uint8_t Addr=RFM69_FIFO;
  SPI1_Select();
  SPI1_TransferByte(Addr);
  for(uint8_t Idx=0; Idx<Len; Idx++)
  { uint8_t ByteH = 0;
    ByteH = SPI1_TransferByte(ByteH);
    ByteH = ManchesterDecode[ByteH]; uint8_t ErrH=ByteH>>4; ByteH&=0x0F;
    uint8_t ByteL = 0;
    ByteL = SPI1_TransferByte(ByteL);
    ByteL = ManchesterDecode[ByteL]; uint8_t ErrL=ByteL>>4; ByteL&=0x0F;
    Data[Idx]=(ByteH<<4) | ByteL;
    Err [Idx]=(ErrH <<4) | ErrL ;
  }
  SPI1_Deselect();
}


// OGN SYNC 0x0AF3656C encoded in Manchester
const uint8_t SYNC[8] = { 0xAA, 0x66, 0x55, 0xA5, 0x96, 0x99, 0x96, 0x5A };

void RFM69_WriteSYNC(uint8_t SyncSize=7, uint8_t SyncTol=4)
{ if(SyncTol>7) SyncTol=7;
  if(SyncSize>8) SyncSize=8;
  RFM69_WriteBytes(SYNC+(8-SyncSize), SyncSize, RFM69_SYNCVALUE1);            // write the SYNC, skip some initial bytes
  RFM69_WriteByte(  0x80 | ((SyncSize-1)<<3) | SyncTol, RFM69_SYNCCONFIG);    // write SYNC length [bytes]
  RFM69_WriteWord( 9-SyncSize, RFM69_PREAMBLEMSB); }                          // preamble length [bytes]

inline void    RFM69_WriteMode(uint8_t Mode=RFM69_OPMODE_STDBY) { RFM69_WriteByte(Mode, RFM69_OPMODE); } // SLEEP/STDBY/FSYNTH/TX/RX
inline uint8_t RFM69_ReadMode (void) { return RFM69_ReadByte(RFM69_OPMODE); }
inline uint8_t RFM69_ModeReady(void) { return RFM69_ReadByte(RFM69_IRQFLAGS1)&0x80; }

inline uint16_t RFM69_ReadIrqFlags(void) { return RFM69_ReadWord(RFM69_IRQFLAGS1); }
inline void     RFM69_ClearIrqFlags(void) { RFM69_WriteWord(RFM69_IRQ_FifoOverrun | RFM69_IRQ_Rssi, RFM69_IRQFLAGS1); }

void RFM69_WriteTxPower(int8_t TxPower=10)  // [dBm]
{ if(TxPower<(-18)) TxPower=(-18);
  if(TxPower>  20 ) TxPower=  20 ;
  if(TxPower<=13)
  { RFM69_WriteByte(  0x80+(18+TxPower), RFM69_PALEVEL);
    RFM69_WriteByte(  0x1A             , RFM69_OCP);
    RFM69_WriteByte(  0x55             , RFM69_TESTPA1);
    RFM69_WriteByte(  0x70             , RFM69_TESTPA2);
  } else if(TxPower<=17)
  { RFM69_WriteByte(  0x60+(14+TxPower), RFM69_PALEVEL);
    RFM69_WriteByte(  0x1A             , RFM69_OCP);
    RFM69_WriteByte(  0x55             , RFM69_TESTPA1);
    RFM69_WriteByte(  0x70             , RFM69_TESTPA2);
  } else
  { RFM69_WriteByte(  0x60+(11+TxPower), RFM69_PALEVEL);
    RFM69_WriteByte(  0x0F             , RFM69_OCP);
    RFM69_WriteByte(  0x5D             , RFM69_TESTPA1);
    RFM69_WriteByte(  0x7C             , RFM69_TESTPA2);
  }
}

int RFM69_Configure(void)
{ RFM69_WriteMode(RFM69_OPMODE_STDBY);          // mode = STDBY
  RFM69_ClearIrqFlags();
  RFM69_WriteByte(  0x02, RFM69_DATAMODUL);     // Packet mode, FSK, BT=0.5
  RFM69_WriteWord(0x0140, RFM69_BITRATEMSB);    // bit rate = 100kbps
  RFM69_WriteWord(0x0333, RFM69_FDEVMSB);       // FSK deviation = +/-50kHz
  RFM69_WriteFreq(LowFreq);                     // operating freq: 868.2MHz (units are 32MHz Xtal/2^19 = 61Hz)
  RFM69_WriteSYNC(8, 7);
  RFM69_WriteByte(  0x00, RFM69_PACKETCONFIG1); // Fixed size packet, no DC-free encoding, no CRC, no address filtering
  RFM69_WriteByte(0x80+51, RFM69_FIFOTHRESH);   // TxStartCondition=FifoNotEmpty, FIFO threshold = 51 bytes
  RFM69_WriteByte(  2*26, RFM69_PAYLOADLENGTH); // Packet size = 26 bytes Manchester encoded into 52 bytes
  RFM69_WriteByte(  0x02, RFM69_PACKETCONFIG2); // disable encryption (it is permanent between resets !), AutoRxRestartOn=1
  RFM69_WriteByte(  0x00, RFM69_AUTOMODES);
  RFM69_WriteTxPower(10);                       // [dBm] (for reception: TxPower<=13dBm)
  RFM69_WriteByte(  0x08, RFM69_LNA);           // LNA input 50 or 200ohm ?
  RFM69_WriteByte( 2*114, RFM69_RSSITHRESH);    // RSSI threshold = -114dBm
  RFM69_WriteByte(  0x4A, RFM69_RXBW);          // +/-100kHz Rx bandwidth => p.27+67
  RFM69_WriteByte(  0x89, RFM69_AFCBW);         // +/-200kHz Rx bandwidth while AFC
  RFM69_WriteByte(  0xE4, RFM69_RSSITHRESH);
  RFM69_WriteWord(0x4047, RFM69_DIOMAPPING1);   // DIO signals: DIO0=01, DIO4=01, ClkOut=OFF
                                                // RX: DIO0 = PayloadReady, DIO4 = Rssi
                                                // TX: DIO0 = TxReady,      DIO4 = TxReady
  RFM69_WriteByte(  0x2D, RFM69_TESTLNA);       // enable LNA, sensitivity up by 3dB ?
  RFM69_WriteByte(  0x20, RFM69_TESTDAGC);      // 0x20 when AfcLowBetaOn, 0x30 otherwise-> page 25
  RFM69_WriteByte(  0x20, RFM69_AFCCTRL);       // AfcLowBetaOn=1 -> page 64 -> page 33
  RFM69_WriteByte(   +10, RFM69_TESTAFC);       // [488Hz] if AfcLowBetaOn
  return 0; }

inline uint8_t RFM69_ReadVersion(void) { return RFM69_ReadByte(RFM69_VERSION); }  // normally gives answer: 0x24

inline void    RFM69_TriggerRSSI(void) { RFM69_WriteByte(0x01, RFM69_RSSICONFIG); }        // trigger measurement
inline uint8_t RFM69_ReadyRSSI(void)   { return RFM69_ReadByte(RFM69_RSSICONFIG) & 0x02; } // ready ?
inline uint8_t RFM69_ReadRSSI(void)    { return RFM69_ReadByte(RFM69_RSSIVALUE); }         // read value: RSS = -Value/2

inline void    RFM69_TriggerTemp(void) { RFM69_WriteByte(0x08, RFM69_TEMP1); }        // trigger measurement
inline uint8_t RFM69_RunningTemp(void) { return RFM69_ReadByte(RFM69_TEMP1) & 0x04; } // still running ?
inline uint8_t RFM69_ReadTemp(void)    { return RFM69_ReadByte(RFM69_TEMP2); }        // read value: -1 deg/LSB

// ================================================================================

// const uint32_t TestPacket[7] = { 0x00112233, 0x01234567, 0x89ABCDEF, 0xFEDCBA98, 0x76543210, 0xFEDCBA98, 0x76543210 };

uint8_t RxPktData[26];    // received packet data
uint8_t RxPktErr [26];    // received packet error pattern

LDPC_Decoder Decoder;     // error corrector for the Gallager code

OGN_Packet RxPacket;

int    PPS_Phase=0;       // [ms] Time since GPS PPS
uint8_t TX_FreqChan=0;    // 0 = 868.2MHz, 1 = 868.4MHz
uint8_t TX_Credit=0;

int32_t RxRssiLow=0;      // [-0.5dBm] background noise level on two frequencies
int32_t RxRssiUpp=0;      // [-0.5dBm]

uint32_t RxRandom=0;      // Random number from LSB of RSSI readouts

void PPS_PhaseCalc(void)
{ PPS_Phase=xTaskGetTickCount()-PPS_TickCount; PPS_Phase=PPS_Phase%1000; } // [0...999 ms] current time-phase in respect to PPS

uint8_t Receive(void)                                           // see if a packet has arrived
{ if(!RFM69_DIO0_isOn()) return 0;
                                          // if a new packet has been received
  PPS_PhaseCalc();
  Beep(7500, 128);

  UART1_Write('R'); UART1_Write(TX_FreqChan?'u':'d');
  Format_UnsDec(UART1_Write, (uint16_t)PPS_Phase, 3);

  uint8_t RxRSSI = RFM69_ReadRSSI();                           // signal strength for the received packet
  UART1_Write('r'); Format_Hex(UART1_Write, RxRSSI);
  RxPacket.RxRSSI=RxRSSI;

  int16_t RxFreqOfs = RFM69_ReadWord(RFM69_FEIMSB);            // frequency offset ?
  UART1_Write('f'); Format_Hex(UART1_Write, (uint16_t)RxFreqOfs);

  RFM69_ReadPacket(RxPktData, RxPktErr);                       // get the packet data from the FIFO
  uint8_t Check=LDPC_Check(RxPktData);
  UART1_Write('c'); Format_UnsDec(UART1_Write, (uint16_t)Check, 2);

  uint8_t BitErr=0;
  if(Check==0)
  { RxPacket.recvBytes(RxPktData); }
  else                                                         // if errors detected
  { Decoder.Input(RxPktData, RxPktErr);
    for(uint8_t Iter=32; Iter; Iter--)
    { Check=Decoder.ProcessChecks();
      if(Check==0) break; }
    UART1_Write('d'); Format_UnsDec(UART1_Write, (uint16_t)Check, 2);
    if(Check==0) { Decoder.Output(&RxPacket.Header); /* BitErr=Count1s(); */ }
  }
  UART1_Write('\r'); UART1_Write('\n');

  if(Check==0)
  { char Line[64];
    RxPacket.Dewhiten();
    RxPacket.Print(Line);
    Format_String(UART1_Write, Line);
  }

/*
  for(uint8_t Idx=0; Idx<26; Idx++)
  { UART1_Write(' '); Format_Hex(UART1_Write, RxPktData[Idx]); }
  UART1_Write('\r'); UART1_Write('\n');
  for(uint8_t Idx=0; Idx<26; Idx++)
  { UART1_Write(' '); Format_Hex(UART1_Write, RxPktErr[Idx]); }
  UART1_Write('\r'); UART1_Write('\n');
*/
  // RFM69_WriteMode(RFM69_OPMODE_RX);                            // back to receive (but we already have AutoRxRestart)
  Beep(7500, 0);

  return 1; }

uint8_t Receive(int Ticks)                                        // keep receiving packets for given period of time
{ uint8_t Count=0; int Delta=0;
  TickType_t Start=xTaskGetTickCount();
  do
  { Count+=Receive(); vTaskDelay(1);
    Delta=xTaskGetTickCount()-Start;
  } while(Delta<Ticks);
  return Count; }

uint8_t Transmit(const uint8_t *PacketPtr, uint8_t Thresh, int MaxWait=7)
{
  if(PacketPtr==0) return 0;

  for( ; MaxWait; MaxWait--)
  { RFM69_TriggerRSSI();
    vTaskDelay(1);
    uint8_t RxRSSI=RFM69_ReadRSSI();
    if(RxRSSI>=Thresh) break; }
  if(MaxWait==0) return 0;

  PPS_PhaseCalc();

  // UART1_Write('T'); UART1_Write(TX_FreqChan?'u':'d');
  // Format_UnsDec(UART1_Write, (uint16_t)PPS_Phase, 3);
  // UART1_Write('\r'); UART1_Write('\n');

  // Beep(15000, 128);
  RFM69_WriteMode(RFM69_OPMODE_STDBY);                            // switch to standy
  vTaskDelay(1);

  RFM69_WriteTxPower(Parameters.RFchipTxPower);
  RFM69_WriteSYNC(8, 7);                                          // Full SYNC
  RFM69_ClearIrqFlags();
  RFM69_WritePacket(PacketPtr);                                   // write packet into FIFO
  RFM69_WriteMode(RFM69_OPMODE_TX);                               // transmit
  for(uint8_t MaxWait=10; MaxWait; MaxWait--)                     // wait for transmission to end
  { vTaskDelay(1);
    uint8_t Mode=RFM69_ReadMode();
    uint16_t Flags=RFM69_ReadIrqFlags();
    if(Mode!=RFM69_OPMODE_TX) break;
    if(Flags&RFM69_IRQ_PacketSent) break; }
  RFM69_WriteMode(RFM69_OPMODE_STDBY);                            // switch to standy

  RFM69_WriteTxPower(10);
  RFM69_WriteSYNC(7, 7);                                          // 
  RFM69_WriteMode(RFM69_OPMODE_RX);                               // back to receive
  // Beep(15000, 0);

  return 1; }


#ifdef __cplusplus
  extern "C"
#endif
void vTaskRF(void* pvParameters)
{ uint8_t *PacketPtr = 0; // (uint8_t*)TestPacket;
  Beep(30000, 128);
  RFM69_RESET_On();
  vTaskDelay(10);
  RFM69_RESET_Off();
  vTaskDelay(10);
  uint8_t ChipVersion=RFM69_ReadVersion();
  xSemaphoreTake(UART1_Mutex, portMAX_DELAY);
  Format_String(UART1_Write, "TaskRF: ");
  UART1_Write('v'); Format_Hex(UART1_Write, ChipVersion);
  UART1_Write(' '); Format_SignDec(UART1_Write, (int16_t)Parameters.RFchipTxPower); Format_String(UART1_Write, "dBm");
  UART1_Write(' '); Format_SignDec(UART1_Write, ((int32_t)Parameters.RFchipFreqCorr*78125)>>7, 2, 1); Format_String(UART1_Write, "kHz");
  UART1_Write('\r'); UART1_Write('\n');
  xSemaphoreGive(UART1_Mutex);
  Beep(30000, 0);

  RFM69_Configure();
  RFM69_WriteMode(RFM69_OPMODE_STDBY);

  srand(Parameters.AcftID);

  TX_Credit = 0;
  TX_FreqChan=0; RFM69_WriteFreq(LowFreq+Parameters.RFchipFreqCorr);
  RFM69_WriteSYNC(7, 7); RFM69_WriteMode(RFM69_OPMODE_RX);

  for( ; ; )
  { 

    while(uxQueueMessagesWaiting(xQueuePacket)>1)         // see for new packets to be sent
    { xQueueReceive(xQueuePacket, &PacketPtr, 0); }
    if(uxQueueMessagesWaiting(xQueuePacket)>0)
    { xQueuePeek(xQueuePacket, &PacketPtr, 0); }

    uint32_t RxRssiSum=0; uint16_t RxRssiCount=0;         // measure the average RSSI for lower frequency
    do
    { Receive();
      RFM69_TriggerRSSI();
      vTaskDelay(1);
      uint8_t RxRSSI=RFM69_ReadRSSI();
      RxRandom = (RxRandom<<1) | (RxRSSI&1);
      RxRssiSum+=RxRSSI; RxRssiCount++;
      PPS_PhaseCalc();
    } while(PPS_Phase<300);
    RxRssiLow = RxRssiSum/RxRssiCount; // [-0.5dBm]

    xSemaphoreTake(UART1_Mutex, portMAX_DELAY);
    Format_String(UART1_Write,"RSSIl = "); Format_SignDec(UART1_Write, -(RxRssiLow/2), 3); Format_String(UART1_Write,"dBm\n");
    xSemaphoreGive(UART1_Mutex);

    TX_FreqChan=1; RFM69_WriteFreq(UppFreq+Parameters.RFchipFreqCorr);  // switch to upper frequency
    TX_Credit++;                                                        // new half-slot => increment the transmission credit
    int TxTimeUpp = rand()%400;


    RFM69_WriteMode(RFM69_OPMODE_STDBY);                            // switch to standy
    vTaskDelay(1);

/*                                                          // here we could read the chip temperature
    RFM69_TriggerTemp();
    vTaskDelay(1); // while(RFM69_RunningTemp()) taskYIELD();
    int8_t ChipTemp= 155-RFM69_ReadTemp();
    xSemaphoreTake(UART1_Mutex, portMAX_DELAY);
    Format_SignDec(UART1_Write, (int16_t)ChipTemp); Format_String(UART1_Write,"degC\n");
    xSemaphoreGive(UART1_Mutex);
*/
/*
    xSemaphoreTake(UART1_Mutex, portMAX_DELAY);
    uint16_t MCU_Temp = ADC1_Read(ADC_Channel_TempSensor);
    uint16_t MCU_Vref = ADC1_Read(ADC_Channel_Vrefint);
    Format_String(UART1_Write,"MCU Temp/Vref = ");
    Format_UnsDec(UART1_Write, MCU_Temp, 4);
    UART1_Write('/');
    Format_UnsDec(UART1_Write, MCU_Vref, 4);
    UART1_Write('\r'); UART1_Write('\n');
    xSemaphoreGive(UART1_Mutex);
*/

    RFM69_WriteMode(RFM69_OPMODE_RX);                            // switch to receive mode
    vTaskDelay(1);

    RxRssiSum=0; RxRssiCount=0;                                  // measure the average RSSI for the upper frequency
    do
    { Receive();
      RFM69_TriggerRSSI();
      vTaskDelay(1);
      uint8_t RxRSSI=RFM69_ReadRSSI();
      RxRandom = (RxRandom<<1) | (RxRSSI&1);
      RxRssiSum+=RxRSSI; RxRssiCount++;
      PPS_PhaseCalc();
    } while(PPS_Phase<400);
    RxRssiUpp = RxRssiSum/RxRssiCount; // [-0.5dBm]

    // xSemaphoreTake(UART1_Mutex, portMAX_DELAY);
    // Format_String(UART1_Write,"RSSIu = "); Format_SignDec(UART1_Write, -(RxRssiUpp/2), 3); Format_String(UART1_Write,"dBm\r\n");
    // xSemaphoreGive(UART1_Mutex);

    Receive(TxTimeUpp);
    TX_Credit-=Transmit(PacketPtr, RxRssiUpp, 8);
    Receive(390-TxTimeUpp);

    TX_FreqChan=0; RFM69_WriteFreq(LowFreq+Parameters.RFchipFreqCorr);       // switch to lower frequency
    TX_Credit++;                                                             // new half slot => increment transmission credit
    int TxTimeLow = rand()%400;

    Receive(TxTimeLow);
    TX_Credit-=Transmit(PacketPtr, RxRssiLow, 8);
    Receive(390-TxTimeLow);

  }

/*
    if( (Time<800) || (Time>=810) ) continue;

    Beep(30000, 128);

    RFM69_Configure();
    RFM69_WriteMode(RFM69_OPMODE_STDBY);

//    Debug_Output('s');
//    for(uint8_t Idx=1; Idx<=0x3D; Idx++)
//    { uint8_t Byte=RFM69_ReadByte(Idx);
//      Format_Hex(Debug_Output, Byte); Debug_Output( ((Idx&0x03)==0x01) ?'/':'.'); }

//    vTaskDelay(10);
//    uint8_t ChipVersion=RFM69_ReadVersion();
//    Debug_Output('v'); Format_Hex(Debug_Output, ChipVersion);

//    vTaskDelay(9);
//    RFM69_TriggerTemp();
//    vTaskDelay(1); // while(RFM69_RunningTemp()) taskYIELD();
//    uint8_t ChipTemp=RFM69_ReadTemp();
//    Debug_Output('t'); Format_Hex(Debug_Output, ChipTemp);

    while(uxQueueMessagesWaiting(xQueuePacket)>1)
    { xQueueReceive(xQueuePacket, &PacketPtr, 0); }
    if(uxQueueMessagesWaiting(xQueuePacket)>0)
    { xQueuePeek(xQueuePacket, &PacketPtr, 0); }

    RFM69_WriteMode(RFM69_OPMODE_STDBY);
    RFM69_ClearIrqFlags();

    RFM69_WritePacket(PacketPtr);
    RFM69_WriteMode(RFM69_OPMODE_TX);
    Debug_Output('T');
    for(uint8_t MaxWait=10; MaxWait; MaxWait--)
    { vTaskDelay(1);
      uint8_t Mode=RFM69_ReadMode();
      // Debug_Output('m'); Format_Hex(Debug_Output, Mode);
      uint16_t Flags=RFM69_ReadIrqFlags();
      // Debug_Output('f'); Format_Hex(Debug_Output, Flags);
      if(Mode!=RFM69_OPMODE_TX) break;
      if(Flags&RFM69_IRQ_PacketSent) break; }

    RFM69_WriteMode(RFM69_OPMODE_RX);
    vTaskDelay(10);

    uint8_t Mode=RFM69_ReadMode();
    Debug_Output('m'); Format_Hex(Debug_Output, Mode);

//    RFM69_TriggerRSSI();
//    vTaskDelay(1); // while(!RFM69_ReadyRSSI()) taskYIELD();
//    uint8_t RxRSSI=RFM69_ReadRSSI();
//    Debug_Output('r'); Format_Hex(Debug_Output, RxRSSI);

    Beep(30000, 0);
  }
*/

}

// ======================================================================================
