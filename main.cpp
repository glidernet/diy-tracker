#include <stdint.h>
#include <stdlib.h>

#include "stm32f10x.h"
#include "stm32f10x_flash.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_usart.h"
#include "stm32f10x_spi.h"
#include "stm32f10x_rtc.h"
#include "stm32f10x_pwr.h"
#include "stm32f10x_tim.h"
#include "stm32f10x_adc.h"
#include "stm32f10x_exti.h"
#include "misc.h"

#include "format.h"

#include "fifo.h"
#include "ogn.h"
#include "ubx.h"

#include <FreeRTOS.h>
#include <task.h>
#include <semphr.h>
#include <queue.h>

// ======================================================================================

#include "adc.h"
#include "beep.h"

// ======================================================================================

uint32_t * const   FlashStart = (uint32_t *)0x08000000;   // where the Flash memory starts
uint16_t          *FlashSize  = (uint16_t *)0x1FFFF7E0;   // [kB] Flash memory size
uint16_t inline getFlashSize(void) { return *FlashSize; }

uint32_t          *UniqueID = (uint32_t*)(0x1FFFF7E8);
uint32_t inline getUniqueID(uint8_t Idx) { return UniqueID[Idx]; }

// ======================================================================================

// Parameters stored in Flash
class FlashParameters
{ public:
   uint32_t  AcftID;         // identification: Private:AcftType:AddrType:Address - must be different for every tracker
    int16_t  RFchipFreqCorr; // [61Hz] frequency correction for crystal frequency offset
    int8_t   RFchipTxPower;  // [dBm]
    int8_t   RFchipTempCorr; // [degC]
   uint32_t  CONbaud;        // [bps] Console baud rate
   uint32_t  GPSbaud;        // [bps] GPS baud rate

   // static const uint32_t Words=sizeof(FlashParameters)/sizeof(uint32_t);

   uint32_t getAddress(void) const { return AcftID&0x00FFFFFF; }
   uint8_t getAddrType(void) const { return (AcftID>>24)&0x03; }
   uint8_t getAcftType(void) const { return (AcftID>>26)&0x0F; }
   uint8_t getNoTrack (void) const { return (AcftID>>30)&0x01; }
   uint8_t getStealth (void) const { return (AcftID>>31)&0x01; }

   void setAddress (uint32_t Address)  { AcftID = (AcftID&0xFF000000) | (Address&0x00FFFFFF); }
   void setAddrType(uint8_t  AddrType) { AcftID = (AcftID&0xFCFFFFFF) | ((uint32_t)(AddrType&0x03)<<24); }
   void setAcftType(uint8_t  AcftType) { AcftID = (AcftID&0xC3FFFFFF) | ((uint32_t)(AcftType&0x0F)<<26); }
   void setNoTrack(void) { AcftID |= 0x40000000; }
   void clrNoTrack(void) { AcftID &= 0xBFFFFFFF; }
   void setStealth(void) { AcftID |= 0x80000000; }
   void clrStealth(void) { AcftID &= 0x7FFFFFFF; }

 public:
  void setDefault(void)
  { AcftID = UniqueID[0] ^ UniqueID[1] ^ UniqueID[2]; 
    AcftID = 0x07000000 | (AcftID&0x00FFFFFF);
    RFchipFreqCorr =   0;
    RFchipTxPower  = +14; // +13dBm for RFM69W and +14dBm for RFM69HW - at this time we cannot recognize which RF chip is being used
    RFchipTempCorr =   0;
    CONbaud  = 115200;
    GPSbaud  =   9600;
  }

  uint32_t static CheckSum(const uint32_t *Word, uint32_t Words)
  { uint32_t Check=0x12345678;
    for(uint32_t Idx=0; Idx<Words; Words++)
    { Check+=Word[Idx]; }
    return Check; }

  uint32_t CheckSum(void) const
  { return CheckSum((uint32_t *)this, sizeof(FlashParameters)/sizeof(uint32_t) ); }

  static uint32_t *DefaultFlashAddr(void) { return FlashStart+((uint32_t)(getFlashSize()-1)<<8); }

  int8_t ReadFromFlash(uint32_t *Addr=0)
  { if(Addr==0) Addr = DefaultFlashAddr();
    const uint32_t Words=sizeof(FlashParameters)/sizeof(uint32_t);
    uint32_t Check=CheckSum(Addr, Words);
    if(Check!=Addr[Words]) return -1;
    uint32_t *Dst = (uint32_t *)this;
    for(uint32_t Idx=0; Idx<Words; Idx++)
    { Dst[Idx] = Addr[Idx]; }
    return 1; }

  int8_t WriteToFlash(uint32_t *Addr=0) const
  { if(Addr==0) Addr = DefaultFlashAddr();
    const uint32_t Words=sizeof(FlashParameters)/sizeof(uint32_t);
    FLASH_Unlock();
    FLASH_ErasePage((uint32_t)Addr);
    uint32_t *Data=(uint32_t *)this;
    for(uint32_t Idx=0; Idx<Words; Idx++)
    { FLASH_ProgramWord((uint32_t)Addr, Data[Idx]); Addr++; }
    FLASH_ProgramWord((uint32_t)Addr, CheckSum(Data, Words) );
    FLASH_Lock();
    if(CheckSum(Addr, Words)!=Addr[Words]) return -1;
    return 0; }

} ;

FlashParameters Parameters;

// ======================================================================================

// Board pin-out: "no name" STM32F103R8T6, CPU chip facing up

//                  Vbat     3.3V
//           LED <- PC13     GND
//           XTAL - PC14     5.0V
//           XTAL - PC15     PB 9 TIM4.CH4
// ENA <- TIM2.CH1  PA 0     PB 8 TIM4.CH3  -> Buzzer
// PPS -> TIM2.CH2  PA 1     PB 7 I2C1.SDA <-> Baro/Gyro
// GPS <- USART2.Tx PA 2     PB 6 I2C1.SCL <-> Baro/Gyro
// GPS -> USART2.Rx PA 3     PB 5           -> RF.RESET
// RF  <- SPI1.SS   PA 4     PB 4           <- RF.DIO0
// RF  <- SPI1.SCK  PA 5     PB 3           <- RF.DIO4
// RF  -> SPI1.MISO PA 6     PA15
// RF  <- SPI1.MOSI PA 7     PA12 TIM1.ETR
//        TIM3.CH3  PB 0     PA11 TIM1.CH4
//        TIM3.CH4  PB 1     PA10 USART1.Rx <- Console
// BT  <- USART3.Tx PB10     PA 9 USART1.Tx -> Console
// BT  -> USART3.Rx PB11     PA 8 TIM1.CH1
//                 RESET     PB15 SPI2.MOSI
//                  3.3V     PB14 SPI2.MISO
//                   GND     PB13 SPI2.SCK
//                   GND     PB12 SPI2.SS


// Board pin-out: Maple Mini: CPU chip facing up

//                     VCC            VCC
//                     GND            GND
//                     Vbat           Vbat
//             LED <-  PC13 14    15  PB 7 I2C1.SDA <-> Gyro/Baro
//             XTAL    PC14 13    16  PB 6 I2C1.SCL <-> Gyro/Baro
//             XTAL    PC15 12    17  PB 5           -> RF.RESET
//                    RESET       18  PB 4           <- RF.DIO0
//           TIM2.CH1  PA 0 11    19  PB 3           <- RF.DIO4
//           TIM2.CH2  PA 1 10    20  PA15
//           USART2.Rx PA 2  9    21  PA14 SWCLK
//           USART2.Tx PA 3  8    22  PA13 SWDIO
//    RF  <- SPI1.SS   PA 4  7    23  PA12 TIM1.ETR
//    RF  <- SPI1.SCK  PA 5  6    24  PA11 TIM1.CH4
//    RF  -> SPI1.MISO PA 6  5    25  PA10 USART.Rx <- Console
//    RF  <- SPI1.MOSI PA 7  4    26  PA 9 USART.Tx -> Console
//           TIM3.CH3  PB 0  3    27  PA 8 TIM1.CH1
//           Boot1     PB 2  2    28  PB15 SPI2.MOSI
//    BT  <- USART3.Tx PB10  1    29  PB14 SPI2.MISO
//    BT  -> USART3.Tx PB11  0    30  PB13 SPI2.SCK
//                     Vin        31  PB12 SPI2.SS

// PB 8 = push button => Boot0
// PB 1 = PCB LED

// ======================================================================================

void RCC_Configuration(void)
{
  RCC_DeInit ();                        // RCC system reset(for debug purpose)
  RCC_HSEConfig (RCC_HSE_ON);           // Enable HSE (High Speed External clock = Xtal)

  while (RCC_GetFlagStatus(RCC_FLAG_HSERDY) == RESET);   // Wait till HSE is ready

  RCC_HCLKConfig   (RCC_SYSCLK_Div1);                    // HCLK   = SYSCLK  (for AHB bus)
  RCC_PCLK2Config  (RCC_HCLK_Div1);                      // PCLK2  = HCLK    (for APB2 periph.  max. 72MHz)
  RCC_PCLK1Config  (RCC_HCLK_Div2);                      // PCLK1  = HCLK/2  (for APB1 periph.  max. 36MHz)
  RCC_ADCCLKConfig (RCC_PCLK2_Div4);                     // ADCCLK = PCLK2/4 (for ADC:          max. 12MHz)

  // *(vu32 *)0x40022000 = 0x01;                            // Flash 2 wait state
  FLASH_SetLatency(FLASH_Latency_2);                     // Flash 2 wait state
  FLASH_PrefetchBufferCmd(FLASH_PrefetchBuffer_Enable);  // Enable Prefetch Buffer

  RCC_PLLConfig (RCC_PLLSource_HSE_Div2, RCC_PLLMul_15); // PLLCLK = 4MHz * 15 = 60 MHz
  RCC_PLLCmd (ENABLE);                                   // Enable PLL
  while (RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET);   // Wait till PLL is ready
  RCC_SYSCLKConfig (RCC_SYSCLKSource_PLLCLK);            // Select PLL as system clock source
  while (RCC_GetSYSCLKSource() != 0x08);                 // Wait till PLL is used as system clock source

  // Enable USART1 and GPIOA clock
  // RCC_APB2PeriphClockCmd (RCC_APB2Periph_USART1 | RCC_APB1Periph_USART2 | RCC_APB2Periph_GPIOA |
  // RCC_APB2Periph_GPIOB | RCC_APB2Periph_GPIOC | RCC_APB2Periph_AFIO, ENABLE);
  // RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
}

// ======================================================================================

inline void LED_PCB_On   (void) { GPIO_ResetBits(GPIOC, GPIO_Pin_13); }
inline void LED_PCB_Off  (void) { GPIO_SetBits  (GPIOC, GPIO_Pin_13); }

void LED_Configuration (void)
{
  GPIO_InitTypeDef  GPIO_InitStructure;

  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);

  GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_13;        // Configure PC.13 as output (blue LED on the PCB)
  GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_Out_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIO_Init(GPIOC, &GPIO_InitStructure);
  LED_PCB_Off();
}

volatile uint8_t LED_PCB_Counter = 0;

inline void LED_PCB_Flash(uint8_t Time=100) { LED_PCB_Counter=Time; } // [ms]

// ======================================================================================
// source:
// https://my.st.com/public/STe2ecommunities/mcu/Lists/cortex_mx_stm32/DispForm.aspx?ID=27834&Source=/public/STe2ecommunities/mcu/Tags.aspx?tags=stm32%20usart%20interrupt
// https://my.st.com/public/STe2ecommunities/mcu/Lists/cortex_mx_stm32/DispForm.aspx?ID=24064&Source=/public/STe2ecommunities/mcu/Tags.aspx?tags=stm32%20usart%20interrupt
// http://electronics.stackexchange.com/questions/100073/stm32-usart-rx-interrupts
// https://my.st.com/public/STe2ecommunities/mcu/Lists/STM32Discovery/Flat.aspx?RootFolder=%2Fpublic%2FSTe2ecommunities%2Fmcu%2FLists%2FSTM32Discovery%2FUART%20example%20code%20for%20STM32F0&FolderCTID=0x01200200770978C69A1141439FE559EB459D75800084C20D8867EAD444A5987D47BE638E0F&currentviews=5401

// UART pins:
// Pin 	Function
//
// PA8 	USART1_CK
// PA11 USART1_CTS
// PA12 USART1_RTS
// PA9 	USART1_TX
// PA10 USART1_RX
//
// PA4 	USART2_CK
///PA0 	USART2_CTS
// PA1 	USART2_RTS
// PA2 	USART2_TX
// PA3 	USART2_RX
//
// PB12 USART3_CK
// PB13 USART3_CTS
// PB14 USART3_RTS
// PB10 USART3_TX
// PB11 USART3_RX

// ------------------------------------------------------------------------------------------

VolatileFIFO<uint8_t, 32> UART1_RxFIFO;
VolatileFIFO<uint8_t, 32> UART1_TxFIFO;

// UART1 pins:
// PA8 	USART1_CK
// PA11 USART1_CTS
// PA12 USART1_RTS
// PA9 	USART1_TX
// PA10 USART1_RX

void UART1_Configuration (int BaudRate=115200)
{
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
}

int  inline UART1_TxDone(void)    { return USART_GetFlagStatus(USART1, USART_FLAG_TC)   != RESET; }
int  inline UART1_TxEmpty(void)   { return USART_GetFlagStatus(USART1, USART_FLAG_TXE)  != RESET; }
int  inline UART1_RxReady(void)   { return USART_GetFlagStatus(USART1, USART_FLAG_RXNE) != RESET; }

void inline UART1_TxChar(char ch) { USART_SendData(USART1, ch); }
char inline UART1_RxChar(void)    { return (uint8_t)USART_ReceiveData(USART1); }

// int inline UART1_TxEmpty(void)    { return USART1->SR & USART_FLAG_TXE; }

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

inline int UART1_Read(uint8_t &Byte) { return UART1_RxFIFO.Read(Byte); } // return number of bytes read (0 or 1)

inline void UART1_TxKick(void) { USART_ITConfig(USART1, USART_IT_TXE, ENABLE); }

void UART1_Write(char Byte)
{ if(UART1_TxFIFO.isEmpty()) { UART1_TxFIFO.Write(Byte); UART1_TxKick(); return; }
  if(UART1_TxFIFO.Write(Byte)>0) return;
  UART1_TxKick();
  while(UART1_TxFIFO.Write(Byte)<=0) taskYIELD();
  return; }

// ------------------------------------------------------------------------------------------

VolatileFIFO<uint8_t, 32> UART2_RxFIFO;
VolatileFIFO<uint8_t, 32> UART2_TxFIFO;

// UART2 pins:
// PA4 	USART2_CK
///PA0 	USART2_CTS
// PA1 	USART2_RTS
// PA2 	USART2_TX
// PA3 	USART2_RX

void UART2_Configuration (int BaudRate=9600)
{
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

int  inline UART2_TxDone(void)    { return USART_GetFlagStatus(USART2, USART_FLAG_TC)   != RESET; }
int  inline UART2_TxEmpty(void)   { return USART_GetFlagStatus(USART2, USART_FLAG_TXE)  != RESET; }
int  inline UART2_RxReady(void)   { return USART_GetFlagStatus(USART2, USART_FLAG_RXNE) != RESET; }

void inline UART2_TxChar(char ch) { USART_SendData(USART2, ch); }
char inline UART2_RxChar(void)    { return (uint8_t)USART_ReceiveData(USART2); }

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

inline int UART2_Read(uint8_t &Byte) { return UART2_RxFIFO.Read(Byte); }

inline void UART2_TxKick(void) { USART_ITConfig(USART2, USART_IT_TXE, ENABLE); }

void UART2_Write(char Byte)
{ if(UART2_TxFIFO.isEmpty()) { UART2_TxFIFO.Write(Byte); UART2_TxKick(); return; }
  if(UART2_TxFIFO.Write(Byte)>0) return;
  UART2_TxKick();
  while(UART2_TxFIFO.Write(Byte)<=0) taskYIELD();
  return; }

// Note: UARTx_Write() can only be used after the RTOS is started as they use taskYIELD()

// ======================================================================================

// ======================================================================================

// ======================================================================================

#include "rtc.h"

// ======================================================================================

void NVIC_Configuration (void)
{ 
  NVIC_InitTypeDef NVIC_InitStructure;

  NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;                  // Enable the USART1 Interrupt
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);

  NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;                  // Enable the USART2 Interrupt
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);

  NVIC_InitStructure.NVIC_IRQChannel = RTC_IRQn;                     // Enable the RTC Interrupt
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
}

// ======================================================================================

SemaphoreHandle_t UART1_Mutex = 0;

#include "gps.h" // GPS task
#include "rf.h"  // RF chip and RF task

// ======================================================================================

#ifdef __cplusplus
  extern "C"
#endif
void vTaskCTRL(void* pvParameters)
{ 
  UART1_Mutex = xSemaphoreCreateMutex();
  vTaskDelay(5);
  xSemaphoreTake(UART1_Mutex, portMAX_DELAY);
  Format_String(UART1_Write, "TaskCTRL: MCU ID: ");
  Format_Hex(UART1_Write, UniqueID[0]); UART1_Write(' ');
  Format_Hex(UART1_Write, UniqueID[1]); UART1_Write(' ');
  Format_Hex(UART1_Write, UniqueID[2]); UART1_Write(' ');
  Format_UnsDec(UART1_Write, getFlashSize()); Format_String(UART1_Write, "kB\n");
  xSemaphoreGive(UART1_Mutex);
  while(1)
  { vTaskDelay(1);
  }
}

// ======================================================================================

/*
non-trivial part to make the FreeRTOS run - possibly can be done as well by:

#define vPortSVCHandler SVC_Handler
#define xPortPendSVHandler PendSV_Handler
#define xPortSysTickHandler SysTick_Handler

The above #define are now done int FreeRTOSConfig.h

#ifdef __cplusplus
  extern "C"
  {
#endif
void vPortSVCHandler();
void xPortPendSVHandler();
void xPortSysTickHandler();

void SVC_Handler    (void) { vPortSVCHandler();     }
void PendSV_Handler (void) { xPortPendSVHandler();  }
void SysTick_Handler(void) { xPortSysTickHandler(); }
#ifdef __cplusplus
  }
#endif
*/
// ======================================================================================

#ifdef __cplusplus
  extern "C"
#endif
void prvSetupHardware(void)
{ RCC_Configuration();

  NVIC_Configuration();

  LED_Configuration();

  if(Parameters.ReadFromFlash()<0)
  { Parameters.setDefault();
    Parameters.WriteToFlash(); }

  UART1_Configuration(Parameters.CONbaud);
  UART2_Configuration(Parameters.GPSbaud);
  GPS_Configuration();

  Beep_Configuration();
  // RTC_Configuration();

  SPI1_Configuration();
  RFM69_GPIO_Configuration();

  ADC_Configuration();

  // to overwrite parameters
  // Parameters.RFchipTxPower = +14; // for RFM69HW (H = up to +20dBm Tx power)
  // Parameters.WriteToFlash();
}

#ifdef __cplusplus
  extern "C"
#endif
void vApplicationIdleHook(void) // when RTOS is idle: should call "sleep until an interrupt"
{ __WFI(); }

#ifdef __cplusplus
  extern "C"
#endif
void vApplicationTickHook(void) // RTOS timer tick hook
{ uint8_t Counter=LED_PCB_Counter;
  if(Counter)
  { Counter--;
    if(Counter) LED_PCB_On();
           else LED_PCB_Off();
    LED_PCB_Counter=Counter; }
}

int main(void)
{ 
  prvSetupHardware();

  UART1_TxFIFO.Write('\r');
  UART1_TxFIFO.Write('\n');
  UART1_TxKick();

/*
  LED_PCB_On();

  UART1_TxFIFO.Write((const uint8_t *)"\r\nMini-Tracker\r\n", 16);
  UART1_TxKick();
  while(!UART1_TxEmpty());

  LED_PCB_Off();
*/

  xTaskCreate(vTaskCTRL,  "CTRL",  120, 0, tskIDLE_PRIORITY+2, 0);
  xTaskCreate(vTaskGPS,   "GPS",   120, 0, tskIDLE_PRIORITY+2, 0);
  xTaskCreate(vTaskRF,    "RF",    120, 0, tskIDLE_PRIORITY+2, 0);

  vTaskStartScheduler();

  while(1)
  { }

}

// lot of things to do:
// . read NMEA user input
// . set Parameters in Flash from NMEA
// . send received positions to console
// . packet retransmission: strategy
// . send received positions to console as NMEA
// . play melodies on events
// . separate the UART code
// . optimize receiver sensitivity
// . AFC ?
// . continues AGC/RSSI ?
// . periodically refresh the RF chip config
// . print task information for check up
// . try to run on Maple Mini (there is more Flash, but likely no xtal)
// . simple time/position log in Flash
// . auto-detect RFM69W or RFM69HW
// . measure the CPU temperature
// . read RF chip temperature and compensate Rx/Tx frequency
// . measure VCC voltage: low battery indicator
// . detect when VK16u6 GPS fails below 2.7V supply
// . GPS try higher baud rates
// . GPS auto-baud
// . detect GPS stop (below 2.7V) and then sleep

