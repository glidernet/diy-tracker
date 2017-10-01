#include <FreeRTOS.h>
#include <task.h>
#include <semphr.h>
#include <queue.h>

#include "stm32f10x.h"
#include "stm32f10x_flash.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_pwr.h"
#include "stm32f10x_tim.h"
#include "stm32f10x_adc.h"
#include "stm32f10x_exti.h"
#include "stm32f10x_iwdg.h"
#include "misc.h"

#include "hal.h"

#include "uart.h"
#include "uart1.h"
#include "uart2.h"

#include "spi1.h"
#include "adc.h"

#include "systick.h"

#ifdef WITH_BEEPER
  #include "beep.h"
#endif

// ======================================================================================
// to deal with the HC-06 Bluetooth module:
//   http://wiki.mikrokopter.de/en/HC-06
//   http://mcuoneclipse.com/2013/06/19/using-the-hc-06-bluetooth-module/
// ask version:     AT+VERSION
// set name:        AT+NAMEOGN
// set pin:         AT+PIN1234
// set baud 115200: AT+BAUD8
// ======================================================================================

// RF board pin-out: RFM69HW

// RESET  1     16  NC
// DIO0   2     15  NSS
// DIO1   3     14  MOSI
// DIO2   4     13  MISO
// DIO3   5     12  SCK
// DIO4   6     11  GND
// DIO5   7     10  ANT
// 3.3V   8      9  GND


// RF board pin-out: RFM95

//   GND  1     16  DIO2
//  MISO  2     15  DIO1
//  MOSI  3     14  DIO0
//   SCK  4     13  3.3V
//   NSS  5     12  DIO4
// RESET  6     11  DIO3
//  DIO5  7     10  ANT
//   GND  8      9  GND


// ======================================================================================

// Blue Pill pin-out: "no name" STM32F103C8T6, CPU chip facing up

//                  Vbat     3.3V           -> LED, I2C pull-up
// LED <-           PC13     GND            <- Li-Ion battery
//           XTAL - PC14     5.0V           <- Li-Ion battery
//           XTAL - PC15     PB 9 TIM4.CH4  -> Buzzer
// ENA <- TIM2.CH1  PA 0     PB 8 TIM4.CH3  -> Buzzer
// PPS -> TIM2.CH2  PA 1     PB 7 I2C1.SDA <-> Baro/Gyro/...
// GPS <- USART2.Tx PA 2     PB 6 I2C1.SCL <-> Baro/Gyro/...
// GPS -> USART2.Rx PA 3     PB 5           -> RF.RESET
// RF  <- SPI1.SS   PA 4     PB 4           <- RF.DIO0
// RF  <- SPI1.SCK  PA 5     PB 3           <- RF.DIO4
// RF  -> SPI1.MISO PA 6     PA15
// RF  <- SPI1.MOSI PA 7     PA12 TIM1.ETR <-> USB
// POT -> TIM3.CH3  PB 0     PA11 TIM1.CH4 <-> USB
// BAT -> TIM3.CH4  PB 1     PA10 USART1.Rx <- Console/BT
// BT  <- USART3.Tx PB10     PA 9 USART1.Tx -> Console/BT
// BT  -> USART3.Rx PB11     PA 8 TIM1.CH1
//                 RESET     PB15 SPI2.MOSI -> SD card
// RF  <-           3.3V     PB14 SPI2.MISO <- SD card
// RF  <-            GND     PB13 SPI2.SCK  -> SD card
//                   GND     PB12 SPI2.SS   -> SD card

// ---------------------------------------------------------------------------------------

// Maple Mini pin-out: CPU chip facing up

//                     VCC            VCC
//                     GND            GND
//                     Vbat           Vbat
//             LED <-  PC13 14    15  PB 7 I2C1.SDA <-> Gyro/Baro
//             XTAL    PC14 13    16  PB 6 I2C1.SCL <-> Gyro/Baro
//             XTAL    PC15 12    17  PB 5           -> RF.RESET
//                    RESET       18  PB 4           <- RF.DIO0
//    ENA <- TIM2.CH1  PA 0 11    19  PB 3           <- RF.DIO4
//    PPS -> TIM2.CH2  PA 1 10    20  PA15
//    GPS <- USART2.Rx PA 2  9    21  PA14 SWCLK
//    GPS -> USART2.Tx PA 3  8    22  PA13 SWDIO
//    RF  <- SPI1.SS   PA 4  7    23  PA12 TIM1.ETR <-> USB
//    RF  <- SPI1.SCK  PA 5  6    24  PA11 TIM1.CH4 <-> USB
//    RF  -> SPI1.MISO PA 6  5    25  PA10 USART1.Rx <- Console/BT
//    RF  <- SPI1.MOSI PA 7  4    26  PA 9 USART1.Tx -> Console/BT
//    POT -> TIM3.CH3  PB 0  3    27  PA 8 TIM1.CH1
//           Boot1     PB 2  2    28  PB15 SPI2.MOSI -> SD card
//    BT  <- USART3.Tx PB10  1    29  PB14 SPI2.MISO <- SD card
//    BT  -> USART3.Tx PB11  0    30  PB13 SPI2.SCK  -> SD card
//                     Vin        31  PB12 SPI2.SS   -> DC card

// PB 8 = push button => Boot0
// PB 1 = PCB LED
// where to put the buzzer ? PB8/9 are not available

// Good document: https://www.mikrocontroller.net/attachment/275556/test.pdf

// ---------------------------------------------------------------------------------------

// OGN-CUBE-1 board by Miroslav: different GPIO, swapped Console and GSP UART, I2C2 for baro

// CPU: STM32F103CBT6 (same as Maple Mini)

// LED    <-  PA 1

// Console <- PA 2 USART2.Tx
// COnsole -> PA 3 USART2.Rx

// GPS    <-  PA 9 USART1.Tx
// GPS    ->  PA10 USART1.Rx

// Baro <->   PB10 I2C2.SCL
// Baro <->   PB11 I2C2.SDA

// RF.SS    <- PB 0
// RF.RESET <- PB 1
// RF.DIO0  -> PB 2
// RF.SCK   <- PA 5 SPI1.SCK
// RF.MISO  -> PA 6 SPI1.MISO
// RF.MOSI  <- PA 7 SPI1.MOSI

// ======================================================================================

void RCC_Configuration(void)
{
  RCC_DeInit ();                                         // RCC system reset(for debug purpose)

  uint32_t Timeout=80000;
  RCC_HSEConfig (RCC_HSE_ON);                            // Enable HSE (High Speed External clock = Xtal)
  while (RCC_GetFlagStatus(RCC_FLAG_HSERDY) == RESET)    // Wait till HSE is not ready
  { Timeout--; if(Timeout==0) break; }                   // but it may never come up... as some boards have no Xtal !
  if(Timeout==0) RCC_HSEConfig (RCC_HSE_OFF);            // if Timeout went down to zero: Xtal did not come up
/*
  RCC_HSEConfig (RCC_HSE_OFF);
  while (RCC_GetFlagStatus(RCC_FLAG_HSIRDY) == RESET);   // Wait till HSI is not ready
  Timeout=0;
*/
  RCC_HCLKConfig   (RCC_SYSCLK_Div1);                    // HCLK   = SYSCLK  (for AHB bus)
  RCC_PCLK2Config  (RCC_HCLK_Div1);                      // PCLK2  = HCLK    (for APB2 periph.  max. 72MHz)
  RCC_PCLK1Config  (RCC_HCLK_Div2);                      // PCLK1  = HCLK/2  (for APB1 periph.  max. 36MHz)
  RCC_ADCCLKConfig (RCC_PCLK2_Div4);                     // ADCCLK = PCLK2/4 (for ADC:          max. 12MHz)

  // *(vu32 *)0x40022000 = 0x01;                            // Flash 2 wait state
  FLASH_SetLatency(FLASH_Latency_2);                     // Flash 2 wait state
  FLASH_PrefetchBufferCmd(FLASH_PrefetchBuffer_Enable);  // Enable Prefetch Buffer

  if(Timeout)                                            // if HSE came up: use it
    RCC_PLLConfig (RCC_PLLSource_HSE_Div2, RCC_PLLMul_15); // PLLCLK = 4MHz * 15 = 60 MHz
  else                                                   // if HSE did not come up: use internal oscilator
    RCC_PLLConfig (RCC_PLLSource_HSI_Div2, RCC_PLLMul_15); // PLLCLK = 4MHz * 15 = 60 MHz

  RCC_PLLCmd (ENABLE);                                   // Enable PLL
  while (RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET);   // Wait till PLL is ready
  RCC_SYSCLKConfig (RCC_SYSCLKSource_PLLCLK);            // Select PLL as system clock source
  while (RCC_GetSYSCLKSource() != 0x08);                 // Wait till PLL is used as system clock source
}

// -------------------------------------------------------------------------------------------------------

#ifdef WITH_BLUE_PILL
void LED_PCB_On   (void) { GPIO_ResetBits(GPIOC, GPIO_Pin_13); }  // LED is on PC.13 and LOW-active
void LED_PCB_Off  (void) { GPIO_SetBits  (GPIOC, GPIO_Pin_13); }
#endif

#ifdef WITH_MAPLE_MINI
void LED_PCB_On  (void) { GPIO_SetBits  (GPIOB, GPIO_Pin_1); }    // LED is on PB.1 and HIGH-active
void LED_PCB_Off (void) { GPIO_ResetBits(GPIOB, GPIO_Pin_1); }
#endif

#ifdef WITH_OGN_CUBE_1
void LED_PCB_On   (void) { GPIO_ResetBits(GPIOA, GPIO_Pin_1); }   // LED is on PA1 and LOW-active
void LED_PCB_Off  (void) { GPIO_SetBits  (GPIOA, GPIO_Pin_1); }
#endif

static void LED_GPIO_Configuration (void)             // LED on the PCB
{ GPIO_InitTypeDef  GPIO_InitStructure;

#ifdef WITH_BLUE_PILL
  GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_13;        // Configure PC.13 as output (blue LED on the PCB)
  GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_Out_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIO_Init(GPIOC, &GPIO_InitStructure);
#endif

#ifdef WITH_MAPLE_MINI
  GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_1;         // Configure PB.1 as output (blue LED on the PCB)
  GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_Out_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIO_Init(GPIOB, &GPIO_InitStructure);
#endif

#ifdef WITH_OGN_CUBE_1
  GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_1;         // Configure PA.1 as output (blue LED on the PCB)
  GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_Out_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
#endif

  LED_PCB_Off(); }

// -------------------------------------------------------------------------------------------------------

// function to control and read status of the RF chip

#if defined(WITH_BLUE_PILL) || defined(WITH_MAPLE_MINI) // classical DIY-Tracker

// PB5: RF chip RESET: active HIGH for RFM69, active low for RFM95
#ifdef SPEEDUP_STM_LIB
inline void RFM_RESET_High  (void) { GPIOB->BSRR = GPIO_Pin_5; }
inline void RFM_RESET_Low   (void) { GPIOB->BRR  = GPIO_Pin_5; }
#else
inline void RFM_RESET_High  (void) { GPIO_SetBits  (GPIOB, GPIO_Pin_5); }
inline void RFM_RESET_Low   (void) { GPIO_ResetBits(GPIOB, GPIO_Pin_5); }
#endif

void RFM_Select  (void) { SPI1_Select(); }
void RFM_Deselect(void) { SPI1_Deselect(); }

// PB4: RF chip IRQ: active HIGH
#ifdef SPEEDUP_STM_LIB
bool RFM_DIO0_isOn(void)   { return (GPIOB->IDR & GPIO_Pin_4) != 0; }
// bool RFM_DIO4_isOn(void)   { return (GPIOB->IDR & GPIO_Pin_3) != 0; }
#else
bool RFM_DIO0_isOn(void)   { return GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_4) != Bit_RESET; }
// bool RFM_DIO4_isOn(void)   { return GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_3) != Bit_RESET; }
#endif

#endif // BLUE_PILL or MAPLE_MINI

#ifdef WITH_OGN_CUBE_1 // OGN-CUBE-1

// PB1: RF chip RESET: active HIGH for RFM69, active low for RFM95
#ifdef SPEEDUP_STM_LIB
inline void RFM_RESET_High  (void) { GPIOB->BSRR = GPIO_Pin_1; }
inline void RFM_RESET_Low   (void) { GPIOB->BRR  = GPIO_Pin_1; }
#else
inline void RFM_RESET_High  (void) { GPIO_SetBits  (GPIOB, GPIO_Pin_1); }
inline void RFM_RESET_Low   (void) { GPIO_ResetBits(GPIOB, GPIO_Pin_1); }
#endif

// PB0: RF chip SELECT: active LOW
#ifdef SPEEDUP_STM_LIB
void RFM_Select  (void) { GPIOB->BRR  = GPIO_Pin_0; }
void RFM_Deselect(void) { GPIOB->BSRR = GPIO_Pin_0; }
#else
void RFM_Select  (void) { GPIO_WriteBit(GPIOB, GPIO_Pin_0, Bit_RESET); }
void RFM_Deselect(void) { GPIO_WriteBit(GPIOB, GPIO_Pin_0, Bit_SET  ); }
#endif

// PB2: RF chip IRQ: active HIGH
#ifdef SPEEDUP_STM_LIB
bool RFM_DIO0_isOn(void)   { return (GPIOB->IDR & GPIO_Pin_2) != 0; }
#else
bool RFM_DIO0_isOn(void)   { return GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_2) != Bit_RESET; }
#endif

#endif // OGN_CUBE

#ifdef WITH_RFM95                     // RESET is active LOW
void RFM_RESET(uint8_t On)
{ if(On) RFM_RESET_Low();
    else RFM_RESET_High(); }
#endif

#ifdef WITH_RFM69                     // RESET is active HIGH
void RFM_RESET(uint8_t On)
{ if(On) RFM_RESET_High();
    else RFM_RESET_Low(); }
#endif

// RF chip interface for OGN-CUBE-1:
// RF.SS    <- PB 0
// RF.RESET <- PB 1
// RF.DIO0  -> PB 2
// RF.SCK   <- PA 5 SPI1.SCK
// RF.MISO  -> PA 6 SPI1.MISO
// RF.MOSI  <- PA 7 SPI1.MOSI

static void RFM_GPIO_Configuration(void)
{ GPIO_InitTypeDef  GPIO_InitStructure;

#if defined(WITH_BLUE_PILL) || defined(WITH_MAPLE_MINI)
  GPIO_InitStructure.GPIO_Pin   = /* GPIO_Pin_3 | */ GPIO_Pin_4;      // PB4 = DIO0 and PB3 = DIO4 of RFM69
  GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_IN_FLOATING;
  // GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_IPU;
  GPIO_Init(GPIOB, &GPIO_InitStructure);

  GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_5;                         // PB5 = RESET
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_Out_PP;
  GPIO_Init(GPIOB, &GPIO_InitStructure);

#ifdef WITH_RF_IRQ
  NVIC_InitTypeDef NVIC_InitStructure;
  NVIC_InitStructure.NVIC_IRQChannel = EXTI4_IRQn;                  // Enable the external I/O Interrupt
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;         // 0 = highest, 15 = lowest priority
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);

  GPIO_EXTILineConfig(GPIO_PortSourceGPIOB, GPIO_PinSource4);
  EXTI_InitTypeDef EXTI_InitStructure;
  EXTI_InitStructure.EXTI_Line = EXTI_Line4;
  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
  EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  EXTI_Init(&EXTI_InitStructure);

  NVIC_EnableIRQ(EXTI4_IRQn);
#endif

#endif

#ifdef WITH_OGN_CUBE_1
  GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_2;                        // PB2 = DIO0 RFM69/95
  GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_IN_FLOATING;
  // GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_IPU;
  GPIO_Init(GPIOB, &GPIO_InitStructure);

  GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_0 | GPIO_Pin_1;            // PB1 = RESET, PB0 = SS
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_Out_PP;
  GPIO_Init(GPIOB, &GPIO_InitStructure);

#ifdef WITH_RF_IRQ
  NVIC_InitTypeDef NVIC_InitStructure;
  NVIC_InitStructure.NVIC_IRQChannel = EXTI2_IRQn;                  // Enable the external I/O Interrupt
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;         // 0 = highest, 15 = lowest priority
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);

  GPIO_EXTILineConfig(GPIO_PortSourceGPIOB, GPIO_PinSource2);
  EXTI_InitTypeDef EXTI_InitStructure;
  EXTI_InitStructure.EXTI_Line = EXTI_Line2;
  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
  EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  EXTI_Init(&EXTI_InitStructure);

  NVIC_EnableIRQ(EXTI2_IRQn);
#endif

#endif

RFM_RESET(0);
RFM_Deselect();

}

#ifdef WITH_RF_IRQ
void (*RF_IRQ_Callback)(void) = 0;
#endif

#ifdef WITH_RF_IRQ
#ifdef __cplusplus
  extern "C"
#endif
#ifdef WITH_BLUE_PILL
void EXTI4_IRQHandler(void)                                        // RF chip DIO0 interrupt
{
  if(EXTI_GetITStatus(EXTI_Line4) != RESET)
  { if(RF_IRQ_Callback) (*RF_IRQ_Callback)(); }                  // execute the callback
  EXTI_ClearITPendingBit(EXTI_Line4);
}
#endif
#ifdef WITH_OGN_CUBE_1
void EXTI2_IRQHandler(void)                                        // RF chip DIO0 interrupt
{
  if(EXTI_GetITStatus(EXTI_Line2) != RESET)
  { if(RF_IRQ_Callback) (*RF_IRQ_Callback)(); }                  // execute the callback
  EXTI_ClearITPendingBit(EXTI_Line2);
}
#endif
#endif

uint8_t RFM_TransferByte(uint8_t Byte)
{ return SPI1_TransferByte(Byte); }

// -------------------------------------------------------------------------------------------------------

uint16_t ADC_Read_MCU_Vtemp(void) { return ADC1_Read(ADC_Channel_TempSensor); }
uint16_t ADC_Read_MCU_Vref (void) { return ADC1_Read(ADC_Channel_Vrefint); }
uint16_t ADC_Read_Vbatt    (void) { return ADC1_Read(ADC_Channel_9); }
uint16_t ADC_ReadKnob      (void) { return ADC1_Read(ADC_Channel_8); }

// -------------------------------------------------------------------------------------------------------

#ifdef WITH_GPS_ENABLE
// PA0 is GPS enable
void GPS_DISABLE(void) { GPIO_ResetBits(GPIOA, GPIO_Pin_0); }
void GPS_ENABLE (void) { GPIO_SetBits  (GPIOA, GPIO_Pin_0); }
#endif

#ifdef WITH_GPS_PPS
// PA1 is GPS PPS
bool GPS_PPS_isOn(void) { return GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_1) != Bit_RESET; }
#endif

static void GPS_GPIO_Configuration (void)
{ GPIO_InitTypeDef  GPIO_InitStructure;

#ifdef WITH_GPS_ENABLE
  GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_0;        // Configure PA.00 as output: GPS Enable(HIGH) / Shutdown(LOW)
  GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_Out_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
#endif

#ifdef WITH_GPS_PPS
  GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_1;         // Configure PA.01 as input: PPS from GPS
  GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_IPD;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
#endif

#ifdef WITH_PPS_IRQ
  NVIC_InitTypeDef NVIC_InitStructure;
  NVIC_InitStructure.NVIC_IRQChannel = EXTI1_IRQn;                  // Enable the external I/O Interrupt
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;         // 0 = highest, 15 = lowest priority
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);

  GPIO_EXTILineConfig(GPIO_PortSourceGPIOA, GPIO_PinSource1);
  EXTI_InitTypeDef EXTI_InitStructure;
  EXTI_InitStructure.EXTI_Line = EXTI_Line1;
  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
  EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  EXTI_Init(&EXTI_InitStructure);
  NVIC_EnableIRQ(EXTI1_IRQn);
#endif

#ifdef WITH_GPS_ENABLE
  GPS_ENABLE();
#endif
}

#ifdef WITH_PPS_IRQ
void (*GPS_PPS_IRQ_Callback)(uint32_t TickCount, uint32_t TickTime) = 0;
#endif

#ifdef WITH_PPS_IRQ
#ifdef __cplusplus
  extern "C"
#endif
void EXTI1_IRQHandler(void)                                        // PPS interrupt
{ uint32_t TickTime = getSysTick_Count();                          // [CPU tick] what time before the next RTOS tick the interrupt arrived
  uint32_t Load     = getSysTick_Reload();                         // [CPU tick] period of the SysTick - 1
  TickTime          = Load-TickTime;                               // [CPU tick] what time after RTOS tick the PPS arrived
  TickType_t TickCount = xTaskGetTickCountFromISR();               // [RTOS tick] RTOS tick counter

  if(EXTI_GetITStatus(EXTI_Line1) != RESET)
  { if( GPS_PPS_IRQ_Callback && GPS_PPS_isOn() ) (*GPS_PPS_IRQ_Callback)(TickCount, TickTime); }          // execute the callback
  EXTI_ClearITPendingBit(EXTI_Line1);
}
#endif

// -------------------------------------------------------------------------------------------------------

void UART_Configuration(int CONS_BaudRate, int GPS_BaudRate)
{
#ifdef WITH_SWAP_UARTS
  UART2_Configuration(CONS_BaudRate);
  UART1_Configuration(GPS_BaudRate);
#else
  UART1_Configuration(CONS_BaudRate);
  UART2_Configuration(GPS_BaudRate);
#endif
}

#ifdef WITH_SWAP_UARTS
int  CONS_UART_Read  (uint8_t &Byte)  { return UART2_Read (Byte); }
void CONS_UART_Write (char     Byte)  {        UART2_Write(Byte); }
void CONS_UART_SetBaudrate(int BaudRate) { UART2_SetBaudrate(BaudRate); }
int   GPS_UART_Read  (uint8_t &Byte)  { return UART1_Read (Byte); }
void  GPS_UART_Write (char     Byte)  {        UART1_Write(Byte); }
void  GPS_UART_SetBaudrate(int BaudRate) { UART1_SetBaudrate(BaudRate); }
#else
int  CONS_UART_Read  (uint8_t &Byte)  { return UART1_Read (Byte); }
void CONS_UART_Write (char     Byte)  {        UART1_Write(Byte); }
void CONS_UART_SetBaudrate(int BaudRate) { UART1_SetBaudrate(BaudRate); }
int   GPS_UART_Read  (uint8_t &Byte)  { return UART2_Read (Byte); }
void  GPS_UART_Write (char     Byte)  {        UART2_Write(Byte); }
void  GPS_UART_SetBaudrate(int BaudRate) { UART2_SetBaudrate(BaudRate); }
#endif

// -------------------------------------------------------------------------------------------------------

void IO_Configuration(void)
{
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);

  LED_GPIO_Configuration();                              // LED
  GPS_GPIO_Configuration();                              // GPS PPS, Enable, PPS IRQ

  RFM_GPIO_Configuration();                              // RF Reset, IRQ
  SPI1_Configuration();                                  // RF SPI

  ADC_Configuration();                                   // ADC

#ifdef WITH_BEEPER
  Beep_Configuration();
#endif

}

// -------------------------------------------------------------------------------------------------------

void IWDG_Configuration(void)                       // setup watch-dog
{ IWDG_WriteAccessCmd(IWDG_WriteAccess_Enable);
  IWDG_SetPrescaler(IWDG_Prescaler_8);              // about 40kHz/8 = 5kHz counter
  IWDG_SetReload(250); }                            // reload with 250 thus 50ms timeout ?

// -------------------------------------------------------------------------------------------------------
