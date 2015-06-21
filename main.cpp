#include <stdint.h>
#include <stdlib.h>

#include "stm32f10x.h"
#include "stm32f10x_flash.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_pwr.h"
#include "stm32f10x_tim.h"
#include "stm32f10x_adc.h"
// #include "stm32f10x_exti.h"
#include "stm32f10x_iwdg.h"
#include "misc.h"

#include "format.h"

#include <FreeRTOS.h>
#include <task.h>
#include <semphr.h>
#include <queue.h>


#include "fifo.h"

#include "uart1.h"
#include "uart2.h"
#include "adc.h"

#ifdef WITH_BEEPER
  #include "beep.h"
#endif

#ifdef WITH_I2C1
  #include "i2c.h"
#endif

#include "flashsize.h"
#include "uniqueid.h"

#include "parameters.h"              // Parameters in Flash

#include "gps.h"                     // GPS task
#include "rf.h"                      // RF chip and RF task
#include "ctrl.h"                    // CTRL task
#include "sens.h"                    // SENS task

FlashParameters Parameters; // parameters to be stored in Flash, on the last page

// extern "C" void __cxa_pure_virtual (void) {} // to reduce the code size ?

// ======================================================================================
// to deal with the HC-06 Bluetooth module:
//   http://wiki.mikrokopter.de/en/HC-06
//   http://mcuoneclipse.com/2013/06/19/using-the-hc-06-bluetooth-module/
// ask version:     AT+VERSION
// set name:        AT+NAMEOGN
// set pin:         AT+PIN1234
// set baud 115200: AT+BAUD8
// ======================================================================================

// #include "adc.h"

// ======================================================================================

// ======================================================================================

// ======================================================================================

// Board pin-out: "no name" STM32F103C8T6, CPU chip facing up

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
// RF  <- SPI1.MOSI PA 7     PA12 TIM1.ETR
// POT -> TIM3.CH3  PB 0     PA11 TIM1.CH4
//        TIM3.CH4  PB 1     PA10 USART1.Rx <- Console/BT
//     <- USART3.Tx PB10     PA 9 USART1.Tx -> Console/BT
//     -> USART3.Rx PB11     PA 8 TIM1.CH1
//                 RESET     PB15 SPI2.MOSI -> SD card
// RF  <-           3.3V     PB14 SPI2.MISO <- SD card
// RF  <-            GND     PB13 SPI2.SCK  -> SD card
//                   GND     PB12 SPI2.SS   -> SD card


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

// #define USE_XTAL

void RCC_Configuration(void)
{
  RCC_DeInit ();                                         // RCC system reset(for debug purpose)
  RCC_HSEConfig (RCC_HSE_ON);                            // Enable HSE (High Speed External clock = Xtal)
  uint32_t Timeout=80000;
  while (RCC_GetFlagStatus(RCC_FLAG_HSERDY) == RESET)    // Wait till HSE is not ready
  { Timeout--; if(Timeout==0) break; }                   // but it may never come up... as some boards have no Xtal !
  if(Timeout==0) RCC_HSEConfig (RCC_HSE_OFF);            // if Timeout went down to zero: Xtal did not come up

  // while (RCC_GetFlagStatus(RCC_FLAG_HSIRDY) == RESET);   // Wait till HSI is not ready

  RCC_HCLKConfig   (RCC_SYSCLK_Div1);                    // HCLK   = SYSCLK  (for AHB bus)
  RCC_PCLK2Config  (RCC_HCLK_Div1);                      // PCLK2  = HCLK    (for APB2 periph.  max. 72MHz)
  RCC_PCLK1Config  (RCC_HCLK_Div2);                      // PCLK1  = HCLK/2  (for APB1 periph.  max. 36MHz)
  RCC_ADCCLKConfig (RCC_PCLK2_Div4);                     // ADCCLK = PCLK2/4 (for ADC:          max. 12MHz)

  // *(vu32 *)0x40022000 = 0x01;                            // Flash 2 wait state
  FLASH_SetLatency(FLASH_Latency_2);                     // Flash 2 wait state
  FLASH_PrefetchBufferCmd(FLASH_PrefetchBuffer_Enable);  // Enable Prefetch Buffer

  if(Timeout)                                            // if HSE came up: use it
    RCC_PLLConfig (RCC_PLLSource_HSE_Div2, RCC_PLLMul_15); // PLLCLK = 4MHz * 15 = 60 MHz
  else                                                   // if HSE did not come up: us internal oscilator
    RCC_PLLConfig (RCC_PLLSource_HSI_Div2, RCC_PLLMul_15); // PLLCLK = 4MHz * 15 = 60 MHz

  RCC_PLLCmd (ENABLE);                                   // Enable PLL
  while (RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET);   // Wait till PLL is ready
  RCC_SYSCLKConfig (RCC_SYSCLKSource_PLLCLK);            // Select PLL as system clock source
  while (RCC_GetSYSCLKSource() != 0x08);                 // Wait till PLL is used as system clock source
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

void LED_PCB_Flash(uint8_t Time=100) { LED_PCB_Counter=Time; } // [ms]

static void LED_PCB_TimerCheck(void)
{ uint8_t Counter=LED_PCB_Counter;
  if(Counter)
  { Counter--;
    if(Counter) LED_PCB_On();
           else LED_PCB_Off();
    LED_PCB_Counter=Counter; }
}

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

// ------------------------------------------------------------------------------------------

// ======================================================================================

// ======================================================================================

// ======================================================================================

// #include "rtc.h"

// ======================================================================================

/*
void NVIC_Configuration (void)
{
  NVIC_InitTypeDef NVIC_InitStructure;

  NVIC_InitStructure.NVIC_IRQChannel = RTC_IRQn;                     // Enable the RTC Interrupt
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
}
*/
// ======================================================================================

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

#ifdef WITH_BEEPER

uint8_t  Vario_Note=0x40;
uint16_t Vario_Period=800;
uint16_t Vario_Fill=50;

static volatile uint16_t Vario_Time=0;

static volatile uint8_t Play_Note=0;             // Note being played
static volatile uint8_t Play_Counter=0;          // time counter

static VolatileFIFO<uint16_t, 8> Play_FIFO;      // queue of notes to play

void Play(uint8_t Note, uint8_t Len)             // put a new not to play in the queue
{ uint16_t Word = Note; Word<<=8; Word|=Len; Play_FIFO.Write(Word); }

uint8_t Play_Busy(void) { return Play_Counter; } // is a note being played right now ?

static void Play_TimerCheck(void)                // every ms serve the note playing
{ uint8_t Counter=Play_Counter;
  if(Counter)                                    // if counter non-zero
  { Counter--;                                   // decrement it
    if(!Counter) Beep_Note(Play_Note=0x00);      // if reached zero, stop playing the note
  }
  if(!Counter)                                   // if counter reached zero
  { if(!Play_FIFO.isEmpty())                     // check for notes in the queue
    { uint16_t Word=0; Play_FIFO.Read(Word);     // get the next note
      Beep_Note(Play_Note=Word>>8); Counter=Word&0xFF; }   // start playing it, load counter with the note duration
  }
  Play_Counter=Counter;

  uint16_t Time=Vario_Time;
  Time++; if(Time>=Vario_Period) Time=0;
  Vario_Time = Time;

  if(Counter==0)                            // when no notes are being played, make the vario sound
  { if(Time<=Vario_Fill)
    { if(Play_Note!=Vario_Note) Beep_Note(Play_Note=Vario_Note); }
    else
    { if(Play_Note!=0) Beep_Note(Play_Note=0x00); }
  }
}

#endif // WITH_BEEPER

// ======================================================================================

void prvSetupHardware(void)
{ RCC_Configuration();

  // NVIC_Configuration();

  if(Parameters.ReadFromFlash()<0)
  { Parameters.setDefault();
    Parameters.WriteToFlash(); }

  UART1_Configuration(Parameters.CONbaud); // Console/Bluetooth serial adapter

  UART2_Configuration(Parameters.GPSbaud); // GPS UART
  GPS_Configuration();                     // GPS PPS and Enable

#ifdef WITH_I2C1
  I2C1_Configuration(400000);              // 400kHz I2C bus speed
#endif

  SPI1_Configuration();                    // SPI1 for the RF chip
  RFM69_GPIO_Configuration();              // RFM69(H)W Reset/DIO0/...
  ADC_Configuration();                     // to measure Vref and CPU temp.

  LED_Configuration();

#ifdef WITH_BEEPER
  Beep_Configuration();
#endif

  // to overwrite parameters
  // Parameters.setTxTypeHW();
  // Parameters.setTxPower(+14); // for RFM69HW (H = up to +20dBm Tx power)
  // Parameters.WriteToFlash();

  // setup watch-dog
  IWDG_WriteAccessCmd(IWDG_WriteAccess_Enable);
  IWDG_SetPrescaler(IWDG_Prescaler_8);              // about 40kHz/8 = 5kHz counter
  IWDG_SetReload(100);                              // reload with 100 thus 20ms timeout ?
  IWDG_ReloadCounter();                             // reload timer = reset the watch-dog
  IWDG_Enable();                                    // enable RESET at timeout
}

extern "C"
void vApplicationIdleHook(void) // when RTOS is idle: should call "sleep until an interrupt"
{ __WFI(); }                    // wait-for-interrupt

extern "C"
void vApplicationTickHook(void) // RTOS timer tick hook
{
  IWDG_ReloadCounter(); // reset watch-dog at every tick (primitive, but enough to start)

  LED_PCB_TimerCheck(); // LED flash periodic check

#ifdef WITH_BEEPER
  Play_TimerCheck();    // Play note periodic check
#endif
}

int main(void)
{
  prvSetupHardware();

  // CTRL: UART1, Console, SD log
  xTaskCreate(vTaskCTRL,  "CTRL",   120, 0, tskIDLE_PRIORITY  , 0);
  // GPS: GPS NMEA/PPS, packet encoding
  xTaskCreate(vTaskGPS,   "GPS",     96, 0, tskIDLE_PRIORITY+1, 0);
  // RF: RF chip, time slots, frequency switching, packet reception and error correction
  xTaskCreate(vTaskRF,    "RF",      96, 0, tskIDLE_PRIORITY+2, 0);
  // SENS: BMP180 pressure, correlate with GPS
  xTaskCreate(vTaskSENS,  "SENS",    96, 0, tskIDLE_PRIORITY+1, 0);

  vTaskStartScheduler();

  while(1)
  { }

}

// lot of things to do:
// + read NMEA user input
// . set Parameters in Flash from NMEA
// + send received positions to console
// + send received positions to console as $POGNT
// + print number of detected transmission errors
// . avoid printing same position twice (from both time slots)
// + send Rx noise and packet stat. as $POGNR
// . optimize receiver sensitivity
// . use RF chip AFC or not ?
// . user RF chip continues AGC/RSSI or not ?
// + periodically refresh the RF chip config (after 60 seconds of Rx inactivity)
// . packet pools for queing
// . separate task for FEC correction
// . separate task for RX processing (retransmission decision)
// . good packets go to RX, bad packets go to FEC first
// . packet retransmission and strategy
// + queue for sounds to be played on the buzzer
// + separate the UART code
// + use watchdog to restart in case of a hangup
// + print heap and task information when Ctlr-C pressed on the console
// . try to run on Maple Mini (there is more Flash, but visibly no xtal)
// + SD card slot and FatFS
// + simple log system onto SD
// + regular log close and auto-resume when card inserted
// . IGC log
// . auto-detect RFM69W or RFM69HW - possible at all ?
// + read RF chip temperature
// . compensate Rx/Tx frequency by RF chip temperature
// . measure the CPU temperature
// . measure VCC voltage: low battery indicator
// + resolve unstable ADC readout
// . detect when VK16u6 GPS fails below 2.7V supply
// . audible alert when GPS fails or absent ?
// . GPS: set higher baud rates
// . GPS: auto-baud
// . check for loss of GPS data and declare fix loss
// + keep/count time (from GPS)
// + connect BMP180 pressure sensor
// . pressure sensor correction in Flash parameters ?
// . support MS5611 pressure sensor
// + correlate pressure and GPS altitude
// . resolve extra dummy byte transfer for I2C_Read()
// + send pressure data in $POGNB
// + vario sound
// + adapt vario integration time to link/sink
// + separate task for BMP180 and other I2C sensors
// . send standard/pressure altitude in the packet ?
// . when measuring pressure avoid times when TX or LOG is active to reduce noise ?
// . stop transmission 60 sec after GPS lock is lost ?
// . audible alert when RF chip fails ?
// + all hardware configure to main() before tasks start ?
// + objective code for RF chip
// . CC1101/CC1120/SPIRIT1 code
// . properly handle TX position when GPS looses lock
// . NMEA commands to make sounds on the speaker
// + use TIM4.CH4 to drive the buzzer with double voltage
// . read compass, gyro, accel.
// + int math into a single file
// + bitcount: option to save flash storage: reduce lookup table from 256 to 16 bytes
// . thermal circling detection
// . measure/transmit/receive QNH
// . measure/transmit/receive wind

