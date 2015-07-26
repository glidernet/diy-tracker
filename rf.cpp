#include <stdlib.h>

#include <FreeRTOS.h>
#include <task.h>
#include <semphr.h>
#include <queue.h>

#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_spi.h"

#include "rf.h"                            // RF (this) task
#include "ctrl.h"                          // CTRL task
#include "gps.h"                           // GPS task
#include "knob.h"

#include "rfm69.h"                         // RFM69(H)W RF chip

#include "format.h"

#include "bitcount.h"

#include "uart1.h"                         // console UART
// #include "adc.h"

#include "ogn.h"                           // OGN packet

#include "parameters.h"                    // parameters in Flash

#include "main.h"

#include "spi1.h"       // SPI1 basic calls

// ================================================================================

// function to control and read status of the RF chip

#ifdef SPEEDUP_STM_LIB
inline void RFM69_RESET_On  (void) { GPIOB->BSRR = GPIO_Pin_5; }
inline void RFM69_RESET_Off (void) { GPIOB->BRR  = GPIO_Pin_5; }
#else
inline void RFM69_RESET_On  (void) { GPIO_SetBits  (GPIOB, GPIO_Pin_5); }
inline void RFM69_RESET_Off (void) { GPIO_ResetBits(GPIOB, GPIO_Pin_5); }
#endif

static void RFM69_RESET(uint8_t On)
{ if(On) RFM69_RESET_On();
    else RFM69_RESET_Off(); }

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
  // GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_IPU;
  GPIO_Init(GPIOB, &GPIO_InitStructure);

  GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_5;                // PB5 = RESET (active high)
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_Out_PP;
  GPIO_Init(GPIOB, &GPIO_InitStructure);
}

// ==================================================================

// OGN SYNC: 0x0AF3656C encoded in Manchester
static const uint8_t OGN_SYNC[8] = { 0xAA, 0x66, 0x55, 0xA5, 0x96, 0x99, 0x96, 0x5A };

// OGN frequencies: 868.2 and 868.4 MHz
static const double OGN_LowFreq = 868.200e6; // [MHz] 868MHz band frequencies
static const double OGN_UppFreq = 868.400e6; // [MHz]
static const double    XtalFreq = 32e6;      // [MHz] RF chip crystal frequency

static const uint32_t LowFreq = floor(OGN_LowFreq/(XtalFreq/(1<<19))+0.5); // conversion from RF frequency
static const uint32_t UppFreq = floor(OGN_UppFreq/(XtalFreq/(1<<19))+0.5); // to RF chip synthesizer setting

static RFM69             TRX;    // radio transceiver

static uint8_t RxPktData[26];    // received packet data
static uint8_t RxPktErr [26];    // received packet error pattern
static OGN_Packet  RxPacket;     // received packet
static LDPC_Decoder Decoder;     // error corrector for the Gallager code

static uint8_t TX_FreqChan=0;     // 0 = 868.2MHz, 1 = 868.4MHz
static uint8_t TX_Credit  =0;     // counts transmitted packets vs. time to avoid using more than 1% of the time

static uint8_t RX_Packets=0;      // [packets] counts received packets
static uint8_t RX_Idle   =0;      // [sec]     time the receiver did not get any packets
static int32_t RX_RssiLow=0;      // [-0.5dBm] background noise level on two frequencies
static int32_t RX_RssiUpp=0;      // [-0.5dBm]

      uint32_t RX_Random=0x12345678; // Random number from LSB of RSSI readouts

static char Line[88];

static uint8_t Receive(void)                                 // see if a packet has arrived
{ if(!TRX.DIO0_isOn()) return 0;

#ifdef WITH_BEEPER
                                                             // if a new packet has been received
  if(KNOB_Tick) Play(0x69, 3);
#endif

  uint8_t RxRSSI = TRX.ReadRSSI();                           // signal strength for the received packet
  RxPacket.RxRSSI=RxRSSI;

  TRX.ReadPacket(RxPktData, RxPktErr);                       // get the packet data from the FIFO
  RX_Packets++;
  uint8_t Check=LDPC_Check(RxPktData);

  // taskYIELD();
  RxPacket.RxErr=0;
  if(Check==0)
  { RxPacket.recvBytes(RxPktData); }
  else                                                         // if errors detected
  { Decoder.Input(RxPktData, RxPktErr);
    for(uint8_t Iter=24; Iter; Iter--)                         // more loops i smore chance to recover the packet
    { Check=Decoder.ProcessChecks();                           // but it takes time
      if(Check==0) break; }
    if(Check==0)
    { Decoder.Output(&RxPacket.Header);
      uint8_t Count=0;
      for(uint8_t Idx=0; Idx<26; Idx++)                        // count detected manchester errors
        Count+=Count1s(RxPktErr[Idx]);
      const uint8_t *Corr = (const uint8_t *)&RxPacket.Header;
      for(uint8_t Idx=0; Idx<26; Idx++)
        Count+=Count1s((uint8_t)((RxPktData[Idx]^Corr[Idx])&(~RxPktErr[Idx])));
      RxPacket.RxErr=Count;
    }
  }

  // taskYIELD();
  if(Check==0)
  { if(RxPacket.isOther())
    { }
    else
    { RxPacket.Dewhiten();
      uint8_t Len=RxPacket.WriteNMEA(Line);
      xSemaphoreTake(UART1_Mutex, portMAX_DELAY);
      Format_String(UART1_Write, Line, Len);
      xSemaphoreGive(UART1_Mutex);
      LogLine(Line);
    }
  }

/*
  for(uint8_t Idx=0; Idx<26; Idx++)
  { UART1_Write(' '); Format_Hex(UART1_Write, RxPktData[Idx]); }
  UART1_Write('\r'); UART1_Write('\n');
  for(uint8_t Idx=0; Idx<26; Idx++)
  { UART1_Write(' '); Format_Hex(UART1_Write, RxPktErr[Idx]); }
  UART1_Write('\r'); UART1_Write('\n');
*/
  // TRX.WriteMode(RFM69_OPMODE_RX);                            // back to receive (but we already have AutoRxRestart)

  return 1; }

static uint8_t Receive(int Ticks)                                        // keep receiving packets for given period of time
{ uint8_t Count=0; int Delta=0;
  TickType_t Start=xTaskGetTickCount();
  do
  { Count+=Receive(); vTaskDelay(1);
    Delta=xTaskGetTickCount()-Start;
  } while(Delta<Ticks);
  return Count; }

static uint8_t Transmit(const uint8_t *PacketPtr, uint8_t Thresh, int MaxWait=7)
{
  if(PacketPtr==0) return 0;

  for( ; MaxWait; MaxWait--)
  { TRX.TriggerRSSI();
    vTaskDelay(1);
    uint8_t RxRSSI=TRX.ReadRSSI();
    if(RxRSSI>=Thresh) break; }
  if(MaxWait==0) return 0;

  TRX.WriteMode(RFM69_OPMODE_STDBY);                            // switch to standby
  vTaskDelay(1);

  TRX.WriteTxPower(Parameters.getTxPower(), Parameters.isTxTypeHW()); // set TX for transmission
  TRX.WriteSYNC(8, 7, OGN_SYNC);                                // Full SYNC for TX
  TRX.ClearIrqFlags();
  TRX.WritePacket(PacketPtr);                                   // write packet into FIFO
  TRX.WriteMode(RFM69_OPMODE_TX);                               // transmit
  for(uint8_t MaxWait=10; MaxWait; MaxWait--)                   // wait for transmission to end
  { vTaskDelay(1);
    uint8_t Mode=TRX.ReadMode();
    uint16_t Flags=TRX.ReadIrqFlags();
    if(Mode!=RFM69_OPMODE_TX) break;
    if(Flags&RFM69_IRQ_PacketSent) break; }
  TRX.WriteMode(RFM69_OPMODE_STDBY);                             // switch to standy

  TRX.WriteTxPowerMin();                                         // setup for RX
  TRX.WriteSYNC(7, 7, OGN_SYNC);                                 //
  TRX.WriteMode(RFM69_OPMODE_RX);                                // back to receive

  return 1; }

static uint8_t StartRFchip(void)
{ TRX.RESET_On();
  vTaskDelay(10);
  TRX.RESET_Off();
  vTaskDelay(10);
  TRX.Configure(LowFreq, OGN_SYNC);
  TRX.WriteMode(RFM69_OPMODE_STDBY);
  return TRX.ReadVersion(); }

#ifdef __cplusplus
  extern "C"
#endif
void vTaskRF(void* pvParameters)
{
                                          // set hardware interface
  TRX.Select       = SPI1_Select;         //
  TRX.Deselect     = SPI1_Deselect;
  TRX.TransferByte = SPI1_TransferByte;
  TRX.DIO0_isOn    = RFM69_DIO0_isOn;
  TRX.DIO4_isOn    = RFM69_DIO4_isOn;
  TRX.RESET_On     = RFM69_RESET_On;
  TRX.RESET_Off    = RFM69_RESET_Off;

  uint8_t *PacketPtr = 0;

  vTaskDelay(5);

  // SPI1_Configuration();
  // RFM69_GPIO_Configuration();
  // ADC_Configuration();

  for( ; ; )
  { uint8_t ChipVersion = StartRFchip();

    xSemaphoreTake(UART1_Mutex, portMAX_DELAY);
    Format_String(UART1_Write, "TaskRF: ");
    UART1_Write('v'); Format_Hex(UART1_Write, ChipVersion);
    UART1_Write('\r'); UART1_Write('\n');
    xSemaphoreGive(UART1_Mutex);

    if( (ChipVersion!=0x00) && (ChipVersion!=0xFF) ) break;
    vTaskDelay(1000);
  }

  TX_Credit =  0;
  RX_Packets = 0;    // count received packets per every second (two time slots)
  RX_Idle    = 0;    // count receiver idle time (slots without any packets received)

  TX_FreqChan=0; TRX.WriteFreq(LowFreq+Parameters.RFchipFreqCorr);
  TRX.WriteSYNC(7, 7, OGN_SYNC); TRX.WriteMode(RFM69_OPMODE_RX);
  for( ; ; )
  {

    while(uxQueueMessagesWaiting(xQueuePacket)>1)         // see for new packets to be sent
    { xQueueReceive(xQueuePacket, &PacketPtr, 0); }
    if(uxQueueMessagesWaiting(xQueuePacket)>0)
    { xQueuePeek(xQueuePacket, &PacketPtr, 0); }

    uint32_t RxRssiSum=0; uint16_t RxRssiCount=0;         // measure the average RSSI for lower frequency
    do
    { Receive();
      TRX.TriggerRSSI();
      vTaskDelay(1);
      uint8_t RxRSSI=TRX.ReadRSSI();
      RX_Random = (RX_Random<<1) | (RxRSSI&1);
      RxRssiSum+=RxRSSI; RxRssiCount++;
    } while(PPS_Phase()<300);
    RX_RssiLow = RxRssiSum/RxRssiCount; // [-0.5dBm]

    TRX.WriteMode(RFM69_OPMODE_STDBY);                               // switch to standy
    vTaskDelay(1);

    if(RX_Idle>=60)                                                    // if no reception within one minute
    { StartRFchip();                                                   // reset and rewrite the RF chip config
      RX_Idle=0; }
                                                                        // here we can read the chip temperature
    TX_FreqChan=1; TRX.WriteFreq(UppFreq+Parameters.RFchipFreqCorr);    // switch to upper frequency
    TX_Credit++; if(!TX_Credit) TX_Credit--;                            // new half-slot => increment the transmission credit
    int TxTimeUpp = RX_Random%400;                                      // when to transmit the packet

    TRX.TriggerTemp();
    vTaskDelay(1); // while(TRX.RunningTemp()) taskYIELD();
    int8_t ChipTemp= 165-TRX.ReadTemp();

    TRX.WriteMode(RFM69_OPMODE_RX);                            // switch to receive mode
    vTaskDelay(1);

    RxRssiSum=0; RxRssiCount=0;                                // measure the average RSSI for the upper frequency
    do
    { Receive();                                               // check for packets being received ?
      TRX.TriggerRSSI();                                       // start RSSI measurement
      vTaskDelay(1);
      uint8_t RxRSSI=TRX.ReadRSSI();                           // read RSSI
      RX_Random = (RX_Random<<1) | (RxRSSI&1);                 // take lower bit for random number generator
      RxRssiSum+=RxRSSI; RxRssiCount++;
    } while(PPS_Phase()<400);                                  // keep going until 400 ms after PPS
    RX_RssiUpp = RxRssiSum/RxRssiCount;                        // average RSSI [-0.5dBm]

    // uint16_t MCU_Temp = ADC1_Read(ADC_Channel_TempSensor);
    // uint16_t MCU_Vref = ADC1_Read(ADC_Channel_Vrefint);

    { uint8_t Len=0;
      // memcpy(Line+Len, "$POGNR,", 7); Len+=7;                               // prepare NMEA of status report
      Len+=Format_String(Line+Len, "$POGNR,");
      Len+=Format_UnsDec(Line+Len, RX_Packets);                             // number of packets received
      Line[Len++]=',';
      Len+=Format_SignDec(Line+Len, -(RX_RssiLow/2));                       // average RF level on the lower frequency
      Line[Len++]=',';
      Len+=Format_SignDec(Line+Len, -(RX_RssiUpp/2));                       // average RF level on the upper frequency
      Line[Len++]=',';
      Len+=Format_SignDec(Line+Len, (int16_t)ChipTemp);
      Line[Len++]=',';
      // Len+=Format_UnsDec(Line+Len, MCU_Temp);
      // Line[Len++]=',';
      // Len+=Format_UnsDec(Line+Len, MCU_Vref);
      // Line[Len++]=',';
      Len+=Format_UnsDec(Line+Len, (uint16_t)TX_Credit);
      Len+=NMEA_AppendCheckCRNL(Line, Len);                                 // append NMEA check-sum and CR+NL
      LogLine(Line);
      xSemaphoreTake(UART1_Mutex, portMAX_DELAY);
      Format_String(UART1_Write, Line, Len);                                 // send the NMEA out to the console
      xSemaphoreGive(UART1_Mutex);
      if(RX_Packets) RX_Idle=0;
               else  RX_Idle++;
      RX_Packets=0;                                                          // clear th ereceived packet count
    }

    Receive(TxTimeUpp);                                                      // keep receiving until the transmit time
    if(TX_Credit)
      TX_Credit-=Transmit(PacketPtr, RX_RssiUpp, 8);                         // attempt to transmit the packet
    Receive(390-TxTimeUpp);                                                  // receive till the end of the time slot

    TX_FreqChan=0; TRX.WriteFreq(LowFreq+Parameters.RFchipFreqCorr);         // switch to lower frequency
    TX_Credit++; if(!TX_Credit) TX_Credit--;                                 // new half slot => increment transmission credit
    int TxTimeLow = RX_Random%400;                                           // decide when to transmit

    Receive(TxTimeLow);                                                      // receive until transmit time comes
    if(TX_Credit)
      TX_Credit-=Transmit(PacketPtr, RX_RssiLow, 8);                         // attempt to transmit the packet
    Receive(390-TxTimeLow);                                                  // receive till the end of the time-slot

  }

}

// ======================================================================================
