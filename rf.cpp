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

#include "fifo.h"

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

static OGN_PrioQueue<16> RelayQueue;  // received packets and candidates to be relayed
static OGN_Packet        RelayPacket; // received packet to be re-transmitted
static LDPC_Decoder      Decoder;     // error corrector for the OGN Gallager code

static uint8_t TX_FreqChan=0;     // 0 = 868.2MHz, 1 = 868.4MHz
static uint8_t TX_Credit  =0;     // counts transmitted packets vs. time to avoid using more than 1% of the time

static uint8_t RX_Packets=0;      // [packets] counts received packets
static uint8_t RX_Idle   =0;      // [sec]     time the receiver did not get any packets
static int32_t RX_RssiLow=0;      // [-0.5dBm] background noise level on lower RF channel
static int32_t RX_RssiUpp=0;      // [-0.5dBm] background noise level on upper RF channel

static Delay<uint8_t, 64> RX_CountDelay;
static uint16_t           RX_Count64=0; // counts received packets for the last 64 seconds

      uint32_t RX_Random=0x12345678; // Random number from LSB of RSSI readouts

static char Line[88];

static uint8_t Receive(void)                                 // see if a packet has arrived
{ if(!TRX.DIO0_isOn()) return 0;                             // DIO0 line HIGH signals a new packet has arrived

#ifdef WITH_BEEPER
                                                             // if a new packet has been received
  if(KNOB_Tick>12) Play(0x69, 3);                            // if Knob>12 => make a beep for every received packet
#endif

  uint8_t RxPacketIdx  = RelayQueue.getNew();                // get place for this new packet
  OGN_Packet *RxPacket = RelayQueue[RxPacketIdx];

  uint8_t RxRSSI = TRX.ReadRSSI();                           // signal strength for the received packet
  RX_Random = (RX_Random<<1) | (RxRSSI&1);
  RxPacket->RxRSSI=RxRSSI;

  TRX.ReadPacket(RxPktData, RxPktErr);                       // get the packet data from the FIFO
  RX_Packets++;
  uint8_t Check=LDPC_Check(RxPktData);

  // taskYIELD();
  RxPacket->RxErr=0;
  if(Check==0)                                                 // if FEC is all fine
  { RxPacket->recvBytes(RxPktData); }                          // copy received data directly into the RxPacket
  else                                                         // if errors detected:
  { Decoder.Input(RxPktData, RxPktErr);                        // put data into the FEC decoder
    for(uint8_t Iter=24; Iter; Iter--)                         // more loops is more chance to recover the packet
    { Check=Decoder.ProcessChecks();                           // do an iteration
      if(Check==0) break; }                                    // if FEC all fine: break
    if(Check==0)                                               // if FEC all fine after correction:
    { Decoder.Output(RxPacket->Word);                          // put corrected data into the RxPacket
      uint8_t Count=0;
      for(uint8_t Idx=0; Idx<26; Idx++)                        // count detected manchester errors
        Count+=Count1s(RxPktErr[Idx]);
      const uint8_t *Corr = RxPacket->Byte;                    // pointer to corrected data
      for(uint8_t Idx=0; Idx<26; Idx++)                        // count bit errors in the data
        Count+=Count1s((uint8_t)((RxPktData[Idx]^Corr[Idx])&(~RxPktErr[Idx])));
      RxPacket->RxErr=Count;
    }
  }

  // taskYIELD();
  if(Check==0)
  { if(RxPacket->isOther())
    { }
    else
    { RxPacket->Dewhiten();
      RxPacket->calcRelayRank(GPS_Altitude);
      RelayQueue.addNew(RxPacketIdx);
      uint8_t Len=RxPacket->WriteNMEA(Line);
      xSemaphoreTake(UART1_Mutex, portMAX_DELAY);
      Format_String(UART1_Write, Line, Len);
      xSemaphoreGive(UART1_Mutex);
#ifdef WITH_SDLOG
      xSemaphoreTake(Log_Mutex, portMAX_DELAY);
      Format_String(Log_Write, Line, Len);
      xSemaphoreGive(Log_Mutex);
#endif
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

  return 1; }                                                   // return: 1 packet we have received

static uint32_t Receive(uint32_t Ticks)                          // keep receiving packets for given period of time
{ uint32_t Count=0; uint32_t Delta=0;
  TickType_t Start=xTaskGetTickCount();                          // remember when you started
  do
  { Count+=Receive(); vTaskDelay(1);
    Delta=xTaskGetTickCount()-Start;                             // time since you started
  } while(Delta<Ticks);                                          // keep going until time since started equals to the given time period
  return Count; }                                                // return number of received packets

static uint8_t Transmit(const uint8_t *PacketByte, uint8_t Thresh, uint8_t MaxWait=7)
{
  if(PacketByte==0) return 0;                                   // if no data: simply return

  for( ; MaxWait; MaxWait--)                                    // wait for a given maximum time for a free radio channel
  { TRX.TriggerRSSI();
    vTaskDelay(1);
    uint8_t RxRSSI=TRX.ReadRSSI();
    RX_Random = (RX_Random<<1) | (RxRSSI&1);
    if(RxRSSI>=Thresh) break; }
  if(MaxWait==0) return 0;

  TRX.WriteMode(RFM69_OPMODE_STDBY);                            // switch to standby
  vTaskDelay(1);

  TRX.WriteTxPower(Parameters.getTxPower(), Parameters.isTxTypeHW()); // set TX for transmission
  TRX.WriteSYNC(8, 7, OGN_SYNC);                                // Full SYNC for TX
  TRX.ClearIrqFlags();
  TRX.WritePacket(PacketByte);                                  // write packet into FIFO
  TRX.WriteMode(RFM69_OPMODE_TX);                               // transmit
  for(uint8_t Wait=10; Wait; Wait--)                            // wait for transmission to end
  { vTaskDelay(1);
    uint8_t Mode=TRX.ReadMode();
    uint16_t Flags=TRX.ReadIrqFlags();
    if(Mode!=RFM69_OPMODE_TX) break;
    if(Flags&RFM69_IRQ_PacketSent) break; }
  TRX.WriteMode(RFM69_OPMODE_STDBY);                             // switch to standy

  TRX.WriteTxPowerMin();                                         // setup for RX
  TRX.WriteSYNC(7, 7, OGN_SYNC);                                 // write (not complete) SYNC
  TRX.WriteMode(RFM69_OPMODE_RX);                                // back to receive mode

  return 1; }

static void GetRelayPacket(void)                                 // prepare a packet to be relayed
{ RelayPacket.clrReady();
  if(RelayQueue.Sum==0) return;
  uint8_t Idx=RelayQueue.getRand(RX_Random);
  if(RelayQueue.Packet[Idx].Rank==0) return;
  RelayPacket = RelayQueue.Packet[Idx]; 
  RelayPacket.incRelayCount();
  RelayPacket.Whiten(); RelayPacket.calcFEC();
  RelayPacket.setReady(); }

static void TimeSlot(uint32_t Len, uint8_t *PacketByte, uint8_t Rx_RSSI, uint8_t MaxWait=8)
{ Len-=MaxWait;
  uint32_t TxTime = RX_Random%Len;                                         // when to transmit the packet
  Receive(TxTime);                                                         // keep receiving until the transmit time
  if( (TX_Credit) && (PacketByte) )
  TX_Credit-=Transmit(PacketByte, RX_RssiLow, MaxWait);                    // attempt to transmit the packet
  Receive(Len-TxTime);
}

static void TimeSlot(uint32_t Len, OGN_Packet &Packet, uint8_t Rx_RSSI, uint8_t MaxWait=8)
{ TimeSlot(Len, Packet.isReady() ? Packet.Byte:0, Rx_RSSI, MaxWait); }

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

  OGN_Packet CurrPosPacket;

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

  CurrPosPacket.clrReady();
  TX_Credit  = 0;    // count slots and packets transmitted: to keep the rule of 1% transmitter duty cycle
  RX_Packets = 0;    // count received packets per every second (two time slots)
  RX_Idle    = 0;    // count receiver idle time (slots without any packets received)

  RX_Count64 = 0;
  RX_CountDelay.Clear();

  TX_FreqChan=0; TRX.WriteFreq(LowFreq+Parameters.RFchipFreqCorr);
  TRX.WriteSYNC(7, 7, OGN_SYNC); TRX.WriteMode(RFM69_OPMODE_RX);
  for( ; ; )
  {

    if(GPS_TimeSinceLock>2)                                                    // if GPS lock is already there for more than 2 seconds
    { OgnPosition *Position = GPS_getPosition();                               // get the most recent valid GPS position
      if(Position && Position->isComplete() && Position->isValid() )                                                             //
      { int8_t Sec=Position->Sec;
        Sec-=2; if(Sec<0) Sec+=60;
        OgnPosition *RefPos = GPS_getPosition(Sec);
        if(RefPos && RefPos->isComplete() && RefPos->isValid() )
        { Position->calcDifferences(*RefPos);
          CurrPosPacket.setAddress(Parameters.getAddress());                     // prepare the current position packet
          CurrPosPacket.setAddrType(Parameters.getAddrType());
          CurrPosPacket.clrOther(); CurrPosPacket.calcAddrParity();
          CurrPosPacket.clrEmergency(); CurrPosPacket.clrEncrypted(); CurrPosPacket.setRelayCount(0);
          Position->Encode(CurrPosPacket);
          CurrPosPacket.setStealth(Parameters.getStealth());
          CurrPosPacket.setAcftType(Parameters.getAcftType());
          CurrPosPacket.Whiten(); CurrPosPacket.calcFEC();
          CurrPosPacket.setReady();
	  // CurrPosPacket_Sec=GPS_Sec;
        }
      }
    } else                                                                     // if GPS lock is not there
    { // if( (CurrPosPacket_Sec == GPS_Sec) && () )
    }                                                                          // we should invalidate position after some time
/*
    TRX.WriteMode(RFM69_OPMODE_STDBY);                                         // switch to standy
    vTaskDelay(1);

    TX_FreqChan=0; TRX.WriteFreq(LowFreq+Parameters.RFchipFreqCorr);           // switch to upper frequency

    TRX.WriteMode(RFM69_OPMODE_RX);                                            // switch to receive mode
    vTaskDelay(1);
*/
    uint32_t RxRssiSum=0; uint16_t RxRssiCount=0;                              // measure the average RSSI for lower frequency
    do
    { Receive();                                                               // keep checking for received packets
      TRX.TriggerRSSI();
      vTaskDelay(1);
      uint8_t RxRSSI=TRX.ReadRSSI();                                           // measure the channel noise level
      RX_Random = (RX_Random<<1) | (RxRSSI&1);
      RxRssiSum+=RxRSSI; RxRssiCount++;
    } while(PPS_Phase()<300);                                                  // until 300ms from the PPS
    RX_RssiLow = RxRssiSum/RxRssiCount;                                        // [-0.5dBm] average noise on channel

    TRX.WriteMode(RFM69_OPMODE_STDBY);                                         // switch to standy
    vTaskDelay(1);

    if(RX_Idle>=60)                                                            // if no reception within one minute
    { StartRFchip();                                                           // reset and rewrite the RF chip config
      RX_Idle=0; }
                                                                               // here we can read the chip temperature
    TX_FreqChan=1; TRX.WriteFreq(UppFreq+Parameters.RFchipFreqCorr);           // switch to upper frequency

    TRX.TriggerTemp();                                                         // trigger RF chip temperature readout
    vTaskDelay(1); // while(TRX.RunningTemp()) taskYIELD();                    // wait for conversion to be ready
    int8_t ChipTemp= 165-TRX.ReadTemp();                                       // read RF chip temperature

    TRX.WriteMode(RFM69_OPMODE_RX);                                            // switch to receive mode
    vTaskDelay(1);

    RxRssiSum=0; RxRssiCount=0;                                                // measure the average RSSI for the upper frequency
    do
    { Receive();                                                               // check for packets being received ?
      TRX.TriggerRSSI();                                                       // start RSSI measurement
      vTaskDelay(1);
      uint8_t RxRSSI=TRX.ReadRSSI();                                           // read RSSI
      RX_Random = (RX_Random<<1) | (RxRSSI&1);                                 // take lower bit for random number generator
      RxRssiSum+=RxRSSI; RxRssiCount++;
    } while(PPS_Phase()<400);                                                  // keep going until 400 ms after PPS
    RX_RssiUpp = RxRssiSum/RxRssiCount;                                        // average RSSI [-0.5dBm]

    // uint16_t MCU_Temp = ADC1_Read(ADC_Channel_TempSensor);                  // now the knob acquisition took over the ADC
    // uint16_t MCU_Vref = ADC1_Read(ADC_Channel_Vrefint);

    uint8_t Time=GPS_Sec+30; if(Time>=60) Time-=60;
    RelayQueue.cleanTime(Time);                                                // clean relay queue past 30 seconds

    RX_Count64 += RX_Packets;                                                  // add packets received
    RX_Count64 -= RX_CountDelay.Input(RX_Packets);                             // subtract packets received 64 seconds ago

    { uint8_t Len=0;
      // memcpy(Line+Len, "$POGNR,", 7); Len+=7;                               // prepare NMEA of status report
      Len+=Format_String(Line+Len, "$POGNR,");
      Len+=Format_UnsDec(Line+Len, RX_Count64);                                // number of packets received
      Line[Len++]=',';
      Len+=Format_SignDec(Line+Len, -(RX_RssiLow/2));                          // average RF level on the lower frequency
      Line[Len++]=',';
      Len+=Format_SignDec(Line+Len, -(RX_RssiUpp/2));                          // average RF level on the upper frequency
      Line[Len++]=',';
      Len+=Format_SignDec(Line+Len, (int16_t)ChipTemp);
      Line[Len++]=',';
      // Len+=Format_UnsDec(Line+Len, MCU_Temp);
      // Line[Len++]=',';
      // Len+=Format_UnsDec(Line+Len, MCU_Vref);
      // Line[Len++]=',';
      Len+=Format_UnsDec(Line+Len, (uint16_t)TX_Credit);
      Len+=NMEA_AppendCheckCRNL(Line, Len);                                    // append NMEA check-sum and CR+NL
      // LogLine(Line);
      xSemaphoreTake(UART1_Mutex, portMAX_DELAY);
      Format_String(UART1_Write, Line, Len);                                   // send the NMEA out to the console
      // RelayQueue.Print(Line);
      // Format_String(UART1_Write, Line);
      xSemaphoreGive(UART1_Mutex);
#ifdef WITH_SDLOG
      xSemaphoreTake(Log_Mutex, portMAX_DELAY);
      Format_String(Log_Write, Line, Len);                                     // send the NMEA out to the log file
      xSemaphoreGive(Log_Mutex);
#endif
      if(RX_Packets) RX_Idle=0;
               else  RX_Idle++;
      RX_Packets=0;                                                            // clear th ereceived packet count
    }

    uint8_t RelaySlot = GPS_Sec&1;                                             // odd slot: we take it for the relay
    uint8_t RelayReady = RelayQueue.Sum>0;                                     // any packets for relaying ?
    uint8_t SplitSlot = RelayReady && (TX_Credit>=4);                          // split the 400ms time slot into 2x200ms.
    GetRelayPacket();                                                          // get ready the packet to be relayed (if any)

    TX_Credit++; if(!TX_Credit) TX_Credit--;                                   // new half-slot => increment the transmission credit

    if(SplitSlot)                                                              // if split time slot
    { TimeSlot(200, CurrPosPacket, RX_RssiUpp);                                // send position
      TimeSlot(200, RelayPacket,   RX_RssiUpp); }                              // relay prepared packet
    else                                                                       // no split
    { if(RelaySlot && RelayReady) TimeSlot(400, RelayPacket,   RX_RssiUpp);    // if there is a packet for relaying: send it
                             else TimeSlot(400, CurrPosPacket, RX_RssiUpp);    // otherwise send current position packet
    }

    TX_FreqChan=0; TRX.WriteFreq(LowFreq+Parameters.RFchipFreqCorr);           // switch to lower frequency

    GetRelayPacket();

    TX_Credit++; if(!TX_Credit) TX_Credit--;                                   // new half slot => increment transmission credit

    if(SplitSlot)
    { TimeSlot(200, CurrPosPacket, RX_RssiLow);
      TimeSlot(200, RelayPacket,   RX_RssiLow); }
    else
    { if(RelaySlot && RelayReady) TimeSlot(400, RelayPacket,   RX_RssiLow);
                             else TimeSlot(400, CurrPosPacket, RX_RssiLow);
    }

  }

}

// ======================================================================================
