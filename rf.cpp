#include <stdlib.h>

#include <FreeRTOS.h>
#include <task.h>
#include <semphr.h>
#include <queue.h>

#include "hal.h"

#include "rf.h"                            // RF (this) task
#include "ctrl.h"                          // CTRL task
#include "gps.h"                           // GPS task
#include "knob.h"

#include "rfm.h"                           // RFM69 and later RFM95
#include "freqplan.h"                      // frequency hopping pattern

#include "format.h"

#include "bitcount.h"

#include "fifo.h"

#include "uart1.h"                         // console UART

#include "ogn.h"                           // OGN packet

#include "parameters.h"                    // parameters in Flash

#include "main.h"

#include "lowpass2.h"

// ================================================================================

// OGN SYNC:       0x0AF3656C encoded in Manchester
static const uint8_t OGN_SYNC[8] = { 0xAA, 0x66, 0x55, 0xA5, 0x96, 0x99, 0x96, 0x5A };

static RFM_TRX           TRX;    // radio transceiver

static uint32_t  RX_UnixTime;    // [sec] UTC time which belongs to the current time slot (0.4sec late by GPS UTC)
static FreqPlan  RF_FreqPlan;    // frequency hopping pattern calculator

static uint8_t RxPktData[26];    // received packet data
static uint8_t RxPktErr [26];    // received packet error pattern

static OGN_PrioQueue<16> RelayQueue;  // received packets and candidates to be relayed
static OGN_RxPacket      RelayPacket; // received packet to be re-transmitted
static LDPC_Decoder      Decoder;     // error corrector for the OGN Gallager code

static uint16_t TX_Credit  =0;        // counts transmitted packets vs. time to avoid using more than 1% of the time

static uint8_t RX_OGN_Packets=0;      // [packets] counts received packets
// static uint8_t RX_Idle   =0;      // [sec]     time the receiver did not get any packets
static LowPass2<uint32_t, 4,2,4> RX_RSSI;   // low pass filter to average the RX noise

static Delay<uint8_t, 64> RX_OGN_CountDelay;
static uint16_t           RX_OGN_Count64=0; // counts received packets for the last 64 seconds

      uint32_t RX_Random=0x12345678; // Random number from LSB of RSSI readouts

static void XorShift32(uint32_t &Seed)
{ Seed ^= Seed << 13;
  Seed ^= Seed >> 17;
  Seed ^= Seed << 5; }


static char Line[100];            // for printing out to serial port, etc.

static uint8_t RX_Channel=0;      // (hopping) channel currently being received

static void SetTxChannel(uint8_t TxChan=RX_Channel)         // default channel to transmit is same as the receive channel
{
#ifdef WITH_RFM69
  TRX.WriteTxPower(Parameters.getTxPower(), Parameters.isTxTypeHW()); // set TX for transmission
#endif
#if defined(WITH_RFM95) || defined(WITH_SX1272)
  TRX.WriteTxPower(Parameters.getTxPower());                          // set TX for transmission
#endif
  TRX.setChannel(TxChan&0x7F);
  TRX.WriteSYNC(8, 7, OGN_SYNC); }   // Full SYNC for TX

static void SetRxChannel(uint8_t RxChan=RX_Channel)
{ TRX.WriteTxPowerMin();                                    // setup for RX
  TRX.setChannel(RxChan&0x7F);
  TRX.WriteSYNC(7, 7, OGN_SYNC); }   // Full SYNC for TX

static void PrintPktData(void)
{ xSemaphoreTake(CONS_Mutex, portMAX_DELAY);
  // uint8_t ManchErr = Count1s(RxPktErr, 26);
  Format_String(CONS_UART_Write, "Pkt ");
  Format_Hex(CONS_UART_Write, RX_UnixTime);
  // CONS_UART_Write(' '); Format_Hex(CONS_UART_Write, RxRSSI);
  CONS_UART_Write(' '); Format_Hex(CONS_UART_Write, RX_Channel);
  CONS_UART_Write('\r'); CONS_UART_Write('\n');
  for(uint8_t Idx=0; Idx<26; Idx++)
  { CONS_UART_Write(' '); Format_Hex(CONS_UART_Write, RxPktData[Idx]); }
  CONS_UART_Write('\r'); CONS_UART_Write('\n');
  for(uint8_t Idx=0; Idx<26; Idx++)
  { CONS_UART_Write(' '); Format_Hex(CONS_UART_Write, RxPktErr[Idx]); }
  CONS_UART_Write('\r'); CONS_UART_Write('\n');
  xSemaphoreGive(CONS_Mutex);
}

static void PrintRelayQueue(uint8_t Idx)                    // for debug
{ uint8_t Len=0;
  // Len+=Format_String(Line+Len, "");
  xSemaphoreTake(CONS_Mutex, portMAX_DELAY);
  // Format_String(CONS_UART_Write, Line, Len);
  Line[Len++]='['; Len+=Format_Hex(Line+Len, Idx); Line[Len++]=']'; Line[Len++]=' ';
  Len+=RelayQueue.Print(Line+Len);
  Format_String(CONS_UART_Write, Line);
  xSemaphoreGive(CONS_Mutex);
}

static uint8_t ReceivePacket(void)                           // see if a packet has arrived
{ if(!TRX.DIO0_isOn()) return 0;                             // DIO0 line HIGH signals a new packet has arrived

#ifdef WITH_BEEPER
                                                             // if a new packet has been received
  if(KNOB_Tick>12) Play(0x69, 3);                            // if Knob>12 => make a beep for every received packet
#endif

  uint8_t RxPacketIdx  = RelayQueue.getNew();                // get place for this new packet
  OGN_RxPacket *RxPacket = RelayQueue[RxPacketIdx];
  // PrintRelayQueue(RxPacketIdx);                              // for debug

  uint8_t RxRSSI = TRX.ReadRSSI();                           // signal strength for the received packet
  RX_Random = (RX_Random<<1) | (RxRSSI&1);
  RxPacket->RxRSSI=RxRSSI;

  TRX.ReadPacket(RxPktData, RxPktErr);                       // get the packet data from the FIFO
  // PrintPktData();                                            // for debug

  // TickType_t ExecTime=xTaskGetTickCount();

  { RX_OGN_Packets++;
    uint8_t Check=LDPC_Check(RxPktData);                       // check the OGN FEC

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
      { Decoder.Output(RxPacket->Packet.Byte());                 // put corrected data into the RxPacket
        uint8_t Count=0;
        for(uint8_t Idx=0; Idx<26; Idx++)                        // count detected manchester errors
          Count+=Count1s(RxPktErr[Idx]);
        const uint8_t *Corr = RxPacket->Packet.Byte();           // pointer to corrected data
        for(uint8_t Idx=0; Idx<26; Idx++)                        // count bit errors in the data
          Count+=Count1s((uint8_t)((RxPktData[Idx]^Corr[Idx])&(~RxPktErr[Idx])));
        RxPacket->RxErr=Count;
      }
    }

    // taskYIELD();
    if( (Check==0) && (RxPacket->RxErr<16) )                     // what limit on number of detected bit errors ?
    { int32_t LatDist, LonDist;
      if( RxPacket->Packet.Header.Other || RxPacket->Packet.Header.Encrypted ) // non-position packet: ignore
      { }
      else                                                       // de-whiten
      { uint8_t MyOwnPacket = ( RxPacket->Packet.Header.Address == Parameters.getAddress() )
                           && ( RxPacket->Packet.Header.AddrType == Parameters.getAddrType() );
        RxPacket->Packet.Dewhiten();
        bool DistOK = RxPacket->Packet.calcDistanceVector(LatDist, LonDist, GPS_Latitude, GPS_Longitude, GPS_LatCosine)>=0;
        if(DistOK)
        { if(!MyOwnPacket)
          { RxPacket->calcRelayRank(GPS_Altitude/10);              // calculate the relay-rank (priority for relay)
            RelayQueue.addNew(RxPacketIdx); }
          uint8_t Len=RxPacket->WritePOGNT(Line);                  // print on the console as $POGNT
          if(!MyOwnPacket)
          { xSemaphoreTake(CONS_Mutex, portMAX_DELAY);
            Format_String(CONS_UART_Write, Line, Len);
            xSemaphoreGive(CONS_Mutex); }
#ifdef WITH_SDLOG
          xSemaphoreTake(Log_Mutex, portMAX_DELAY);
          Format_String(Log_Write, Line, Len);
          xSemaphoreGive(Log_Mutex);
#endif
          if(!MyOwnPacket)
          { Len=RxPacket->Packet.WritePFLAA(Line, 0, LatDist, LonDist, RxPacket->Packet.DecodeAltitude()-GPS_Altitude/10); // print on the console as $PFLAA
            xSemaphoreTake(CONS_Mutex, portMAX_DELAY);
            Format_String(CONS_UART_Write, Line, Len);
            xSemaphoreGive(CONS_Mutex); }
        }
      }
    }
  }

  // TRX.WriteMode(RFM69_OPMODE_RX);                            // back to receive (but we already have AutoRxRestart)
/*
  ExecTime=xTaskGetTickCount()-ExecTime;
  xSemaphoreTake(CONS_Mutex, portMAX_DELAY);
  Format_String(CONS_UART_Write, "ReceivePacket: "); Format_UnsDec(CONS_UART_Write, ExecTime); CONS_UART_Write('\r'); CONS_UART_Write('\n');
  xSemaphoreGive(CONS_Mutex);
*/
  return 1; }                                                    // return: 1 packet we have received

static uint32_t ReceiveUntil(TickType_t End)
{ uint32_t Count=0;
  for( ; ; )
  { Count+=ReceivePacket();
    int32_t Left = End-xTaskGetTickCount();
    if(Left<=0) break;
    vTaskDelay(1); }
  return Count; }

static uint32_t ReceiveFor(TickType_t Ticks)                     // keep receiving packets for given period of time
{ return ReceiveUntil(xTaskGetTickCount()+Ticks); }
/*
static uint32_t ReceiveFor(uint32_t Ticks)                       // keep receiving packets for given period of time
{ uint32_t Count=0; uint32_t Delta=0;
  TickType_t Start=xTaskGetTickCount();                          // remember when you started
  do
  { Count+=ReceivePacket(); vTaskDelay(1);
    Delta=xTaskGetTickCount()-Start;                             // time since you started
  } while(Delta<Ticks);                                          // keep going until time since started equals to the given time period
  return Count; }                                                // return number of received packets
*/

static uint8_t Transmit(uint8_t TxChan, const uint8_t *PacketByte, uint8_t Thresh, uint8_t MaxWait=7)
{
  if(PacketByte==0) return 0;                                   // if no packet to send: simply return

  if(MaxWait)
  { for( ; MaxWait; MaxWait--)                                  // wait for a given maximum time for a free radio channel
    {
#ifdef WITH_RFM69
      TRX.TriggerRSSI();
#endif
      vTaskDelay(1);
      uint8_t RxRSSI=TRX.ReadRSSI();
      RX_Random = (RX_Random<<1) | (RxRSSI&1);
      if(RxRSSI>=Thresh) break; }
    if(MaxWait==0) return 0; }

  TRX.WriteMode(RF_OPMODE_STANDBY);                              // switch to standby
  vTaskDelay(1);
  SetTxChannel(TxChan);

  TRX.ClearIrqFlags();
  TRX.WritePacket(PacketByte);                                   // write packet into FIFO
  TRX.WriteMode(RF_OPMODE_TRANSMITTER);                          // transmit
  vTaskDelay(5);
  for(uint8_t Wait=200; Wait; Wait--)                            // wait for transmission to end
  { // vTaskDelay(1);
    uint8_t Mode=TRX.ReadMode();
    uint16_t Flags=TRX.ReadIrqFlags();
    if(Mode!=RF_OPMODE_TRANSMITTER) break;
    if(Flags&RF_IRQ_PacketSent) break; }
  TRX.WriteMode(RF_OPMODE_STANDBY);                              // switch to standy

  SetRxChannel();
  TRX.WriteMode(RF_OPMODE_RECEIVER);                             // back to receive mode

  return 1; }

/*                                                                           // make a time-slot: keep receiving and make a single transmission
static void TimeSlot(uint8_t TxChan, uint32_t SlotLen, uint8_t *PacketByte, uint8_t Rx_RSSI, uint8_t MaxWait=8, uint32_t TxTime=0)
{ SlotLen-=MaxWait;
  if(TxTime==0) TxTime = RX_Random%SlotLen;                                    // decide when to transmit the packet
  ReceiveFor(TxTime);                                                      // keep receiving until the transmit time
  if( (TX_Credit) && (PacketByte) )                                        // when packet to transmit is given and there is still TX credit left:
    TX_Credit-=Transmit(TxChan, PacketByte, Rx_RSSI, MaxWait);             // attempt to transmit the packet
  ReceiveFor(SlotLen-TxTime);
}                                                                          // the total time varies: is not exactly = Len
*/

static void TimeSlot(uint8_t TxChan, uint32_t SlotLen, uint8_t *PacketByte, uint8_t Rx_RSSI, uint8_t MaxWait=8, uint32_t TxTime=0)
{ TickType_t Start = xTaskGetTickCount();                                  // when the slot started
  TickType_t End   = Start + SlotLen;                                      // when should it end
  uint32_t MaxTxTime = SlotLen-8-MaxWait;                                  // time limit when transmision could start
  if( (TxTime==0) || (TxTime>=MaxTxTime) ) TxTime = RX_Random%MaxTxTime;   // if TxTime out of limits, setup a random TxTime
  TickType_t Tx    = Start + TxTime;                                       // Tx = the moment to start transmission
  ReceiveUntil(Tx);                                                        // listen until this time comes
  if( (TX_Credit) && (PacketByte) )                                        // when packet to transmit is given and there is still TX credit left:
    TX_Credit-=Transmit(TxChan, PacketByte, Rx_RSSI, MaxWait);             // attempt to transmit the packet
  ReceiveUntil(End);                                                       // listen till the end of the time-slot
}

// static void TimeSlot(uint8_t TxChan, uint32_t Len, OGN_Packet &Packet, uint8_t Rx_RSSI, uint8_t MaxWait=8)
// { TimeSlot(TxChan, Len, Packet.isReady() ? Packet.Byte:0, Rx_RSSI, MaxWait); }

// static void TimeSlot(uint8_t TxChan, uint32_t SlotLen, OGN_RxPacket &Packet, uint8_t Rx_RSSI, uint8_t MaxWait=8, uint32_t TxTime=0)
// { TimeSlot(TxChan, SlotLen, Packet.isReady() ? Packet.Packet.Byte():0, Rx_RSSI, MaxWait, TxTime); }

static void SetFreqPlan(void)
{ TRX.setBaseFrequency(RF_FreqPlan.BaseFreq);                // set the base frequency (recalculate to RFM69 internal synth. units)
  TRX.setChannelSpacing(RF_FreqPlan.ChanSepar);              // set the channel separation
  TRX.setFrequencyCorrection(10*Parameters.RFchipFreqCorr);  // set the fine correction (to counter the Xtal error)
}

static uint8_t StartRFchip(void)
{
  TRX.RESET(1);
  vTaskDelay(10);
  TRX.RESET(0);
  vTaskDelay(10);
  SetFreqPlan();
  TRX.WriteMode(RF_OPMODE_STANDBY);                          // set RF chip mode to STANDBY
  TRX.Configure(0, OGN_SYNC);                                // setup RF chip parameters and set to channel #0
  return TRX.ReadVersion(); }                                // read the RF chip version and return it

// ---------------------------------------------------------------------------------------------------------------------------------------

static void GetRelayPacket(void)                                 // prepare a packet to be relayed
{ RelayPacket.clrReady();
  if(RelayQueue.Sum==0) return;
  uint8_t Idx=RelayQueue.getRand(RX_Random);
  if(RelayQueue.Packet[Idx].Rank==0) return;
  // RelayPacket = RelayQueue.Packet[Idx];
  memcpy(RelayPacket.Packet.Byte(), RelayQueue[Idx], OGN_Packet::Bytes);
  RelayPacket.Packet.Header.RelayCount++;
  RelayPacket.Packet.Whiten(); RelayPacket.calcFEC();
  // PrintRelayQueue(Idx);  // for debug
  RelayQueue.decrRank(Idx);
  RelayPacket.setReady(); }

// ---------------------------------------------------------------------------------------------------------------------------------------

#ifdef __cplusplus
  extern "C"
#endif
void vTaskRF(void* pvParameters)
{
                                          // set interface for the RF chip
  TRX.Select       = RFM_Select;          // separate SS
  TRX.Deselect     = RFM_Deselect;
  TRX.TransferByte = RFM_TransferByte;
  TRX.DIO0_isOn    = RFM_DIO0_isOn;
  TRX.RESET        = RFM_RESET;

  RF_FreqPlan.setPlan(0);                  // 1 = Europe/Africa, 2 = USA/CA, 3 = Australia and South America

  OGN_RxPacket CurrPosPacket;              // should be OGN_Packet but we need to resolve the use of setReady()
  OGN_RxPacket CurrStatPacket;             // status report packet
#ifdef WITH_PPM
  OGN_PPM_Packet PPM_Packet;               // pulse-position-modulation apcket for long range reception
#endif
  RelayQueue.Clear();

  vTaskDelay(5);

  for( ; ; )
  { uint8_t ChipVersion = StartRFchip();

    xSemaphoreTake(CONS_Mutex, portMAX_DELAY);
    Format_String(CONS_UART_Write, "TaskRF: ");
    CONS_UART_Write('v'); Format_Hex(CONS_UART_Write, ChipVersion);
    CONS_UART_Write('\r'); CONS_UART_Write('\n');
    xSemaphoreGive(CONS_Mutex);

    if( (ChipVersion!=0x00) && (ChipVersion!=0xFF) ) break;
    vTaskDelay(1000);
  }

  CurrPosPacket.clrReady();
  TX_Credit  = 0;    // count slots and packets transmitted: to keep the rule of 1% transmitter duty cycle
  RX_OGN_Packets = 0;    // count received packets per every second (two time slots)
  // RX_Idle    = 0;    // count receiver idle time (slots without any packets received)

  RX_OGN_Count64 = 0;
  RX_OGN_CountDelay.Clear();

  RX_Channel = RF_FreqPlan.getChannel(GPS_UnixTime, 0, 1);
  SetRxChannel();
  TRX.WriteMode(RF_OPMODE_RECEIVER);

  RX_RSSI.Set(2*112);

  uint8_t TimeSinceNoPos=0xFF;
  for( ; ; )
  {

    TimeSinceNoPos++; if(TimeSinceNoPos==0) TimeSinceNoPos--;
    // if(GPS_TimeSinceLock>4)
    // { }
    if(GPS_TimeSinceLock>2)                                                    // if GPS lock is already there for more than 2 seconds
    { GPS_Position *Position = GPS_getPosition();                              // get the most recent valid GPS position
      RF_FreqPlan.setPlan(Position->Latitude, Position->Longitude);            // set the frequency plan according to the GPS position
      if(Position && Position->Complete && Position->isValid() )               // position must be complete and valid
      { int8_t Sec=Position->Sec;
        Sec-=2; if(Sec<0) Sec+=60;
        GPS_Position *RefPos = GPS_getPosition(Sec);                           // reference position 2 seconds ago (or just 1 second ?)
        if(RefPos && RefPos->Complete && RefPos->isValid() )                   // reference position must be complete and valid
        { Position->calcDifferences(*RefPos);                                  // calculate the differentials
          CurrPosPacket.Packet.HeaderWord=0;
          CurrPosPacket.Packet.Header.Address  = Parameters.getAddress();               // set address
          CurrPosPacket.Packet.Header.AddrType = Parameters.getAddrType();              // address-type
          CurrPosPacket.Packet.Header.Emergency=0; CurrPosPacket.Packet.Header.Encrypted=0; CurrPosPacket.Packet.Header.RelayCount=0;
          CurrPosPacket.Packet.Header.Other=0; CurrPosPacket.Packet.calcAddrParity();    // rest of header
          Position->Encode(CurrPosPacket.Packet);                                        // encode position/altitude/speed/etc. from GPS position
          CurrPosPacket.Packet.Position.Stealth = Parameters.getStealth();
          CurrPosPacket.Packet.Position.AcftType = Parameters.getAcftType();             // aircraft-type
          CurrPosPacket.Packet.Whiten(); CurrPosPacket.calcFEC();                        // whiten and calculate FEC code
          CurrPosPacket.setReady();                                                      // mark packet as ready to go
          CurrStatPacket.Packet.HeaderWord=CurrPosPacket.Packet.HeaderWord;
          CurrStatPacket.Packet.Status.Hardware=0;
          CurrStatPacket.Packet.Status.Firmware=0;
          CurrStatPacket.Packet.Header.Other=1;
          Position->EncodeStatus(CurrStatPacket.Packet);
          TimeSinceNoPos=0;
        }
      }
    }
    else if(CurrPosPacket.isReady() )                                          // if GPS lock is not there but previous position is already in the position packet
    { if(TimeSinceNoPos>30)
      { CurrPosPacket.Packet.Dewhiten();
        CurrPosPacket.Packet.Position.Time=0x3F;
        CurrPosPacket.Packet.Whiten();
        CurrPosPacket.calcFEC(); }
    }                                                                          // we should invalidate position after some time

#ifdef WITH_PPM
    if(CurrPosPacket.isReady() && ((GPS_Sec==0) || (GPS_Sec==30)) )
    { CurrPosPacket.Packet.Dewhiten();
      PPM_Packet.Packet=CurrPosPacket.Packet;
      CurrPosPacket.Packet.Whiten();
      PPM_Packet.Packet.Whiten();
      PPM_Packet.calcFEC();
    }
#endif
    int16_t AverSpeed=GPS_AverageSpeed();
    uint32_t RxRssiSum=0; uint16_t RxRssiCount=0;                              // measure the average RSSI for lower frequency
    do
    { ReceivePacket();                                                         // keep checking for received packets
#ifdef WITH_RFM69
      TRX.TriggerRSSI();
#endif
      vTaskDelay(1);
      uint8_t RxRSSI=TRX.ReadRSSI();                                           // measure the channel noise level
      RX_Random = (RX_Random<<1) | (RxRSSI&1);
      RxRssiSum+=RxRSSI; RxRssiCount++;
    } while(PPS_Phase()<300);                                                  // until 300ms from the PPS
    RX_RSSI.Process(RxRssiSum/RxRssiCount);                                    // [-0.5dBm] average noise on channel

    uint8_t RX_Rssi=RX_RSSI.getOutput();

    TRX.WriteMode(RF_OPMODE_STANDBY);                                         // switch to standy
    vTaskDelay(1);
    SetFreqPlan();

    // uint8_t TxSlotIdx=GPS_Sec%15;
    // uint8_t TxSlot0 = CurrPosPacket.Packet.getTxSlot(Idx  );
    // uint8_t TxSLot1 = CurrPosPacket.Packet.getTxSlot(Idx+1);

    StartRFchip();
                                                                               // Note: on RFM95 temperature sens does not work in STANDBY
#ifdef WITH_RFM69
    TRX.TriggerTemp();                                                         // trigger RF chip temperature readout
#endif
    vTaskDelay(1); // while(TRX.RunningTemp()) taskYIELD();                    // wait for conversion to be ready
    int8_t ChipTemp= 165-TRX.ReadTemp();                                       // read RF chip temperature

    xSemaphoreTake(ADC1_Mutex, portMAX_DELAY);
    uint16_t MCU_Vtemp  = ADC_Read_MCU_Vtemp();                                // T = 25+(V25-Vtemp)/Avg_Slope; V25=1.43+/-0.1V, Avg_Slope=4.3+/-0.3mV/degC
    uint16_t MCU_Vref   = ADC_Read_MCU_Vref();                                 // VDD = 1.2*4096/Vref
             MCU_Vtemp += ADC_Read_MCU_Vtemp();                                // measure again and average
             MCU_Vref  += ADC_Read_MCU_Vref();
#ifdef WITH_BATT_SENSE
    uint16_t Vbatt       = ADC_Read_Vbatt();                                   // measure voltage on PB1
             Vbatt      += ADC_Read_Vbatt();
#endif
    xSemaphoreGive(ADC1_Mutex);
    int16_t MCU_Temp = -999;
    uint16_t MCU_VCC = 0;
    if(MCU_Vref)
    { MCU_Temp = 250 + ( ( ( (int32_t)1430 - ((int32_t)1200*(int32_t)MCU_Vtemp+(MCU_Vref>>1))/MCU_Vref )*(int32_t)37 +8 )>>4); // [0.1degC]
      MCU_VCC  = ( ((uint32_t)240<<12)+(MCU_Vref>>1))/MCU_Vref; }              // [0.01V]
    CurrStatPacket.Packet.EncodeVoltage(((MCU_VCC<<4)+12)/25);                 // [1/64V]  write supply voltage to the status packet
    CurrStatPacket.Packet.Status.RadioNoise = RX_Rssi;                         // [-0.5dBm] write radio noise to the status packet
#ifdef WITH_BATT_SENSE
    if(MCU_Vref)
      CurrStatPacket.Packet.EncodeVoltage(((int32_t)154*(int32_t)Vbatt+(MCU_Vref>>1))/MCU_Vref); // [1/64V] battery voltage assuming 1:1 divider form battery to PB1
#endif
    if(CurrStatPacket.Packet.Status.Pressure==0) CurrStatPacket.Packet.EncodeTemperature(MCU_Temp);
    CurrStatPacket.Packet.Status.TxPower = Parameters.getTxPower()-4;
    uint16_t RxRate = RX_OGN_Count64+1;
    uint8_t RxRateLog2=0; RxRate>>=1; while(RxRate) { RxRate>>=1; RxRateLog2++; }
    CurrStatPacket.Packet.Status.RxRate = RxRateLog2;

    uint8_t Time=GPS_Sec+30; if(Time>=60) Time-=60;
    RelayQueue.cleanTime(Time);                                                // clean relay queue past 30 seconds

    RX_OGN_Count64 += RX_OGN_Packets;                                          // add packets received
    RX_OGN_Count64 -= RX_OGN_CountDelay.Input(RX_OGN_Packets);                 // subtract packets received 64 seconds ago

    { uint8_t Len=0;
      Len+=Format_String(Line+Len, "$POGNR,");                                 // NMEA report: radio status
      Len+=Format_UnsDec(Line+Len, RF_FreqPlan.Plan);                          // which frequency plan
      Line[Len++]=',';
      Len+=Format_UnsDec(Line+Len, RX_OGN_Count64);                            // number of OGN packets received
      Line[Len++]=',';
      // Len+=Format_UnsDec(Line+Len, GPS_UnixTime);                           // GPS time: for debug
      Line[Len++]=',';
      Len+=Format_SignDec(Line+Len, -5*RX_Rssi, 2, 1);                         // average RF level (over all channels)
      Line[Len++]=',';
      Len+=Format_UnsDec(Line+Len, (uint16_t)TX_Credit);
      Line[Len++]=',';
      Len+=Format_SignDec(Line+Len, (int16_t)ChipTemp);                        // the temperature of the RF chip
      if(MCU_Vref)
      { Line[Len++]=',';
        Len+=Format_SignDec(Line+Len, MCU_Temp, 2, 1);
        Line[Len++]=',';
        Len+=Format_UnsDec(Line+Len, MCU_VCC, 3, 2); }

      Len+=NMEA_AppendCheckCRNL(Line, Len);                                    // append NMEA check-sum and CR+NL
      // LogLine(Line);
      xSemaphoreTake(CONS_Mutex, portMAX_DELAY);
      Format_String(CONS_UART_Write, Line, Len);                                   // send the NMEA out to the console
      // RelayQueue.Print(Line);
      // Format_String(CONS_UART_Write, Line);
      xSemaphoreGive(CONS_Mutex);
#ifdef WITH_SDLOG
      xSemaphoreTake(Log_Mutex, portMAX_DELAY);
      Format_String(Log_Write, Line, Len);                                     // send the NMEA out to the log file
      xSemaphoreGive(Log_Mutex);
#endif
      // if(RX_OGN_Packets) RX_Idle=0;
      //              else  RX_Idle++;
      RX_OGN_Packets=0;                                                        // clear th ereceived packet count
    }

    RX_UnixTime = GPS_UnixTime;
    uint8_t TxChan = RF_FreqPlan.getChannel(RX_UnixTime, 0, 1);                // tranmsit channel
    RX_Channel = TxChan;                                                       // listen channel
    SetRxChannel();
                                                                               // here we can read the chip temperature
    TRX.WriteMode(RF_OPMODE_RECEIVER);                                         // switch to receive mode
    vTaskDelay(1);

    RxRssiSum=0; RxRssiCount=0;                                                // measure the average RSSI for the upper frequency
    do
    { ReceivePacket();                                                         // check for packets being received ?
#ifdef WITH_RFM69
      TRX.TriggerRSSI();                                                       // start RSSI measurement
#endif
      vTaskDelay(1);
      uint8_t RxRSSI=TRX.ReadRSSI();                                           // read RSSI
      RX_Random = (RX_Random<<1) | (RxRSSI&1);                                 // take lower bit for random number generator
      RxRssiSum+=RxRSSI; RxRssiCount++;
    } while(PPS_Phase()<400);                                                  // keep going until 400 ms after PPS
    RX_RSSI.Process(RxRssiSum/RxRssiCount);                                    // [-0.5dBm] average noise on channel

    TX_Credit+=2; if(TX_Credit>7200) TX_Credit=7200;                           // count the transmission credit

    XorShift32(RX_Random);

    uint32_t Seed = GPS_UnixTime ^ CurrStatPacket.Packet.HeaderWord;
    XorShift32(Seed);
    XorShift32(Seed);
    uint8_t StatSlot  = (Seed&0x1F) == 0x00;
    if(StatSlot) { CurrStatPacket.Packet.Whiten(); CurrStatPacket.calcFEC(); }

    uint8_t RelaySlot = GPS_UnixTime&1;                                        // relay
    uint8_t RelayReady = (RelayQueue.Sum>0) && ((RX_Random&0x80)==0);          // any packets for relaying ?

    if(RelayReady) GetRelayPacket();                                           // get ready the packet to be relayed (if any)
             else  RelayPacket.clrReady();

    uint32_t TxTime = (RX_Random&0x3F)+1; TxTime*=6;

    uint8_t *Packet=0;
    if(CurrPosPacket.isReady()) Packet = (uint8_t *)&CurrPosPacket;
    if(RelaySlot)
    { if(StatSlot) Packet = (uint8_t *)&CurrStatPacket;
      else if(RelayReady) Packet = (uint8_t *)&RelayPacket; }
    TimeSlot(TxChan, 400, Packet,   RX_Rssi, 0, TxTime);

    TRX.WriteMode(RF_OPMODE_STANDBY);
    // vTaskDelay(1);                                                             // do we need a delay here ?
    TxChan = RF_FreqPlan.getChannel(RX_UnixTime, 1, 1);                        // transmit channel
    RX_Channel = TxChan;                                                       // listen channel
    SetRxChannel();
    TRX.WriteMode(RF_OPMODE_RECEIVER);

    if(RelaySlot) RelaySlot=0;
             else RelaySlot = (GPS_UnixTime&1)==0;
                                                                               // here we can read the chip temperature
    // GetRelayPacket();                                                       // get readt the packet to be relayed (if any)

    TxTime = ((RX_Random>>8)&0x3F)+1; TxTime*=6;
    Packet=0;
    if(CurrPosPacket.isReady()) Packet = (uint8_t *)&CurrPosPacket;
    if(RelaySlot)
    { if(StatSlot) Packet = (uint8_t *)&CurrStatPacket;
      else if(RelayReady) Packet = (uint8_t *)&RelayPacket; }
    TimeSlot(TxChan, 400, Packet,   RX_Rssi, 0, TxTime);

  }

}

// ======================================================================================
