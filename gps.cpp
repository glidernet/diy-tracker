#include <stdint.h>
#include <stdlib.h>

#include "hal.h"

#include "nmea.h"
#include "ubx.h"

#include "ogn.h"

#include "parameters.h"

#include "gps.h"
#include "ctrl.h"
#include "knob.h"

#include "systick.h"

#include "lowpass2.h"

// ----------------------------------------------------------------------------

#include "main.h"

#ifdef WITH_PPS_IRQ
uint32_t   PPS_IRQ_TickCount;                       // [RTOS tick] (coarse) time of previous PPS
uint32_t   PPS_IRQ_TickTime;                        // [CPU tick] (fine) time of previous PPS
 int32_t   PPS_IRQ_TickCountDiff;                   // [RTOS tick] diff.
 int32_t   PPS_IRQ_TickTimeDiff;                    // [CPU tick] diff. of (fine) times between PPS'es
 int32_t   PPS_IRQ_Correction=0;                    // [1/16 CPU tick]
#endif

#ifdef WITH_PPS_IRQ
static void PPS_IRQ(uint32_t TickCount, uint32_t TickTime)         // GPS PPS IRQ handler
{ PPS_IRQ_TickCountDiff = TickCount - PPS_IRQ_TickCount;
  PPS_IRQ_TickTimeDiff = TickTime - PPS_IRQ_TickTime;
  const int32_t OneSec = 1000;
  if(abs((int32_t)(PPS_IRQ_TickCountDiff-OneSec))<=2)
  { for( ; ; )
    { if(PPS_IRQ_TickCountDiff==OneSec) break;
      if(PPS_IRQ_TickCountDiff>OneSec)
      { PPS_IRQ_TickTimeDiff+=SysTickPeriod; PPS_IRQ_TickCountDiff-=1; }
      if(PPS_IRQ_TickCountDiff<OneSec)
      { PPS_IRQ_TickTimeDiff-=SysTickPeriod; PPS_IRQ_TickCountDiff+=1; }
    }
    int32_t Diff = PPS_IRQ_TickTimeDiff - ((PPS_IRQ_Correction+8)>>4);
    PPS_IRQ_Correction += Diff;
  }
  PPS_IRQ_TickCount = TickCount;
  PPS_IRQ_TickTime  = TickTime;
}
#endif

// void Debug_Print(uint8_t Byte) { while(!UART1_TxEmpty()) taskYIELD(); UART1_TxChar(Byte); }

static NMEA_RxMsg  NMEA;                 // NMEA sentences catcher
static UBX_RxMsg   UBX;                  // UBX messages catcher

static GPS_Position Position[4];         // GPS position pipe
static uint8_t      PosIdx;              // Pipe index, increments with every GPS position received

static   TickType_t Burst_TickCount;     // [msec] TickCount when the data burst from GPS started

         TickType_t PPS_TickCount;       // [msec] TickCount of the most recent PPS pulse
         uint32_t   GPS_TimeSinceLock;   // [sec] time since the GPS has a lock
volatile  uint8_t   GPS_Sec=0;           // [sec] UTC time: second

         uint32_t   GPS_UnixTime  = 0;    // [sec] UTC date/time in Unix format
         uint32_t   GPS_FatTime   = 0;    // [sec] UTC date/time in FAT format
          int32_t   GPS_Altitude  = 0;    // [0.1m] last valid altitude
          int32_t   GPS_Latitude  = 0;    //
          int32_t   GPS_Longitude = 0;    //
          int16_t   GPS_GeoidSepar= 0;    // [0.1m]
         uint16_t   GPS_LatCosine = 3000; //
//          uint8_t   GPS_FreqPlan=0;      //

         Status     GPS_Status;

static const uint8_t  BaudRates=7;                                                                 // number of possible baudrates choices
static       uint8_t  BaudRateIdx=0;                                                               // actual choice
static const uint32_t BaudRate[BaudRates] = { 4800, 9600, 19200, 38400, 57600, 115200, 230400 } ;  // list of baudrate the autobaud scans through

uint32_t GPS_getBaudRate (void) { return BaudRate[BaudRateIdx]; }
uint32_t GPS_nextBaudRate(void) { BaudRateIdx++; if(BaudRateIdx>=BaudRates) BaudRateIdx=0; return GPS_getBaudRate(); }

const uint32_t GPS_TargetBaudRate = 57600; // BaudRate[4]; // [bps] must be one of the baud rates known by the autbaud
const uint8_t  GPS_dynModel       =     7; // for UBX GPS's: 6 = airborne with >1g, 7 = with >2g

// ----------------------------------------------------------------------------

int16_t GPS_AverageSpeed(void)                        // get average speed based on stored GPS positions
{ uint8_t Count=0;
  int16_t Speed=0;
  for(uint8_t Idx=0; Idx<4; Idx++)                    // loop over GPS positions
  { GPS_Position *Pos = Position+Idx;
    if( !Pos->Complete || !Pos->isValid() ) continue; // skip invalid positions
    Speed += Pos->Speed +abs(Pos->ClimbRate); Count++;
  }
  if(Count==0) return -1;
  if(Count>1) Speed/=Count;
  return Speed; }

// static char Line[64];                  // for console output formating

uint16_t PPS_Phase(void)                              //
{ TickType_t Phase=xTaskGetTickCount()-PPS_TickCount; // time after last PPS pulse ?
  if(Phase<1000) return Phase;                        // is less than 1sec then return it directly
  return Phase%1000; }                                // [0...999 ms] current time-phase in respect to PPS

// ----------------------------------------------------------------------------

static void GPS_PPS_On(uint8_t Delay=0)               // called on rising edge of PPS
{ static TickType_t PrevTickCount=0;
  TickType_t TickCount = xTaskGetTickCount();
  TickType_t Delta = TickCount-PrevTickCount;
  PrevTickCount = TickCount;
  if(abs(Delta-1000)>10) return;
  PPS_TickCount=TickCount-Delay;
  GPS_Status.PPS=1;
  LED_PCB_Flash(50);
  uint8_t Sec=GPS_Sec; Sec++; if(Sec>=60) Sec=0; GPS_Sec=Sec;
  GPS_UnixTime++; }

static void GPS_PPS_Off(void)                       // called on falling edge of PPS
{ }

// ----------------------------------------------------------------------------

static void GPS_LockStart(void)                     // called when GPS catches a lock
{ GPS_Status.Lock=1;
/*
#ifdef WITH_BEEPER
  if(KNOB_Tick)
  { Play(0x50, 100);
    Play(0x10, 100);
    Play(0x52, 100);
    Play(0x12, 100); }
#endif
*/
}

static void GPS_LockEnd(void)                       // called when GPS looses a lock
{ GPS_Status.Lock=0;
/*
#ifdef WITH_BEEPER
  if(KNOB_Tick)
  { Play(0x52, 100);
    Play(0x12, 100);
    Play(0x50, 100);
    Play(0x10, 100); }
#endif
*/
}

// ----------------------------------------------------------------------------

const uint8_t GPS_BurstDelay=70;                                           // [ms] time after the PPS when the data burst starts on the UART

static void GPS_BurstStart(void)                                           // when GPS starts sending the data on the serial port
{ Burst_TickCount=xTaskGetTickCount();
  // TickType_t TicksSincePPS=Burst_TickCount-PPS_TickCount;
  // if(TicksSincePPS>10500)
#ifndef WITH_GPS_PPS
  GPS_PPS_On(GPS_BurstDelay);                                              // if there is no PPS from GPS use the burst start to simulate the PPS with given delay.
#endif
  static uint16_t QueryWait=0;
  if(GPS_Status.NMEA)                                                      // if there is communication with the GPS already
  { if(QueryWait)
    { QueryWait--; }
    else
    { if(!GPS_Status.ModeConfig)                                             // if GPS navigation mode is not done yet
      { // Format_String(CONS_UART_Write, "CFG_NAV5 query...\n");
        UBX_RxMsg::SendPoll(0x06, 0x24, GPS_UART_Write); }                   // send the query for the navigation mode setting
      if(!GPS_Status.BaudConfig)                                             // if GPS baud config is not done yet
      { // Format_String(CONS_UART_Write, "CFG_PRT query...\n");
        UBX_RxMsg::SendPoll(0x06, 0x00, GPS_UART_Write); }                   // send the query for the port config to have a template configuration packet
      QueryWait=300; }
  }
  else { QueryWait=0; }
}

static void GPS_BurstEnd(void)                                             // when GPS stops sending data on the serial port
{ if(Position[PosIdx].Complete)                                            // position data complete
  { if(Position[PosIdx].isTimeValid())                                     // if time is valid already
    { GPS_Sec=Position[PosIdx].Sec;                                        // get the second - this is important for time in the OGN packets
      if(Position[PosIdx].isDateValid())                                   // if date is valid as well
      { GPS_UnixTime=Position[PosIdx].getUnixTime(); GPS_FatTime=Position[PosIdx].getFatTime(); }
    }
    if(Position[PosIdx].isValid())                                         // position is complete and locked
    { Position[PosIdx].calcLatitudeCosine();
      GPS_TimeSinceLock++;
      GPS_Altitude=Position[PosIdx].Altitude;
      GPS_Latitude=Position[PosIdx].Latitude;
      GPS_Longitude=Position[PosIdx].Longitude;
      GPS_GeoidSepar=Position[PosIdx].GeoidSeparation;
      GPS_LatCosine=Position[PosIdx].LatitudeCosine;
      // GPS_FreqPlan=Position[PosIdx].getFreqPlan();
      if(GPS_TimeSinceLock==1)
      { GPS_LockStart(); }
      if(GPS_TimeSinceLock>2)
      { uint8_t PrevIdx=(PosIdx+2)&3;
        Position[PosIdx].calcDifferences(Position[PrevIdx]);
        LED_PCB_Flash(100); }
    }
    else                                                                  // complete but no valid lock
    { if(GPS_TimeSinceLock) { GPS_LockEnd(); GPS_TimeSinceLock=0; }
    }
  }
  else                                                                    // posiiton not complete, no GPS lock
  { if(GPS_TimeSinceLock) { GPS_LockEnd(); GPS_TimeSinceLock=0; }
  }
  uint8_t NextPosIdx = (PosIdx+1)&3;                                      // next position to be recorded
  Position[NextPosIdx].Clear();                                           // clear the next position
  int8_t Sec = Position[PosIdx].Sec;                                      //
  Sec++; if(Sec>=60) Sec=0;
  Position[NextPosIdx].Sec=Sec;                                           // set the correct time for the next position
  // Position[NextPosIdx].copyTime(Position[PosIdx]);                        // copy time from current position
  // Position[NextPosIdx].incrTime();                                        // increment time by 1 sec
  PosIdx=NextPosIdx;                                                      // advance the index
}

GPS_Position *GPS_getPosition(void)                                       // retunr most recent, valid GPS_Position
{ uint8_t PrevIdx=PosIdx;
  GPS_Position *PrevPos = Position+PrevIdx;
  if(PrevPos->Complete) return PrevPos;
  PrevIdx=(PrevIdx+3)&3;
  PrevPos = Position+PrevIdx;
  if(PrevPos->Complete) return PrevPos;
  return 0; }

GPS_Position *GPS_getPosition(int8_t Sec)                                // return the GPS_Position which corresponds to given Sec
{ for(uint8_t Idx=0; Idx<4; Idx++)
  { if(Sec==Position[Idx].Sec) return Position+Idx; }
  return 0; }

// ----------------------------------------------------------------------------

static void GPS_NMEA(void)                                                 // when GPS gets a correct NMEA sentence
{ GPS_Status.NMEA=1;
  LED_PCB_Flash(2);                                                        // Flash the LED for 2 ms
  Position[PosIdx].ReadNMEA(NMEA);                                         // read position elements from NMEA
  if( NMEA.isP() || NMEA.isGxRMC() || NMEA.isGxGGA() || NMEA.isGxGSA() || NMEA.isGPTXT() )
  { static char CRNL[3] = "\r\n";
    xSemaphoreTake(CONS_Mutex, portMAX_DELAY);
    Format_Bytes(CONS_UART_Write, NMEA.Data, NMEA.Len);
    Format_Bytes(CONS_UART_Write, (const uint8_t *)CRNL, 2);
    xSemaphoreGive(CONS_Mutex);
#ifdef WITH_SDLOG
    xSemaphoreTake(Log_Mutex, portMAX_DELAY);
    Format_Bytes(Log_Write, NMEA.Data, NMEA.Len);
    Format_Bytes(Log_Write, (const uint8_t *)CRNL, 2);
    xSemaphoreGive(Log_Mutex);
#endif
  }
}

static void DumpUBX(void)
{ Format_String(CONS_UART_Write, "UBX:");
  for(uint8_t Idx=0; Idx<20; Idx++)
  { CONS_UART_Write(' '); Format_Hex(CONS_UART_Write, UBX.Byte[Idx]); }
  Format_String(CONS_UART_Write, "\n"); }

static void GPS_UBX(void)                                                         // when GPS gets an UBX packet
{ GPS_Status.UBX=1;
  LED_PCB_Flash(2);
#ifdef WITH_GPS_UBX_PASS
  { xSemaphoreTake(CONS_Mutex, portMAX_DELAY);                                    // send ther UBX packet to the console
    UBX.Send(CONS_UART_Write);
    // Format_String(CONS_UART_Write, "UBX");
    // Format_Hex(CONS_UART_Write, UBX.Class);
    // Format_Hex(CONS_UART_Write, UBX.ID);
    xSemaphoreGive(CONS_Mutex); }
#endif
  if(UBX.isCFG_PRT())                                                             // if port configuration
  { class UBX_CFG_PRT *CFG = (class UBX_CFG_PRT *)UBX.Word;                       // create pointer to the packet content
    // xSemaphoreTake(CONS_Mutex, portMAX_DELAY);
    // Format_String(CONS_UART_Write, "TaskGPS: CFG_PRT\n");
    // DumpUBX();
    // Format_Hex(CONS_UART_Write, CFG->portID);
    // CONS_UART_Write(':');
    // Format_UnsDec(CONS_UART_Write, CFG->baudRate);
    // Format_String(CONS_UART_Write, "bps\n");
    // xSemaphoreGive(CONS_Mutex);
    if(CFG->baudRate==GPS_TargetBaudRate) GPS_Status.BaudConfig=1;                // if baudrate same as our target then declare the baud config is done
    else                                                                          // otherwise use the received packet as the template
    { CFG->baudRate=GPS_TargetBaudRate;                                           // set the baudrate to our target
      UBX.RecalcCheck();                                                          // reclaculate the check sum
      // xSemaphoreTake(CONS_Mutex, portMAX_DELAY);
      // Format_UnsDec(CONS_UART_Write, GPS_TargetBaudRate);
      // Format_String(CONS_UART_Write, "bps\n");
      // DumpUBX();
      // xSemaphoreGive(CONS_Mutex);
      UBX.Send(GPS_UART_Write);                                                   // send this UBX packet to the GPS
    }
  }
  if(UBX.isCFG_NAV5())                                                            // Navigation config
  { class UBX_CFG_NAV5 *CFG = (class UBX_CFG_NAV5 *)UBX.Word;
    // xSemaphoreTake(CONS_Mutex, portMAX_DELAY);
    // Format_String(CONS_UART_Write, "TaskGPS: CFG_NAV5 ");
    // Format_Hex(CONS_UART_Write, CFG->dynModel);
    // Format_String(CONS_UART_Write, "\n");
    // xSemaphoreGive(CONS_Mutex);
    if(CFG->dynModel==GPS_dynModel) GPS_Status.ModeConfig=1;                      // dynamic model = 6 => Airborne with >1g acceleration
    else
    { CFG->dynModel=GPS_dynModel; CFG->mask = 0x01;                               //
      UBX.RecalcCheck();                                                          // reclaculate the check sum
      UBX.Send(GPS_UART_Write);                                                   // send this UBX packet
    }
  }
/*
  if(UBX.isACK())
  { xSemaphoreTake(CONS_Mutex, portMAX_DELAY);
    Format_String(CONS_UART_Write, "TaskGPS: ACK_ ");
    Format_Hex(CONS_UART_Write,  UBX.ID);
    CONS_UART_Write(' ');
    Format_Hex(CONS_UART_Write,  UBX.Byte[0]);
    CONS_UART_Write(':');
    Format_Hex(CONS_UART_Write,  UBX.Byte[1]);
    Format_String(CONS_UART_Write, "\n");
    xSemaphoreGive(CONS_Mutex);
  }
*/
}

// ----------------------------------------------------------------------------

// Baud setting for SIRF GPS:
//    9600/8/N/1      $PSRF100,1,9600,8,1,0*0D<cr><lf>
//   19200/8/N/1      $PSRF100,1,19200,8,1,0*38<cr><lf>
//   38400/8/N/1      $PSRF100,1,38400,8,1,0*3D<cr><lf>
//                    $PSRF100,1,57600,8,1,0*36
//                    $PSRF100,1,115200,8,1,0*05

// static const char *SiRF_SetBaudrate_57600  = "$PSRF100,1,57600,8,1,0*36\r\n";
// static const char *SiRF_SetBaudrate_115200 = "$PSRF100,1,115200,8,1,0*05\r\n";


// Baud setting for MTK GPS:
// $PMTK251,38400*27<CR><LF>
// $PMTK251,57600*2C<CR><LF>
// $PMTK251,115200*1F<CR><LF>

// static const char *MTK_SetBaudrate_115200 = "$PMTK251,115200*1F\r\n";


// Baud setting for UBX GPS:
// "$PUBX,41,1,0003,0001,19200,0*23\r\n"
// "$PUBX,41,1,0003,0001,38400,0*26\r\n"
// "$PUBX,41,1,0003,0001,57600,0*2D\r\n"
// static const char *UBX_SetBaudrate_115200 = "$PUBX,41,1,0003,0001,115200,0*1E\r\n";

// ----------------------------------------------------------------------------

#ifdef __cplusplus
  extern "C"
#endif
void vTaskGPS(void* pvParameters)
{
  GPS_Status.Flags = 0;
#ifdef WITH_PPS_IRQ
  GPS_PPS_IRQ_Callback = PPS_IRQ;
#endif

  PPS_TickCount=0;
  Burst_TickCount=0;

  vTaskDelay(5);

  xSemaphoreTake(CONS_Mutex, portMAX_DELAY);
  Format_String(CONS_UART_Write, "TaskGPS:");
  Format_String(CONS_UART_Write, "\n");
  xSemaphoreGive(CONS_Mutex);

  // Format_String(GPS_UART_Write, UBX_SetBaudrate_115200);

  int Burst=(-1);                                                         // GPS transmission ongoing or line is idle ?
  { int LineIdle=0;                                                       // [ms] counts idle time for the GPS data
    int NoValidData=0;                                                    // [ms] count time without valid data (to decide to change baudrate)
    int PPS=0;
    NMEA.Clear(); UBX.Clear();                                            // scans GPS input for NMEA and UBX frames
    for(uint8_t Idx=0; Idx<4; Idx++)
      Position[Idx].Clear();
    PosIdx=0;

    TickType_t RefTick = xTaskGetTickCount();
    for( ; ; )                                                              // every milisecond (RTOS time tick)
    { vTaskDelay(1);                                                        // wait for the next time tick (but apparently it can wait more than one OS tick)
      TickType_t NewTick = xTaskGetTickCount();
      TickType_t Delta = NewTick-RefTick; RefTick=NewTick;
#ifdef WITH_GPS_PPS
      if(GPS_PPS_isOn()) { if(!PPS) { PPS=1; GPS_PPS_On();  } }             // monitor GPS PPS signal
                    else { if( PPS) { PPS=0; GPS_PPS_Off(); } }             // and call handling calls
#endif
      LineIdle+=Delta;                                                      // count idle time
      NoValidData+=Delta;                                                   // count time without any valid NMEA nor UBX packet
      for( ; ; )
      { uint8_t Byte; int Err=GPS_UART_Read(Byte); if(Err<=0) break;        // get Byte from serial port
        LineIdle=0;                                                         // if there was a byte: restart idle counting
        NMEA.ProcessByte(Byte); UBX.ProcessByte(Byte);                      // process through the NMEA interpreter
        if(NMEA.isComplete())                                               // NMEA completely received ?
        { if(NMEA.isChecked()) { GPS_NMEA(); NoValidData=0; }               // NMEA check sum is correct ?
          NMEA.Clear(); }
        if(UBX.isComplete()) { GPS_UBX(); NoValidData=0; UBX.Clear(); }
      }

      if(LineIdle==0)                                                        // if any bytes were received ?
      { if(Burst==0) GPS_BurstStart();                                       // burst started
        Burst=1; }
      else if(LineIdle>=10)                                                  // if GPS sends no more data for 10 time ticks
      { if(Burst>0)                                                          // if still in burst
        { GPS_BurstEnd();                                                    // burst just ended
        } else if(LineIdle>=1000)                                            // if idle for more than 1 sec
	{ GPS_Status.Flags=0; }
        Burst=0;
      }

      if(NoValidData>=2000)                                                  // if no valid data from GPS for 2sec, then decide to switch the baudrate
      { GPS_Status.Flags=0;
        uint32_t NewBaudRate = GPS_nextBaudRate();
        xSemaphoreTake(CONS_Mutex, portMAX_DELAY);
        Format_String(CONS_UART_Write, "TaskGPS: switch to ");
        Format_UnsDec(CONS_UART_Write, NewBaudRate);
        Format_String(CONS_UART_Write, "bps\n");
        xSemaphoreGive(CONS_Mutex);
        GPS_UART_SetBaudrate(NewBaudRate);
        NoValidData=0; }

    }
  }
}


