#include <stdint.h>
#include <stdlib.h>

#include "stm32f10x_rcc.h"
#include "stm32f10x_gpio.h"

#include "nmea.h"
#include "ubx.h"

#include "uart1.h"
#include "uart2.h"

#include "parameters.h"

#include "gps.h"
#include "ctrl.h"

#include "knob.h"

// ----------------------------------------------------------------------------

#include "main.h"

inline void GPS_DISABLE(void) { GPIO_ResetBits(GPIOA, GPIO_Pin_0); }
inline void GPS_ENABLE (void) { GPIO_SetBits  (GPIOA, GPIO_Pin_0); }

inline int GPS_PPS_isOn(void) { return GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_1) != Bit_RESET; }

void GPS_Configuration (void)
{
  GPIO_InitTypeDef  GPIO_InitStructure;

  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);

  GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_0;        // Configure PA.00 as output: GPS Enable(HIGH) / Shutdown(LOW)
  GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_Out_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIO_Init(GPIOA, &GPIO_InitStructure);

  GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_1;         // Configure PA.01 as input: PPS from GPS
  GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_IPD;
  GPIO_Init(GPIOA, &GPIO_InitStructure);

  GPS_ENABLE(); }

// void Debug_Print(uint8_t Byte) { while(!UART1_TxEmpty()) taskYIELD(); UART1_TxChar(Byte); }

xQueueHandle xQueuePacket;

static NMEA_RxMsg  NMEA;               // NMEA sentences catcher
static UBX_RxMsg   UBX;                // UBX messages catcher

static OgnPosition Position[4];        // four GPS position pipe
static uint8_t     PosIdx;

static OGN_Packet  Packet[2];          // two OGN packet pipe
static uint8_t     PktIdx;

static TickType_t Burst_TickCount;     // [msec] TickCount when the data burst from GPS started

         TickType_t PPS_TickCount;       // [msec] TickCount of the most recent PPS pulse
static   uint32_t   GPS_TimeSinceMSG;    // [msec] time since the last GPS message received
         uint32_t   GPS_TimeSinceLock;   // [sec] time since the GPS has a lock
volatile  uint8_t   GPS_Sec=0;           // [sec] UTC time: second
         uint32_t   GPS_UnixTime=0;      // [sec] UTC date/time in Unix format
         uint32_t   GPS_FatTime=0;       // [sec] UTC date/time in FAT format
          int32_t   GPS_Altitude=0;      // [0.1m] last valid altitude
          int16_t   GPS_GeoidSepar=0;    // [0.1m]

// static char Line[64];                  // for console output formating

uint16_t PPS_Phase(void)
{ TickType_t Phase=xTaskGetTickCount()-PPS_TickCount;
  if(Phase<1000) return Phase;
  return Phase%1000; }          // [0...999 ms] current time-phase in respect to PPS

// ----------------------------------------------------------------------------

static void GPS_PPS_On(void)                        // called on rising edge of PPS
{ LED_PCB_Flash(50);
  PPS_TickCount=xTaskGetTickCount();
  uint8_t Sec=GPS_Sec; Sec++; if(Sec>=60) Sec=0; GPS_Sec=Sec;
  GPS_UnixTime++; }

static void GPS_PPS_Off(void)                       // called on falling edge of PPS
{ }

// ----------------------------------------------------------------------------

static void GPS_LockStart(void)                     // called when GPS catches a lock
{
#ifdef WITH_BEEPER
  if(KNOB_Tick)
  { Play(0x50, 100);
    Play(0x10, 100);
    Play(0x52, 100);
    Play(0x12, 100); }
#endif
}

static void GPS_LockEnd(void)                       // called when GPS looses a lock
{
#ifdef WITH_BEEPER
  if(KNOB_Tick)
  { Play(0x52, 100);
    Play(0x12, 100);
    Play(0x50, 100);
    Play(0x10, 100); }
#endif
}

// ----------------------------------------------------------------------------

const OgnPosition& GetGPSStatus()
{
  // after 10 sec rx timeout, return empty data (reseting -1 also solves GPS_TimeSinceMSG overflow)
  if (GPS_TimeSinceMSG > 10*1000)
    Position[(PosIdx-1)&3].Clear();

  return Position[(PosIdx-1)&3];
}


static void GPS_BurstStart(void)                                           // when GPS starts sending the data on the serial port
{ Burst_TickCount=xTaskGetTickCount(); }

static void GPS_BurstEnd(void)                                             // when GPS stops sending data on the serial port
{ if(Position[PosIdx].isComplete())                                        // position data complete
  { if(Position[PosIdx].isTimeValid())
    { GPS_Sec=Position[PosIdx].Sec;
      if(Position[PosIdx].isDateValid())
      { GPS_UnixTime=Position[PosIdx].getUnixTime(); GPS_FatTime=Position[PosIdx].getFatTime(); }
    }
    if(Position[PosIdx].isValid())                                         // position is complete and locked
    { GPS_TimeSinceLock++;
      GPS_Altitude=Position[PosIdx].Altitude;
      GPS_GeoidSepar=Position[PosIdx].GeoidSeparation;
      if(GPS_TimeSinceLock==1)
      { GPS_LockStart(); }
      if(GPS_TimeSinceLock>2)
      { uint8_t PrevIdx=(PosIdx+2)&3;
        int Delta=Position[PosIdx].calcDifferences(Position[PrevIdx]);
        Packet[PktIdx].setAddress(Parameters.getAddress());                     // prepare the packet
        Packet[PktIdx].setAddrType(Parameters.getAddrType());
        Packet[PktIdx].clrOther(); Packet[PktIdx].calcAddrParity();
        Packet[PktIdx].clrEmergency(); Packet[PktIdx].clrEncrypted(); Packet[PktIdx].setRelayCount(0);
        Position[PosIdx].Encode(Packet[PktIdx]);
        Packet[PktIdx].setStealth(Parameters.getStealth());
        Packet[PktIdx].setAcftType(Parameters.getAcftType());
        Packet[PktIdx].Whiten(); Packet[PktIdx].setFEC(); Packet[PktIdx].setReady();
        OGN_Packet *PktPtr = &Packet[PktIdx];
        xQueueSend(xQueuePacket, &PktPtr, 10);                            // send the new packet to the RF task
        PktIdx^=1; LED_PCB_Flash(100);
      }
    }
    else                                                                  // complete but not valid lock
    { if(GPS_TimeSinceLock) { GPS_LockEnd(); GPS_TimeSinceLock=0; }
    }
  }
  else
  { if(GPS_TimeSinceLock) { GPS_LockEnd(); GPS_TimeSinceLock=0; }
  }
  PosIdx=(PosIdx+1)&3; Position[PosIdx].Clear();

}

// ----------------------------------------------------------------------------

static void GPS_NMEA(void)                                                 // when GPS gets a correct NMEA sentence
{ LED_PCB_Flash(2);                                                        // Flash the LED for 2 ms
  GPS_TimeSinceMSG = 0;
  Position[PosIdx].ReadNMEA(NMEA);                                         // read position elements from NMEA
  if( NMEA.isGPRMC() || NMEA.isGPGGA() )
  { static char CRNL[3] = "\r\n";
    LogLine((char *)NMEA.Data);
    LogLine(CRNL);
    xSemaphoreTake(UART1_Mutex, portMAX_DELAY);
    Format_Bytes(UART1_Write, NMEA.Data, NMEA.Len); // UART1_Write('\r'); UART1_Write('\n');
    Format_Bytes(UART1_Write, (const uint8_t *)CRNL, 2);
    xSemaphoreGive(UART1_Mutex);
    vTaskDelay(10);  // to give time for the log to write the data before it is overwritten by the next sentence
  }
}

static void GPS_UBX(void)                                                         // when GPS gets an UBX packet
{ }

// ----------------------------------------------------------------------------

// ----------------------------------------------------------------------------

#ifdef __cplusplus
  extern "C"
#endif
void vTaskGPS(void* pvParameters)
{ xQueuePacket = xQueueCreate(2, sizeof(OGN_Packet *));
  PPS_TickCount=0;
  Burst_TickCount=0;

  // UART2_Configuration(Parameters.GPSbaud);
  // GPS_Configuration();

  vTaskDelay(5);

  xSemaphoreTake(UART1_Mutex, portMAX_DELAY);
  Format_String(UART1_Write, "TaskGPS:");
  Format_String(UART1_Write, "\n");
  xSemaphoreGive(UART1_Mutex);

  int Burst=(-1);                                                         // GPS transmission ongoing or line is idle ?
  { int LineIdle=0;                                                       // counts idle time for the GPS data
    int PPS=0;
    NMEA.Clear(); UBX.Clear();                                            // scans GPS input for NMEA and UBX frames
    for(uint8_t Idx=0; Idx<4; Idx++)
      Position[Idx].Clear();
    PosIdx=0;
    PktIdx=0;

    for( ; ; )                                                             //
    { vTaskDelay(1);                                                       // wait for the next time tick

      if(GPS_PPS_isOn()) { if(!PPS) { PPS=1; GPS_PPS_On();  } }            // monitor GPS PPS signal
                    else { if( PPS) { PPS=0; GPS_PPS_Off(); } }

      GPS_TimeSinceMSG++;
      LineIdle++;                                                           // count idle time
      for( ; ; )
      { uint8_t Byte; int Err=UART2_Read(Byte); if(Err<=0) break;           // get Byte from serial port
        LineIdle=0;                                                         // if there was a byte: restart idle counting
        NMEA.ProcessByte(Byte); UBX.ProcessByte(Byte);                      // process through the NMEA interpreter
        if(NMEA.isComplete())                                               // NMEA completely received ?
        { if(NMEA.isChecked()) GPS_NMEA();                                  // NMEA check sum is correct ?
          NMEA.Clear(); }
        if(UBX.isComplete()) { GPS_UBX(); UBX.Clear(); }
      }

      if(LineIdle==0)                                                        // if any bytes were received ?
      { if(Burst==0) GPS_BurstStart();                                       // burst started
        Burst=1; }
      else if(LineIdle>10)                                                   // if GPS sends no more data for 10 time ticks
      { if(Burst>0)                                                          // if still in burst
        { GPS_BurstEnd();                                                    // burst just ended
        } else if(LineIdle>1000)                                             // if idle for more than 1 sec
	{ }
        Burst=0;
      }

    }
  }
}


