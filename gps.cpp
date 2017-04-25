#include <stdint.h>
#include <stdlib.h>

#include "stm32f10x.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_exti.h"
#include "stm32f10x_rcc.h"
#include "misc.h"

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

#ifdef WITH_GPS_ENABLE
// PA0 is GPS enable
inline void GPS_DISABLE(void) { GPIO_ResetBits(GPIOA, GPIO_Pin_0); }
inline void GPS_ENABLE (void) { GPIO_SetBits  (GPIOA, GPIO_Pin_0); }
#endif

#ifdef WITH_GPS_PPS
// PA1 is GPS PPS
inline int GPS_PPS_isOn(void) { return GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_1) != Bit_RESET; }
#endif

#ifdef WITH_PPS_IRQ
TickType_t PPS_IRQ_TickCount;                       // [RTOS tick] (coarse) time of previous PPS
uint32_t PPS_IRQ_TickTime;                          // [CPU tick] (fine) time of previous PPS
 int32_t PPS_IRQ_TickTimeDiff;                      // [CPU tick] diff. of (fine) times between PPS'es
uint32_t PPS_IRQ_InitLoad;                          // [CPU tick] initial period of SysTick
uint32_t PPS_IRQ_InitPeriod;
uint32_t PPS_IRQ_Period;                            // [CPU tick] time between two PPS'es
LowPass2<int32_t,6,4,8> PPS_IRQ_AverPeriod;
#endif

void GPS_Configuration (void)
{
#ifdef WITH_PPS_IRQ
  NVIC_InitTypeDef NVIC_InitStructure;
  // NVIC_InitTypeDef NVIC_InitStructure = { .NVIC_IRQChannel = EXTI1_IRQn, .NVIC_IRQChannelPreemptionPriority = 1, .NVIC_IRQChannelSubPriority = 1, .NVIC_IRQChannelCmd = ENABLE };

  NVIC_InitStructure.NVIC_IRQChannel = EXTI1_IRQn;                  // Enable the USART1 Interrupt
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;         // 0 = highest, 15 = lowest priority
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;

  NVIC_Init(&NVIC_InitStructure);

#endif

  GPIO_InitTypeDef  GPIO_InitStructure;

  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);

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
  // we want interrupt on rising PPS from GPS
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
#ifdef __cplusplus
  extern "C"
#endif
void EXTI1_IRQHandler(void)               // PPS interrupt
{
  if(EXTI_GetITStatus(EXTI_Line1) != RESET)
  { uint32_t Load=getSysTick_Reload();                             // [CPU tick] period of the SysTick - 1
    uint32_t TickTime=Load-getSysTick_Count();                     // [CPU tick] what time after RTOS tick the PPS arrived
    TickType_t TickCount = xTaskGetTickCountFromISR();             // [RTOS tick] RTOS tick counter
    TickType_t TickSincePPS = TickCount-PPS_IRQ_TickCount;         // [RTOS tick] how many RTOS ticks after the previos PPS
    PPS_IRQ_TickCount=TickCount;                                   //
    if((TickSincePPS>900)&&(TickSincePPS<5100))                    // if this PPS between 0.9sec and 5.1sec after the previous PPS
    { uint32_t SecsSincePPS=(TickSincePPS+500)/1000;               // [sec] number of seconds since previous PPS
       int32_t TickTimeDiff=TickTime-PPS_IRQ_TickTime;             // [CPU tick] this is zero if the Xtal is exactly on frequency
      if(SecsSincePPS==1) { PPS_IRQ_Period=TickSincePPS*(Load+1)+TickTimeDiff; PPS_IRQ_AverPeriod.Process(PPS_IRQ_Period-PPS_IRQ_InitPeriod); }
      if(SecsSincePPS>1) TickTimeDiff/=SecsSincePPS;
      PPS_IRQ_TickTimeDiff=TickTimeDiff;
      int32_t LoadIncr=0;
           if (TickTimeDiff >    500 ) LoadIncr++;
      else if (TickTimeDiff < ( -500)) LoadIncr--;
      // if(LoadIncr==0)
      { int32_t TickPhaseDiff = TickTime-(Load>>1);
             if (TickPhaseDiff>   1000 ) LoadIncr++;
        else if (TickPhaseDiff< (-1000)) LoadIncr--;
      }
      if(LoadIncr!=0)
      { Load+=LoadIncr;
        if(abs((int32_t)(Load-PPS_IRQ_InitLoad))<8) setSysTick_Reload(Load);
      }
    }
    else
    {
    }
    PPS_IRQ_TickTime=TickTime;
  }
  EXTI_ClearITPendingBit(EXTI_Line1);
}
#endif

// void Debug_Print(uint8_t Byte) { while(!UART1_TxEmpty()) taskYIELD(); UART1_TxChar(Byte); }

static NMEA_RxMsg  NMEA;                 // NMEA sentences catcher
static UBX_RxMsg   UBX;                  // UBX messages catcher

static GPS_Position Position[4];         // four GPS position pipe
static uint8_t     PosIdx;

static TickType_t Burst_TickCount;       // [msec] TickCount when the data burst from GPS started

         TickType_t PPS_TickCount;       // [msec] TickCount of the most recent PPS pulse
         uint32_t   GPS_TimeSinceLock;   // [sec] time since the GPS has a lock
volatile  uint8_t   GPS_Sec=0;           // [sec] UTC time: second

         uint32_t   GPS_UnixTime=0;      // [sec] UTC date/time in Unix format
         uint32_t   GPS_FatTime=0;       // [sec] UTC date/time in FAT format
          int32_t   GPS_Altitude=0;      // [0.1m] last valid altitude
          int32_t   GPS_Latitude=0;
          int32_t   GPS_Longitude=0;
          int16_t   GPS_GeoidSepar=0;    // [0.1m]
         uint16_t   GPS_LatCosine=0x3000;  //
//          uint8_t   GPS_FreqPlan=0;      //

// static char Line[64];                  // for console output formating

uint16_t PPS_Phase(void)                              //
{ TickType_t Phase=xTaskGetTickCount()-PPS_TickCount; // time after last PPS pulse ?
  if(Phase<1000) return Phase;                        // is less than 1sec then return it directly
  return Phase%1000; }                                // [0...999 ms] current time-phase in respect to PPS

// ----------------------------------------------------------------------------

static void GPS_PPS_On(uint8_t Delay=0)               // called on rising edge of PPS
{ LED_PCB_Flash(50);
  PPS_TickCount=xTaskGetTickCount()-Delay;
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

const uint8_t GPS_BurstDelay=70;                                           // [ms] time after the PPS when the data burst starts on the UART

static void GPS_BurstStart(void)                                           // when GPS starts sending the data on the serial port
{ Burst_TickCount=xTaskGetTickCount();
  // TickType_t TicksSincePPS=Burst_TickCount-PPS_TickCount;
  // if(TicksSincePPS>10500)
#ifndef WITH_GPS_PPS
  GPS_PPS_On(GPS_BurstDelay);                                              // if there is no PPS from GPS use the burst start to simulate the PPS with given delay.
#endif
}

static void GPS_BurstEnd(void)                                             // when GPS stops sending data on the serial port
{ if(Position[PosIdx].isComplete())                                        // position data complete
  { if(Position[PosIdx].isTimeValid())
    { GPS_Sec=Position[PosIdx].Sec;                                        // get the second - this is important for time in the OGN packets
      if(Position[PosIdx].isDateValid())
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
        /* int Delta = */ Position[PosIdx].calcDifferences(Position[PrevIdx]);
        LED_PCB_Flash(100); }
    }
    else                                                                  // complete but not valid lock
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

GPS_Position *GPS_getPosition(void)
{ uint8_t PrevIdx=PosIdx;
  GPS_Position *PrevPos = Position+PrevIdx;
  if(PrevPos->isComplete()) return PrevPos;
  PrevIdx=(PrevIdx+3)&3;
  PrevPos = Position+PrevIdx;
  if(PrevPos->isComplete()) return PrevPos;
  return 0; }

GPS_Position *GPS_getPosition(int8_t Sec)
{ for(uint8_t Idx=0; Idx<4; Idx++)
  { if(Sec==Position[Idx].Sec) return Position+Idx; }
  return 0; }

// ----------------------------------------------------------------------------

static void GPS_NMEA(void)                                                 // when GPS gets a correct NMEA sentence
{ LED_PCB_Flash(2);                                                        // Flash the LED for 2 ms
  Position[PosIdx].ReadNMEA(NMEA);                                         // read position elements from NMEA
  if( NMEA.isGxRMC() || NMEA.isGxGGA() || NMEA.isGxGSA() || NMEA.isGPTXT() )
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

static void GPS_UBX(void)                                                         // when GPS gets an UBX packet
{ }

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

// ----------------------------------------------------------------------------

static const uint8_t BaudRates=7;
static       uint8_t BaudRateIdx=1;
static const uint32_t BaudRate[BaudRates] = { 4800, 9600, 19200, 38400, 57600, 115200, 230400 } ;

#ifdef __cplusplus
  extern "C"
#endif
void vTaskGPS(void* pvParameters)
{
#ifdef WITH_PPS_IRQ
  PPS_IRQ_InitLoad = getSysTick_Reload();
  PPS_IRQ_InitPeriod = 1000*(PPS_IRQ_InitLoad+1);
  PPS_IRQ_Period = PPS_IRQ_InitPeriod;
  PPS_IRQ_AverPeriod.Set(0);
#endif

  PPS_TickCount=0;
  Burst_TickCount=0;

  vTaskDelay(5);

  xSemaphoreTake(CONS_Mutex, portMAX_DELAY);
  Format_String(CONS_UART_Write, "TaskGPS:");
  Format_String(CONS_UART_Write, "\n");
  xSemaphoreGive(CONS_Mutex);

  // Format_String(GPS_UART_Write, MTK_SetBaudrate_115200);

  int Burst=(-1);                                                         // GPS transmission ongoing or line is idle ?
  { int LineIdle=0;                                                       // counts idle time for the GPS data
    int NoValidData=0;                                                    // count time without valid data (to decide to change baudrate)
    int PPS=0;
    NMEA.Clear(); UBX.Clear();                                            // scans GPS input for NMEA and UBX frames
    for(uint8_t Idx=0; Idx<4; Idx++)
      Position[Idx].Clear();
    PosIdx=0;
    // PktIdx=0;

    for( ; ; )                                                             // every milisecond (RTOS time tick)
    { vTaskDelay(1);                                                       // wait for the next time tick
#ifdef WITH_GPS_PPS
      if(GPS_PPS_isOn()) { if(!PPS) { PPS=1; GPS_PPS_On();  } }            // monitor GPS PPS signal
                    else { if( PPS) { PPS=0; GPS_PPS_Off(); } }            // and call handling calls
#endif
      LineIdle++;                                                           // count idle time
      for( ; ; )
      { uint8_t Byte; int Err=GPS_UART_Read(Byte); if(Err<=0) break;        // get Byte from serial port
        LineIdle=0;                                                         // if there was a byte: restart idle counting
        NoValidData++;
        NMEA.ProcessByte(Byte); UBX.ProcessByte(Byte);                      // process through the NMEA interpreter
        if(NMEA.isComplete())                                               // NMEA completely received ?
        { if(NMEA.isChecked()) { GPS_NMEA(); NoValidData=0; }               // NMEA check sum is correct ?
          NMEA.Clear(); }
        if(UBX.isComplete()) { GPS_UBX(); NoValidData=0; UBX.Clear(); }
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
#ifdef WITH_GPS_AUTOBAUD
      if(NoValidData>1000)                                                   // decide to switch the baudrate
      { BaudRateIdx++; if(BaudRateIdx>=BaudRates) BaudRateIdx=0;
        uint32_t NewBaudRate = BaudRate[BaudRateIdx];
        xSemaphoreTake(CONS_Mutex, portMAX_DELAY);
        Format_String(CONS_UART_Write, "TaskGPS: switch to ");
        Format_UnsDec(CONS_UART_Write, NewBaudRate);
        Format_String(CONS_UART_Write, "bps\n");
        xSemaphoreGive(CONS_Mutex);
        GPS_UART_SetBaudrate(NewBaudRate);
        NoValidData=0; }
#endif
    }
  }
}


