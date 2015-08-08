#include "FreeRTOS.h"
#include "task.h"

#include "parameters.h"

#include "sens.h"

#include "uart1.h"
#include "main.h"
#include "ctrl.h"
#include "gps.h"
#include "knob.h"

#include "i2c.h"

#ifdef WITH_BMP180

#include "bmp180.h"
#include "atmosphere.h"
#include "slope.h"
#include "lowpass2.h"
#include "intmath.h"

#include "fifo.h"

// static const uint8_t  VarioVolume     =    2; // [0..3]
static const uint16_t VarioBasePeriod = 800;  // [ms]

#ifdef WITH_BEEPER
void VarioSound(int32_t ClimbRate)
{
  uint8_t VarioVolume = KNOB_Tick>>1; if(VarioVolume>3) VarioVolume=3;  // take vario volume from the user knob
  if(ClimbRate>=50)                                                     // if climb > 1/2 m/s
  { uint8_t Note=(ClimbRate-50)/50;                                     // one semitone per 1/2 m/s
    if(Note>=0x0F) Note=0x0F;                                           // stop at 15 thus 8 m/s
    uint16_t Period=(VarioBasePeriod+(Note>>1))/(1+Note);               // beeping period
    Vario_Period=Period; Vario_Fill=Period>>1;                          // period shorter (faster beeping) with stronger climb
    Vario_Note = (VarioVolume<<6) | (0x10+Note); }                      // note to play: higher for stronger climb
  else if(ClimbRate<=(-100))                                            // if sink > 1 m/s
  { uint8_t Note=(-ClimbRate-100)/100;                                  // one semitone per 1 m/s
    if(Note>=0x0B) Note=0x0B;                                           //
    Vario_Period=VarioBasePeriod; Vario_Fill=VarioBasePeriod;           // continues tone
    Vario_Note = (VarioVolume<<6) | (0x0B-Note); }                      // note to play: lower for stronger sink
  else                                                                  // if climb less than 1/2 m/s or sink less than 1 m/s
  { Vario_Note=0x00; }                                                  // no sound
}
#endif // WITH_BEEPER

static BMP180   Baro;                       // BMP180 barometer sensor

static uint32_t AverPress;                  // [ Pa] summed Pressure over several readouts
static uint8_t  AverCount;                  // [int] number of summed Pressure readouts

static SlopePipe<int32_t>        BaroPipe;  // 4-point slope-fit pipe for pressure
static LowPass2<int32_t,6,4,8>   BaroNoise; // low pass (average) filter for pressure noise

static LowPass2<int64_t,10,9,12> PressAver, // low pass (average) filter for pressure
                                 AltAver;   // low pass (average) filter for GPS altitude

static Delay<int32_t, 8>        PressDelay; // 4-second delay for long-term climb rate

static char Line[64];                       // line to prepare the barometer NMEA sentence


static bool InitBaro()
{

// #ifdef WITH_BEEPER
//   VarioSound(0);
// #endif

  Baro.Bus=I2C1;
  BaroPipe.Clear  (90000);
  BaroNoise.Set(3*16);                 // guess the pressure noise level

                                       // initialize the GPS<->Baro correlator
  AltAver.Set(0);                      // [m] Altitude at sea level
  PressAver.Set(4*101300);             // [Pa] Pressure at sea level
  PressDelay.Clear(4*101300);

  uint8_t Err=Baro.CheckID();
  if(Err==0) Err=Baro.ReadCalib();
  if(Err==0) Err=Baro.AcquireRawTemperature();
  if(Err==0) { Baro.CalcTemperature(); AverPress=0; AverCount=0; }
  return Err==0;
}

static void ProcBaro()
{
    static uint8_t PipeCount=0;

    int16_t Sec  = 10*GPS_Sec;                                            // [0.1sec]
    uint16_t Phase = PPS_Phase();                                         // sync to the GPS PPS
    if(Phase>=500) { Sec+=10; vTaskDelay(1000-Phase); }
              else { Sec+= 5; vTaskDelay( 500-Phase); }
    if(Sec>=600) Sec-=600;

    TickType_t Start=xTaskGetTickCount();
    uint8_t Err=Baro.AcquireRawTemperature();                             // measure temperature
    if(Err==0) { Baro.CalcTemperature(); AverPress=0; AverCount=0; }      // clear the average
          else { PipeCount=0; return; }

    for(uint8_t Idx=0; Idx<24; Idx++)
    { uint8_t Err=Baro.AcquireRawPressure();                              // take pressure measurement
      if(Err==0) { Baro.CalcPressure(); AverPress+=Baro.Pressure; AverCount++; } // sum-up average pressure
      TickType_t Time=xTaskGetTickCount()-Start; if(Time>=450) break; }   // but no longer than 450ms to fit into 0.5 second slot

    if(AverCount)                                                         // and we summed-up some measurements
    { AverPress = (AverPress+(AverCount>>1))/AverCount;                   // make the average

      BaroPipe.Input(AverPress);                                          // [Pa]
      if(PipeCount<255) PipeCount++;                                      // count data going to the slope fitting pipe
      if(PipeCount>=4)
      { BaroPipe.FitSlope();                                             // fit the average and slope from the four most recent pressure points
        int32_t PLR = Atmosphere::PressureLapseRate(AverPress, Baro.Temperature); // [0.0001m/Pa]
        int32_t ClimbRate = (BaroPipe.Slope*PLR)/200;                             // [0.25Pa/0.5sec] * [0.0001m/Pa] x 200 => [0.01m/sec]

        BaroPipe.CalcNoise();                                            // calculate the noise (average square residue)
        uint32_t Noise=BaroNoise.Process(BaroPipe.Noise);                // pass the noise through the low pass filter
                 Noise=(IntSqrt(100*Noise)+32)>>6;                       // [0.1 Pa] noise (RMS) measured on the pressure

        int32_t Pressure=BaroPipe.Aver;                                  // [0.25 Pa]
        int32_t StdAltitude = Atmosphere::StdAltitude((Pressure+2)>>2);  // [0.1 m]
        int32_t ClimbRate4sec = ((Pressure-PressDelay.Input(Pressure))*PLR)/800; // [0.01m/sec] climb rate over 4 sec.
#ifdef WITH_BEEPER
        VarioSound(ClimbRate);
        // if(abs(ClimbRate4sec)>50) VarioSound(ClimbRate);
	//                      else VarioSound(2*ClimbRate4sec);
#endif
        if( (Phase>=500) && GPS_TimeSinceLock)
        { PressAver.Process(Pressure);                                   // [0.25 Pa] pass pressure through low pass filter
          AltAver.Process(GPS_Altitude);                                 // [0.1 m] pass GPS altitude through same low pass filter
        }
        int32_t PressDiff=Pressure-((PressAver.Out+2048)>>12);           // [0.25 Pa] pressure - <pressure>
        int32_t AltDiff = (PressDiff*(PLR>>4))/250;                      // [0.1 m]
        int32_t Altitude=((AltAver.Out+2048)>>12)+AltDiff;               // [0.1 m]

        uint8_t Frac = Sec%10;
        if(Frac==0)
        { OgnPosition *PosPtr = GPS_getPosition(Sec/10);
          PosPtr->StdAltitude = StdAltitude;
          PosPtr->Temperature = Baro.Temperature;
          PosPtr->setBaro(); }

        uint8_t Len=0;                                                   // start preparing the barometer NMEA sentence
        Len+=Format_String(Line+Len, "$POGNB,");
        Len+=Format_UnsDec(Line+Len, Sec, 3, 1);                         // [sec] measurement time
        Line[Len++]=',';
        Len+=Format_SignDec(Line+Len, Baro.Temperature, 2, 1);           // [degC] temperature
        Line[Len++]=',';
        Len+=Format_UnsDec(Line+Len, (uint32_t)(10*Pressure+2)>>2, 2, 1); // [Pa] pressure
        Line[Len++]=',';
        Len+=Format_UnsDec(Line+Len, Noise, 2, 1);                       // [Pa] pressure noise
        Line[Len++]=',';
        Len+=Format_SignDec(Line+Len, StdAltitude, 2, 1);                // [m] standard altitude (calc. from pressure)
        Line[Len++]=',';
        Len+=Format_SignDec(Line+Len, Altitude,    2, 1);                // [m] altitude (from cross-calc. with the GPS)
        Line[Len++]=',';
        Len+=Format_SignDec(Line+Len, ClimbRate,   3, 2);                // [m/s] climb rate
        Line[Len++]=',';
        Len+=NMEA_AppendCheckCRNL(Line, Len);
        LogLine(Line);                                                   // send NMEA sentence to the LOG
        xSemaphoreTake(UART1_Mutex, portMAX_DELAY);
        Format_String(UART1_Write, Line, Len);                           // send NMEA sentence to the console (UART1)
        xSemaphoreGive(UART1_Mutex);
      }

    } else PipeCount=0;
}

#endif // WITH_BMP

extern "C"
void vTaskSENS(void* pvParameters)
{ vTaskDelay(20);   // this delay seems to be essential - if you don't wait long enough, the BMP180 won't respond properly.

#ifdef WITH_BMP180
  bool withBaro = InitBaro();
#else
  bool withBaro = false;
#endif

  xSemaphoreTake(UART1_Mutex, portMAX_DELAY);
  Format_String(UART1_Write, "TaskSENS:");
  if(withBaro) Format_String(UART1_Write, " BMP180");
  Format_String(UART1_Write, "\n");
  xSemaphoreGive(UART1_Mutex);

  while(1)
  {
#ifdef WITH_BMP180
    ProcBaro();
#else
    vTaskDelay(1000);
#endif
  }
}

