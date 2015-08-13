#include <stdio.h>

#define NO_RTOS

#include "bmp180.h"

BMP180 Baro;

const int32_t Alt300  = 91652;
const int32_t Alt400  = 71864;
const int32_t Alt800  = 19493;
const int32_t Alt900  =  9886;
const int32_t Alt1000 =  1109;

int32_t StdAlt(int32_t Press)
{ int32_t Alt = (Alt800*(int64_t)(100000-Press) + Alt1000*(int64_t)(Press-80000) )/(int64_t)(20000);
          Alt -= ( (int64_t)(100000-Press)*(Press-80000) * 415 ) /((int64_t)10000*10000);
  return Alt; }

int main(int argc, char *argv[])
{ Baro.Temperature  = 235;     // [0.1degC]
  Baro.Pressure     = 96055;   // [Pa]
  Baro.PrevPressure = 96050;   // [Pa]
  // Baro.CalcStdAltitude();

  char NMEA[64];
  uint8_t Len=Baro.WriteNMEA(NMEA);
  NMEA[Len]=0;
  printf("%s", NMEA);

  for(uint32_t P=20000; P<=110000; P+=1000)
  { Baro.Pressure=P; Baro.CalcStdAltitude();
    int32_t Est = StdAlt(P);
    printf("%7.2f mBar: %7.1f  %7.1f  %+4.1f m\n", 0.01*P, 0.1*Baro.StdAltitude, 0.1*Est, 0.1*(Est-Baro.StdAltitude) );
  }

  return 0; }
