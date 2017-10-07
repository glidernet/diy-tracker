#include <stdio.h>
#include <stdint.h>

#define NO_RTOS

#include "atmosphere.h"

Atmosphere Atm;

int main(int argc, char *argv[] )
{
  printf("StdTemperature(Altitude)\n");
  for(int32_t Altitude=0; Altitude<=200000; Altitude+=1000)                          // [0.1m]
  { int32_t Temperature = Atm.StdTemperature(Altitude);                              // [0.1degC]
    printf("%7.1f m => %+5.1f degC\n", 0.1*Altitude, 0.1*Temperature);
  }
  printf("\n");

  int32_t Pressure    = Atm.StdPressureAtSeaLevel;
  int32_t Temperature = Atm.StdTemperatureAtSeaLevel;
  printf("AltitudeDelta(PressureDelta)\n");
  for(int32_t PressureDelta=(-100000); PressureDelta<=100000; PressureDelta+=1000)    // [Pa]
  { int32_t AltitudeDelta = Atm.AltitudeDelta(PressureDelta, Pressure, Temperature);  // [Pa], [Pa], [0.1 degC] => [0.01 m]
    printf("%+7d Pa %+8.2f hPa => %+8.2f m\n", PressureDelta, 0.01*PressureDelta, 0.01*AltitudeDelta);
  }
  printf("\n");

  printf("StdAltitude(Pressure)\n");
  for( Pressure=110000; Pressure>=20000; Pressure-=1000)
  { printf("%7d Pa %8.2f hPa => %8.1f/%8.1f/%8.1f/%8.1f m\n",
       Pressure, 0.01*Pressure,
       0.1*Atm.StdAltitude(Pressure, 200),
       0.1*Atm.StdAltitude(Pressure, 100),
       0.1*Atm.StdAltitude(Pressure,  50),
       0.1*Atm.StdAltitude_float(Pressure) );
  }

  return 0; }
