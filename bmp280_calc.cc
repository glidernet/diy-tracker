#include <stdio.h>
#include <stdint.h>

#define NO_RTOS
#include "bmp280.h"

int main(int argc, char *argv[])
{ BMP280 Baro;

  Baro.DefaultCalib();
  Baro.RawTemp  = 519888;
  Baro.RawPress = 415148;

  Baro.CalcTemperature();
  Baro.CalcPressure();

  printf("Temperature = %+4.1fdegC Pressure = %3.1fPa\n", 0.1*Baro.Temperature, 0.25*Baro.Pressure);
}

