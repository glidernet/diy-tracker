#include <stdio.h>
#include <stdint.h>

class BMP180
{ private:
   static const uint8_t MEAS_OSS     = 0x00; // oversampling factor: 0..3

  private:
   int16_t AC1, AC2, AC3;  // 11 calibration values from EEPROM
  uint16_t AC4, AC5, AC6;
   int16_t  B1,  B2;
   int16_t  MB,  MC,  MD;
   int32_t  B5;            // temperature compensation for pressure ?
  public:
    uint16_t RawTemp;
     int16_t Temperature; // [0.1 degC]
    uint32_t RawPress;
    uint32_t Pressure;    // [Pa]

 public:
  void DefaultCalib(void)
  { AC1 =    408;
    AC2 =    -72;
    AC3 = -14383;
    AC4 =  32741;
    AC5 =  32757;
    AC6 =  23153;
    B1  =   6190;
    B2  =      4;
    MB  = -32768;
    MC  =  -8711;
    MD  =   2868; }

  void CalcTemperature(void) // calculate the temperature from raw readout and calibration values
  { int32_t X1 = ( (( (int32_t)RawTemp - (int32_t)AC6 )*(int32_t)AC5) )>>15;
    printf("X1 = %d\n", X1);
    int32_t X2 = ((int32_t)MC<<11)/(X1+(int32_t)MD);
    printf("X2 = %d\n", X2);
            B5 = X1+X2;
    printf("B5 = %d\n", B5);
    Temperature = (B5+8)>>4; }

  void CalcPressure(void) // calculate the pressure - must calculate the temperature first !
  { int32_t B6 = B5-4000;
    printf("B6 = %d\n", B6);
    int32_t X1 = ((int32_t)B2*((B6*B6)>>12))>>11;
    printf("X1 = %d\n", X1);
    int32_t X2 = ((int32_t)AC2*B6)>>11;
    printf("X2 = %d\n", X2);
    int32_t X3 = X1+X2;
    printf("X3 = %d\n", X3);
    int32_t B3 = (((((int32_t)AC1<<2)+X3)<<MEAS_OSS)+2)>>2;
    printf("B3 = %d\n", B3);
            X1 = ((int32_t)AC3*B6)>>13;
    printf("X1 = %d\n", X1);
            X2 = ((int32_t)B1*((B6*B6)>>12))>>16;
    printf("X2 = %d\n", X2);
	    X3 = ((X1+X2)+2)>>2;
    printf("X3 = %d\n", X3);
   uint32_t B4 = ((uint32_t)AC4*(uint32_t)(X3+32768))>>15;
    printf("B4 = %d\n", B4);
   uint32_t B7 = (RawPress-B3)*((uint32_t)50000>>MEAS_OSS);
    printf("B7 = %d\n", B7);
    if(B7&0x8000000) { Pressure = (B7/B4)<<1; }
                else { Pressure = (B7<<1)/B4; }
            X1 = (Pressure>>8)*(Pressure>>8);
	    X1 = (X1*3038)>>16;
	    X2 = (-7357*(int32_t)Pressure)>>16;
	    Pressure += (X1+X2+3791)>>4;
   }


} ;

int main(int argc, char *argv[])
{ BMP180 Baro;

  Baro.DefaultCalib();
  Baro.RawTemp  = 27898;
  Baro.RawPress = 23843;

  Baro.CalcTemperature();
  Baro.CalcPressure();

  printf("Temperature = %d, Pressure = %d\n", Baro.Pressure, Baro.Temperature);
}

