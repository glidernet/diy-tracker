#include "FreeRTOS.h"
#include "task.h"

#include "bmp180.h"

#include "stm32f10x_i2c.h"

#define BMP180_BUS I2C1

const uint8_t BMP180_ADDR         = 0x77; // BMP180 I2C address
const uint8_t BMP180_REG_CALIB    = 0xAA; // calibration register: 11x16bit (MSB first)
const uint8_t BMP180_REG_ID       = 0xD0; // ID register: always reads 0x55
const uint8_t BMP180_REG_RESET    = 0xE0; // write 0xB6 to perform soft-reset
const uint8_t BMP180_REG_MEAS     = 0xF4; // measurement control: SSCMMMMM SS=oversampling, C=conversion, MMMMM=mode
const uint8_t BMP180_MEAS_TEMP    = 0x2E; // measure temperature
const uint8_t BMP180_MEAS_PRESS   = 0x34; // measure pressure
const uint8_t BMP180_MEAS_BUSY    = 0x20; // measurement-busy flag
const uint8_t BMP180_REG_ADC      = 0xF6; // ADC result: 2 or 3 bytes
const uint8_t BMP180_REG_ADC_MSB  = 0xF6; // ADC result: MSB
const uint8_t BMP180_REG_ADC_LSB  = 0xF7; // ADC result: LSB
const uint8_t BMP180_REG_ADC_XLSB = 0xF8; // ADC result: more LSB

static        uint16_t BMP180_SwapBytes(uint16_t Word) { return (Word>>8) | (Word<<8); }
static inline uint16_t BMP180_SwapBytes( int16_t Word) { return BMP180_SwapBytes((uint16_t)Word); }
static void BMP180_SwapBytes(uint16_t *Word, uint8_t Bytes)
{ uint8_t Words=Bytes>>1;
  for(uint8_t Idx=0; Idx<Words; Idx++)
    Word[Idx]=BMP180_SwapBytes(Word[Idx]);
}

uint8_t BMP180_CheckID(void)
{ uint8_t ID, Err;
  Err=I2C_Read(BMP180_BUS, BMP180_ADDR, BMP180_REG_ID, ID); if(Err) return Err;
  // Err=I2C_Read(BMP180_BUS, BMP180_ADDR, BMP180_REG_ID, &ID, 1); if(Err) return Err;
  return ID!=0x55; }

uint8_t BMP180_ReadRawTemperature(int16_t &RawTemp)
{ uint8_t Err;
  Err=I2C_Write(BMP180_BUS, BMP180_ADDR, BMP180_REG_MEAS, BMP180_MEAS_TEMP); if(Err) return Err;
  for( uint8_t Timeout=8; ; )
  { vTaskDelay(1);
    uint8_t MeasCtrl;
    Err=I2C_Read(BMP180_BUS, BMP180_ADDR, BMP180_REG_MEAS, MeasCtrl); if(Err) return Err;
    if((MeasCtrl&BMP180_MEAS_BUSY)==0) break;
    Timeout--; if(Timeout==0) return 0xFF; }
  Err=I2C_Read(BMP180_BUS, BMP180_ADDR, BMP180_REG_ADC, RawTemp); if(Err) return Err;
  RawTemp=BMP180_SwapBytes(RawTemp); return 0; }

uint8_t BMP180_ReadRawPressure(uint16_t &RawPressure)
{
  return 0; }

uint8_t BMP180_ReadCalib(BMP180_Calib &Calib)
{ uint8_t Err=I2C_Read(BMP180_BUS, BMP180_ADDR, BMP180_REG_CALIB, Calib); if(Err) return Err;
  BMP180_SwapBytes((uint16_t *)(&Calib), sizeof(Calib) ); return Err; }
