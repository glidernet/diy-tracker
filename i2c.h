#ifndef __I2C_H__
#define __I2C_H__

#include "stm32f10x_i2c.h"

void I2C1_Configuration(uint32_t ClockSpeed=100000);

uint8_t I2C_Write(I2C_TypeDef *I2Cx, uint8_t Addr, uint8_t Reg, const uint8_t *Data, uint8_t Len);

uint8_t I2C_Read (I2C_TypeDef *I2Cx, uint8_t Addr, uint8_t Reg,       uint8_t *Data, uint8_t Len);

template <class Type>
 inline uint8_t I2C_Write(I2C_TypeDef *I2Cx, uint8_t Addr, uint8_t Reg, Type &Object)
{ return I2C_Write(I2Cx, Addr, Reg, (uint8_t *)&Object, sizeof(Type)); }

template <class Type>
 inline uint8_t I2C_Read(I2C_TypeDef *I2Cx, uint8_t Addr, uint8_t Reg, Type &Object)
{ return I2C_Read(I2Cx, Addr, Reg, (uint8_t *)&Object, sizeof(Type)); }

#endif // __I2C_H__
