#include <math.h>
#include <stdint.h>

#ifndef __INTMATH_H__
#define __INTMATH_H__

const uint32_t SineScale=0x80000000;

// get Sine from the SineTable
// Angle is 0..255 which corresponds to <0..2*PI)
int32_t IntSine(uint8_t Angle);

// precise Sine with for 16-bit angles 2nd derivative interpolation
// max. result error is about 2.3e-7
int32_t IntSine(uint16_t Angle);

// precise Sine for 32-bit angles with 2nd derivative interpolation
// max. result error is about 2.3e-7
int32_t IntSine(uint32_t Angle);

// integer square root
uint32_t IntSqrt(uint32_t Inp);

#endif // of __INTMATH_H__
