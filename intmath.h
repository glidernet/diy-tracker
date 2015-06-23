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

// atan2(Y, X)
// max. result error is 1/6 degree
int16_t IntAtan2(int16_t Y, int16_t X);

// integer square root
uint32_t IntSqrt(uint32_t Inp);

// Distance = sqrt(dX*dX+dY*dY)
template <class IntType>
 IntType IntDistance(IntType dX, IntType dY) // after: http://www.flipcode.com/archives/Fast_Approximate_Distance_Functions.shtml
 { IntType min, max, approx;

   if(dX<0) dX = -dX;
   if(dY<0) dY = -dY;

   if(dX<dY) { min = dX; max = dY; }
        else { min = dY; max = dX; }

   approx = max*1007 + min*441;
   if( max < (min<<4) ) approx -= max*40;

   return (approx+512)>>10; }

#endif // of __INTMATH_H__
