#include <stdio.h>
#include <stdint.h>
#include <math.h>

#include "slope.h"

SlopePipe<int32_t> Slope;

int main(int argc, char *argv[])
{
  Slope.Input(10);
  Slope.Input(20);
  Slope.Input(30);
  Slope.Input(40);
  Slope.CalcSlope();
  Slope.CalcNoise();

  printf("Aver=%+5.1f, Slope=%+5.1f, Noise=%4.2f\n",
         0.25*Slope.Aver, 0.25*Slope.Slope, sqrt(0.0625*Slope.Noise));

  Slope.Input(50);
  Slope.Input(40);
  Slope.Input(30);
  Slope.Input(20);
  Slope.CalcSlope();
  Slope.CalcNoise();

  printf("Aver=%+5.1f, Slope=%+5.1f, Noise=%4.2f\n",
         0.25*Slope.Aver, 0.25*Slope.Slope, sqrt(0.0625*Slope.Noise));

  Slope.Input(10);
  Slope.Input(20);
  Slope.Input(10);
  Slope.Input(20);
  Slope.CalcSlope();
  Slope.CalcNoise();

  printf("Aver=%+5.1f, Slope=%+5.1f, Noise=%4.2f\n",
         0.25*Slope.Aver, 0.25*Slope.Slope, sqrt(0.0625*Slope.Noise));

  Slope.Input(40);
  Slope.Input(30);
  Slope.Input(40);
  Slope.Input(30);
  Slope.CalcSlope();
  Slope.CalcNoise();

  printf("Aver=%+5.1f, Slope=%+5.1f, Noise=%4.2f\n",
         0.25*Slope.Aver, 0.25*Slope.Slope, sqrt(0.0625*Slope.Noise));

  return 0; }

