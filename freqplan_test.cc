#include <stdio.h>
#include <time.h>

#include "freqplan.h"

int main(int argc, char *argv[])
{ time_t Now; time(&Now);

  FreqPlan Plan;
  Plan.setPlan(2);

  for(int Ofs=0; Ofs<10000; Ofs++)
  { uint32_t Time = Now+Ofs;
    uint8_t FLR1 = Plan.getChannel(Time, 0, 0);
    uint8_t OGN1 = Plan.getChannel(Time, 0, 1);
    uint8_t FLR2 = Plan.getChannel(Time, 1, 0);
    uint8_t OGN2 = Plan.getChannel(Time, 1, 1);
    printf("%10d: [%02d, %02d] [%02d, %02d] %+2d %+2d\n", Time, FLR1, FLR2, OGN1, OGN2, (int)(OGN1-FLR1), (int)(OGN2-OGN1) );
  }

  return 0; }

