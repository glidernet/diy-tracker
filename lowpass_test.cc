#include <stdio.h>
#include <stdlib.h>
#include <time.h>

#include "lowpass2.h"

/* 4% overshoot
class LowPass2
{ public:
   int32_t Out1, Out;
  public:
   void Set(uint32_t Out=0) { Out1=Out2=Out; }
   int32_t Process( int32_t Inp)
   { Out1 = (Inp-Out2+32*Out1+16)>>5;
     Out  = (    Out1+15*Out2+ 8)>>4;
     return Out; }

} ;
*/
/*
// possibly optimal, (almost) no overshoot
template <class Int>
class LowPass2
{ public:
   static const Int InpScale    = 16;
   static const Int IntegScale1 = 10;
   static const Int IntegScale2 = IntegScale1-2;
   Int Out1, Out;
  public:
   void Set(Int Inp=0) { Out1=Out = Inp<<InpScale; }
   Int  Process( Int Inp)
   { Inp<<=InpScale;
     Out1 = ( Inp -Out + (Out1<<IntegScale1) + (1<<(IntegScale1-1)) )>>IntegScale1;
     Out  = ( Out1-Out + (Out <<IntegScale2) + (1<<(IntegScale2-1)) )>>IntegScale2;
     return  Out ; } // return fractional result

} ;

     // Out  = (              Out1-Out  +  276*Out   + 138)/276;
*/
/*
class LowPass2
{ public:
   int32_t Out1, Out2;
  public:
   void Set(uint32_t Out=0) { Out1=Out2=Out; }
   int32_t Process( int32_t Inp)
   { Out1 = (Inp-Out2+256*Out1+128)>>8;
     Out2 = (    Out1+ 72*Out2+ 36)/73;
     return Out2; }

} ;
*/

/*
class LowPass2 // IIR bilinear ?
{ public:
   int32_t Hist1, Hist2;
  public:
   void Set(uint32_t Inp=0) { Hist1=Hist2=Inp; }
   int32_t Process( int32_t Inp)
   { int32_t Hist = (  Inp  + (Hist1<<5)  -  Hist2 + 16)>>5;
     int32_t Out  = ( Hist  + (Hist1<<2)  +  Hist2 +  2)>>2;
     Hist2 = Hist1; Hist1 = Hist; return Out; }

} ;
*/

int main(int argc, char *argv[])
{ LowPass2<int64_t,10,9,16> LowPass;

  LowPass.Set(0);

  time_t Now; time(&Now); srand(Now);

  for(int T=(-100); T<20000; T++)
  { int32_t Inp = (T>=0) ? 1000000:0;
    //      Inp+= (rand()%33)-16;
    int64_t Out = LowPass.Process(Inp)>>16;
    printf("%+8d: %+10d %+15.3f %c %c\n",
            T, Inp, (double)LowPass.Out/(1<<16), (Out>=500000)?'=':' ', (Out>=900000)?'^':' ' );
  }

  return 0; }
