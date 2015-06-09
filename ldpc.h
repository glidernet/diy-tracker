#ifndef __LDPC_H__
#define __LDPC_H__

#include <stdint.h>
#include <stdlib.h>

#include "bitcount.h"

#ifndef __AVR__
// #include <stdio.h>
#include <math.h>
#endif

#ifdef __AVR__
#include <avr/pgmspace.h>
#endif

#ifdef __AVR__

// encode Parity from Data: Data is 5x 32-bit words = 160 bits, Parity is 1.5x 32-bit word = 48 bits
void LDPC_Encode(const uint32_t *Data, uint32_t *Parity, const uint32_t ParityGen[48][5]);

void LDPC_Encode(const uint32_t *Data, uint32_t *Parity);

// encode Parity from Data: Data is 20 bytes = 160 bits, Parity is 6 bytes = 48 bits
void LDPC_Encode(const uint8_t *Data, uint8_t *Parity, const uint32_t ParityGen[48][5]);
void LDPC_Encode(const uint8_t *Data, uint8_t *Parity);
void LDPC_Encode(uint8_t *Data);
                                        // check Data against Parity (run 48 parity checks) - return number of failed checks
int8_t LDPC_Check(const uint8_t *Data); // 20 data bytes followed by 6 parity bytes
int8_t LDPC_Check(const uint32_t *Packet);

#else // if not 8-bit AVR

void LDPC_Encode(const uint8_t *Data, uint8_t *Parity, const uint32_t ParityGen[48][5]);
void LDPC_Encode(const uint8_t *Data, uint8_t *Parity);
void LDPC_Encode(uint8_t *Data);
                                                              // encode Parity from Data: Data is 5x 32-bit words = 160 bits, Parity is 1.5x 32-bit word = 48 bits
void LDPC_Encode(const uint32_t *Data, uint32_t *Parity, const uint32_t ParityGen[48][5]);
void LDPC_Encode(const uint32_t *Data, uint32_t *Parity);
                                                              // check Data against Parity (run 48 parity checks) - return number of failed checks
int LDPC_Check(const uint32_t *Data, const uint32_t *Parity); // Data and Parity are 32-bit words
int LDPC_Check(const uint32_t *Packet);
// int LDPC_Check(const uint8_t *Data); // Data and Parity are 8-bit bytes
int8_t LDPC_Check(const uint8_t *Data); // 20 data bytes followed by 6 parity bytes
  
#endif // __AVR__

#ifndef __AVR__

extern const uint8_t LDPC_ParityCheckIndex[48][24];

class LDPC_Decoder
{ public:
   const static uint8_t UserBits   = 160;                 // 5 32-bit bits = 20 bytes
   const static uint8_t UserWords  = UserBits/32;
   const static uint8_t ParityBits =  48;                 // 6 bytes (total packet is 26 bytes)
   const static uint8_t CodeBits   = UserBits+ParityBits; // 160+48 = 208 code bits = 26 bytes
   const static uint8_t CodeBytes  = (CodeBits+ 7)/ 8;    //
   const static uint8_t CodeWords  = (CodeBits+31)/32;    // 
   const static uint8_t MaxCheckWeight = 24;
   // const static uint8_t MaxBitWeight   =  8;

  public:

   int8_t  InpBit[CodeBits]; // a-priori bits
   int16_t OutBit[CodeBits]; // a-posteriori bits
/*
   void PrintInpBit(void)
   { printf("InpBit[%d]\n", CodeBits);
     for(uint8_t Bit=0; Bit<CodeBits; Bit++)
     { if((Bit&0xF)==0x0) printf("%03d:", Bit);
       printf(" %+5d", InpBit[Bit]);
       if((Bit&0xF)==0xF) printf("\n"); }
   }
*/
   void Input(const uint8_t *Data, uint8_t *Err)
   { uint8_t Mask=1; uint8_t Idx=0; uint8_t DataByte=0; uint8_t ErrByte=0;
     for(uint8_t Bit=0; Bit<CodeBits; Bit++)
     { if(Mask==1) { DataByte=Data[Idx];  ErrByte=Err[Idx]; }
       int8_t Inp;
       if(ErrByte&Mask) Inp=0;
                   else Inp=(DataByte&Mask) ? +32:-32;
       InpBit[Bit] = Inp;
       Mask<<=1; if(Mask==0) { Idx++; Mask=1; }
     }
   }

   void Input(const uint32_t Data[CodeWords])
   { uint32_t Mask=1; uint8_t Idx=0; uint32_t Word=Data[Idx];
     for(uint8_t Bit=0; Bit<CodeBits; Bit++)
     { InpBit[Bit] = (Word&Mask) ? +32:-32;
       Mask<<=1; if(Mask==0) { Word=Data[++Idx]; Mask=1; }
     }
   }

   void Input(const float *Data, float RefAmpl=1.0)
   { for(int Bit=0; Bit<CodeBits; Bit++)
     { int Inp = floor(32*Data[Bit^7]/RefAmpl+0.5);
       if(Inp>127) Inp=127; else if(Inp<(-127)) Inp=(-127);
       InpBit[Bit] = Inp; }
   }

   void Output(uint32_t Data[CodeWords])
   { uint32_t Mask=1; uint8_t Idx=0; uint32_t Word=0;
     for(uint8_t Bit=0; Bit<CodeBits; Bit++)
     { if(InpBit[Bit]>0) Word|=Mask;
       Mask<<=1; if(Mask==0) { Data[Idx++]=Word; Word=0; Mask=1; }
     } if(Mask>1) Data[Idx++]=Word;
   }

   void Output(uint8_t Data[CodeBytes])
   { uint8_t Mask=1; uint8_t Idx=0; uint8_t Byte=0;
     for(uint8_t Bit=0; Bit<CodeBits; Bit++)
     { if(InpBit[Bit]>0) Byte|=Mask;
       Mask<<=1; if(Mask==0) { Data[Idx++]=Byte; Byte=0; Mask=1; }
     } if(Mask>1) Data[Idx++]=Byte;
   }

   int8_t ProcessChecks(void)
   { for(uint8_t Bit=0; Bit<CodeBits; Bit++)
       OutBit[Bit]=0;
     uint8_t Count=0;
     for(uint8_t Row=0; Row<ParityBits; Row++)
     { int8_t Ret=ProcessCheck(Row); 
       if(Ret<=0) Count++; }
     // printf("%d parity checks fail\n", Count);
     if(Count==0) return 0;
     for(uint8_t Bit=0; Bit<CodeBits; Bit++)
     { int16_t Ampl = ((uint16_t)4*InpBit[Bit] + OutBit[Bit])/(uint16_t)5;
       if(Ampl<(-128)) Ampl=(-127);
       else if(Ampl>127) Ampl=127;
       InpBit[Bit] = Ampl; }
     // PrintInpBit();
     return Count; }

   int8_t ProcessCheck(uint8_t Row)
   { int8_t MinAmpl=127; uint8_t MinBit=0; int8_t MinAmpl2=127; uint32_t Word=0; uint32_t Mask=1;
     const uint8_t *CheckIndex = LDPC_ParityCheckIndex[Row];
     uint8_t CheckWeight = *CheckIndex++;
     for(uint8_t Bit=0; Bit<CheckWeight; Bit++)
     { uint8_t BitIdx=CheckIndex[Bit]; int8_t Ampl=InpBit[BitIdx];
       if(Ampl>0) Word|=Mask;
       Mask<<=1;
       if(Ampl<0) Ampl=(-Ampl);
       if(Ampl<MinAmpl) { MinAmpl2=MinAmpl; MinAmpl=Ampl; MinBit=Bit; }
       else if(Ampl<MinAmpl2) { MinAmpl2=Ampl; }
     }
     uint8_t CheckFails = Count1s(Word)&1;
     Mask=1;
     for(uint8_t Bit=0; Bit<CheckWeight; Bit++)
     { uint8_t BitIdx=CheckIndex[Bit];
       int8_t Ampl = Bit==MinBit ? MinAmpl2 : MinAmpl;
       if(CheckFails) Ampl=(-Ampl);
       // if( (BitIdx==166) || (BitIdx==183) )
       //   printf("OutBit[%d] += %+d\n", BitIdx, (Word&Mask) ? Ampl:-Ampl);
       OutBit[BitIdx] += (Word&Mask) ? Ampl:-Ampl;
       Mask<<=1; }
     return CheckFails?-MinAmpl:MinAmpl; }

} ;

#endif // __AVR__

#endif // of __LDPC_H__
