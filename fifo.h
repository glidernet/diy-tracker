#ifndef __FIFO_H__
#define __FIFO_H__

#include <stdint.h>

template <class Type, const uint8_t Size=8> // size must be (!) a power of 2 like 4, 8, 16, 32, etc.
 class VolatileFIFO
{ public:
   static const uint8_t Len = Size;
   static const uint8_t PtrMask = Size-1;

   volatile Type Data[Len];
   volatile uint8_t ReadPtr;
   volatile uint8_t WritePtr;

  public:
   void Clear(void)
   { ReadPtr=0; WritePtr=0; }

   uint8_t Write(Type Byte)
   { uint8_t Ptr=WritePtr;
     Data[Ptr]=Byte;
     Ptr++; Ptr&=PtrMask;
     if(Ptr==ReadPtr) return 0;
     WritePtr=Ptr; return 1; }

   uint8_t Read(Type &Byte)
   { uint8_t Ptr=ReadPtr;
     if(Ptr==WritePtr) return 0;
     Byte=Data[Ptr];
     Ptr++; Ptr&=PtrMask;
     ReadPtr=Ptr; return 1; }

   uint8_t isEmpty(void) const
   { return ReadPtr==WritePtr; }

   uint8_t Write(const Type *Data, uint8_t Len)
   { uint8_t Idx;
     for(Idx=0; Idx<Len; Idx++)
     { if(Write(Data[Idx])==0) break; }
     return Idx; }

/*
   Type Read(void)
   { uint8_t Ptr=ReadPtr;
     if(Ptr==WritePtr) return 0x00;
     uint8_t Byte=Data[Ptr];
     Ptr++; Ptr&=PtrMask;
     ReadPtr=Ptr; return Byte; }

   uint8_t ReadReady(void) const
   { return ReadPtr!=WritePtr; }

   uint8_t WriteReady(void) const
   { uint8_t Ptr=WritePtr;
     Ptr++; Ptr&=PtrMask;
     return Ptr!=ReadPtr; }
*/
} ;

#endif // __FIFO_H__
