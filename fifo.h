#ifndef __FIFO_H__
#define __FIFO_H__

#include <stdint.h>

template <class Type, const size_t Size=8> // size must be (!) a power of 2 like 4, 8, 16, 32, etc.
 class VolatileFIFO
{ public:
   static const size_t Len = Size;
   static const size_t PtrMask = Size-1;

   volatile Type Data[Len];
   volatile size_t ReadPtr;
   volatile size_t WritePtr;

  public:
   void Clear(void)
   { ReadPtr=0; WritePtr=0; }

   size_t Write(Type Byte)
   { size_t Ptr=WritePtr;
     Data[Ptr]=Byte;
     Ptr++; Ptr&=PtrMask;
     if(Ptr==ReadPtr) return 0;
     WritePtr=Ptr; return 1; }

   size_t Read(Type &Byte)
   { size_t Ptr=ReadPtr;
     if(Ptr==WritePtr) return 0;
     Byte=Data[Ptr];
     Ptr++; Ptr&=PtrMask;
     ReadPtr=Ptr; return 1; }

   size_t getReadBlock(volatile Type *&Byte)
   { if(ReadPtr==WritePtr) { Byte=0; return 0; }
     Byte = Data+ReadPtr;
     if(ReadPtr<WritePtr) return WritePtr-ReadPtr;
     return Size-ReadPtr; }

   void flushReadBlock(size_t Len)
   { ReadPtr+=Len; ReadPtr&=PtrMask; }

   size_t isEmpty(void) const
   { return ReadPtr==WritePtr; }

   size_t Write(const Type *Data, size_t Len)
   { size_t Idx;
     for(Idx=0; Idx<Len; Idx++)
     { if(Write(Data[Idx])==0) break; }
     return Idx; }

/*
   Type Read(void)
   { uint16_t Ptr=ReadPtr;
     if(Ptr==WritePtr) return 0x00;
     uint8_t Byte=Data[Ptr];
     Ptr++; Ptr&=PtrMask;
     ReadPtr=Ptr; return Byte; }

   uint16_t ReadReady(void) const
   { return ReadPtr!=WritePtr; }

   uint8_t WriteReady(void) const
   { uint8_t Ptr=WritePtr;
     Ptr++; Ptr&=PtrMask;
     return Ptr!=ReadPtr; }
*/
} ;

template <class Type, const uint8_t Size=8> // size must be (!) a power of 2 like 4, 8, 16, 32, etc.
 class Delay
{ public:
   static const uint8_t Len = Size;
   static const uint8_t PtrMask = Size-1;

   Type Data[Len];
   uint8_t Ptr;

  public:
   void Clear(Type Zero=0)
   { for(uint8_t Idx=0; Idx<Len; Idx++)
       Data[Idx]=Zero;
     Ptr=0; }

   Type Input(Type Inp)
   { Ptr--; Ptr&=PtrMask; 
     Type Out=Data[Ptr];
     Data[Ptr]=Inp;
     return Out; }

   Type & operator[](uint8_t Idx)
   { Idx = Ptr-Idx; Idx&=PtrMask;
     return Data[Idx]; }

} ;

#endif // __FIFO_H__
