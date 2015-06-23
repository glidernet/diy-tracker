#ifndef __PARAMETERS_H__
#define __PARAMETERS_H__

#include "flashsize.h"
#include "uniqueid.h"

#include "stm32f10x_flash.h"

#include "ogn.h"
#include "format.h"

// Parameters stored in Flash
class FlashParameters
{ public:
   uint32_t  AcftID;         // identification: Private:AcftType:AddrType:Address - must be different for every tracker
    int16_t  RFchipFreqCorr; // [61Hz] frequency correction for crystal frequency offset
    int8_t   RFchipTxPower;  // [dBm] highest bit set => HW module (up to +20dBm Tx power)
    int8_t   RFchipTempCorr; // [degC]
   uint32_t  CONbaud;        // [bps] Console baud rate
   uint32_t  GPSbaud;        // [bps] GPS baud rate

   // static const uint32_t Words=sizeof(FlashParameters)/sizeof(uint32_t);

   uint32_t getAddress(void) const { return AcftID&0x00FFFFFF; }
   uint8_t getAddrType(void) const { return (AcftID>>24)&0x03; }
   AcftType getAcftType(void) const { return (AcftType) ((AcftID>>26)&0x0F); }
   uint8_t getNoTrack (void) const { return (AcftID>>30)&0x01; }
   uint8_t getStealth (void) const { return (AcftID>>31)&0x01; }

   void setAddress (uint32_t Address)  { AcftID = (AcftID&0xFF000000) | (Address&0x00FFFFFF); }
   void setAddrType(uint8_t  AddrType) { AcftID = (AcftID&0xFCFFFFFF) | ((uint32_t)(AddrType&0x03)<<24); }
   void setAcftType(uint8_t  AcftType) { AcftID = (AcftID&0xC3FFFFFF) | ((uint32_t)(AcftType&0x0F)<<26); }
   void setNoTrack(void) { AcftID |= 0x40000000; }
   void clrNoTrack(void) { AcftID &= 0xBFFFFFFF; }
   void setStealth(void) { AcftID |= 0x80000000; }
   void clrStealth(void) { AcftID &= 0x7FFFFFFF; }

   int8_t getTxPower(void) const { int8_t Pwr=RFchipTxPower&0x7F; if(Pwr&0x40) Pwr|=0x80; return Pwr; }
   void   setTxPower(int8_t Pwr) { RFchipTxPower = (RFchipTxPower&0x80) | (Pwr&0x7F); }

   void   setTxTypeHW(void)       {        RFchipTxPower|=0x80; }
   void   clrTxTypeHW(void)       {        RFchipTxPower&=0x7F; }
   uint8_t isTxTypeHW(void) const { return RFchipTxPower& 0x80; } // if this RFM69HW (Tx power up to +20dBm) ?

   static const uint32_t CheckInit = 0x89ABCDEF;

 public:
  void setDefault(void)
  { AcftID = UniqueID[0] ^ UniqueID[1] ^ UniqueID[2];
    AcftID = 0x07000000 | (AcftID&0x00FFFFFF);
    RFchipFreqCorr =         0; // [61Hz]
    // RFchipTxPower  =        13; // [dBm] for RFM69W
    RFchipTxPower  = 0x80 | 14; // [dBm] for RFM69HW
    RFchipTempCorr =         0; // [degC]
    CONbaud        =    115200; // [bps]
    GPSbaud        =      9600; // [bps]
  }

  uint32_t static CheckSum(const uint32_t *Word, uint32_t Words)
  { uint32_t Check=CheckInit;
    for(uint32_t Idx=0; Idx<Words; Words++)
    { Check+=Word[Idx]; }
    return Check; }

  uint32_t CheckSum(void) const
  { return CheckSum((uint32_t *)this, sizeof(FlashParameters)/sizeof(uint32_t) ); }

  static uint32_t *DefaultFlashAddr(void) { return FlashStart+((uint32_t)(getFlashSize()-1)<<8); }

  int8_t ReadFromFlash(uint32_t *Addr=0)
  { if(Addr==0) Addr = DefaultFlashAddr();
    const uint32_t Words=sizeof(FlashParameters)/sizeof(uint32_t);
    uint32_t Check=CheckSum(Addr, Words);
    if(Check!=Addr[Words]) return -1;
    uint32_t *Dst = (uint32_t *)this;
    for(uint32_t Idx=0; Idx<Words; Idx++)
    { Dst[Idx] = Addr[Idx]; }
    return 1; }

  int8_t WriteToFlash(uint32_t *Addr=0) const
  { if(Addr==0) Addr = DefaultFlashAddr();
    const uint32_t Words=sizeof(FlashParameters)/sizeof(uint32_t);
    FLASH_Unlock();
    FLASH_ErasePage((uint32_t)Addr);
    uint32_t *Data=(uint32_t *)this;
    for(uint32_t Idx=0; Idx<Words; Idx++)
    { FLASH_ProgramWord((uint32_t)Addr, Data[Idx]); Addr++; } // !=FLASH_COMPLETE ?
    FLASH_ProgramWord((uint32_t)Addr, CheckSum(Data, Words) );
    FLASH_Lock();
    if(CheckSum(Addr, Words)!=Addr[Words]) return -1;
    return 0; }

  uint8_t Print(char *Line)
  { uint8_t Len=0;
    Line[Len++]=HexDigit(getAcftType()); Line[Len++]=':';
    Line[Len++]=HexDigit(getAddrType()); Line[Len++]=':';
    Len+=Format_Hex(Line+Len, getAddress(), 6);
    Len+=Format_String(Line+Len, " GPS:");
    Len+=Format_UnsDec(Line+Len, GPSbaud);
    Len+=Format_String(Line+Len, "bps ");
    Len+=Format_SignDec(Line+Len, (int16_t)getTxPower());
    Len+=Format_String(Line+Len, "dBm");
    Line[Len++]='/';
    if(isTxTypeHW()) Line[Len++]='H';
    Line[Len++]='W';
    Line[Len++]=' '; Len+=Format_SignDec(Line+Len, ((int32_t)RFchipFreqCorr*15625+128)>>8, 1); Len+=Format_String(Line+Len, "Hz\n");
    Line[Len]=0;
    return Len; }

} ;

#endif // __PARAMETERS_H__

