#include "timesync.h"
#include "flashsize.h"

#include "stm32f10x_flash.h"

#include "flashlog.h"

#include "format.h"

static uint8_t  PageSizeLog2  = 0; //              10;             // [log2([B])]
static uint32_t PageSize      = 0; // 1<<PageSizeLog2;             // [Bytes]
static uint32_t PageMask      = 0; //     PageSize-1;
// const uint32_t PageSizeWords = PageSize/sizeof(uint32_t);   // [Words]

static uint16_t  Pages      = 0; // number of Flash pages to be used for logging
static uint32_t *FirstPage  = 0; // address of the first Flash page available for logging
static uint16_t  WriteIdx   = 0; // index of the current Flash page being written
static uint32_t *WriteAddr  = 0; // current address to write next record

static uint32_t *PrevPacket = 0; // most recent packet stored
static uint32_t  PrevTime   = 0; // [sec] and timestamup of this packet

static uint16_t  ReadIdx    = 0; // index of the current Flash page being read
static uint32_t *ReadAddr   = 0; // current address to read the next packet

// static uint32_t *getPageAddr(uint16_t  Idx)  { return FlashStart + ((uint32_t)(getFlashSizeKB()-1-Pages+Idx)*PageSizeWords); } // address of the given page (the last page is reserved for parameters)
static uint32_t *getPageAddr(uint16_t  Idx)  { return FirstPage + ((uint32_t)Idx<<(PageSizeLog2-2)); } // address of the given page (the last page is reserved for parameters)
static uint32_t *getPageAddr(uint32_t *Addr) { return (uint32_t *)((uint32_t)Addr&(~PageMask)); }
static bool       isPageAddr(uint32_t *Addr) { return ((uint32_t)Addr&PageMask)==0; }

extern char __etext;              // end of code in the Flash: linker symbol

static void FlashLog_Init(void)
{ PageSizeLog2 = getFlashPageSizeLog2();
  PageSize     = 1<<PageSizeLog2;
  PageMask     = PageSize-1;
  uint32_t CodeEnd    = (uint32_t)(&__etext);                                        // [Bytes] end of code in Flash
  uint32_t CodePages  = ((CodeEnd-(uint32_t)FlashStart)+PageMask)>>PageSizeLog2; // [pages] size of the code in Flash
  uint16_t FlashPages = getFlashSizeKB()>>(PageSizeLog2-10);
  uint16_t FreePages  = FlashPages-CodePages;                                        // [pages]
  Pages = FreePages-1;
  FirstPage = (uint32_t *)((CodeEnd+PageMask)&(~PageMask)); }

uint8_t FlashLog_Print(char *Output)
{ uint8_t Len=0;
  Len+=Format_String(Output+Len, "FlashWrite: ");
  Len+=Format_UnsDec(Output+Len, WriteIdx);
  Output[Len++]='/';
  Len+=Format_UnsDec(Output+Len, Pages);
  Output[Len++]=' ';
  Len+=Format_HHMMSS(Output+Len, PrevTime);
  Output[Len++]=' '; Output[Len++]='@';
  Len+=Format_Hex(Output+Len, (uint32_t)WriteAddr);
  Output[Len++]='\r'; Output[Len++]='\n';
  Output[Len]=0; return Len; }

// Every Flash log page would start with the following uint32_t words:
// 0. header word of the position packets: written just after the page is erased
// 1. ...
// 2. Time of the first stored position: written just after page is erased
// 3. Time of the last stored position: written when last postion is stored

static void Read(OGN_Packet &Packet, uint32_t *ReadAddr)
{ uint32_t *PageAddr = getPageAddr(ReadAddr);
  Packet.HeaderWord = PageAddr[0];
  memcpy(Packet.Data, ReadAddr, 4*sizeof(uint32_t)); }

static bool Write(OGN_Packet &Packet, uint32_t Time, bool NewPage=0)            // write the Packet (without Header) into the Flash log
{ int16_t Delay=0;
  vTaskDelay(1);                                                             // be as close to the start of the tick as possible
  taskDISABLE_INTERRUPTS();                                                  // disable all interrupts: Flash can not be read while being erased
  FLASH_Unlock();
  uint32_t *PageAddr = getPageAddr(WriteAddr);                               // address of the beginning of the page
  if(NewPage && (PageAddr!=WriteAddr) )                                      // if jumping to a new page was requested (and not at the start of a page)
  { WriteIdx++; if(WriteIdx>=Pages) WriteIdx=0;                                 // increment the page index
    WriteAddr = getPageAddr(WriteIdx);                                        // WriteAddr at the beginning of the page
    PageAddr = getPageAddr(WriteAddr); }
  if(PageAddr==WriteAddr)                                                    // if at the beginning of the page
  { FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_PGERR | FLASH_FLAG_WRPRTERR);
    // FLASH_ErasePage((uint32_t)WriteAddr); Delay=15;                       // erages the new Flash page
    while(FLASH_ErasePage((uint32_t)WriteAddr)!=FLASH_COMPLETE); Delay=20;   // erages the new Flash page
    FLASH_ProgramWord((uint32_t)WriteAddr, Packet.HeaderWord); WriteAddr+=2; // Write packet header word
    FLASH_ProgramWord((uint32_t)WriteAddr, Time); WriteAddr+=2;              // write the time of the first record on page
    if(PageAddr[2]!=Time)                                                    // if Flash erase or write failed
    { WriteAddr=PageAddr;                                                    // keep WriteAddr at the start of the page
      FLASH_Lock();                                                          // re-lock the Flash
      taskENABLE_INTERRUPTS();                                               // restore RTOS interrupts
      return 0; }                                                            // if page erase failed, then return failure

  }
  PrevPacket=WriteAddr;                                                      // keep the address of the last position record
  PrevTime=Time;                                                             // and its Time
  for(uint8_t Idx=0; Idx<4; Idx++)                                           // write to Flash the four words of position (but not the header)
  { FLASH_ProgramWord((uint32_t)WriteAddr, Packet.Data[Idx]); WriteAddr++; }
  if(isPageAddr(WriteAddr))                                                  // if at the end of the page
  { FLASH_ProgramWord((uint32_t)(PageAddr+3), Time);                         // write the time of the last record on page
    WriteIdx++; if(WriteIdx>=Pages) WriteIdx=0;                                 // push PageIdx to the next Flash page
    WriteAddr = getPageAddr(WriteIdx); }                                      // set the WriteAddr to the start of this page
  FLASH_Lock();
  if(Delay)                                                                  // when page is erased we hole the CPU for some 15-20ms
  { TimeSync_CorrRef(-Delay); }                                              // correct the time refernece bu the delay
  taskENABLE_INTERRUPTS();                                                   // restore RTOS interrupts
  return 1; }                                                                // return success

uint16_t FlashLog_OpenForWrite(void)
{ if(PageSize==0) FlashLog_Init();
  uint32_t Latest=0; WriteIdx=0;                    // find the page with the latest end-timestamp
  for(uint16_t Idx=0; Idx<Pages; Idx++)
  { uint32_t *Page=getPageAddr(Idx);
    uint32_t First=Page[2]; // uint32_t Last=Page[3];
    if(First>Latest) { Latest=First; WriteIdx=Idx; }
    if(Latest==0xFFFFFFFF) break; }                // if this is an erased page stop the search
  if(Latest!=0xFFFFFFFF)
  { WriteIdx++; if(WriteIdx>=Pages) WriteIdx=0; }  // take the page following the latest or the unfinished one
  WriteAddr=getPageAddr(WriteIdx);
  return Pages<<(PageSizeLog2-10); }               // [KB] return the number of Flash space available for logging

static bool Process(OGN_Packet &Packet, uint32_t Time)                        // process position packet: decide whether to store it or not
{ uint32_t TimeDelta = Time-PrevTime;                                         // [sec] time since previously stored packet
  if( (PrevPacket==0) || (TimeDelta>=50) ) return 1;                          // [sec]
  int16_t Climb = Packet.DecodeClimbRate();                                   // [0.1m/s]
  if(abs(Climb)>=100) return 1;                                               // if climb/decent rate more than 10m/s
  OGN_Packet *Prev = (OGN_Packet *)(PrevPacket-1);                            // address (tweaked) of the previously stored packet
  int32_t AltDelta=Packet.DecodeAltitude()-Prev->DecodeAltitude();            // [m] altitude change
  if(abs(AltDelta)>=20)  return 1;                                            // if more than 50m altitude change
  int16_t PrevClimb = Prev->DecodeClimbRate();                                    // [0.1m/s]
  int32_t DistDeltaV = (int32_t)(Climb-PrevClimb)*TimeDelta;                  // [0.1m]
  if(abs(DistDeltaV)>=200) return 1;
  int16_t Speed = Packet.DecodeSpeed();                                       // [0.1m/s]
  int16_t PrevSpeed = Prev->DecodeSpeed();                                    // [0.1m/s]
  int32_t DistDeltaH = (int32_t)(Speed-PrevSpeed)*TimeDelta;                  // [0.1m] speed change * time since last recorded packet
  if(abs(DistDeltaH)>=200) return 1;                                           // if extrapolation error more than 50m
  int16_t Turn = Packet.DecodeTurnRate();                                     // [0.1deg/s]
  int16_t CFaccel = ((int32_t)Turn*Speed*229+0x10000)>>17;                    // [0.1m/s^2] centrifugal acceleration in turn
  if(abs(CFaccel)>=50) return 1;                                              // CFaccel at or above 5m/s^2 (0.5g)
  int16_t PrevTurn = Prev->DecodeTurnRate();                                  // [0.1deg/s]
  int16_t PrevCFaccel = ((int32_t)PrevTurn*PrevSpeed*229+0x10000)>>17;        // [0.1m/s^2]
  int32_t DistDeltaR = abs(CFaccel-PrevCFaccel)*TimeDelta*TimeDelta/2;        // [0.1m]
  if(abs(DistDeltaR)>=200) return 1;                                          // [0.1m]
  return 0; }

bool FlashLog_Process(OGN_Packet &Packet, uint32_t Time)                      // process position packet: decide whether to store it or not
{ if(WriteAddr==0) FlashLog_OpenForWrite();
  if(Process(Packet, Time)==0) return 0;                                      // decide whetehr to save this packet
  return Write(Packet, Time); }                                               // return 1 if this packet was saved in the flash

bool FlashLog_PageIsValid(const uint32_t *Page)
{ uint32_t First=Page[2]; uint32_t Last=Page[3];
  if(First>=Last) return 0;
  if(First<1500000000) return 0;
  if( (Last!=0xFFFFFFFF) && ((Last-First)>(PageSize/16*50)) ) return 0;
  OGN_Packet *Packet = (OGN_Packet *)Page;
  if(Packet->Header.Other) return 0;
  if(Packet->Header.RelayCount!=0) return 0;
  if(!Packet->goodAddrParity()) return 0;
  return 1; }

uint16_t FlashLog_OpenForRead(uint32_t *StartTime)
{ if(PageSize==0) FlashLog_Init();
  uint32_t Earliest=0xFFFFFFFF; ReadIdx=0; uint16_t Count=0;                  // find the page with the latest end-timestamp
  for(uint16_t Idx=0; Idx<Pages; Idx++)
  { uint32_t *Page=getPageAddr(Idx);
    if(!FlashLog_PageIsValid(Page)) continue;
    uint32_t First=Page[2]; // uint32_t Last=Page[3];
    if(First<Earliest) { Earliest=First; ReadIdx=Idx; }
    Count++; }
  ReadAddr=getPageAddr(ReadIdx);
  if(StartTime) *StartTime=Earliest;
  return Count; }                                                             // [KB] return the number of Flash space available for logging

uint32_t FlashLog_ReadPage(const uint32_t * &Page)
{ if(ReadAddr==0) FlashLog_OpenForRead();
  uint16_t StartIdx=ReadIdx;
  for( ; ; )
  { Page=ReadAddr;
    ReadIdx++; if(ReadIdx>=Pages) ReadIdx=0;
    ReadAddr=getPageAddr(ReadIdx);
    if(FlashLog_PageIsValid(Page)) return PageSize;
    if(ReadIdx==StartIdx) break; }
  return 0; }

