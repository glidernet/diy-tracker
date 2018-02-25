#include "hal.h"

#include "hal.h"

#include <FreeRTOS.h>
#include <task.h>
#include <semphr.h>
#include <queue.h>

#include "stm32f10x_iwdg.h"

#include <string.h>

#include "ctrl.h"

#include "nmea.h"        // NMEA
#include "ubx.h"         // UBX
#include "parameters.h"  // Parameters in Flash
#include "format.h"      // output formatting

#include "flashlog.h"

#include "main.h"
#include "gps.h"

#include "systick.h"

#ifdef WITH_SDCARD
  #include "fifo.h"
  #include "diskio.h"
  #include "ff.h"
#endif

uint32_t get_fattime(void) { return GPS_FatTime; } // for FatFS to have the correct time

// ======================================================================================

static char Line[64];

static void PrintParameters(void)                               // print parameters stored in Flash
{ Parameters.Print(Line);
  xSemaphoreTake(CONS_Mutex, portMAX_DELAY);                   // ask exclusivity on UART1
  Format_String(CONS_UART_Write, Line);
  xSemaphoreGive(CONS_Mutex);                                  // give back UART1 to other tasks
}

static void ProcessCtrlC(void)                                  // print system state to the console
{
  PrintParameters();

  size_t FreeHeap = xPortGetFreeHeapSize();

  xSemaphoreTake(CONS_Mutex, portMAX_DELAY);                   // ask exclusivity on UART1

  Format_String(CONS_UART_Write, "GPS: ");
  Format_UnsDec(CONS_UART_Write, GPS_getBaudRate(), 1);
  Format_String(CONS_UART_Write, "bps");
  CONS_UART_Write(',');
  Format_UnsDec(CONS_UART_Write, GPS_PosPeriod, 3, 2);
  CONS_UART_Write('s');
  if(GPS_Status.PPS)         Format_String(CONS_UART_Write, ",PPS");
  if(GPS_Status.NMEA)        Format_String(CONS_UART_Write, ",NMEA");
  if(GPS_Status.UBX)         Format_String(CONS_UART_Write, ",UBX");
  if(GPS_Status.MAV)         Format_String(CONS_UART_Write, ",MAV");
  if(GPS_Status.BaudConfig)  Format_String(CONS_UART_Write, ",BaudOK");
  if(GPS_Status.ModeConfig)  Format_String(CONS_UART_Write, ",ModeOK");
  // if(GPS_Status.Lock)        Format_String(CONS_UART_Write, ",Lock");
#ifdef WITH_PPS_IRQ
  // Format_String(CONS_UART_Write, "Xtal/PPS: ");
  CONS_UART_Write(',');
  Format_SignDec(CONS_UART_Write, (int32_t)(((10000>>4)*PPS_IRQ_Correction+SysTickPeriod/2)/SysTickPeriod), 1, 1);
  Format_String(CONS_UART_Write, "ppm");
#endif
  CONS_UART_Write('\r'); CONS_UART_Write('\n');

  Format_String(CONS_UART_Write, "Task  Pr. Stack, ");
  Format_UnsDec(CONS_UART_Write, (uint32_t)FreeHeap, 4, 3);
  Format_String(CONS_UART_Write, "kB free\n");

  UBaseType_t uxArraySize = uxTaskGetNumberOfTasks();
  TaskStatus_t *pxTaskStatusArray = (TaskStatus_t *)pvPortMalloc( uxArraySize * sizeof( TaskStatus_t ) );
  if(pxTaskStatusArray==0) goto Exit;
  uxArraySize = uxTaskGetSystemState( pxTaskStatusArray, uxArraySize, NULL );
  for(UBaseType_t T=0; T<uxArraySize; T++)
  { TaskStatus_t *Task = pxTaskStatusArray+T;
    // uint8_t Len=strlen(Task->pcTaskName);
    // memcpy(Line, Task->pcTaskName, Len);
    uint8_t Len=Format_String(Line, Task->pcTaskName);
    for( ; Len<=configMAX_TASK_NAME_LEN; )
      Line[Len++]=' ';
    Line[Len++]='0'+Task->uxCurrentPriority; Line[Len++]=' ';
    Len+=Format_UnsDec(Line+Len, Task->usStackHighWaterMark, 3);
    Line[Len++]='\n'; Line[Len++]=0;
    Format_String(CONS_UART_Write, Line);
  }
  vPortFree( pxTaskStatusArray );

Exit:
  Parameters.Write(CONS_UART_Write);
  xSemaphoreGive(CONS_Mutex);                                       // give back UART1 to other tasks
}

// ================================================================================================

static NMEA_RxMsg NMEA;
#ifdef WITH_GPS_UBX_PASS
static UBX_RxMsg  UBX;
#endif

#ifdef WITH_CONFIG
static void ReadParameters(void)  // read parameters requested by the user in the NMEA sent.
{ if((!NMEA.hasCheck()) || NMEA.isChecked() )
  { PrintParameters();
    if(NMEA.Parms==0)                                                      // if no parameter given
    { xSemaphoreTake(CONS_Mutex, portMAX_DELAY);                           // print a help message
      // Format_String(CONS_UART_Write, "$POGNS,<aircraft-type>,<addr-type>,<address>,<RFM69(H)W>,<Tx-power[dBm]>,<freq.corr.[Hz]>,<console baudrate[bps]>\n");
      Format_String(CONS_UART_Write, "$POGNS[,<Name>=<Value>]\n");
      xSemaphoreGive(CONS_Mutex);                                          //
      return; }
    Parameters.ReadPOGNS(NMEA);
/*
    const char *Parm; int8_t Val;
    Parm = (const char *)NMEA.ParmPtr(0);                                  // [0..15] aircraft-type: 1=glider, 2=tow plane, 3=helicopter, ...
    if(Parm)
    { Val=Read_Hex1(Parm[0]);
      if( (Val>=0) && (Val<16) ) Parameters.AcftType=Val; }
    Parm = (const char *)NMEA.ParmPtr(1);                                  // [0..3] addr-type: 1=ICAO, 2=FLARM, 3=OGN
    if(Parm)
    { Val=Read_Hex1(Parm[0]);
      if( (Val>=0) && (Val<4) ) Parameters.AddrType=Val; }
    Parm = (const char *)NMEA.ParmPtr(2);                                  // [HHHHHH] Address (ID): 6 hex digits, 24-bit
    uint32_t Addr;
    int8_t Len=Read_Hex(Addr, Parm);
    if( (Len==6) && (Addr<0x01000000) ) Parameters.Address=Addr;
    Parm = (const char *)NMEA.ParmPtr(3);                                  // [0..1] RFM69HW (up to +20dBm) or W (up to +13dBm)
    if(Parm)
    { Val=Read_Dec1(Parm[0]);
           if(Val==0) Parameters.clrTxTypeHW();
      else if(Val==1) Parameters.setTxTypeHW(); }
    Parm = (const char *)NMEA.ParmPtr(4);                                  // [dBm] Tx power
    int32_t TxPower;
    Len=Read_SignDec(TxPower, Parm);
    if( (Len>0) && (TxPower>=(-10)) && (TxPower<=20) ) Parameters.setTxPower(TxPower);
    Parm = (const char *)NMEA.ParmPtr(5);                                  // [Hz] Tx/Rx frequency correction
    int32_t FreqCorr;
    Len=Read_SignDec(FreqCorr, Parm);
    if( (Len>0) && (FreqCorr>=(-100000)) && (FreqCorr<=100000) ) Parameters.RFchipFreqCorr = FreqCorr/10;
    Parm = (const char *)NMEA.ParmPtr(6);                                  // [bps] Console baud rate
    uint32_t BaudRate;
    Len=Read_UnsDec(BaudRate, Parm);
    if( (Len>0) && (BaudRate<=230400) ) Parameters.CONbaud = BaudRate;
*/
    PrintParameters();
    taskDISABLE_INTERRUPTS();                                              // disable all interrupts: Flash can not be read while being erased
    // IWDG_ReloadCounter();                                                  // kick the watch-dog
    Parameters.WriteToFlash();                                             // erase and write the parameters into the last page of Flash
    if(Parameters.ReadFromFlash()<0) Parameters.setDefault();              // read the parameters back: if invalid, set defaults
    TimeSync_CorrRef(-20);                                                 // correct the time reference by about the time where he halted the CPU
    taskENABLE_INTERRUPTS();                                               // bring back interrupts and the system
                                                                           // page erase lasts 20ms tus about 20 system ticks are lost here
  }
  PrintParameters();
}
#endif

static void ProcessNMEA(void)     // process a valid NMEA that got to the console
{
#ifdef WITH_CONFIG
  if(NMEA.isPOGNS()) ReadParameters();
#endif
}

static void ProcessInput(void)
{
  for( ; ; )
  { uint8_t Byte; int Err=CONS_UART_Read(Byte); if(Err<=0) break; // get byte from console, if none: exit the loop
    if(Byte==0x03) ProcessCtrlC();                            // if Ctrl-C received
    NMEA.ProcessByte(Byte);                                   // pass the byte through the NMEA processor
    if(NMEA.isComplete())                                     // if complete NMEA:
    {
#ifdef WITH_GPS_NMEA_PASS
      if(NMEA.isChecked())
        NMEA.Send(GPS_UART_Write);
#endif
      ProcessNMEA();                                          // interpret the NMEA
      NMEA.Clear(); }                                         // clear the NMEA processor for the next sentence
#ifdef WITH_GPS_UBX_PASS
    UBX.ProcessByte(Byte);
    if(UBX.isComplete())
    { UBX.Send(GPS_UART_Write);                               // is there a need for a Mutex on the GPS UART ?
      UBX.Clear(); }
#endif
  }
}

// ================================================================================================

#ifdef WITH_SDLOG
static FATFS       FatFs;                                         // FatFS object for the file system (FAT)
       SemaphoreHandle_t FatFs_Mutex;                             // Mutex for more than one file at a time access
static       char  FileName[14];                                  // File name
static FRESULT     FileErr;                                       // most recent error/state of the File  operation
static FIL         File;                                          // FatFS object for the File

static   uint16_t  LogDate = 0;                                   // [~days] date = FatTime>>16
static TickType_t  LogOpenTime;                                   // [msec] when was the log file (re)open
static const  TickType_t  LogReopen = 20000;                      // [msec] when to close and re-open the log file

static FIFO<char, 2048> Log_FIFO;                                 // buffer for SD-log
       SemaphoreHandle_t Log_Mutex;                               // Mutex for the FIFO to prevent mixing between threads

void Log_Write(char Byte)                                         // write a byte into the log file buffer (FIFO)
{ if(Log_FIFO.Write(Byte)>0) return;                              // if byte written into FIFO return
  while(Log_FIFO.Write(Byte)<=0) vTaskDelay(1); }                 // wait while the FIFO is full - we have to use vTaskDelay not TaskYIELD

int Log_Free(void) { return Log_FIFO.Free(); }                    // how much space left in the buffer
                                                                  // TaskYIELD would not give time to lower priority task like log-writer
static void Log_Open(void)
{ LogDate=get_fattime()>>16;                                      // get the FAT-time date part
  int32_t Day   =  LogDate    &0x1F;                              // get day, month, year
  int32_t Month = (LogDate>>5)&0x0F;
  int32_t Year  = (LogDate>>9)-20;
  uint32_t Date = 0;
  if(Year>=0) Date = Day*10000 + Month*100 + Year;                // create DDMMYY number for easy printout
  strcpy(FileName, "TR000000.LOG");
  Format_UnsDec(FileName+2,    Date, 6);                          // format the date into the log file name
  FileErr=f_open(&File, FileName, FA_WRITE | FA_OPEN_ALWAYS);     // open the log file
  if(FileErr)
  { // xSemaphoreTake(CONS_Mutex, portMAX_DELAY);                   // ask exclusivity on UART1
    // Format_String(CONS_UART_Write, "TaskCTRL: cannot open "); // report open error
    // Format_String(CONS_UART_Write, FileName);
    // Format_String(CONS_UART_Write, "\n");
    // xSemaphoreGive(CONS_Mutex);                                  // give back UART1 to other tasks
    return ; }
  FileErr=f_lseek(&File, f_size(&File));                          // move to the end of the file (for append)
  LogOpenTime=xTaskGetTickCount();                                // record the system time when log was open
  if(!FileErr)
  { xSemaphoreTake(CONS_Mutex, portMAX_DELAY);                    // ask exclusivity on UART1
    Format_String(CONS_UART_Write, "TaskCTRL: writing to ");      // report open file name
    Format_String(CONS_UART_Write, FileName);
    Format_String(CONS_UART_Write, "\n");
    xSemaphoreGive(CONS_Mutex); }                                 // give back UART1 to other tasks
}

void Log_WriteData(const char *Data, int DataLen)                 // write the Line to the log file
{ // xSemaphoreTake(CONS_Mutex, portMAX_DELAY);
  // Format_String(CONS_UART_Write, "TaskCTRL: Log_WriteData: ");
  // Format_UnsDec(CONS_UART_Write, (uint32_t)DataLen, 1);
  // Format_String(CONS_UART_Write, "B \n");
  // xSemaphoreGive(CONS_Mutex);
  if(FileErr)                                                     // if last operation was in error
  { f_close(&File);                                               // attempt to reopen the file system
    FileErr=f_mount(&FatFs, "", 0);                               // here it should quickly catch if the SD card is not there
    if(!FileErr) Log_Open();                                      // if file system OK, thne open the file
    else                                                          // if an error, report it
    { // xSemaphoreTake(CONS_Mutex, portMAX_DELAY);
      // Format_String(CONS_UART_Write, "TaskCTRL: cannot mount FAT filesystem\n"); // report mount error
      // Format_String(CONS_UART_Write, "\n");
      // xSemaphoreGive(CONS_Mutex);
    }
  }
  if(FileErr) return;                                             // if still in error: quit
  UINT WrLen;
  FileErr=f_write(&File, Data, DataLen, &WrLen);                  // write the data to the log file
  if(!FileErr) return;
  xSemaphoreTake(CONS_Mutex, portMAX_DELAY);                      // ask exclusivity on UART1
  Format_String(CONS_UART_Write, "TaskCTRL: error when writing to "); // report write error
  Format_String(CONS_UART_Write, FileName);
  Format_String(CONS_UART_Write, "\n");
  xSemaphoreGive(CONS_Mutex);
}

// static void Log_WriteLine(const char *Line) { Log_Write(Line, strlen(Line)); }

static void Log_Check(void)                                    // time check:
{ if(FileErr) return;                                           // if last operation in error then don't do anything
  TickType_t TimeSinceOpen = xTaskGetTickCount()-LogOpenTime;  // when did we (re)open the log file last time
  if(LogDate)
  { if(TimeSinceOpen<LogReopen) return; }                      // if fresh (less than 30 seconds) then nothing to do
  else
  { if(TimeSinceOpen<(LogReopen/4)) return; }
  f_close(&File);                                           // close and reopen the log file when older than 10 seconds
  Log_Open();
}

static void ProcessLog(void)                                   // process the queue of lines to be written to the log
{ for( ; ; )
  { // char Byte; if(Log_FIFO.Read(Byte)<=0) break; Log_WriteData(&Byte, 1);
    char *Block; size_t Len=Log_FIFO.getReadBlock(Block); if(Len==0) break;
    if(Len>128) Len=128;
    Log_WriteData((const char *)Block, Len);
    Log_FIFO.flushReadBlock(Len);
  }
  Log_Check();                                            // time check the log file
}
#endif // WITH_SDLOG

// ================================================================================================

#if defined(WITH_SDLOG) && defined(WITH_FLASHLOG)

static void SaveFlashLog(uint16_t Pages, uint32_t StartTime)
{ // strcpy(FileName, "00000000.FLG");
  Format_Hex(FileName, StartTime); strcpy(FileName+8, ".FLG");
#ifdef WITH_BEEPER
  Play(Play_Vol_1 | Play_Oct_1 | 0, 100);
  Play(             Play_Oct_1 | 0,  50);
#endif
  FileErr=f_open(&File, FileName, FA_WRITE | FA_CREATE_ALWAYS);    // open the file to save the Flash log
  if(!FileErr)
  { for(uint16_t Page=0; Page<Pages; Page++)
    { const uint32_t *PageAddr;
      uint16_t PageLen = FlashLog_ReadPage(PageAddr); if(PageLen==0) break;
      UINT WrLen;
      FileErr=f_write(&File, PageAddr, PageLen, &WrLen); if(FileErr) break;
#ifdef WITH_BEEPER
      Play(Play_Vol_1 | Play_Oct_1 | (4-(Page&1)), 25);
      Play(             Play_Oct_1 |  4          , 25);
#endif
    }
  }
  f_close(&File);
  if(!FileErr)
  { xSemaphoreTake(CONS_Mutex, portMAX_DELAY);
    Format_String(CONS_UART_Write, "FlashLog saved to ");
    Format_String(CONS_UART_Write, FileName);
    CONS_UART_Write('\r'); CONS_UART_Write('\n');
    xSemaphoreGive(CONS_Mutex); }
#ifdef WITH_BEEPER
  if(FileErr)
    Play(Play_Vol_1 | Play_Oct_1 | 0, 100);
  else
    Play(Play_Vol_1 | Play_Oct_1 | 7, 100);
  Play(             Play_Oct_1 | 0,  50);
#endif
}

#endif

// ================================================================================================

#ifdef WITH_SDLOG
static uint8_t ReadLine(char *Line, uint8_t MaxLen, FIL *File, bool SkipComment=1)
{ uint8_t Len=0; MaxLen--;
  bool Comment=0;
  for( ; Len<MaxLen; )
  { UINT Read; char Ch;
    f_read(File, &Ch, 1, &Read);
    if(Read!=1) break;
    if(Ch=='\n') break;
    if( (Ch==';') || (Ch=='#') ) Comment=1;
    if(!Comment)
    { Line[Len++] = Ch; MaxLen--; }
  }
  Line[Len]=0;
  return Len; }

static int ReadParmFile(const char *ParmFileName="TRACKER.CFG")
{ int Count=0;
  xSemaphoreTake(CONS_Mutex, portMAX_DELAY);
  FileErr=f_open(&File, ParmFileName, FA_READ );               // open the parameter file
  Format_String(CONS_UART_Write, "Config file ");
  Format_String(CONS_UART_Write, ParmFileName);
  if(!FileErr)                                                 // if file opens correctly
  { CONS_UART_Write(':');
    CONS_UART_Write('\r'); CONS_UART_Write('\n');
    for( ; ; )
    { if(ReadLine(Line, 64, &File)<=0) break;                  // read line from the config file
      Format_String(CONS_UART_Write, Line);                    // print out the parameter line
      bool OK=Parameters.ReadLine(Line); if(OK) Count++;       // parse the line for parameter input
      CONS_UART_Write(' '); CONS_UART_Write(OK?'*':'?');       // for correct lines print '*'
      CONS_UART_Write('\r'); CONS_UART_Write('\n'); }
  }
  else
  { Format_String(CONS_UART_Write, " could not be open\n"); }
  xSemaphoreGive(CONS_Mutex);
  f_close(&File);                                              // close the parameter file
  return Count; }                                              // return: how many parameters has been correctly read
#endif // WITH_SDLOG

// ================================================================================================

// ================================================================================================

// ================================================================================================

extern "C"
void vTaskCTRL(void* pvParameters)
{

#ifdef WITH_FLASHLOG
  uint32_t LogStartTime=0;
  uint16_t LogPages=FlashLog_OpenForRead(&LogStartTime);
#endif

#ifdef WITH_SDLOG
  FatFs_Mutex = xSemaphoreCreateMutex();

  Log_Mutex = xSemaphoreCreateMutex();
  Log_FIFO.Clear();

  FileErr=f_mount(&FatFs, "", 0);
#endif

  vTaskDelay(5);

  xSemaphoreTake(CONS_Mutex, portMAX_DELAY);                   // ask exclusivity on UART1
  Format_String(CONS_UART_Write, "TaskCTRL: MCU ID: ");
  Format_Hex(CONS_UART_Write, UniqueID[0]); CONS_UART_Write(' ');
  Format_Hex(CONS_UART_Write, UniqueID[1]); CONS_UART_Write(' ');
  Format_Hex(CONS_UART_Write, UniqueID[2]); CONS_UART_Write(' ');
  Format_UnsDec(CONS_UART_Write, getFlashSizeKB()); Format_String(CONS_UART_Write, "kB\n");

#ifdef WITH_SDLOG
  if(!FileErr)
  { Format_String(CONS_UART_Write, "SD card: ");
    Format_UnsDec(CONS_UART_Write, (uint32_t)FatFs.csize * (uint32_t)(FatFs.free_clust>>1), 4, 3 );
    Format_String(CONS_UART_Write, "MB free\n"); }
#endif

#ifdef WITH_FLASHLOG
  if(LogPages)
  { Format_String(CONS_UART_Write, "FlashLog: ");
    Format_UnsDec(CONS_UART_Write, LogPages);
    Format_String(CONS_UART_Write, "pages @");
    Format_UnsDec(CONS_UART_Write, LogStartTime);
    Format_String(CONS_UART_Write, "\n");
  }
#endif
  xSemaphoreGive(CONS_Mutex);                                  // give back UART1 to other tasks

#ifdef WITH_SDLOG
  if(ReadParmFile()>0)
  { if(Parameters.CompareToFlash()<=0)                         // if new parameters are difference from those in Flash
    { // Parameters.WriteToFlash();                               // write the new parameters to Flash
    }
  }
#endif
  PrintParameters();

#ifdef WITH_SD_LOG

#ifdef WITH_FLASHLOG
  // if( (!FileErr) && LogPages) SaveFlashLog(LogPages, LogStartTime);
  if(LogPages) SaveFlashLog(LogPages, LogStartTime);
#endif // WITH_FLASH_LOG
  // if(!FileErr) Log_Open();
  Log_Open();

#endif // WITH_SD_LOG

  // vTaskDelay(1000);  // give chance to the GPS to catch the date

  NMEA.Clear();

  while(1)
  { vTaskDelay(1);

    ProcessInput();                                             // process console input
#ifdef WITH_SDLOG
    ProcessLog();                                               // process lines to written to the log file
#endif

  }
}

