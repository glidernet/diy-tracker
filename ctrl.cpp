#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "queue.h"

#include "stm32f10x_iwdg.h"

#include <string.h>

#include "ctrl.h"

#include "uart1.h"       // UART1 (console)
#include "nmea.h"        // NMEA
#include "parameters.h"  // Parameters in Flash
#include "format.h"      // output formatting

#include "main.h"
#include "gps.h"

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
  xSemaphoreTake(UART1_Mutex, portMAX_DELAY);                   // ask exclusivity on UART1
  Format_String(UART1_Write, Line);
  xSemaphoreGive(UART1_Mutex);                                  // give back UART1 to other tasks
}

static void ProcessCtrlC(void)                                  // print system state to the console
{
  PrintParameters();

  size_t FreeHeap = xPortGetFreeHeapSize();

  xSemaphoreTake(UART1_Mutex, portMAX_DELAY);                   // ask exclusivity on UART1

  Format_String(UART1_Write, "Task  Pr. Stack, ");
  Format_UnsDec(UART1_Write, (uint32_t)FreeHeap, 4, 3);
  Format_String(UART1_Write, "kB free\n");
  // xSemaphoreGive(UART1_Mutex);                                  // give back UART1 to other tasks

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
    // xSemaphoreTake(UART1_Mutex, portMAX_DELAY);                   // ask exclusivity on UART1
    Format_String(UART1_Write, Line);
    // xSemaphoreGive(UART1_Mutex);                                  // give back UART1 to other tasks
  }
  vPortFree( pxTaskStatusArray );
Exit:
  xSemaphoreGive(UART1_Mutex);                                       // give back UART1 to other tasks
}

// ================================================================================================

static NMEA_RxMsg NMEA;

#ifdef WITH_CONFIG
static void ReadParameters(void)  // read parameters requested by the user in the NMEA sent.
{ if((!NMEA.hasCheck()) || NMEA.isChecked() )
  { const char *Parm; int8_t Val;
    Parm = (const char *)NMEA.ParmPtr(0);                                  // [0..15] aircraft-type: 1=glider, 2=towa plane, 3=helicopter, ...
    if(Parm)
    { Val=Read_Hex1(Parm[0]);
      if( (Val>=0) && (Val<16) ) Parameters.setAcftType(Val); }
    Parm = (const char *)NMEA.ParmPtr(1);                                  // [0..3] addr-type: 1=ICAO, 2=FLARM, 3=OGN
    if(Parm)
    { Val=Read_Hex1(Parm[0]);
      if( (Val>=0) && (Val<4) ) Parameters.setAddrType(Val); }
    Parm = (const char *)NMEA.ParmPtr(2);                                  // [HHHHHH] Address (ID): 6 hex digits, 24-bit
    uint32_t Addr;
    int8_t Len=Read_Hex(Addr, Parm);
    if( (Len==6) && (Addr<0x01000000) ) Parameters.setAddress(Addr);
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
    if( (Len>0) && (FreqCorr>=(-100000)) && (FreqCorr<=100000) ) Parameters.RFchipFreqCorr = (FreqCorr<<8)/15625;

    taskDISABLE_INTERRUPTS();                                              // disable all interrupts: Flash can not be read while being erased
    IWDG_ReloadCounter();                                                  // kick the watch-dog
    Parameters.WriteToFlash();                                             // erase and write the parameters into the last page of Flash
    if(Parameters.ReadFromFlash()<0) Parameters.setDefault();              // read the parameters back: if invalid, set defaults
    taskENABLE_INTERRUPTS();                                               // bring back interrupts and the system
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
  { uint8_t Byte; int Err=UART1_Read(Byte); if(Err<=0) break; // get byte from console, if none: exit the loop
    if(Byte==0x03) ProcessCtrlC();                            // if Ctrl-C received
    NMEA.ProcessByte(Byte);                                   // pass the byte through the NMEA processor
    if(NMEA.isComplete())                                     // if complete NMEA:
    { /* if(NMEA.isChecked()) */ ProcessNMEA();               // and if CRC is good: interpret the NMEA
      NMEA.Clear(); }                                         // clear the NMEA processor for the next sentence
  }
}

// ================================================================================================

#ifdef WITH_SDLOG
static FATFS       FatFs;                                         // FatFS object for the file system (FAT)
       SemaphoreHandle_t FatFs_Mutex;                             // Mutex for more than one file at a time access

static   uint16_t  LogDate     =              0;                  // [~days] date = FatTime>>16
static       char  LogName[14] = "TR000000.LOG";                  // log file name
static FRESULT     LogErr;                                        // most recent error/state of the logging system
static FIL         LogFile;                                       // FatFS object for the log file
static TickType_t  LogOpenTime;                                   // [msec] when was the log file (re)open
static const  TickType_t  LogReopen = 20000;                      // [msec] when to close and re-open the log file

static VolatileFIFO<char, 512> Log_FIFO;                          // buffer for SD-log
       SemaphoreHandle_t Log_Mutex;                               // Mutex for the FIFO to prevent mixing between threads

void Log_Write(char Byte)                                         // write a byte into the log file buffer (FIFO)
{ if(Log_FIFO.Write(Byte)>0) return;                              // if byte written into FIFO return
  while(Log_FIFO.Write(Byte)<=0) vTaskDelay(1); }                 // wait while the FIFO is full - we have to use vTaskDelay not TaskYIELD
                                                                  // TaskYIELD would not give time to lower priority task like log-writer
static void Log_Open(void)
{ LogDate=get_fattime()>>16;                                      // get the FAT-time date part
  int32_t Day   =  LogDate    &0x1F;                              // get day, month, year
  int32_t Month = (LogDate>>5)&0x0F;
  int32_t Year  = (LogDate>>9)-20;
  uint32_t Date = 0;
  if(Year>=0) Date = Day*10000 + Month*100 + Year;                // create DDMMYY number for easy printout
  Format_UnsDec(LogName+2,    Date, 6);                           // format the date into the log file name
  LogErr=f_open(&LogFile, LogName, FA_WRITE | FA_OPEN_ALWAYS);    // open the log file
  if(LogErr)
  { // xSemaphoreTake(UART1_Mutex, portMAX_DELAY);                   // ask exclusivity on UART1
    // Format_String(UART1_Write, "TaskCTRL: cannot open "); // report open error
    // Format_String(UART1_Write, LogName);
    // Format_String(UART1_Write, "\n");
    // xSemaphoreGive(UART1_Mutex);                                  // give back UART1 to other tasks
    return ; }
  LogErr=f_lseek(&LogFile, f_size(&LogFile));                     // move to the end of the file (for append)
  LogOpenTime=xTaskGetTickCount();                                // record the system time when log was open
  if(!LogErr)
  { xSemaphoreTake(UART1_Mutex, portMAX_DELAY);                   // ask exclusivity on UART1
    Format_String(UART1_Write, "TaskCTRL: writing to ");          // report open file name
    Format_String(UART1_Write, LogName);
    Format_String(UART1_Write, "\n");
    xSemaphoreGive(UART1_Mutex); }                                // give back UART1 to other tasks
}

void Log_WriteData(const char *Data, int DataLen)                 // write the Line to the log file
{ // xSemaphoreTake(UART1_Mutex, portMAX_DELAY);
  // Format_String(UART1_Write, "TaskCTRL: Log_WriteData: ");
  // Format_UnsDec(UART1_Write, (uint32_t)DataLen, 1);
  // Format_String(UART1_Write, "B \n");
  // xSemaphoreGive(UART1_Mutex);
  if(LogErr)                                                      // if last operation was in error
  { f_close(&LogFile);                                            // attempt to reopen the file system
    LogErr=f_mount(&FatFs, "", 0);                                // here it should quickly catch if the SD card is not there
    if(!LogErr) Log_Open();                                       // if file system OK, thne open the file
    else                                                          // if an error, report it
    { // xSemaphoreTake(UART1_Mutex, portMAX_DELAY);
      // Format_String(UART1_Write, "TaskCTRL: cannot mount FAT filesystem\n"); // report mount error
      // Format_String(UART1_Write, "\n");
      // xSemaphoreGive(UART1_Mutex);
    }
  }
  if(LogErr) return;                                              // if still in error: quit
  UINT WrLen;
  LogErr=f_write(&LogFile, Data, DataLen, &WrLen);                // write the data to the log file
  if(!LogErr) return;
  xSemaphoreTake(UART1_Mutex, portMAX_DELAY);                     // ask exclusivity on UART1
  Format_String(UART1_Write, "TaskCTRL: error when writing to "); // report write error
  Format_String(UART1_Write, LogName);
  Format_String(UART1_Write, "\n");
  xSemaphoreGive(UART1_Mutex);
}

// static void Log_WriteLine(const char *Line) { Log_Write(Line, strlen(Line)); }

static void Log_Check(void)                                    // time check:
{ if(LogErr) return;                                           // if last operation in error then don't do anything
  TickType_t TimeSinceOpen = xTaskGetTickCount()-LogOpenTime;  // when did we (re)open the log file last time
  if(LogDate)
  { if(TimeSinceOpen<LogReopen) return; }                      // if fresh (less than 30 seconds) then nothing to do
  else
  { if(TimeSinceOpen<(LogReopen/4)) return; }
  f_close(&LogFile);                                           // close and reopen the log file when older than 10 seconds
  Log_Open();
}

static void ProcessLog(void)                                   // process the queue of lines to be written to the log
{ for( ; ; )
  { // char Byte; if(Log_FIFO.Read(Byte)<=0) break; Log_WriteData(&Byte, 1);
    volatile char *Block; size_t Len=Log_FIFO.getReadBlock(Block); if(Len==0) break;
    Log_WriteData((const char *)Block, Len);
    Log_FIFO.flushReadBlock(Len);
  }
  Log_Check();                                            // time check the log file
}
#endif // WITH_SDLOG

// ================================================================================================

extern "C"
void vTaskCTRL(void* pvParameters)
{
#ifdef WITH_SDLOG
  FatFs_Mutex = xSemaphoreCreateMutex();

  Log_Mutex = xSemaphoreCreateMutex();
  Log_FIFO.Clear();
  LogErr=f_mount(&FatFs, "", 0);
  if(!LogErr) Log_Open();
#endif

  vTaskDelay(5);

  xSemaphoreTake(UART1_Mutex, portMAX_DELAY);                   // ask exclusivity on UART1
  Format_String(UART1_Write, "TaskCTRL: MCU ID: ");
  Format_Hex(UART1_Write, UniqueID[0]); UART1_Write(' ');
  Format_Hex(UART1_Write, UniqueID[1]); UART1_Write(' ');
  Format_Hex(UART1_Write, UniqueID[2]); UART1_Write(' ');
  Format_UnsDec(UART1_Write, getFlashSize()); Format_String(UART1_Write, "kB\n");
#ifdef WITH_SDLOG
  if(!LogErr)
  { Format_String(UART1_Write, "SD card: ");
    Format_UnsDec(UART1_Write, (uint32_t)FatFs.csize * (uint32_t)(FatFs.free_clust>>1), 4, 3 );
    Format_String(UART1_Write, "MB free\n"); }
#endif
  xSemaphoreGive(UART1_Mutex);                                  // give back UART1 to other tasks
  PrintParameters();

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

