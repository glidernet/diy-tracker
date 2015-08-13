#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <time.h>
#include <math.h>
#include <stdint.h>
#include <ctype.h>

#include "serial.h"
#include "nmea.h"
#include "ogn.h"

class LowPass2
{ public:
   int32_t Out1, Out2;
  public:
   void Set(uint32_t Out=0) { Out1=Out2=Out; }
   int32_t Process( int32_t Inp)
   { Out1 = (Inp-Out2+1024*Out1+512)>>10;
     Out2 = (    Out1+ 291*Out2+146)/292;
     return Out2; }

} ;

LowPass2    AltLowPass;

SerialPort  Port;
NMEA_RxMsg  RxMsg;
OgnPosition Position;

int ProcessPort(void)                   // process serial port bytes (but only up to a point where the NMEA sequence is complete)
{ char Byte; int Read=0;
  while(!RxMsg.isComplete())             // while NMEA message is not (yet) complete
  { int Err=Port.Read(Byte);             // try to read a byte from the serial port
    if(Err>0)
    { RxMsg.ProcessByte(Byte); Read++; } // feed NMEA message with with received bytes
    else { if(Err<0) Read=Err; break; }  // break if no more bytes in the serial port
  }
 return Read; }                          // return the number of processed bytes

int ProcessString(const char *Input)
{ int Len=0;
  for( ; ; )
  { char Byte=(*Input++); if(Byte==0) break;
    RxMsg.ProcessByte(Byte); Len++; }
  return Len; }

int ProcessRxMsg(void)
{ if(!RxMsg.isComplete())                       return 0;   // if NMEA message not complete yet: return
  if(!RxMsg.isChecked())       { RxMsg.Clear(); return 0; } // if complete but checksum is bad: return
  // printf("RxMsg[%02d]: %s\n", RxMsg.Len, RxMsg.Data);
  if(RxMsg.isGPS())
    Position.ReadNMEA(RxMsg);
  if(RxMsg.isPOGNB())
  { uint32_t Press=0; Read_UnsDec(Press, (const char *)RxMsg.ParmPtr(2));
     int32_t Temp=0;  Read_Float1(Temp,  (const char *)RxMsg.ParmPtr(0));
    int32_t AverAlt = AltLowPass.Process(Position.Altitude<<6);
    printf("%5d %+5.1f %7.2f  %6.1f %+10.6f %+10.6f  %4.1f %4.1f %2d %6.1f\n", 
           Position.getUnixTime()%86400, 0.1*Temp, 0.01*Press,
           0.1*Position.Altitude, 0.0001/60*Position.Latitude, 0.0001/60*Position.Longitude,
           0.1*Position.Speed, 0.1*Position.HDOP, Position.Satellites,
           0.1/64*AverAlt ); }
  RxMsg.Clear(); return 1; }

int Process(void)                       // process data from the serial port, return positive when any new NMEA sentence
{ int Err=0;
  for( ; ; )
  { Err=ProcessRxMsg();                 // process the NMEA sentence
    if(Err!=0) break;                   // break the loop if GGA or RMC sentence or if an error
    Err=ProcessPort();                  // read data from serial port until message complete or no more data to read
    if(Err<=0) break; }                 // break the loop if  port empty or data error
  return Err; }                         // otherwise return 0

int main(int argc, char *argv[])
{ int Error;

  int PortNumber=0;
  int PortBaud=0;
  char PortName[32];
  PortName[0]=0;

  if(argc>1)
  { if(sscanf(argv[1], "COM%d:%d", &PortNumber, &PortBaud)==2)
    { PortNumber--; sprintf(PortName, "/dev/ttyS%d", PortNumber); }
    else if(sscanf(argv[1], "/dev/ttyS%d:%d", &PortNumber, &PortBaud)==2)
    { sprintf(PortName, "/dev/ttyS%d", PortNumber); }
    else if(sscanf(argv[1], "/dev/ttyUSB%d:%d", &PortNumber, &PortBaud)==2)
    { sprintf(PortName, "/dev/ttyUSB%d", PortNumber); }
    else if(sscanf(argv[1], "/dev/ttyACM%d:%d", &PortNumber, &PortBaud)==2)
    { sprintf(PortName, "/dev/ttyACM%d", PortNumber); }
    else
    { sprintf(PortName, "%s", argv[1]); PortBaud=0; }
  } else
  { strcpy(PortName,"/dev/ttyS0"); PortBaud=9600; }

  if(PortBaud)
  { Error=Port.Open(PortName, PortBaud);
    printf("Port.Open(%s, %d) => %d\n",PortName, PortBaud, Error); }
  else
  { Error=Port.OpenFileForRead(PortName);
    printf("Port.OpenFileForRead(%s) => %d\n",PortName, Error); }
  if(Error<0) exit(0);

  AltLowPass.Set(4600<<6);

  for( ; ; )
  { int Err=Process(); if(Err<0) break;
    // if(Err>0) printf("New NMEA message\n");
    if(PortBaud) usleep(1000);
  }

  Port.Close();
  return 0; }

