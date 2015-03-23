
inline void GPS_DISABLE(void) { GPIO_ResetBits(GPIOA, GPIO_Pin_0); }
inline void GPS_ENABLE (void) { GPIO_SetBits  (GPIOA, GPIO_Pin_0); }

inline int GPS_PPS_isOn(void) { return GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_1) != Bit_RESET; }

void GPS_Configuration (void)
{
  GPIO_InitTypeDef  GPIO_InitStructure;

  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);

  GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_0;        // Configure PA.00 as output: GPS Enable(HIGH) / Shutdown(LOW)
  GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_Out_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIO_Init(GPIOA, &GPIO_InitStructure);

  GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_1;         // Configure PA.01 as input: PPS from GPS
  GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_IPD;
  GPIO_Init(GPIOA, &GPIO_InitStructure);

  GPS_ENABLE(); }

void Debug_Print(uint8_t Byte) { while(!UART1_TxEmpty()) taskYIELD(); UART1_TxChar(Byte); }

xQueueHandle xQueuePacket;

NMEA_RxMsg  NMEA;               // NMEA sentences catcher
UBX_RxMsg   UBX;                // UBX messages catcher
OgnPosition Position[4];        // four GPS position pipe
uint8_t     PosIdx;
OGN_Packet  Packet[2];          // two OGN packet pipe
uint8_t     PktIdx;

TickType_t PPS_TickCount;       // [msec] TickCount of the most recent PPS pulse
TickType_t Burst_TickCount;     // [msec]
uint32_t   TimeSinceLock;       // [sec] time since the GPS has a lock
// uint8_t    UTC_Sec;             // [sec] UTC time: second

void GPS_PPS_On(void)                        // called on rising edge of PPS
{ LED_PCB_Flash(50);
  PPS_TickCount=xTaskGetTickCount(); }

void GPS_PPS_Off(void)                       // called on falling edge of PPS
{ }

void GPS_LockStart(void)                     // called when GPS catches a lock
{ Beep(20000, 128); vTaskDelay(100);
  Beep(20000,   0); vTaskDelay( 50);
  Beep(18000, 128); vTaskDelay(100);
  Beep(18000,   0); }

void GPS_LockEnd(void)                       // called when GPS looses a lock
{ Beep(18000, 128); vTaskDelay(100);
  Beep(18000,   0); vTaskDelay( 50);
  Beep(20000, 128); vTaskDelay(100);
  Beep(20000,   0); }

void GPS_BurstStart(void)                    // when GPS starts sending the data on the serial port
{ Burst_TickCount=xTaskGetTickCount(); }

void GPS_BurstEnd(void)                      // when GPS stops sending data on the serial port
{ if(Position[PosIdx].isComplete())                                        // position data complete
  { if(Position[PosIdx].isValid())                                         // position is complete and locked
    { 
      TimeSinceLock++;
      if(TimeSinceLock==1)
      { GPS_LockStart(); }
      if(TimeSinceLock>2)
      { uint8_t PrevIdx=(PosIdx+2)&3;
        int Delta=Position[PosIdx].calcDifferences(Position[PrevIdx]);
        Packet[PktIdx].setAddress(Parameters.getAddress());                     // prepare the packet
        Packet[PktIdx].setAddrType(Parameters.getAddrType());
        Packet[PktIdx].clrMeteo(); Packet[PktIdx].calcAddrParity();
        Packet[PktIdx].clrEmergency(); Packet[PktIdx].clrEncrypted(); Packet[PktIdx].setRelayCount(0);
        Position[PosIdx].Encode(Packet[PktIdx]);
        Packet[PktIdx].setStealth(Parameters.getStealth());
        Packet[PktIdx].setAcftType(Parameters.getAcftType());
        Packet[PktIdx].Whiten(); Packet[PktIdx].setFEC(); Packet[PktIdx].setReady();
        OGN_Packet *PktPtr = &Packet[PktIdx];
        xQueueSend(xQueuePacket, &PktPtr, 10);                            // send the new packet to the RF task
        PktIdx^=1; LED_PCB_Flash(100);
      }
    }
    else                                                                  // complete but not valid lock
    { if(TimeSinceLock) { GPS_LockEnd(); TimeSinceLock=0; }
    }
  }
  else
  { if(TimeSinceLock) { GPS_LockEnd(); TimeSinceLock=0; }
  }
  PosIdx=(PosIdx+1)&3; Position[PosIdx].Clear();
}

void GPS_NMEA(void)                                                        // when GPS gets a correct NMEA sentence
{ LED_PCB_Flash(2);                                                        // Flash the LED for 2 ms
  Position[PosIdx].ReadNMEA(NMEA);                                         // read position elements from NMEA
  if( NMEA.isGPRMC() || NMEA.isGPGGA() )
  { xSemaphoreTake(UART1_Mutex, portMAX_DELAY);
    Format_Bytes(UART1_Write, NMEA.Data, NMEA.Len); UART1_Write('\r'); UART1_Write('\n');
    xSemaphoreGive(UART1_Mutex); }
}

void GPS_UBX(void)                                                         // when GPS gets an UBX packet
{ }

#ifdef __cplusplus
  extern "C"
#endif
void vTaskGPS(void* pvParameters)
{ xQueuePacket = xQueueCreate(2, sizeof(OGN_Packet *));
  PPS_TickCount=0;
  Burst_TickCount=0;

  vTaskDelay(10);

  xSemaphoreTake(UART1_Mutex, portMAX_DELAY);
  Format_String(UART1_Write, "TaskGPS\n");
  xSemaphoreGive(UART1_Mutex);

  int Burst=(-1);                                                         // GPS transmission ongoing or line is idle ?
  { int LineIdle=0;
    int PPS=0;
    NMEA.Clear(); UBX.Clear();
    for(uint8_t Idx=0; Idx<4; Idx++)
      Position[Idx].Clear();
    PosIdx=0;
    PktIdx=0;

    for( ; ; )
    { vTaskDelay(1);                                                       // wait for the next time tick

      if(GPS_PPS_isOn()) { if(!PPS) { PPS=1; GPS_PPS_On();  } }
                    else { if( PPS) { PPS=0; GPS_PPS_Off(); } }

      LineIdle++;
      for( ; ; )
      { uint8_t Byte; int Err=UART2_Read(Byte); if(Err<=0) break;           // get Byte from serial port
        LineIdle=0;
        NMEA.ProcessByte(Byte); UBX.ProcessByte(Byte);                      // process through the NMEA interpreter
        if(NMEA.isComplete())                                               // NMEA completely received ?
        { if(NMEA.isChecked()) GPS_NMEA();                                  // NMEA check sum is correct ?
          NMEA.Clear(); }
        if(UBX.isComplete()) { GPS_UBX(); UBX.Clear(); }
      }

      if(LineIdle==0)                                                        // if any bytes were received ?
      { if(Burst==0) GPS_BurstStart();                                       // burst started
        Burst=1; }
      else if(LineIdle>10)                                                   // if GPS sends no more data for 10 time ticks
      { if(Burst>0) GPS_BurstEnd();                                          // burst ended
        Burst=0; }

    }
  }
}


