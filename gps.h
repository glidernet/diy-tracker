
#include <FreeRTOS.h>
#include <task.h>
#include <semphr.h>
#include <queue.h>

#include "ogn.h"

#include "uart1.h"
#include "i2c.h"

extern          uint32_t GPS_UnixTime;      // [sec] UTC time in Unix format
extern          uint32_t GPS_FatTime;       // [2 sec] UTC time in FAT format (for FatFS)
extern volatile  uint8_t GPS_Sec;           // [sec]
extern           int32_t GPS_Altitude;      // [0.1m] altitude (height above Geoid)
extern           int16_t GPS_GeoidSepar;    // [0.1m]
extern          uint32_t GPS_TimeSinceLock; // [sec] time since GPS has a valid lock

extern TickType_t PPS_TickCount;   // [ms] time since the most recent rising edge of PPS pulse
uint16_t PPS_Phase(void);          // [ms]

extern xQueueHandle xQueuePacket;

const OgnPosition& GetGPSStatus();

void GPS_Configuration (void);

#ifdef __cplusplus
  extern "C"
#endif
void vTaskGPS(void* pvParameters);

