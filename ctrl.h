//------------------------------------------------------------------------------

#ifndef __CTRL_H
#define __CTRL_H

//------------------------------------------------------------------------------

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

//------------------------------------------------------------------------------

// various control commands
enum ControlCmd
{
  button_up,
  button_down,
  button_set,
};

//------------------------------------------------------------------------------

#ifdef WITH_SDLOG
extern xQueueHandle LogQueue;
inline void LogLine(char *Line, TickType_t Wait=2) { xQueueSend(LogQueue, &Line, Wait); }
#else
inline void LogLine(char *Line, TickType_t Wait=2) { }
#endif

#ifdef __cplusplus
  extern "C"
#endif
void vTaskCTRL(void* pvParameters);


//------------------------------------------------------------------------------
#endif // __CTRL_H
