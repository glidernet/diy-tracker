//------------------------------------------------------------------------------

#ifndef __CTRL_H
#define __CTRL_H

//------------------------------------------------------------------------------

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"


#ifdef WITH_BUTTONS
  enum Buttons {
    btn_up,
    btn_down,
    btn_set,
  };
#endif


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
