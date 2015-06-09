#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "queue.h"

#define LOG_ENABLE

extern SemaphoreHandle_t UART1_Mutex;
extern xQueueHandle      LogQueue;

#ifdef LOG_ENABLE
inline void LogLine(char *Line, TickType_t Wait=2) { xQueueSend(LogQueue, &Line, Wait); }
#else
inline void LogLine(char *Line, TickType_t Wait=2) { }
#endif

#ifdef __cplusplus
  extern "C"
#endif
void vTaskCTRL(void* pvParameters);

