#include "FreeRTOS.h"
#include "task.h"
// #include "queue.h"

#ifdef WITH_SDLOG
extern SemaphoreHandle_t Log_Mutex;
void Log_Write(char Byte);
// #else
// inline void Log_Write(char Byte) { }
#endif

#ifdef __cplusplus
  extern "C"
#endif
void vTaskCTRL(void* pvParameters);
