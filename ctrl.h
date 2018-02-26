#include "FreeRTOS.h"
#include "task.h"

#ifdef WITH_SDLOG
extern SemaphoreHandle_t Log_Mutex;
void Log_Write(char Byte);
int Log_Free(void);
#endif

#ifdef __cplusplus
  extern "C"
#endif
void vTaskCTRL(void* pvParameters);
