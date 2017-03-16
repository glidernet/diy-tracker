#include "spi1.h"

void RFM_GPIO_Configuration(); // prepare GPIO pins to talk to RFM69 module

extern uint32_t RX_Random;       // random number derived from reception noise

#ifdef __cplusplus
  extern "C"
#endif
void vTaskRF(void* pvParameters);



