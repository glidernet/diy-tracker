#include <stdlib.h>

#include "hal.h"

#include <FreeRTOS.h>
#include <task.h>
#include <semphr.h>
#include <queue.h>

#include "knob.h"

#include "format.h"

#include "main.h"

volatile uint8_t KNOB_Tick=15;

#ifdef __cplusplus
  extern "C"
#endif
void vTaskKNOB(void* pvParameters)
{ // ADC_Configuration();

  uint8_t Tick=KNOB_Tick;
  for( ; ; )
  { vTaskDelay(40);
    xSemaphoreTake(ADC1_Mutex, portMAX_DELAY);
    uint16_t Knob     = ADC_Read_Knob();
    xSemaphoreGive(ADC1_Mutex);
    uint16_t PrevKnob = ((uint16_t)Tick<<8)+0x80;
     int16_t Err      = Knob-PrevKnob;
    if(abs(Err)>=(0x80+0x20))                              // 0x20 is the histeresis to avoid noisy input
    { KNOB_Tick = (Tick = (Knob>>8)); Play(0x40+Tick, 10);
      // xSemaphoreTake(CONS_Mutex, portMAX_DELAY);
      // Format_UnsDec(CONS_UART_Write, (uint16_t)Tick);
      // CONS_UART_Write('\r'); CONS_UART_Write('\n');
      // xSemaphoreGive(CONS_Mutex);
    }

  }

}
