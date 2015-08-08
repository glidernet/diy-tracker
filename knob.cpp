#include <stdlib.h>

#include <FreeRTOS.h>
#include <task.h>
#include <semphr.h>
#include <queue.h>

#include "knob.h"

#include "format.h"
#include "uart1.h"                         // console UART
#include "adc.h"

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

    uint16_t Knob     = ADC1_Read(ADC_Channel_8);
    uint16_t PrevKnob = ((uint16_t)Tick<<8)+0x80;
     int16_t Err      = Knob-PrevKnob;
    if(abs(Err)>=(0x80+0x20))                              // 0x20 is the histeresis to avoid noisy input
    { KNOB_Tick = (Tick = (Knob>>8)); Play(0x40+Tick, 10);
      // xSemaphoreTake(UART1_Mutex, portMAX_DELAY);
      // Format_UnsDec(UART1_Write, (uint16_t)Tick);
      // UART1_Write('\r'); UART1_Write('\n');
      // xSemaphoreGive(UART1_Mutex);
    }

    // uint16_t MCU_Temp = ADC1_Read(ADC_Channel_TempSensor);
    // uint16_t MCU_Vref = ADC1_Read(ADC_Channel_Vrefint);

  }

}
