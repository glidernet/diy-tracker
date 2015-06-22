#include <stdint.h>

#include "stm32f10x_adc.h"

void ADC_Configuration(void);

uint16_t ADC1_Read(uint8_t Channel);  // convert and read given channel

// temperatue sensor channel = ADC_Channel_TempSensor = ADC_Channel_16
// internal reference channel = ADC_Channel_Vrefint   = ADC_Channel_17

