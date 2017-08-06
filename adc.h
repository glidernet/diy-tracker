#include <stdint.h>

#include "stm32f10x_adc.h"

void ADC_Configuration(void);

uint16_t ADC1_Read(uint8_t Channel);  // convert and read given channel

// #define            VREFIN_CAL   ((uint16_t *)0x1FFF7A2A)   //
// uint16_t inline getVREFIN_CAL(void) { return *VREFIN_CAL; }

// temperatue sensor channel = ADC_Channel_TempSensor = ADC_Channel_16
// internal reference channel = ADC_Channel_Vrefint   = ADC_Channel_17

