#include "adc.h"

#include "stm32f10x_rcc.h"

void ADC_Configuration(void)
{
  ADC_InitTypeDef  ADC_InitStructure;
  RCC_ADCCLKConfig(RCC_PCLK2_Div6);                                    // PCLK2 is the APB2 clock, ADCCLK = PCLK2/6 = 60/6 = 10MHz
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);                 // Enable ADC1 clock so that we can talk to it
  ADC_DeInit(ADC1);                                                    // Put everything back to power-on defaults

  ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;                   // ADC2 not depenedent on ADC1
  ADC_InitStructure.ADC_ScanConvMode = DISABLE;                        // Disable the scan conversion so we do one at a time
  ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;                  // Don't do contimuous conversions - do them on demand
  ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;  // Start conversin by software, not an external trigger
  ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;               // Conversions are 12 bit - put them in the lower 12 bits of the result
  ADC_InitStructure.ADC_NbrOfChannel = 1;                              // How many channels would be used by the sequencer
  ADC_Init(ADC1, &ADC_InitStructure);
  ADC_Cmd(ADC1, ENABLE);

  ADC_ResetCalibration(ADC1);                                           // Enable ADC1 reset calibaration register
  while(ADC_GetResetCalibrationStatus(ADC1));                           // Check the end of ADC1 reset calibration register
  ADC_StartCalibration(ADC1);                                           // Start ADC1 calibaration
  while(ADC_GetCalibrationStatus(ADC1));                                // Check the end of ADC1 calibration
  ADC_TempSensorVrefintCmd(ENABLE);                                     // enable Vrefint and Temperature sensor
}

uint16_t ADC1_Read(uint8_t Channel)                                     // convert and read given channel
{
  ADC_RegularChannelConfig(ADC1, Channel, 1, ADC_SampleTime_7Cycles5);
  ADC_SoftwareStartConvCmd(ADC1, ENABLE);                               // Start the conversion
  while(ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC) == RESET);                // Wait until conversion complete
  return ADC_GetConversionValue(ADC1);                                  // Get the conversion value
}

// temperatue sensor channel = ADC_Channel_TempSensor = ADC_Channel_16
// internal reference channel = ADC_Channel_Vrefint   = ADC_Channel_17

