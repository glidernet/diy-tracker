// Beeper is on PB8 = TIM4.CH3

void Beep_Configuration(void)
{ 
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);

  TIM_TimeBaseInitTypeDef SetupTimer;
  SetupTimer.TIM_ClockDivision = TIM_CKD_DIV1;
  SetupTimer.TIM_CounterMode = TIM_CounterMode_Up;
  SetupTimer.TIM_Prescaler = 1-1;
  SetupTimer.TIM_Period    = 60000-1;
  SetupTimer.TIM_RepetitionCounter = 0;
  TIM_TimeBaseInit(TIM4, &SetupTimer);
  TIM_Cmd(TIM4, ENABLE);

  TIM_OCInitTypeDef TIM_OCStruct;
  TIM_OCStruct.TIM_OCMode = TIM_OCMode_PWM2;
  TIM_OCStruct.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCStruct.TIM_OCPolarity = TIM_OCPolarity_Low;
  TIM_OCStruct.TIM_Pulse = 0;                                  // 0% duty cycle
  TIM_OC3Init(TIM4, &TIM_OCStruct);                            // TIM4.CH3
  TIM_OC3PreloadConfig(TIM4, TIM_OCPreload_Enable);

  // GPIO_PinAFConfig(GPIOB, GPIO_PinSource8, GPIO_AF_TIM4);

  RCC_APB2PeriphClockCmd (RCC_APB2Periph_GPIOB | RCC_APB2Periph_AFIO, ENABLE);

  GPIO_InitTypeDef  GPIO_InitStruct;
  GPIO_InitStruct.GPIO_Pin = GPIO_Pin_8;                      // PB8
  GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOB, &GPIO_InitStruct);

}

void Beep(uint16_t Period, uint8_t Duty=128)
{ 
  uint32_t Limit = ((uint32_t)Period * (uint32_t)Duty ) >> 8;
  TIM_SetCompare3(TIM4, (uint16_t)Limit); // TIM4->CCR3 = Limit;
  TIM_SetAutoreload(TIM4, Period); // TIM4->ARR = Period;
}
