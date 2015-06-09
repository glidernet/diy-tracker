
// void DisableRTCInterrupt(void) { NVIC->ICER[0] = (1<< (3 & 0x1F)); }
// void EnableRTCInterrupt(void)  { NVIC->ISER[0] = (1<< (3 & 0x1F)); }

#ifdef __cplusplus
  extern "C"
#endif
void RTC_IRQHandler (void)
{ if(RTC_GetITStatus(RTC_IT_SEC) != RESET)
  { RTC_ClearITPendingBit(RTC_IT_SEC); // Clear the RTC Second interrupt
    RTC_WaitForLastTask();
    /* UART1_TxChar('.'); */ }
}

void RTC_Configuration (void)
{
  RCC_APB1PeriphClockCmd (RCC_APB1Periph_PWR, ENABLE);          // enable clock for Power interface

  PWR_BackupAccessCmd(ENABLE);                                  // enable access to RTC, BDC registers

  RCC_LSEConfig (RCC_LSE_ON);                                   // enable LSE
  while (RCC_GetFlagStatus(RCC_FLAG_LSERDY) == RESET);          // Wait for LSERDY = 1 (LSE is ready)

  RTC_EnterConfigMode ();
  RCC_RTCCLKConfig (RCC_RTCCLKSource_LSE);                      // set RTC clock source
  RCC_RTCCLKCmd (ENABLE);                                       // enable RTC clock

  RTC_SetCounter(0);
  RTC_WaitForLastTask ();

  RTC_ITConfig(RTC_IT_SEC, ENABLE);                             // enable every-second interrupt
  RTC_WaitForLastTask ();

  RTC_SetPrescaler(32767);
  RTC_WaitForLastTask ();

  RTC_ExitConfigMode ();
  RTC_WaitForLastTask ();                                        // wait until write is finished

  RCC_BackupResetCmd(DISABLE);                                   // disable access to RTC registers

  NVIC_EnableIRQ(RTC_IRQn);
}


