#include "spi1.h"

struct RFStatus {
  uint8_t ChipTemp;     // [degC]    trx chip temperature
  uint8_t RX_Packets;   // [packets] counts received packets
  uint8_t RX_Idle;      // [sec]     time the receiver did not get any packets
  int32_t RX_RssiLow;   // [-0.5dBm] background noise level on two frequencies
  int32_t RX_RssiUpp;   // [-0.5dBm]
};

const RFStatus& GetRFStatus();

void RFM69_GPIO_Configuration(); // prepare GPIO pins to talk to RFM69 module

extern uint32_t RX_Random;       // random number derived from reception noise

#ifdef __cplusplus
  extern "C"
#endif
void vTaskRF(void* pvParameters);



