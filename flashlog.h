#include <stdint.h>

#include "ogn.h"

uint16_t FlashLog_OpenForRead(uint32_t *StartTime=0);
uint32_t FlashLog_ReadPage(const uint32_t *&Page);

uint16_t FlashLog_OpenForWrite(void);
bool     FlashLog_Process(OGN_Packet &Packet, uint32_t Time);

uint8_t FlashLog_Print(char *Output);
