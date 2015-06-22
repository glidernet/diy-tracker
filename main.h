#include "parameters.h"

extern FlashParameters Parameters;

extern uint8_t  Vario_Note;
extern uint16_t Vario_Period;
extern uint16_t Vario_Fill;

void LED_PCB_Flash(uint8_t Time);

void Play(uint8_t Note, uint8_t Len);
uint8_t Play_Busy(void);

