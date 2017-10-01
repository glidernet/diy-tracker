#include "parameters.h"

extern FlashParameters Parameters;

extern uint8_t  Vario_Note;
extern uint16_t Vario_Period;
extern uint16_t Vario_Fill;

void LED_PCB_Flash(uint8_t Time);     // [ms] turn on the PCB LED for a given time

void Play(uint8_t Note, uint8_t Len); // put anote to play in the queue
uint8_t Play_Busy(void);              // check is the queue is empty or still busy playing ?

extern SemaphoreHandle_t CONS_Mutex; // console port Mutex
extern SemaphoreHandle_t ADC1_Mutex; // ADC1 Mutex for knob and other access to ADC
