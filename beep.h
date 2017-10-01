#ifndef __BEEP_H__
#define __BEEP_H__

#include <stdint.h>

void Beep_Configuration(void);

void Beep(uint16_t Period, uint8_t Duty=128); // [ms, 1/256] play sound with given period and duty (=volume)
void Beep_Note(uint8_t  Note);                // [VVOONNNN] play given volume/octave/note

#endif // __BEEP_H__
