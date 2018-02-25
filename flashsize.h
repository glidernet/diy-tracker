#ifndef __FLASHSIZE_H__
#define __FLASHSIZE_H__

#include <stdint.h>

uint32_t * const   FlashStart = (uint32_t *)0x08000000;                           // where the Flash memory starts
#define            FlashSize   ((uint16_t *)0x1FFFF7E0)                           // [KB] Flash memory size
uint16_t inline getFlashSizeKB(void) { return *FlashSize; }                       // [KB]
uint16_t inline getFlashPageSizeKB(void) { return 1+(getFlashSizeKB()>=256); }    // [KB]
uint8_t  inline getFlashPageSizeLog2(void) { return 10+(getFlashSizeKB()>=256); } // [log2([B])]

#endif // __FLASHSIZE_H__
