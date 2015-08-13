#ifndef __FLASHSIZE_H__
#define __FLASHSIZE_H__

uint32_t * const   FlashStart = (uint32_t *)0x08000000;   // where the Flash memory starts
#define            FlashSize   ((uint16_t *)0x1FFFF7E0)   // [kB] Flash memory size
uint16_t inline getFlashSize(void) { return *FlashSize; }

#endif // __FLASHSIZE_H__
