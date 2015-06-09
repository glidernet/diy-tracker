#ifndef __UNIQUEID_H__
#define __UNIQUEID_H__

#define UniqueID ((uint32_t*)(0x1FFFF7E8))
uint32_t inline getUniqueID(uint8_t Idx) { return UniqueID[Idx]; }

#endif // __UNIQUEID_H__

