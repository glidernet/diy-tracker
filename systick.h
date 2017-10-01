#ifndef __SYSTICK_H__
#define __SYSTICK_H__

// on SysTick: 4.4 of http://infocenter.arm.com/help/topic/com.arm.doc.dui0552a/DUI0552A_cortex_m3_dgug.pdf

#define SYSTICK_RELOAD_REG             ( * ( ( volatile uint32_t * ) 0xe000e014 ) )         // reload value when counter reaches zero
#define SYSTICK_CURRENT_VALUE_REG      ( * ( ( volatile uint32_t * ) 0xe000e018 ) )         // this 24-bit counter counts down from the reload value
inline uint32_t getSysTick_Count(void)              { return SYSTICK_CURRENT_VALUE_REG; }   // [CPU tick]
inline uint32_t getSysTick_Reload(void)             { return SYSTICK_RELOAD_REG; }          // [CPU tick]
inline void     setSysTick_Reload(uint32_t Reload)  {        SYSTICK_RELOAD_REG = Reload; } // [CPU tick]

const  uint32_t SysTickPeriod = configCPU_CLOCK_HZ/configTICK_RATE_HZ;

#endif // __SYSTICK_H__

