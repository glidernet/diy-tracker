#include "parameters.h"

extern FlashParameters Parameters;

extern uint8_t  Vario_Note;
extern uint16_t Vario_Period;
extern uint16_t Vario_Fill;

void LED_PCB_Flash(uint8_t Time);

void Play(uint8_t Note, uint8_t Len);
uint8_t Play_Busy(void);

#include "uart1.h"
#include "uart2.h"

extern SemaphoreHandle_t CONS_Mutex; // console port Mutex

// here we can tell which UART is to be used for consolse and which for the GPS
#ifdef WITH_SWAP_UARTS

#define CONS_UART_Read  UART2_Read
#define CONS_UART_Write UART2_Write
#define GPS_UART_Read  UART1_Read
#define GPS_UART_Write UART1_Write

#else

#define CONS_UART_Read  UART1_Read
#define CONS_UART_Write UART1_Write
#define GPS_UART_Read  UART2_Read
#define GPS_UART_Write UART2_Write

#endif
