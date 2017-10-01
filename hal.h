#include <stdint.h>



#include "beep.h"

void RCC_Configuration(void);            // Configure CPU clock, memory

void IO_Configuration(void);             // Configure I/O

void IWDG_Configuration(void);           // configure watchdog

void LED_PCB_On   (void);                // LED on the PCB for vizual indications
void LED_PCB_Off  (void);

void    RFM_RESET(uint8_t On);           // RF module reset
void    RFM_Select  (void);              // SPI select
void    RFM_Deselect(void);              // SPI de-select
uint8_t RFM_TransferByte(uint8_t Byte);  // SPI transfer/exchange a byte
bool    RFM_DIO0_isOn(void);             // query the IRQ state

#ifdef WITH_RF_IRQ
extern void (*RF_IRQ_Callback)(void);
#endif

uint16_t ADC_Read_MCU_Vtemp(void);       // MCU internal temperature sensor
uint16_t ADC_Read_MCU_Vref (void);       // MCU internal reference voltage
uint16_t ADC_Read_Vbatt    (void);       // divider wired to the battery
uint16_t ADC_Read_Knob     (void);       // potentiometer (user input)

#ifdef WITH_PPS_IRQ
extern void (*GPS_PPS_IRQ_Callback)(uint32_t TickCount, uint32_t TickTime);
#endif
#ifdef WITH_GPS_ENABLE                    // if there is line to control the GPS ON/OFF
void GPS_DISABLE(void);
void GPS_ENABLE (void);
#endif
#ifdef WITH_GPS_PPS                       // if GPS PPS is connected
bool GPS_PPS_isOn(void);
#endif

void UART_Configuration(int CONS_BaudRate, int GPS_BaudRate);

int  CONS_UART_Read       (uint8_t &Byte); // non-blocking
void CONS_UART_Write      (char     Byte); // blocking
void CONS_UART_SetBaudrate(int BaudRate);
int   GPS_UART_Read       (uint8_t &Byte); // non-blocking
void  GPS_UART_Write      (char     Byte); // blocking
void  GPS_UART_SetBaudrate(int BaudRate);

