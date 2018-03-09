#ifndef __HAL_H__
#define __HAL_H__

#define WITH_STM32

#define HARDWARE_ID 0x01
#define SOFTWARE_ID 0x01

#include <stdint.h>

#include <FreeRTOS.h>
#include <task.h>
#include <semphr.h>
#include <queue.h>

uint32_t getUniqueAddress(void);

#include "uniqueid.h"
#include "parameters.h"
#include "beep.h"
#include "knob.h"

#ifdef WITH_MAVLINK
extern uint8_t  MAV_Seq;                 // sequence number for MAVlink message sent out
const  uint8_t  MAV_SysID = 1;
#endif

extern FlashParameters Parameters;

void RCC_Configuration(void);            // Configure CPU clock, memory

void IO_Configuration(void);             // Configure I/O

void IWDG_Configuration(void);           // configure watchdog

void LED_PCB_On   (void);                // LED on the PCB for vizual indications
void LED_PCB_Off  (void);

void LED_RX_On    (void);
void LED_RX_Off   (void);

void LED_TX_On    (void);
void LED_TX_Off   (void);

// =======================================================================================================

void    RFM_RESET(uint8_t On);           // RF module reset
void    RFM_Select  (void);              // SPI select
void    RFM_Deselect(void);              // SPI de-select
uint8_t RFM_TransferByte(uint8_t Byte);  // SPI transfer/exchange a byte
bool    RFM_IRQ_isOn(void);              // query the IRQ state

#ifdef WITH_RF_IRQ
extern void (*RF_IRQ_Callback)(void);
#endif

// =======================================================================================================

extern SemaphoreHandle_t ADC1_Mutex; // ADC1 Mutex for knob and other access to ADC

// uint16_t ADC_Read_MCU_Vtemp(void);       // MCU internal temperature sensor
// uint16_t ADC_Read_MCU_Vref (void);       // MCU internal reference voltage
// uint16_t ADC_Read_Vbatt    (void);       // divider wired to the battery
uint16_t ADC_Read_Knob     (void);       // potentiometer (user input)

// uint16_t Measure_MCU_Vref(void);
 int16_t Measure_MCU_Temp(void);         // [0.1degC]
uint16_t Measure_MCU_VCC(void);          // [1mV]
uint16_t Measure_Vbatt(void);            // [1mV]

// =======================================================================================================

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

// =======================================================================================================

void UART_Configuration(int CONS_BaudRate, int GPS_BaudRate);

extern SemaphoreHandle_t CONS_Mutex; // console port Mutex

int  CONS_UART_Read       (uint8_t &Byte); // non-blocking
void CONS_UART_Write      (char     Byte); // blocking
int  CONS_UART_Free       (void);          // how many bytes can be written to the transmit buffer
int  CONS_UART_Full       (void);          // how many bytes already in the transmit buffer
void CONS_UART_SetBaudrate(int BaudRate);
int   GPS_UART_Read       (uint8_t &Byte); // non-blocking
void  GPS_UART_Write      (char     Byte); // blocking
void  GPS_UART_SetBaudrate(int BaudRate);

void LED_PCB_Flash(uint8_t Time);     // [ms] turn on the PCB LED for a given time
#ifdef WITH_LED_RX
void LED_RX_Flash(uint8_t Time);      // [ms] turn on the RX LED for a given time
#endif
#ifdef WITH_LED_TX
void LED_TX_Flash(uint8_t Time);      // [ms] turn on the TX LED for a given time
#endif
#ifdef WITH_LED_BAT
void LED_BAT_Flash(uint8_t Time);     // [ms] turn on the BAT LED for a given time
#endif

// =======================================================================================================

extern uint8_t  Vario_Note;
extern uint16_t Vario_Period;
extern uint16_t Vario_Fill;

const uint8_t Play_Vol_0 = 0x00;
const uint8_t Play_Vol_1 = 0x40;
const uint8_t Play_Vol_2 = 0x80;
const uint8_t Play_Vol_3 = 0xC0;

const uint8_t Play_Oct_0 = 0x00;
const uint8_t Play_Oct_1 = 0x10;
const uint8_t Play_Oct_2 = 0x20;
const uint8_t Play_Oct_3 = 0x30;

void Play(uint8_t Note, uint8_t Len); // put anote to play in the queue
uint8_t Play_Busy(void);              // check is the queue is empty or still busy playing ?

// =======================================================================================================

#define I2C_SPEED 400000

// extern SemaphoreHandle_t I2C_Mutex;        // I2C port Mutex (OLED and Baro)

uint8_t I2C_Read (uint8_t Bus, uint8_t Addr, uint8_t Reg, uint8_t *Data, uint8_t Len, uint8_t Wait=10);
uint8_t I2C_Write(uint8_t Bus, uint8_t Addr, uint8_t Reg, uint8_t *Data, uint8_t Len, uint8_t Wait=10);

template <class Type>
 inline uint8_t I2C_Write(uint8_t Bus, uint8_t Addr, uint8_t Reg, Type &Object, uint8_t Wait=10)
{ return I2C_Write(Bus, Addr, Reg, (uint8_t *)&Object, sizeof(Type), Wait); }

template <class Type>
 inline uint8_t I2C_Read (uint8_t Bus, uint8_t Addr, uint8_t Reg, Type &Object, uint8_t Wait=10)
{ return I2C_Read (Bus, Addr, Reg, (uint8_t *)&Object, sizeof(Type), Wait); }

uint8_t I2C_Restart(uint8_t Bus);

// =======================================================================================================

#endif // __HAL_H__

