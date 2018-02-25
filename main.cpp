#include <stdint.h>
#include <stdlib.h>

#include "stm32f10x_iwdg.h"

#include "hal.h"

#include "main.h"

#include "gps.h"                     // GPS task:  read the GPS receiver
#include "rf.h"                      // RF task:   transmit/received packets on radio
#include "proc.h"
#include "ctrl.h"                    // CTRL task: write log file to SD card
#include "sens.h"                    // SENS task: read I2C sensors (baro for now)
#include "knob.h"                    // KNOB task: read user knob


/*
#ifdef WITH_BEEPER

uint8_t  Vario_Note=0x00; // 0x40;
uint16_t Vario_Period=800;
uint16_t Vario_Fill=50;

static volatile uint16_t Vario_Time=0;

static volatile uint8_t Play_Note=0;             // Note being played
static volatile uint8_t Play_Counter=0;          // [ms] time counter

static FIFO<uint16_t, 8> Play_FIFO;              // queue of notes to play

void Play(uint8_t Note, uint8_t Len)             // [Note] [ms] put a new not to play in the queue
{ uint16_t Word = Note; Word<<=8; Word|=Len; Play_FIFO.Write(Word); }

uint8_t Play_Busy(void) { return Play_Counter; } // is a note being played right now ?

static void Play_TimerCheck(void)                // every ms serve the note playing
{ uint8_t Counter=Play_Counter;
  if(Counter)                                    // if counter non-zero
  { Counter--;                                   // decrement it
    if(!Counter) Beep_Note(Play_Note=0x00);      // if reached zero, stop playing the note
  }
  if(!Counter)                                   // if counter reached zero
  { if(!Play_FIFO.isEmpty())                     // check for notes in the queue
    { uint16_t Word=0; Play_FIFO.Read(Word);     // get the next note
      Beep_Note(Play_Note=Word>>8); Counter=Word&0xFF; }   // start playing it, load counter with the note duration
  }
  Play_Counter=Counter;

  uint16_t Time=Vario_Time;
  Time++; if(Time>=Vario_Period) Time=0;
  Vario_Time = Time;

  if(Counter==0)                            // when no notes are being played, make the vario sound
  { if(Time<=Vario_Fill)
    { if(Play_Note!=Vario_Note) Beep_Note(Play_Note=Vario_Note); }
    else
    { if(Play_Note!=0) Beep_Note(Play_Note=0x00); }
  }
}

#endif // WITH_BEEPER
*/

int main(void)
{
  IO_Configuration();                          // GPIO for LED and RF chip, SPI for RF, ADC, GPS GPIO and IRQ

  if(Parameters.ReadFromFlash()<0)             // read parameters from Flash
  { Parameters.setDefault();                   // if nov valid: set defaults
    Parameters.WriteToFlash(); }               // and write the defaults back to Flash
  // to overwrite parameters
  // Parameters.setTxTypeHW();
  // Parameters.setTxPower(+14); // for RFM69HW (H = up to +20dBm Tx power)
  // Parameters.WriteToFlash();

  UART_Configuration(Parameters.CONbaud, GPS_getBaudRate());

  xTaskCreate(vTaskCTRL,  "CTRL",   160, 0, tskIDLE_PRIORITY  , 0);  // CTRL: UART1, Console, SD log
#ifdef WITH_KNOB
  xTaskCreate(vTaskKNOB,  "KNOB",   100, 0, tskIDLE_PRIORITY  , 0);  // KNOB: read the knob (potentiometer wired to PB0)
#endif
  xTaskCreate(vTaskGPS,   "GPS",    100, 0, tskIDLE_PRIORITY+1, 0);  // GPS: GPS NMEA/PPS, packet encoding
  xTaskCreate(vTaskRF,    "RF",     120, 0, tskIDLE_PRIORITY+1, 0);  // RF: RF chip, time slots, frequency switching, packet reception and error correction
  xTaskCreate(vTaskPROC,  "PROC",   160, 0, tskIDLE_PRIORITY  , 0);  // processing received packets and prepare packets for transmission
  xTaskCreate(vTaskSENS,  "SENS",   128, 0, tskIDLE_PRIORITY+1, 0);  // SENS: BMP180 pressure, correlate with GPS

  vTaskStartScheduler();

  while(1)
  { }

}

// lot of things to do:
// + read NMEA user input
// + set Parameters in Flash from $POGNS
//
// + send received positions to console
// + send received positions to console as $POGNT
// + print number of detected transmission errors
// . avoid printing same position twice (from both time slots)
// + send Rx noise and packet stat. as $POGNR
//
// . optimize receiver sensitivity
// . use RF chip AFC or not ?
// . user RF chip continues AGC/RSSI or not ?
// + periodically refresh the RF chip config (after 60 seconds of Rx inactivity)
//
// + packet pools for queing
// + separate task for FEC correction
// + separate task for RX processing (retransmission decision)
// . good packets go to RX, bad packets go to FEC first
// + packet retransmission and strategy
// . limit or receive range to minimize false FEC decode
//
// + queue for sounds to be played on the buzzer
// + separate the UART code
//
// + use watchdog to restart in case of a hangup
// + print heap and task information when Ctlr-C pressed on the console
// + try to run on Maple Mini (there is more Flash)
//
// + SD card slot and FatFS
// + simple log system onto SD
// + regular log close and auto-resume when card inserted
// + DDMMYY in the log file name
// + proper buffering
// . IGC log (detect takeoff/landing ?)
// + FIFO as the log file buffer
// + file error crashes the system - resolved after the bug when baro was writing into a null pointer
//
// . auto-detect RFM69W or RFM69HW - possible at all ?
// + read RF chip temperature
// . compensate Rx/Tx frequency by RF chip temperature
//
// + measure the CPU temperature
// . measure VCC voltage: low battery indicator ?
// + resolve unstable ADC readout
//
// . detect when VK16u6 GPS fails below 2.7V supply
// . audible alert when GPS fails or absent ?
// + GPS: set higher baud rates
// + GPS: auto-baud
// + GPS: keep functioning when GPGSA is not there
// . check for loss of GPS data and declare fix loss
// + keep/count time (from GPS)
// . precise time from GPS and local clock correction
//
// + connect BMP180 pressure sensor
// . pressure sensor correction in Flash parameters ?
// + support BMP280 pressure sensor
// + support MS5607 pressure sensor
// + correlate pressure and GPS altitude
// . resolve extra dummy byte transfer for I2C_Read()
// + recover from I2C hang-up
// - BMP180 readout fails sometimes: initial delay after power-up or something else ?
// + send pressure data in $POGNB
// + vario sound
// - adapt vario integration time to climb/sink
// + separate task for BMP180 and other I2C sensors
// + send standard/pressure altitude in the packet ?
// . when measuring pressure avoid times when TX or LOG is active to reduce noise ?
//
// + stop transmission 60 sec after GPS lock is lost or mark the time as invalid
// . audible alert when RF chip fails ?
// + all hardware configure to main() before tasks start ?
//
// + objective code for RF chip
// . CC1101/CC1120/SPIRIT1/RFM95 code
// . properly handle transmitted position when GPS looses lock
// . NMEA commands to make sounds on the speaker
//
// + use TIM4.CH4 to drive the buzzer with double voltage
// . read compass, gyro, accel.
//
// + int math into a single file
// + bitcount: option to reduce code size: reduce lookup table from 256 to 16 bytes
//
// . thermal circling detection
// . measure/transmit/receive QNH
// . measure/transmit/receive wind
//
// . slow, long range mode
//
//
