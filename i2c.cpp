#include <FreeRTOS.h>
#include <task.h>
#include <semphr.h>
#include <queue.h>

// SemaphoreHandle_t I2C1_Mutex;

#include "stm32f10x_rcc.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_i2c.h"

void I2C_Reset(I2C_TypeDef* I2Cx)
{ I2C_SoftwareResetCmd(I2Cx ,ENABLE);
  vTaskDelay(1);
  I2C_SoftwareResetCmd(I2Cx, DISABLE);
  // I2C_DeInit(I2Cx);
  // I2C_Cmd(I2Cx, ENABLE);
}

void I2C_Restart(I2C_TypeDef* I2Cx, uint32_t ClockSpeed)
{
  I2C_InitTypeDef I2C_InitStructure;

  // I2C_SoftwareResetCmd(I2Cx, ENABLE);
  // I2C_SoftwareResetCmd(I2Cx, DISABLE);
  // I2C_Cmd(I2Cx, DISABLE);

  I2C_DeInit( I2Cx );                                      //

  I2C_InitStructure.I2C_Mode                = I2C_Mode_I2C;
  I2C_InitStructure.I2C_ClockSpeed          = ClockSpeed;
  I2C_InitStructure.I2C_DutyCycle           = I2C_DutyCycle_2;
  I2C_InitStructure.I2C_OwnAddress1         = 0x00;        // what own address should be set ?
  I2C_InitStructure.I2C_Ack                 = I2C_Ack_Enable;
  I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;

  I2C_Cmd(I2Cx, ENABLE);
  I2C_Init(I2Cx, &I2C_InitStructure);
  // I2C_AcknowledgeConfig(I2Cx, ENABLE); // ?

}

void I2C_Configuration(I2C_TypeDef* I2Cx, uint32_t ClockSpeed) // initialize I2C #1 interface and set it to given speed
{
  GPIO_InitTypeDef  GPIO_InitStructure;

  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);    // enable clock for GPIO port B
  if(I2Cx == I2C1)
  { RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1 , ENABLE );   // enable the clock for I2C1

    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_6 | GPIO_Pin_7; // I2C #1 runs on GPIO pins 6/7
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;        // 2MHz ?
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF_OD;         // open-drain, but can't make pull-ups
    GPIO_Init(GPIOB, &GPIO_InitStructure);                   // setup port B
  }
  else if(I2Cx == I2C2)
  { RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C2 , ENABLE );   // enable the clock for I2C1

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11; // I2C #1 runs on GPIO pins 6/7
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;        // 2MHz ?
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF_OD;         // open-drain, but can't make pull-ups
    GPIO_Init(GPIOB, &GPIO_InitStructure);                   // setup port B
  }

  I2C_Restart(I2Cx, ClockSpeed); }

// #define I2C_IdleWait taskYIELD()
static const uint16_t I2C_BusyTimeout  = 200;  // timeout to wait when busy
static const uint16_t I2C_EventTimeout =  20;  // timeout to wait on events during transfer

static uint8_t I2C_WaitWhileBusy(I2C_TypeDef* I2Cx) // wait while the bus is busy
{ for(uint16_t t=I2C_BusyTimeout; t; t--)
  { if(!I2C_GetFlagStatus(I2Cx, I2C_FLAG_BUSY)) return 0;
    taskYIELD(); }                             // do taskYIELD() while waiting - not clear
  return 0xB0; }

static uint8_t I2C_WaitEvent(I2C_TypeDef* I2Cx, uint32_t Event) // wait fot given event during a transfer
{ for(uint16_t t=I2C_EventTimeout; t; t--)
  { if(I2C_CheckEvent(I2Cx, Event)) return 0;
    taskYIELD(); }
  return 0xE0; }

// write Data (Len-gth bytes) into I2C Addr-ess, and given Reg-ister
uint8_t I2C_Write(I2C_TypeDef* I2Cx, uint8_t Addr, uint8_t Reg, const uint8_t *Data, uint8_t Len)
{ uint8_t Err=0;

  if(I2C_WaitWhileBusy(I2Cx) ) return 0xFF;

  I2C_AcknowledgeConfig(I2Cx,  ENABLE);

  I2C_GenerateSTART   (I2Cx, ENABLE);
  Err=I2C_WaitEvent   (I2Cx, I2C_EVENT_MASTER_MODE_SELECT); if(Err) goto Stop;

  I2C_Send7bitAddress (I2Cx, Addr<<1, I2C_Direction_Transmitter);
  Err=I2C_WaitEvent   (I2Cx, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED); if(Err) goto Stop;

  I2C_SendData        (I2Cx, Reg);
  Err=I2C_WaitEvent   (I2Cx, I2C_EVENT_MASTER_BYTE_TRANSMITTED); if(Err) goto Stop;

  for(uint8_t i=0; i<Len; i++)
  { I2C_SendData(I2Cx, Data[i]);
    Err=I2C_WaitEvent(I2Cx, I2C_EVENT_MASTER_BYTE_TRANSMITTED); if(Err) break; }

Stop:
  I2C_GenerateSTOP(I2Cx, ENABLE);
  return Err; }

// read Data (Len bytes) from I2C Addr-ess and given REG-ister
uint8_t I2C_Read(I2C_TypeDef* I2Cx, uint8_t Addr, uint8_t Reg, uint8_t *Data, uint8_t Len)
{ int Err=0;

  if(I2C_WaitWhileBusy   (I2Cx) ) return 0xFF;

  I2C_AcknowledgeConfig(I2Cx,  ENABLE);

  I2C_GenerateSTART      (I2Cx, ENABLE);
  Err=I2C_WaitEvent      (I2Cx, I2C_EVENT_MASTER_MODE_SELECT); if(Err) goto Stop;

  I2C_Send7bitAddress    (I2Cx, Addr<<1, I2C_Direction_Transmitter);
  Err=I2C_WaitEvent      (I2Cx, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED); if(Err) goto Stop;

  I2C_SendData           (I2Cx, Reg);
  Err=I2C_WaitEvent      (I2Cx, I2C_EVENT_MASTER_BYTE_TRANSMITTED); if(Err) goto Stop;

  I2C_GenerateSTART      (I2Cx, ENABLE);
  Err=I2C_WaitEvent      (I2Cx, I2C_EVENT_MASTER_MODE_SELECT); if(Err) goto Stop;

  I2C_Send7bitAddress    (I2Cx, Addr<<1, I2C_Direction_Receiver); 
  Err=I2C_WaitEvent      (I2Cx, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED); if(Err) goto Stop;

  for(uint8_t i=0; i<Len; i++)
  { if(i==(Len-1)) I2C_AcknowledgeConfig(I2Cx, DISABLE);
    // if(i==(Len-1)) I2C_GenerateSTOP(I2Cx, ENABLE); // <- strange, STOP should be here but it is unstable this way
    Err=I2C_WaitEvent(I2Cx, I2C_EVENT_MASTER_BYTE_RECEIVED); if(Err) goto Stop;
    Data[i]=I2C_ReceiveData(I2Cx); }

  I2C_GenerateSTOP(I2Cx, ENABLE); // <- STOP here make an extra "empty" byte being sent on the bus, but it works after all
  return Err;
 Stop:
  I2C_GenerateSTOP(I2C1, ENABLE);
  return Err; }

