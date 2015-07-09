//------------------------------------------------------------------------------
/*
  IO pin configuration.

  Copyright (C) Richard Pecl 2015

  This program is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/
//------------------------------------------------------------------------------

#ifndef __IOPINS_H
#define __IOPINS_H

//------------------------------------------------------------------------------

#include "FreeRTOS.h"
#include "task.h"
#include "adc.h"

//------------------------------------------------------------------------------

// Digital input pin helper.
template<unsigned long int channel, uint16_t pin> struct DigInput
{
  enum Const
  {
    min_btn_press_time = 70,
  };

  static void Init(GPIOMode_TypeDef mode)
  {
    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Pin  = pin;
    GPIO_InitStructure.GPIO_Mode = mode;
    GPIO_Init(reinterpret_cast<GPIO_TypeDef*>(channel), &GPIO_InitStructure);
  }

  // test for high input state
  static bool IsHigh()
  {
    return GPIO_ReadInputDataBit(reinterpret_cast<GPIO_TypeDef*>(channel), pin) == Bit_SET;
  }

  static bool IsLow()
  {
    return !IsHigh();
  }

  // test for button press (pull up, pressed if low state - gnd connected)
  static bool BtnPress()
  {
    static bool lastSt   = false;
    static bool reported = false;
    static TickType_t lastTm = 0;

    if (IsLow() != lastSt)
    {
      lastSt = !lastSt;
      lastTm = xTaskGetTickCount();
      reported = false;
    }
    else if (lastSt && (xTaskGetTickCount() - lastTm) >= min_btn_press_time && !reported)
    //else if (lastSt && !reported)
    {
      reported = true;
      return true;
    }
    return false;
  }
};

//------------------------------------------------------------------------------

// Digital output pin helper.
template<unsigned long int channel, uint16_t pin> struct DigOutput
{
  static void Init(GPIOMode_TypeDef mode, GPIOSpeed_TypeDef speed)
  {
    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Pin  = pin;
    GPIO_InitStructure.GPIO_Mode = mode;
    GPIO_InitStructure.GPIO_Speed = speed;

    GPIO_Init(reinterpret_cast<GPIO_TypeDef*>(channel), &GPIO_InitStructure);
  }

  static void SetHigh()
  {
    GPIO_SetBits(reinterpret_cast<GPIO_TypeDef*>(channel), pin);
  }

  static void SetLow()
  {
    GPIO_ResetBits(reinterpret_cast<GPIO_TypeDef*>(channel), pin);
  }

  static void SetState(bool high)
  {
    GPIO_WriteBit(reinterpret_cast<GPIO_TypeDef*>(channel), pin, high ? Bit_SET : Bit_RESET);
  }

};

//------------------------------------------------------------------------------

// Analog input pin helper.
template<unsigned long int channel, uint16_t pin, uint8_t analogChannel>
struct AnalogInput
{
  static void Init()
  {
    //~ TODO: B0 is initialized in adc.cpp, should be refactored
    //~ GPIO_InitTypeDef GPIO_InitStructure;
    //~ GPIO_InitStructure.GPIO_Pin  = pin;
    //~ GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
    //~ GPIO_Init(reinterpret_cast<GPIO_TypeDef*>(channel), &GPIO_InitStructure);
    value = (uint16_t) -1;
  }

  // perform new A/D conversion and return the value
  // (can block for significant amount of time)
  static uint16_t Read()
  {
    return value = ADC1_Read(ADC_Channel_8);
  }

  // return value from the last conversion
  static uint16_t Get()
  {
    return value;
  }

protected:
  static uint16_t value;
};

template<unsigned long int channel, uint16_t pin, uint8_t analogChannel>
uint16_t AnalogInput<channel, pin, analogChannel>::value;


//------------------------------------------------------------------------------

// Analog button input pin helper.
// mean_value is the value expected on analog input if the button is pressed.
template<typename analogInput, uint16_t mean_value>
struct AnalogButton
{
  enum Const
  {
    min_btn_press_time = 70,
    // maximum  accepted value spread from defined mean_value
    max_spread = 300,
  };

  // test for immediate input state
  // (AnalogInput::Read() has to be called before)
  static bool InputMatched()
  {
    if (analogInput::Get() >= (mean_value - max_spread) && (analogInput::Get() <= (mean_value + max_spread)))
      return true;
    return false;
  }

  // test for button press
  // (AnalogInput::Read() has to be called before)
  static bool BtnPress()
  {
    static bool lastSt   = false;
    static bool reported = false;
    static TickType_t lastTm = 0;

    if (InputMatched() != lastSt)
    {
      lastSt = !lastSt;
      lastTm = xTaskGetTickCount();
      reported = false;
    }
    else if (lastSt && (xTaskGetTickCount() - lastTm) >= min_btn_press_time && !reported)
    //else if (lastSt && !reported)
    {
      reported = true;
      return true;
    }
    return false;
  }
};

//------------------------------------------------------------------------------

#ifdef WITH_BUTTONS
  // analog control buttons PIN configuration
  // (ladder Vcc + 2k2 - down/B0 - 1k0 - up - 2k2 - set: each button connects GND)
  typedef AnalogInput<GPIOB_BASE, GPIO_Pin_0, ADC_Channel_8> inADC8;
  typedef AnalogButton<inADC8, 0> inButtonDown;
  typedef AnalogButton<inADC8, 1280> inButtonUp;
  typedef AnalogButton<inADC8, 2427> inButtonSet;
#endif

#ifdef WITH_LCD5110
  // LCD5110 control pins
  typedef DigOutput<GPIOB_BASE, GPIO_Pin_12> outDisplRST;
  typedef DigOutput<GPIOB_BASE, GPIO_Pin_14> outDisplCE;
  typedef DigOutput<GPIOA_BASE, GPIO_Pin_8>  outDisplDC;
  typedef DigOutput<GPIOB_BASE, GPIO_Pin_15> outDisplDIN;
  typedef DigOutput<GPIOB_BASE, GPIO_Pin_13> outDisplCLK;
#endif

//------------------------------------------------------------------------------

// Initialize configurable IO pins.
void InitPins();

//------------------------------------------------------------------------------

#endif // __IOPINS_H
