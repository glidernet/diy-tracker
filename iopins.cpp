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

#include "stm32f10x_gpio.h"

#include "iopins.h"

//------------------------------------------------------------------------------


// Initialize configurable IO pins.
void InitPins()
{
  #ifdef WITH_BUTTONS
    inButtonUp::Init(GPIO_Mode_IPU);
    inButtonDown::Init(GPIO_Mode_IPU);
    inButtonSet::Init(GPIO_Mode_IPU);
  #endif

  #ifdef WITH_LCD5110
    //RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB, ENABLE);
    outDisplRST::Init(GPIO_Mode_Out_PP, GPIO_Speed_2MHz);
    outDisplCE::Init(GPIO_Mode_Out_PP, GPIO_Speed_2MHz);
    outDisplDC::Init(GPIO_Mode_Out_PP, GPIO_Speed_2MHz);
    outDisplDIN::Init(GPIO_Mode_Out_PP, GPIO_Speed_2MHz);
    outDisplCLK::Init(GPIO_Mode_Out_PP, GPIO_Speed_2MHz);
  #endif
}

//------------------------------------------------------------------------------
