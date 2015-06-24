//------------------------------------------------------------------------------
/*
  84x48 LCD with PCD8544 controller.

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

#ifndef __LCD5110_H
#define __LCD5110_H

//------------------------------------------------------------------------------

#include "ctrl.h"

//------------------------------------------------------------------------------

// Process control command.
// Called from ctrl task!
void DisplProcCtrl(ControlCmd cmd);

//------------------------------------------------------------------------------

#ifdef __cplusplus
  extern "C" {
#endif

//------------------------------------------------------------------------------

void vTaskLcd(void* pvParameters);

//------------------------------------------------------------------------------

} // extern "C"

#endif // __LCD5110_H
