//------------------------------------------------------------------------------
/*
  Raster fonts.

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

#ifndef __BMPFONTS_H
#define __BMPFONTS_H

//------------------------------------------------------------------------------

#include <stdint.h>

//------------------------------------------------------------------------------

enum BmpFonts
{
  font_4x6,
  font_5x7,
  font_7x15,
};

// special characters
#define CH_HALFSP  '\x80' // half space
#define CS_HALFSP  "\x80" // half space

#define CH_DEGREE  '\x81' // degree of Celsius sign
#define CS_DEGREE  "\x81" // degree of Celsius sign

#define CH_ARRUP   '\x82' // arrow up
#define CS_ARRUP   "\x82" // arrow up

#define CH_ARRDOWN '\x83' // arrow down
#define CS_ARRDOWN "\x83" // arrow down

//------------------------------------------------------------------------------

struct BmpFont
{
  uint8_t width;
  uint8_t height;
  uint8_t letterSpace;
  uint8_t lineSpace;
  const uint8_t* data;

  BmpFont(uint8_t width, uint8_t height,
    uint8_t letterSpace, uint8_t lineSpace, const uint8_t* data)
  : width(width), height(height),
    letterSpace(letterSpace), lineSpace(lineSpace), data(data)
  {}
};

//------------------------------------------------------------------------------

const BmpFont& GetBmpFont(BmpFonts font);

//------------------------------------------------------------------------------

#endif // __BMPFONTS_H
