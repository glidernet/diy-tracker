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

#include <bmpfonts.h>

//------------------------------------------------------------------------------


// 4x6 font bitmap:
// This table contains the hex values that represent pixels for a
// font that is 4 pixels wide and 6 pixels high. Each byte in a row
// represents one, 6-pixel, vertical column of a character. 4 bytes
// per character.
static const uint8_t font4x6bmp[][4] = {
  {0x00, 0x00, 0x00, 0x00}, // (space)
  {0x00, 0x2F, 0x00, 0x00}, // !
  {0x03, 0x00, 0x03, 0x00}, // "
  {0x3F, 0x0A, 0x3F, 0x0A}, // #
  {0x03, 0x02, 0x07, 0x00}, // $
  {0x33, 0x0B, 0x34, 0x33}, // %
  {0x1A, 0x25, 0x2A, 0x10}, // &
  {0x00, 0x03, 0x00, 0x00}, // '
  {0x00, 0x1E, 0x21, 0x00}, // (
  {0x00, 0x21, 0x1E, 0x00}, // )
  {0x0A, 0x04, 0x0A, 0x00}, // *
  {0x04, 0x0E, 0x04, 0x00}, // +
  {0x20, 0x10, 0x00, 0x00}, // ,
  {0x04, 0x04, 0x04, 0x00}, // -
  {0x00, 0x20, 0x00, 0x00}, // .
  {0x30, 0x08, 0x04, 0x03}, // /
  {0x1E, 0x29, 0x25, 0x1E}, // 0
  {0x22, 0x3F, 0x20, 0x00}, // 1
  {0x32, 0x29, 0x25, 0x22}, // 2
  {0x12, 0x21, 0x25, 0x1A}, // 3
  {0x0C, 0x0A, 0x3F, 0x08}, // 4
  {0x27, 0x25, 0x25, 0x19}, // 5
  {0x1E, 0x25, 0x25, 0x19}, // 6
  {0x01, 0x39, 0x05, 0x03}, // 7
  {0x1A, 0x25, 0x25, 0x1A}, // 8
  {0x06, 0x29, 0x29, 0x1E}, // 9
  {0x00, 0x14, 0x00, 0x00}, // :
  {0x20, 0x14, 0x00, 0x00}, // ;
  {0x08, 0x14, 0x22, 0x22}, // <
  {0x0A, 0x0A, 0x0A, 0x0A}, // =
  {0x22, 0x22, 0x14, 0x08}, // >
  {0x02, 0x01, 0x2D, 0x02}, // ?
  {0x1E, 0x21, 0x2D, 0x2E}, // @
  {0x3E, 0x09, 0x09, 0x3E}, // A
  {0x3F, 0x25, 0x25, 0x1A}, // B
  {0x1E, 0x21, 0x21, 0x12}, // C
  {0x3F, 0x21, 0x21, 0x1E}, // D
  {0x3F, 0x25, 0x25, 0x21}, // E
  {0x3F, 0x05, 0x05, 0x01}, // F
  {0x1E, 0x21, 0x29, 0x1A}, // G
  {0x3F, 0x04, 0x04, 0x3F}, // H
  {0x21, 0x3F, 0x21, 0x00}, // I
  {0x10, 0x21, 0x21, 0x1F}, // J
  {0x3F, 0x0C, 0x12, 0x21}, // K
  {0x3F, 0x20, 0x20, 0x20}, // L
  {0x3F, 0x02, 0x06, 0x3F}, // M
  {0x3F, 0x04, 0x08, 0x3F}, // N
  {0x1E, 0x21, 0x21, 0x1E}, // O
  {0x3F, 0x09, 0x09, 0x06}, // P
  {0x1E, 0x21, 0x11, 0x2E}, // Q
  {0x3F, 0x09, 0x09, 0x36}, // R
  {0x22, 0x25, 0x25, 0x19}, // S
  {0x01, 0x3F, 0x01, 0x01}, // T
  {0x1F, 0x20, 0x20, 0x1F}, // U
  {0x0F, 0x30, 0x10, 0x0F}, // V
  {0x3F, 0x10, 0x18, 0x3F}, // W
  {0x33, 0x0C, 0x0C, 0x33}, // X
  {0x07, 0x38, 0x04, 0x03}, // Y
  {0x31, 0x29, 0x25, 0x23}, // Z
  {0x00, 0x3F, 0x21, 0x00}, // [
  {0x03, 0x04, 0x08, 0x30}, // "\"
  {0x00, 0x21, 0x3F, 0x00}, // ]
  {0x02, 0x01, 0x02, 0x00}, // ^
  {0x20, 0x20, 0x20, 0x20}, // _
  {0x00, 0x01, 0x02, 0x00}, // `
  {0x10, 0x2A, 0x2A, 0x3C}, // a
  {0x3F, 0x24, 0x24, 0x18}, // b
  {0x1C, 0x22, 0x22, 0x14}, // c
  {0x18, 0x24, 0x24, 0x3F}, // d
  {0x1C, 0x2A, 0x2A, 0x0C}, // e
  {0x3E, 0x09, 0x01, 0x02}, // f
  {0x24, 0x2A, 0x2A, 0x1E}, // g
  {0x3F, 0x08, 0x04, 0x38}, // h
  {0x24, 0x3D, 0x20, 0x00}, // i
  {0x10, 0x20, 0x20, 0x1D}, // j
  {0x3F, 0x08, 0x14, 0x22}, // k
  {0x21, 0x3F, 0x20, 0x00}, // l
  {0x3E, 0x02, 0x1C, 0x3E}, // m
  {0x3E, 0x02, 0x02, 0x3C}, // n
  {0x1C, 0x22, 0x22, 0x1C}, // o
  {0x3E, 0x0A, 0x0A, 0x04}, // p
  {0x04, 0x0A, 0x0A, 0x3E}, // q
  {0x3E, 0x02, 0x02, 0x04}, // r
  {0x24, 0x2A, 0x2A, 0x12}, // s
  {0x04, 0x1E, 0x24, 0x20}, // t
  {0x1E, 0x20, 0x20, 0x3E}, // u
  {0x1E, 0x20, 0x10, 0x0E}, // v
  {0x1E, 0x38, 0x20, 0x1E}, // w
  {0x36, 0x08, 0x08, 0x36}, // x
  {0x26, 0x28, 0x28, 0x1E}, // y
  {0x32, 0x2A, 0x2A, 0x26}, // z
  {0x04, 0x1B, 0x21, 0x00}, // {
  {0x00, 0x3F, 0x00, 0x00}, // |
  {0x00, 0x21, 0x1B, 0x04}, // }
  {0x04, 0x02, 0x04, 0x02}, // ~
  {0x3F, 0x35, 0x35, 0x3F}, // DEL
  {0x00, 0x00, 0x00, 0x00}, // dummy CH_HALFSP
  {0x02, 0x05, 0x02, 0x00}, // CH_DEGREE
  {0x04, 0x3E, 0x04, 0x00}, // CH_ARRUP
  {0x10, 0x3E, 0x10, 0x00}  // CH_ARRDOWN
}; // font4x6bmp

struct BmpFont4x6 : public BmpFont
{
  BmpFont4x6()
  : BmpFont(4, 6, 1, 1, (const uint8_t*) font4x6bmp)
  {}
};

static const BmpFont4x6 font4x6;

//------------------------------------------------------------------------------

// 5x7 font bitmap:
// This table contains the hex values that represent pixels for a
// font that is 5 pixels wide and 7 pixels high. Each byte in a row
// represents one, 7-pixel, vertical column of a character. 5 bytes
// per character.
static const uint8_t font5x7bmp[][5] = {
  // First 32 characters (0x00-0x19) are ignored. These are
  // non-displayable, control characters.
  {0x00, 0x00, 0x00, 0x00, 0x00}, // 0x20
  {0x00, 0x00, 0x5f, 0x00, 0x00}, // 0x21 !
  {0x00, 0x07, 0x00, 0x07, 0x00}, // 0x22 "
  {0x14, 0x7f, 0x14, 0x7f, 0x14}, // 0x23 #
  {0x24, 0x2a, 0x7f, 0x2a, 0x12}, // 0x24 $
  {0x23, 0x13, 0x08, 0x64, 0x62}, // 0x25 %
  {0x36, 0x49, 0x55, 0x22, 0x50}, // 0x26 &
  {0x00, 0x05, 0x03, 0x00, 0x00}, // 0x27 '
  {0x00, 0x1c, 0x22, 0x41, 0x00}, // 0x28 (
  {0x00, 0x41, 0x22, 0x1c, 0x00}, // 0x29 )
  {0x14, 0x08, 0x3e, 0x08, 0x14}, // 0x2a *
  {0x08, 0x08, 0x3e, 0x08, 0x08}, // 0x2b +
  {0x00, 0x50, 0x30, 0x00, 0x00}, // 0x2c ,
  {0x08, 0x08, 0x08, 0x08, 0x08}, // 0x2d -
  {0x00, 0x60, 0x60, 0x00, 0x00}, // 0x2e .
  {0x20, 0x10, 0x08, 0x04, 0x02}, // 0x2f /
  {0x3e, 0x51, 0x49, 0x45, 0x3e}, // 0x30 0
  {0x00, 0x42, 0x7f, 0x40, 0x00}, // 0x31 1
  {0x42, 0x61, 0x51, 0x49, 0x46}, // 0x32 2
  {0x21, 0x41, 0x45, 0x4b, 0x31}, // 0x33 3
  {0x18, 0x14, 0x12, 0x7f, 0x10}, // 0x34 4
  {0x27, 0x45, 0x45, 0x45, 0x39}, // 0x35 5
  {0x3c, 0x4a, 0x49, 0x49, 0x30}, // 0x36 6
  {0x01, 0x71, 0x09, 0x05, 0x03}, // 0x37 7
  {0x36, 0x49, 0x49, 0x49, 0x36}, // 0x38 8
  {0x06, 0x49, 0x49, 0x29, 0x1e}, // 0x39 9
  {0x00, 0x36, 0x36, 0x00, 0x00}, // 0x3a :
  {0x00, 0x56, 0x36, 0x00, 0x00}, // 0x3b ;
  {0x08, 0x14, 0x22, 0x41, 0x00}, // 0x3c <
  {0x14, 0x14, 0x14, 0x14, 0x14}, // 0x3d =
  {0x00, 0x41, 0x22, 0x14, 0x08}, // 0x3e >
  {0x02, 0x01, 0x51, 0x09, 0x06}, // 0x3f ?
  {0x32, 0x49, 0x79, 0x41, 0x3e}, // 0x40 @
  {0x7e, 0x11, 0x11, 0x11, 0x7e}, // 0x41 A
  {0x7f, 0x49, 0x49, 0x49, 0x36}, // 0x42 B
  {0x3e, 0x41, 0x41, 0x41, 0x22}, // 0x43 C
  {0x7f, 0x41, 0x41, 0x22, 0x1c}, // 0x44 D
  {0x7f, 0x49, 0x49, 0x49, 0x41}, // 0x45 E
  {0x7f, 0x09, 0x09, 0x09, 0x01}, // 0x46 F
  {0x3e, 0x41, 0x49, 0x49, 0x7a}, // 0x47 G
  {0x7f, 0x08, 0x08, 0x08, 0x7f}, // 0x48 H
  {0x00, 0x41, 0x7f, 0x41, 0x00}, // 0x49 I
  {0x20, 0x40, 0x41, 0x3f, 0x01}, // 0x4a J
  {0x7f, 0x08, 0x14, 0x22, 0x41}, // 0x4b K
  {0x7f, 0x40, 0x40, 0x40, 0x40}, // 0x4c L
  {0x7f, 0x02, 0x0c, 0x02, 0x7f}, // 0x4d M
  {0x7f, 0x04, 0x08, 0x10, 0x7f}, // 0x4e N
  {0x3e, 0x41, 0x41, 0x41, 0x3e}, // 0x4f O
  {0x7f, 0x09, 0x09, 0x09, 0x06}, // 0x50 P
  {0x3e, 0x41, 0x51, 0x21, 0x5e}, // 0x51 Q
  {0x7f, 0x09, 0x19, 0x29, 0x46}, // 0x52 R
  {0x46, 0x49, 0x49, 0x49, 0x31}, // 0x53 S
  {0x01, 0x01, 0x7f, 0x01, 0x01}, // 0x54 T
  {0x3f, 0x40, 0x40, 0x40, 0x3f}, // 0x55 U
  {0x1f, 0x20, 0x40, 0x20, 0x1f}, // 0x56 V
  {0x3f, 0x40, 0x38, 0x40, 0x3f}, // 0x57 W
  {0x63, 0x14, 0x08, 0x14, 0x63}, // 0x58 X
  {0x07, 0x08, 0x70, 0x08, 0x07}, // 0x59 Y
  {0x61, 0x51, 0x49, 0x45, 0x43}, // 0x5a Z
  {0x00, 0x7f, 0x41, 0x41, 0x00}, // 0x5b [
  {0x02, 0x04, 0x08, 0x10, 0x20}, // 0x5c \.
  {0x00, 0x41, 0x41, 0x7f, 0x00}, // 0x5d ]
  {0x04, 0x02, 0x01, 0x02, 0x04}, // 0x5e ^
  {0x40, 0x40, 0x40, 0x40, 0x40}, // 0x5f _
  {0x00, 0x01, 0x02, 0x04, 0x00}, // 0x60 `
  {0x20, 0x54, 0x54, 0x54, 0x78}, // 0x61 a
  {0x7f, 0x48, 0x44, 0x44, 0x38}, // 0x62 b
  {0x38, 0x44, 0x44, 0x44, 0x20}, // 0x63 c
  {0x38, 0x44, 0x44, 0x48, 0x7f}, // 0x64 d
  {0x38, 0x54, 0x54, 0x54, 0x18}, // 0x65 e
  {0x08, 0x7e, 0x09, 0x01, 0x02}, // 0x66 f
  {0x0c, 0x52, 0x52, 0x52, 0x3e}, // 0x67 g
  {0x7f, 0x08, 0x04, 0x04, 0x78}, // 0x68 h
  {0x00, 0x44, 0x7d, 0x40, 0x00}, // 0x69 i
  {0x20, 0x40, 0x44, 0x3d, 0x00}, // 0x6a j
  {0x7f, 0x10, 0x28, 0x44, 0x00}, // 0x6b k
  {0x00, 0x41, 0x7f, 0x40, 0x00}, // 0x6c l
  {0x7c, 0x04, 0x18, 0x04, 0x78}, // 0x6d m
  {0x7c, 0x08, 0x04, 0x04, 0x78}, // 0x6e n
  {0x38, 0x44, 0x44, 0x44, 0x38}, // 0x6f o
  {0x7c, 0x14, 0x14, 0x14, 0x08}, // 0x70 p
  {0x08, 0x14, 0x14, 0x18, 0x7c}, // 0x71 q
  {0x7c, 0x08, 0x04, 0x04, 0x08}, // 0x72 r
  {0x48, 0x54, 0x54, 0x54, 0x20}, // 0x73 s
  {0x04, 0x3f, 0x44, 0x40, 0x20}, // 0x74 t
  {0x3c, 0x40, 0x40, 0x20, 0x7c}, // 0x75 u
  {0x1c, 0x20, 0x40, 0x20, 0x1c}, // 0x76 v
  {0x3c, 0x40, 0x30, 0x40, 0x3c}, // 0x77 w
  {0x44, 0x28, 0x10, 0x28, 0x44}, // 0x78 x
  {0x0c, 0x50, 0x50, 0x50, 0x3c}, // 0x79 y
  {0x44, 0x64, 0x54, 0x4c, 0x44}, // 0x7a z
  {0x00, 0x08, 0x36, 0x41, 0x00}, // 0x7b {
  {0x00, 0x00, 0x7f, 0x00, 0x00}, // 0x7c |
  {0x00, 0x41, 0x36, 0x08, 0x00}, // 0x7d }
  {0x10, 0x08, 0x08, 0x10, 0x08}, // 0x7e ~
  {0x78, 0x46, 0x41, 0x46, 0x78}, // 0x7f DEL
  {0x00, 0x00, 0x00, 0x00, 0x00}, // dummy CH_HALFSP
  {0xFF, 0xFF, 0xFF, 0xFF, 0xFF}, // CH_DEGREE (TBD)
  {0xFF, 0xFF, 0xFF, 0xFF, 0xFF}, // CH_ARRUP (TBD)
  {0xFF, 0xFF, 0xFF, 0xFF, 0xFF}  // CH_ARRDOWN (TBD)
}; // font5x7

struct BmpFont5x7 : public BmpFont
{
  BmpFont5x7()
  : BmpFont(5, 7, 1, 1, (const uint8_t*) font5x7bmp)
  {}
};

static const BmpFont5x7 font5x7;

//------------------------------------------------------------------------------

static const uint8_t font7x15bmp[][2*7] =
{
  {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}, //
  {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x19, 0xfe, 0x19, 0xfe, 0x00, 0x00, 0x00, 0x00}, // !
  {0x00, 0x00, 0x00, 0x7e, 0x00, 0x7e, 0x00, 0x00, 0x00, 0x00, 0x00, 0x7e, 0x00, 0x7e}, // "
  {0x01, 0x98, 0x07, 0xfe, 0x07, 0xfe, 0x01, 0x98, 0x01, 0x98, 0x07, 0xfe, 0x07, 0xfe}, // #
  {0x00, 0x00, 0x06, 0x3c, 0x06, 0x7e, 0x1e, 0x67, 0x1e, 0x67, 0x07, 0xe6, 0x03, 0xc6}, // $
  {0x00, 0x00, 0x06, 0x0e, 0x07, 0x8e, 0x01, 0xe0, 0x00, 0x78, 0x07, 0x1e, 0x07, 0x06}, // %
  {0x0f, 0x80, 0x1f, 0xe7, 0x18, 0x7f, 0x19, 0xf9, 0x0f, 0x9f, 0x1f, 0x87, 0x19, 0x80}, // &
  {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x7e, 0x00, 0x7e, 0x00, 0x00, 0x00, 0x00}, // '
  {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x07, 0xf8, 0x0f, 0xfc, 0x1c, 0x0e, 0x10, 0x02}, // (
  {0x00, 0x00, 0x10, 0x02, 0x1c, 0x0e, 0x0f, 0xfc, 0x07, 0xf8, 0x00, 0x00, 0x00, 0x00}, // )
  {0x00, 0x60, 0x06, 0x66, 0x07, 0xfe, 0x01, 0xf8, 0x01, 0xf8, 0x07, 0xfe, 0x06, 0x66}, // *
  {0x00, 0x00, 0x00, 0x60, 0x00, 0x60, 0x03, 0xfc, 0x03, 0xfc, 0x00, 0x60, 0x00, 0x60}, // +
  {0x00, 0x00, 0x00, 0x00, 0x60, 0x00, 0x3e, 0x00, 0x1e, 0x00, 0x00, 0x00, 0x00, 0x00}, // ,
  {0x00, 0x00, 0x00, 0x60, 0x00, 0x60, 0x00, 0x60, 0x00, 0x60, 0x00, 0x60, 0x00, 0x60}, // -
  {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1e, 0x00, 0x1e, 0x00, 0x00, 0x00, 0x00, 0x00}, // .
  {0x00, 0x00, 0x1c, 0x00, 0x1f, 0x00, 0x03, 0xc0, 0x00, 0xf0, 0x00, 0x3e, 0x00, 0x0e}, // /
  {0x00, 0x00, 0x0f, 0xfc, 0x1f, 0xfe, 0x18, 0x86, 0x18, 0x46, 0x1f, 0xfe, 0x0f, 0xfc}, // 0
  {0x00, 0x00, 0x18, 0x00, 0x18, 0x18, 0x1f, 0xfe, 0x1f, 0xfe, 0x18, 0x00, 0x18, 0x00}, // 1
  {0x00, 0x00, 0x18, 0x1c, 0x1e, 0x1e, 0x1f, 0x86, 0x19, 0xe6, 0x18, 0x7e, 0x18, 0x1c}, // 2
  {0x00, 0x00, 0x0e, 0x06, 0x1e, 0x06, 0x18, 0x66, 0x19, 0xfe, 0x1f, 0x9e, 0x0e, 0x06}, // 3
  {0x00, 0x00, 0x07, 0x80, 0x07, 0xe0, 0x06, 0x78, 0x1f, 0xfe, 0x1f, 0xfe, 0x06, 0x00}, // 4
  {0x00, 0x00, 0x0c, 0x7e, 0x1c, 0x7e, 0x18, 0x66, 0x18, 0x66, 0x1f, 0xe6, 0x0f, 0xc6}, // 5
  {0x00, 0x00, 0x0f, 0xf8, 0x1f, 0xfc, 0x18, 0xce, 0x18, 0xc6, 0x1f, 0xc6, 0x0f, 0x80}, // 6
  {0x00, 0x00, 0x00, 0x06, 0x1e, 0x06, 0x1f, 0x86, 0x01, 0xe6, 0x00, 0x7e, 0x00, 0x1e}, // 7
  {0x00, 0x00, 0x0f, 0x9c, 0x1f, 0xfe, 0x18, 0x66, 0x18, 0x66, 0x1f, 0xfe, 0x0f, 0x9c}, // 8
  {0x00, 0x00, 0x00, 0x3c, 0x18, 0x7e, 0x18, 0x66, 0x1c, 0x66, 0x0f, 0xfe, 0x07, 0xfc}, // 9
  {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1e, 0x78, 0x1e, 0x78, 0x00, 0x00, 0x00, 0x00}, // :
  {0x00, 0x00, 0x00, 0x00, 0x60, 0x00, 0x3e, 0x78, 0x1e, 0x78, 0x00, 0x00, 0x00, 0x00}, // ;
  {0x00, 0x40, 0x00, 0xe0, 0x01, 0xf0, 0x03, 0xb8, 0x07, 0x1c, 0x06, 0x0c, 0x04, 0x04}, // <
  {0x00, 0x00, 0x01, 0x98, 0x01, 0x98, 0x01, 0x98, 0x01, 0x98, 0x01, 0x98, 0x01, 0x98}, // =
  {0x04, 0x04, 0x06, 0x0c, 0x07, 0x1c, 0x03, 0xb8, 0x01, 0xf0, 0x00, 0xe0, 0x00, 0x40}, // >
  {0x00, 0x00, 0x00, 0x1c, 0x00, 0x1e, 0x1b, 0x86, 0x1b, 0xe6, 0x00, 0x7e, 0x00, 0x1c}, // ?
  {0x07, 0xf8, 0x0f, 0xfc, 0x1c, 0x0e, 0x19, 0xe6, 0x19, 0x26, 0x19, 0xcc, 0x0c, 0xf8}, // @
  {0x00, 0x00, 0x1f, 0xf8, 0x1f, 0xfc, 0x01, 0x8e, 0x01, 0x8e, 0x1f, 0xfc, 0x1f, 0xf8}, // A
  {0x00, 0x00, 0x1f, 0xfe, 0x1f, 0xfe, 0x18, 0x66, 0x18, 0x66, 0x1f, 0xfe, 0x0f, 0xbc}, // B
  {0x00, 0x00, 0x0f, 0xfc, 0x1f, 0xfe, 0x18, 0x06, 0x18, 0x06, 0x1e, 0x1e, 0x0e, 0x1c}, // C
  {0x00, 0x00, 0x1f, 0xfe, 0x1f, 0xfe, 0x18, 0x06, 0x1c, 0x0e, 0x0f, 0xfc, 0x07, 0xf8}, // D
  {0x00, 0x00, 0x1f, 0xfe, 0x1f, 0xfe, 0x18, 0x66, 0x18, 0x66, 0x18, 0x66, 0x18, 0x06}, // E
  {0x00, 0x00, 0x1f, 0xfe, 0x1f, 0xfe, 0x00, 0x66, 0x00, 0x66, 0x00, 0x66, 0x00, 0x06}, // F
  {0x00, 0x00, 0x0f, 0xfc, 0x1f, 0xfe, 0x18, 0x06, 0x18, 0x66, 0x1f, 0xe6, 0x0f, 0xe6}, // G
  {0x00, 0x00, 0x1f, 0xfe, 0x1f, 0xfe, 0x00, 0x60, 0x00, 0x60, 0x1f, 0xfe, 0x1f, 0xfe}, // H
  {0x00, 0x00, 0x18, 0x06, 0x18, 0x06, 0x1f, 0xfe, 0x1f, 0xfe, 0x18, 0x06, 0x18, 0x06}, // I
  {0x00, 0x00, 0x0e, 0x00, 0x1e, 0x00, 0x18, 0x00, 0x18, 0x00, 0x1f, 0xfe, 0x0f, 0xfe}, // J
  {0x1f, 0xfe, 0x1f, 0xfe, 0x00, 0x60, 0x01, 0xf8, 0x07, 0x9e, 0x1e, 0x06, 0x18, 0x00}, // K
  {0x00, 0x00, 0x1f, 0xfe, 0x1f, 0xfe, 0x18, 0x00, 0x18, 0x00, 0x18, 0x00, 0x18, 0x00}, // L
  {0x1f, 0xfe, 0x1f, 0xfe, 0x00, 0x38, 0x00, 0xe0, 0x00, 0x38, 0x1f, 0xfe, 0x1f, 0xfe}, // M
  {0x00, 0x00, 0x1f, 0xfe, 0x1f, 0xfe, 0x00, 0xf0, 0x03, 0xc0, 0x1f, 0xfe, 0x1f, 0xfe}, // N
  {0x00, 0x00, 0x0f, 0xfc, 0x1f, 0xfe, 0x18, 0x06, 0x18, 0x06, 0x1f, 0xfe, 0x0f, 0xfc}, // O
  {0x00, 0x00, 0x1f, 0xfe, 0x1f, 0xfe, 0x01, 0x86, 0x01, 0x86, 0x01, 0xfe, 0x00, 0xfc}, // P
  {0x00, 0x00, 0x0f, 0xfc, 0x1f, 0xfe, 0x18, 0x06, 0x0c, 0x06, 0x1b, 0xfe, 0x17, 0xfc}, // Q
  {0x1f, 0xfe, 0x1f, 0xfe, 0x00, 0xc6, 0x01, 0xc6, 0x07, 0xfe, 0x1e, 0x7c, 0x18, 0x00}, // R
  {0x00, 0x00, 0x18, 0x3c, 0x18, 0x7e, 0x18, 0xe6, 0x19, 0xc6, 0x1f, 0x86, 0x0f, 0x06}, // S
  {0x00, 0x00, 0x00, 0x06, 0x00, 0x06, 0x1f, 0xfe, 0x1f, 0xfe, 0x00, 0x06, 0x00, 0x06}, // T
  {0x00, 0x00, 0x0f, 0xfe, 0x1f, 0xfe, 0x18, 0x00, 0x18, 0x00, 0x1f, 0xfe, 0x0f, 0xfe}, // U
  {0x00, 0x00, 0x01, 0xfe, 0x07, 0xfe, 0x1e, 0x00, 0x1e, 0x00, 0x07, 0xfe, 0x01, 0xfe}, // V
  {0x1f, 0xfe, 0x0f, 0xfe, 0x07, 0x00, 0x03, 0xc0, 0x07, 0x00, 0x0f, 0xfe, 0x1f, 0xfe}, // W
  {0x00, 0x00, 0x1c, 0x0e, 0x1f, 0x3e, 0x03, 0xf0, 0x03, 0xf0, 0x1f, 0x3e, 0x1c, 0x0e}, // X
  {0x00, 0x00, 0x00, 0x1e, 0x00, 0x7e, 0x1f, 0xe0, 0x1f, 0xe0, 0x00, 0x7e, 0x00, 0x1e}, // Y
  {0x00, 0x00, 0x1e, 0x06, 0x1f, 0x86, 0x19, 0xe6, 0x18, 0x7e, 0x18, 0x1e, 0x18, 0x06}, // Z
  {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1f, 0xfe, 0x1f, 0xfe, 0x18, 0x06, 0x18, 0x06}, // [
  {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}, // Y
  {0x00, 0x00, 0x18, 0x06, 0x18, 0x06, 0x1f, 0xfe, 0x1f, 0xfe, 0x00, 0x00, 0x00, 0x00}, // ]
  {0x00, 0xc0, 0x00, 0xf0, 0x00, 0x3c, 0x00, 0x0f, 0x00, 0x3c, 0x00, 0xf0, 0x00, 0xc0}, // ^
  {0x18, 0x00, 0x18, 0x00, 0x18, 0x00, 0x18, 0x00, 0x18, 0x00, 0x18, 0x00, 0x18, 0x00}, // _
  {0x00, 0x00, 0x00, 0x03, 0x00, 0x07, 0x00, 0x0e, 0x00, 0x1c, 0x00, 0x38, 0x00, 0x00}, // `
  {0x00, 0x00, 0x0f, 0x00, 0x1f, 0xb0, 0x19, 0xb0, 0x19, 0xb0, 0x1f, 0xf0, 0x1f, 0xe0}, // a
  {0x00, 0x00, 0x1f, 0xfe, 0x1f, 0xfe, 0x18, 0x30, 0x18, 0x30, 0x1f, 0xf0, 0x0f, 0xe0}, // b
  {0x00, 0x00, 0x0f, 0xe0, 0x1f, 0xf0, 0x18, 0x30, 0x18, 0x30, 0x18, 0x30, 0x18, 0x00}, // c
  {0x00, 0x00, 0x0f, 0xe0, 0x1f, 0xf0, 0x18, 0x30, 0x18, 0x30, 0x1f, 0xfe, 0x1f, 0xfe}, // d
  {0x00, 0x00, 0x0f, 0xe0, 0x1f, 0xf0, 0x19, 0x30, 0x19, 0x30, 0x19, 0xf0, 0x19, 0xe0}, // e
  {0x00, 0x00, 0x00, 0x60, 0x00, 0x60, 0x1f, 0xfc, 0x1f, 0xfe, 0x00, 0x66, 0x00, 0x66}, // f
  {0x00, 0x00, 0x67, 0xe0, 0x6f, 0xf0, 0x6c, 0x30, 0x6c, 0x30, 0x7f, 0xf0, 0x3f, 0xf0}, // g
  {0x00, 0x00, 0x1f, 0xfe, 0x1f, 0xfe, 0x00, 0x30, 0x00, 0x30, 0x1f, 0xf0, 0x1f, 0xe0}, // h
  {0x00, 0x00, 0x00, 0x00, 0x18, 0x30, 0x1f, 0xf6, 0x1f, 0xf6, 0x18, 0x00, 0x00, 0x00}, // i
  {0x00, 0x00, 0x60, 0x00, 0x60, 0x00, 0x60, 0x00, 0x7f, 0xf6, 0x3f, 0xf6, 0x00, 0x00}, // j
  {0x1f, 0xfe, 0x1f, 0xfe, 0x01, 0xc0, 0x03, 0xe0, 0x0f, 0x70, 0x1c, 0x30, 0x18, 0x00}, // k
  {0x00, 0x00, 0x00, 0x00, 0x18, 0x06, 0x1f, 0xfe, 0x1f, 0xfe, 0x18, 0x00, 0x00, 0x00}, // l
  {0x1f, 0xe0, 0x1f, 0xf0, 0x00, 0x70, 0x03, 0xe0, 0x00, 0x70, 0x1f, 0xf0, 0x1f, 0xe0}, // m
  {0x00, 0x00, 0x1f, 0xe0, 0x1f, 0xf0, 0x00, 0x30, 0x00, 0x30, 0x1f, 0xf0, 0x1f, 0xe0}, // n
  {0x00, 0x00, 0x0f, 0xe0, 0x1f, 0xf0, 0x18, 0x30, 0x18, 0x30, 0x1f, 0xf0, 0x0f, 0xe0}, // o
  {0x00, 0x00, 0x7f, 0xf0, 0x7f, 0xf0, 0x18, 0x30, 0x18, 0x30, 0x1f, 0xf0, 0x0f, 0xe0}, // p
  {0x00, 0x00, 0x0f, 0xe0, 0x1f, 0xf0, 0x18, 0x30, 0x18, 0x30, 0x7f, 0xf0, 0x7f, 0xf0}, // q
  {0x00, 0x00, 0x1f, 0xf0, 0x1f, 0xf0, 0x00, 0x30, 0x00, 0x30, 0x00, 0x70, 0x00, 0x60}, // r
  {0x00, 0x00, 0x18, 0xe0, 0x19, 0xf0, 0x19, 0xb0, 0x1b, 0x30, 0x1f, 0x30, 0x0e, 0x30}, // s
  {0x00, 0x00, 0x00, 0x30, 0x00, 0x30, 0x0f, 0xfc, 0x1f, 0xfc, 0x18, 0x30, 0x18, 0x30}, // t
  {0x00, 0x00, 0x0f, 0xf0, 0x1f, 0xf0, 0x18, 0x00, 0x18, 0x00, 0x1f, 0xf0, 0x1f, 0xf0}, // u
  {0x00, 0x00, 0x01, 0xf0, 0x07, 0xf0, 0x1e, 0x00, 0x1e, 0x00, 0x07, 0xf0, 0x01, 0xf0}, // v
  {0x1f, 0xf0, 0x0f, 0xf0, 0x07, 0x00, 0x03, 0xc0, 0x07, 0x00, 0x0f, 0xf0, 0x1f, 0xf0}, // w
  {0x00, 0x00, 0x18, 0x30, 0x1e, 0xf0, 0x07, 0xc0, 0x07, 0xc0, 0x1e, 0xf0, 0x18, 0x30}, // x
  {0x00, 0x00, 0x67, 0xf0, 0x6f, 0xf0, 0x6c, 0x00, 0x6c, 0x00, 0x7f, 0xf0, 0x3f, 0xf0}, // y
  {0x00, 0x00, 0x18, 0x30, 0x1e, 0x30, 0x1f, 0xb0, 0x19, 0xf0, 0x18, 0x70, 0x18, 0x30}, // z
  {0x00, 0xc0, 0x00, 0xc0, 0x01, 0xe0, 0x1f, 0xfe, 0x3f, 0x3f, 0x20, 0x01, 0x20, 0x01}, // {
  {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x3f, 0xff, 0x3f, 0xff, 0x00, 0x00, 0x00, 0x00}, // |
  {0x20, 0x01, 0x20, 0x01, 0x3f, 0x3f, 0x1f, 0xfe, 0x01, 0xe0, 0x00, 0xc0, 0x00, 0xc0}, // }
  {0x00, 0xe0, 0x00, 0x30, 0x00, 0x70, 0x00, 0xe0, 0x00, 0xc0, 0x00, 0xc0, 0x00, 0x70}, // ~
  {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}, // DEL
  {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}, // dummy CH_HALFSP
  {0x00, 0x0C, 0x00, 0x1E, 0x00, 0x33, 0x00, 0x33, 0x00, 0x1E, 0x00, 0x0C, 0x00, 0x00}, // CH_DEGREE
  {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF}, // CH_ARRUP (TBD)
  {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF}  // CH_ARRDOWN (TBD)
};

struct BmpFont7x15 : public BmpFont
{
  BmpFont7x15()
  : BmpFont(7, 15, 2, 2, (const uint8_t*) font7x15bmp)
  {}
};

static const BmpFont7x15 font7x15;

//------------------------------------------------------------------------------

const BmpFont& GetBmpFont(BmpFonts font)
{
  switch (font)
  {
    default:
    case font_4x6:  return font4x6;
    case font_5x7:  return font5x7;
    case font_7x15: return font7x15;
  }
}
