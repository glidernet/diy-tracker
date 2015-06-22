//------------------------------------------------------------------------------
/*
  OGN packet.

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

#include <ogn.h>

//------------------------------------------------------------------------------

const char* GetAcftTypeShort(AcftType type)
{
  // return 4 chars max
  switch (type)
  {
    case acft_unknown          : return "unkn";
    case acft_glider           : return "gld";
    case acft_tow_plane        : return "tow";
    case acft_helicopter       : return "heli";
    case acft_parachute        : return "para";
    case acft_drop_plane       : return "drop";
    case acft_hang_glider      : return "hang";
    case acft_para_glider      : return "pgld";
    case acft_powered_aircraft : return "pwrd";
    case acft_jet_aircraft     : return "jet";
    case acft_ufo              : return "ufo";
    case acft_balloon          : return "baln";
    case acft_airship          : return "ashp";
    case acft_uav              : return "uav";
    case acft_static_object    : return "stat";
    default                    : return "???";
  }
}
