#pragma once

#include "types.h"

////////////// EEPROM MAP
byte EEPROM[0x200] = {
  [0x00..0x41] = 0x00, // oh, rly?
  [0x41..0x73] = 0x00, // oh, rly?
  [0x74..0x7F] = 0x00, // oh, rly?
};


