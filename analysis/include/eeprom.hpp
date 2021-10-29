#pragma once

#include "defines.hpp"
#include "mem-seg-helpers.hpp"
#include "types.hpp"

////////////// EEPROM MAP
extern byte EEPROM[0x200];

#if __E591_HOST_COMPILATION
static inline void INIT_EEPROM() {
  DEF_ARRAY(byte, 0x00, 0x1FF, full_range) = { 0 };

#if 0
  [0x00..0x41] = 0x00, // oh, rly?
  [0x41..0x73] = 0x00, // oh, rly?
  [0x74..0x7F] = 0x00, // oh, rly?
#endif
};
#endif
