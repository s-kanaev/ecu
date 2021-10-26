#pragma once

#include "defines.hpp"
#include "mem-seg-helpers.hpp"
#include "types.hpp"

////////////// RAM MAP
extern byte RAM[0x100];

#if __E591_HOST_COMPILATION
static inline void INIT_RAM() {
  DEF_ARRAY(byte, 0x00, 0xFF, full_range) = { 0 };
  UNUSED(full_range);

#if 0
  [0x00..0x07] = 0, // R0..R7 @ register bank 0
  [0x08..0x0F] = 0, // R0..R7 @ register bank 1
  [0x10..0x17] = 0, // R0..R7 @ register bank 2
  [0x18..0x1F] = 0, // R0..R7 @ register bank 3

  [0x1C] = 0,     // Divisor (of the same as 0x44/0x45), low byte, can be changed by some interrupt?
  [0x1D] = 0,     // Divisor (of the same as 0x44/0x45), high byte, can be changed by some interrupt?

  // the rest (up to 0x7B, incl.) is initialized with nil
  [0x20] = 0x00,  // status byte
                  // bit 2 - ???
                  // bit 6 (bit address 0x06) = copy of PSW.F1 // watchdog triggered?
                  // bit 7 (bit address 0x07) = if xram check sum was not valid

  [0x21] = 0x00,  // bit 0 (bit address 0x08) = 1 if failed to read first 0x42 bytes from eeprom
  [0x22] = 0x00,  // error codes ?
                  // bit 4 - ignition switch voltage less than low limit
                  // bit 5 - ignition switch voltage higher than high limit
                  // bit 6 - EGO #1 sensor processed value less than low limit
                  // bit 7 - EGO #1 sensor processed value larger than high limit
  [0x23] = 0x00,  // error codes?
                  // bit 2 - coolant temperature less than low limit
                  // bit 3 - coolant temperature higher than high limit
                  // bit 4 - CO potentiometer less than low limit
                  // bit 5 - CO potentiometer temperature higher than high limit
  [0x24] = 0x00,  // error codes?
                  // bit 0 - XRAM[0xF6A7] less than low limit
                  // bit 1 - XRAM[0xF6A7] larger than upper limit
                  // bit 4 - intake air temperature less than low limit
                  // bit 5 - intake air temperature higher than high limit
  [0x25] = 0x00,  // ???
                  // bit 5 - ???

  [0x26] = 0,     // status byte:
                  // bit 6 - ??

  [0x27] = 0x00,  // ???
                  // bit 2 (bit address 0x3A) ???
                  // bit 3 (bit address 0x3B) = copy of PSW.F0
                  // bit 4 ???

  [0x28] = 0x00,  // some status byte
                  // bit 0 - ???, bit which allows for main loop to continue
                  // bit 3 - ???
                  // bit 4 - ignition switch voltage above threshold @ FLASH[0x8096]
                  // bit 6 - ???, set when XRAM[0xF679] >= FLASH[0x809B] && RAM[0x49] >= FLASH[0x809A]
  [0x29] = 0x00,
  [0x2A] = 0x00,  // some status word
                  // bit 0 ???
                  // bit 3 ???
  [0x2B] = 0x00,  // ???
                  // bit 4 ???
  [0x2C] = 0x00,
  [0x2D] = 0x00,  // bit 7 = FLASH[0x873F] bit 4, is there camshaft position sensor
  [0x2E] = 0x00,  // bit 0 = FLASH[0x873F] bit 5, camshaft position sensor cross-section is aligned with TDC
                  // bit 1 = FLASH[0x873F] bit 2, is there knock sensor

  [0x2F] = 0,     // bit 0 - ???
                  // bit 1 - ???

  [0x30] = 0,     // Some counter?, can be changed by some interrupt along with 0x1C/0x1D?

  [0x3A] = 0,     // Adjusted coolant temperature

  [0x3B] = 0,     // Adjusted intake air temperature
  [0x3C] = 0,     // Adjusted ignition switch voltage

  [0x3D] = 0,     // Adjusted coolant temperature (output from adjusting RAM[0x3A] with AdjustTemperature proc)
                  // packed offset and factor for FLASH[0xA2FD]
                  // factor - least significant three bits, will be SHL 5 = xxx0 0000
                  // offset - most significant five bits, will be SHR 3, max value = 0x1F

  [0x3E] = 0,     // Adjusted intake air temperature (after RAM[0x3B])
  [0x3F] = 0,     // Adjusted ignition switch voltage
  [0x40] = 0,     // Initial value for XRAM[0xF6AD] and XRAM[0xF6AF]
  [0x41..0x43] = 0,

  [0x44] = 0,     // Divisor (of what?), low byte, can be changed by some interrupt?
  [0x45] = 0,     // Divisor (of what?), high byte, can be changed by some interrupt?
                  // For the previous two, the result is stored @ XRAM[0xF6BB], XRAM[0xF6BC]

  [0x46..0x47] = 0,
  [0x48] = 0,     // ???
  [0x49] = 0,     // XRAM[0xF6BC] + 0x80, saturated at 0xFF
  [0x4A] = 0,     // interpolated value of XRAM[0xF6BC/0xF6BB], table at FLASH[0x83B0], size unknown, TODO
  [0x4B] = 0,     // ((RAM[0x4A] + 4) >> 3) & 0x1F
  [0x4C] = 0,     // ((RAM[0x4A] + 8) >> 4) = offset div 0x11 for table at FLASH[0x8AFB]
  [0x4D..0x5C] = 0,
  [0x5D] = 0,     // ???
  [0x5E] = 0x00,

  [0x5F] = 0x20,  // ???
  [0x60] = 0x03,  // ???
  [0x61] = 0x21,  // ???
  [0x62] = 0x00,  // ???

  [0x63] = 0x00,  // Some derivative from Coolant Temperature

  [0x64..0x71] = 0,
  [0x72] = 0,     // some status word ?
                  // bit 7 - ???
                  // bit 6
                  // bit 5 - ???
                  // bit 4
                  // bit 3
                  // bit 2
                  // bit 1
                  // bit 0
  [0x73] = 0,     // ???
                  // bit 3 - XRAM[0xF681] initialized
                  // bit 4 - XRAM[0xF682] initialized
                  // bit 5 - ???
  [0x74..0x7B] = 0x00,

  [0x7C..0x7D] = 0x00,

  [0x7E..0x7F] = 0x40, // HIP0045 configuration words

#define STACK = 0x00
  [0xB4..0xFF] = STACK
#undef STACK
#endif
};
#endif
