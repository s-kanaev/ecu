#pragma once

#include "types.h"

////////////// XRAM MAP
byte XRAM[0xC00] = /* [0xF400..0x10000] address space */ {
  [0xF400..0xF4FF] = 0,                       // Sum of XRAM[0xF5xx] + FLASH[0x9A1C + xx]
  [0xF500..0xF5FF] = 0,                       // EGO calibration
  [0xF600] = 0,
  [0xF602] = FLASH[0x8761],                   // Set to FLASH[0x8761] if and only if watchdog triggered or xram checksum failed (power-on?)
                                              // Signed byte value.
                                              // Differentiated versus RAM[0x63].
  [0xF603] = 0x01,                            // Set to 0x01 if and only if watchdog triggered or xram checksum failed (power-on?)
  [0xF605..0xF657] = 0,

  [0xF658] = 0,                               // checksum low byte (for 0xFx00..0xF657)
  [0xF659] = 0                                // checksum high byte (for 0xFx00..0xF657)
  
  [0xF681] = 0,                               // Adjusted CO potentiometer
  [0xF683] = 0,                               // Adjusted coolant temperature, copy of RAM[0x3A]
  [0xF686] = 0,                               // ADC Coolant temperature
  [0xF687] = 0,                               // ADC Intake air temperature
  [0xF688] = 0,                               // ADC Ignition switch voltage
  [0xF689] = 0,                               // ADC CO Potentiometer

  [0xF69B] = 0,                               // sum of Ignition switch voltage, low byte
  [0xF69C] = 0,                               // sum of Ignition switch voltage, high byte

  [0xF69E] = 0,                               // sum of scaled ADC Coolant Temperature, low byte
  [0xF69F] = 0,                               // sum of scaled ADC Coolant Temperature, high byte
  [0xF6A0] = 0,                               // sum of scaled ADC Intake Air Temperature, low byte
  [0xF6A1] = 0,                               // sum of scaled ADC Intake Air Temperature, high byte

  [0xF6B9] = 0,                               // ??? TODO

  [0xF6BB] = 0,                               // quotient from dividng smth on RAM[0x44..0x45] or RA<[0x1C/0x1D], low byte
  [0xF6BC] = 0,                               // quotient from dividng smth on RAM[0x44..0x45] or RA<[0x1C/0x1D], high byte
  
  [0xF675..0xF7D4] = 0x00,                    // 0x160 bytes
  [0xF7A4] = 0x00,                            // Filled in with FLASH[0x8789] or FLASH[0x878A] depending
                                              // on adjusted coolant temperature less than some limit,
                                              // i.e. RAM[0x3A] < FLASH[0x8788

  [0xF7BE] = 0x00,                            // Current operationg mode:
                                              // 0 - normal
                                              // 3 - diagnostic (has smth on L-line)
  [0xF7D5..0xF8CC] = FLASH[0xABF1..0xACE8]    // 0xF8 bytes

  [0xF8CD..0xF94C] = 0,                       // 0x80 bytes,
                                              // Eight copies of FLASH[0xADF1..0xADE0] or all zeros depending
                                              // on whether there is a knock sensor or coolant temperature
                                              // below some limit i.e. RAM[0x3A] < FLASH[0x87B7]

  [0xF96D] = 0,                               // counter/pausing word, low byte
  [0xF96E] = 0,                               // counter/pausing word, high byte

  [0xF97E] = 0x00,
  [0xF972] = 0x00,
  [0xF973] = 0x00,
  [0xF974] = 0x00,

  [0xF9B9] = 0,                               // counter?
  
  [0xFF00..0xFF41] = EEPROM[0x00..0x41],
  [0xFF42..0xFF73] = EEPROM[0x41..0x73],
  [0xFF74..0xFF7F] = EEPROM[0x74..0x7F]
};


