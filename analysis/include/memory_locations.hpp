#pragma once

#include "ram.h"
#include "flash.h"
#include "xram.h"

#define DEFINE_MEMORY_RANGE(segment, name, start, end)                          \
  static const int segment ## _ ## name ## _START_PTR = (start);                \
  static const int segment ## _ ## name ## _END_PTR = (end);                    \
  static const int segment ## _ ## name ## _RANGE_LENGTH = (end) - (start) + 1; \
  static inline byte* get_ ## segment ## _ ## name ## _start() {                \
    return &segment[segment ## _ ## name ## _START_PTR];                        \
  };                                                                            \
  static inline byte* get_ ## segment ## _ ## name ## _end() {                  \
    return &segment[segment ## _ ## name ## _END_PTR];                          \
  };

#define RNG_START(segment, name) \
  segment ## _ ## name ## _START_PTR
#define RNG_END(segment, name) \
  segment ## _ ## name ## _END_PTR
#define RNG_LENGTH(segment, name) \
  segment ## _ ## name ## _RANGE_LENGTH
#define GET_RNG_START_PTR(segment, name) \
  get_ ## segment ## _ ## name ## _start()
#define GET_RNG_END_PTR(segment, name) \
  get_ ## segment ## _ ## name ## _end()


#define _MEM_PTR(segment, name) segment ## _ ## name ## _PTR
#define DEFINE_MEMORY_BYTE(segment, name, loc) \
  static const int _MEM_PTR(segment, name) = (loc);

#define MEM_BYTE(segment, name) _MEM_PTR(segment, name)
#define GET_MEM_BYTE_PTR(segment, name) \
  &segment[_MEM_PTR(segment, name)]
#define GET_MEM_BYTE(segment, name) \
  segment[_MEM_PTR(segment, name)]

#define DEFINE_MEMORY_WORD(segment, name, loc) \
  static const int _MEM_PTR(segment, name) = (loc);
#define MEM_WORD(segment, name) _MEM_PTR(segment, name)
#define GET_MEM_WORD_PTR(segment, name) \
  WORD_PTR(&segment[_MEM_PTR(segment, name)])
#define GET_MEM_WORD(segment, name) \
  COMPOSE_WORD(segment[_MEM_PTR(segment, name) + 1], segment[_MEM_PTR(segment, name)])

////////////////////////////////////////////////////////////
// XRAM
DEFINE_MEMORY_RANGE(XRAM, SUM_OF_EGO_CALIBRATION, 0xF400, 0xF4FF);
DEFINE_MEMORY_RANGE(XRAM, EGO_CALIBRATION, 0xF500, 0xF5FF);

DEFINE_MEMORY_WORD(XRAM, CHECKSUM, 0xF658);

DEFINE_MEMORY_BYTE(XRAM, ADJUSTED_CO_POT, 0xF681);

DEFINE_MEMORY_BYTE(XRAM, ADJUSTED_COOLANT_TEMP, 0xF683);

DEFINE_MEMORY_BYTE(XRAM, ADC_THROTTLE_POSITION, 0xF685);
DEFINE_MEMORY_BYTE(XRAM, ADC_COOLANT_TEMP, 0xF686);
DEFINE_MEMORY_BYTE(XRAM, ADC_INTAKE_AIR_TEMP, 0xF687);
DEFINE_MEMORY_BYTE(XRAM, ADC_IGNITION_SWITCH_VOLTAGE, 0xF688);
DEFINE_MEMORY_BYTE(XRAM, ADC_CO_POT, 0xF689);

DEFINE_MEMORY_WORD(XRAM, IGNITION_SW_VOLTAGE_SUM, 0xF69B);
DEFINE_MEMORY_WORD(XRAM, COOLANT_TEMP_SUM, 0xF69E);
DEFINE_MEMORY_WORD(XRAM, INTAKE_AIR_SUM, 0xF6A0);
DEFINE_MEMORY_WORD(XRAM, CO_POT_SUM, 0xF6A4);
DEFINE_MEMORY_WORD(XRAM, THROTTLE_POSITION_SUM, 0xF6A6);
DEFINE_MEMORY_WORD(XRAM, RAM_49_SUM, 0xF6A8);
DEFINE_MEMORY_WORD(XRAM, RAM_49_SUM_STEP_1, 0xF6AA);

////////////////////////////////////////////////////////////
// FLASH
DEFINE_MEMORY_RANGE(FLASH, COOLANT_TEMPERATURE_TABLE_1, 0x831F, 0x832F);
DEFINE_MEMORY_RANGE(FLASH, COOLANT_TEMPERATURE_TABLE_2, 0x8330, 0x8340);
DEFINE_MEMORY_RANGE(FLASH, INTAKE_AIR_TEMPERATURE_TABLE, 0x8341, 0x8351);

DEFINE_MEMORY_BYTE(FLASH, MIMIMUM_INTAKE_AIR_TEMPERATURE, 0x805E);
DEFINE_MEMORY_BYTE(FLASH, MAXIMUM_INTAKE_AIR_TEMPERATURE, 0x805F);

////////////////////////////////////////////////////////////
// RAM


