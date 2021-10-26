#pragma once

#include "memory-locations.hpp"

#include <types.hpp>
#include <ram.hpp>
#include <flash.hpp>
#include <xram.hpp>

#define IGNITION_VOLTAGE_FACTOR BYTE(0x75)

void Inputs_Part1();
word GetAdcValueFromTableAndAdjustForCalculus(word FlashPtr, word ADCValue);
word GetAdcValueFromTable(word FlashPtr, word ADCValue);
word GetValueFromTableImpl(word FlashPtr, byte Offset, byte Factor, bool Negate);
byte AdjustTemperature(word TemperatureTableValue);
word MultiplySigned(byte _M1, byte _M2);

void Inputs_Part1_CoolantTemp();
void Inputs_Part1_IgnitionSwitchVoltage();

inline bool status_watchdog_triggerred() {
  return CHECK_BIT_AT(RAM[0x20], 6);
}

inline bool status_xram_checksum_invalid() {
  return CHECK_BIT_AT(RAM[0x20], 7);
}


inline bool kitting_has_ego_sensor() {
  return CHECK_BIT_AT(GET_RNG_START_PTR(FLASH, KITTING)[0], 1);
}

inline bool kitting_has_additional_ego_sensor() {
  return CHECK_BIT_AT(GET_RNG_START_PTR(FLASH, KITTING)[1], 4);
}

inline bool kitting_has_absorber() {
  return CHECK_BIT_AT(GET_RNG_START_PTR(FLASH, KITTING)[4], 4);
}

inline bool kitting_has_intake_air_temperature_sensor() {
  return CHECK_BIT_AT(GET_RNG_START_PTR(FLASH, KITTING)[0], 3);
}

inline bool kitting_has_co_potentiometer_sensor() {
  return CHECK_BIT_AT(GET_RNG_START_PTR(FLASH, KITTING)[0], 7);
}

inline bool kitting_has_irom() {
  return CHECK_BIT_AT(GET_RNG_START_PTR(FLASH, KITTING)[2], 0);
}

inline bool kitting_has_camshaft_position_sensor() {
  return CHECK_BIT_AT(GET_RNG_START_PTR(FLASH, KITTING)[0], 4);
}

inline bool kitting_camshaft_position_sensor_cross_section_aligned_with_TDC() {
  return CHECK_BIT_AT(GET_RNG_START_PTR(FLASH, KITTING)[0], 5);
}

inline bool kitting_has_knock_sensor() {
  return CHECK_BIT_AT(GET_RNG_START_PTR(FLASH, KITTING)[0], 2);
}

inline bool kitting_should_adapt_throttle_position_sensor() {
  return CHECK_BIT_AT(GET_RNG_START_PTR(FLASH, KITTING)[2], 6);
}
