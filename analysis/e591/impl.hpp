#pragma once

#include "memory-locations.hpp"

#include <types.hpp>
#include <ram.hpp>
#include <flash.hpp>
#include <xram.hpp>
#include <registers.hpp>

#define WAIT_MACHINE_CYCLES_BY_2(x) /* waits for x*2 machine cycles, x = 0 is 0x100 */

#define IGNITION_VOLTAGE_FACTOR BYTE(0x75)

void Inputs_Part1();
word GetAdcValueFromTableAndAdjustForCalculus(word FlashPtr, word ADCValue);
word GetAdcValueFromTable(word FlashPtr, word ADCValue);
word GetValueFromTableImpl(word FlashPtr, byte Offset, byte Factor, bool Negate);
byte AdjustTemperature(word TemperatureTableValue);

// _5FE2:
// M1 is signed
// M2 is unsigned
word MultiplySigned(byte _M1, byte _M2);

void Inputs_Part1_CoolantTemp();
void Inputs_Part1_IgnitionSwitchVoltage();

void mainLoopTrampoline();
void MAIN_LOOP();
void init_xram_f6bb_f6bc_and_ram_48_49_4a_4b_4c(bool SkipIntro);

// _695C
void ClearXramF69E_0C_Bytes();
// _2C09
// Launched when XRAM[0xF69D] == 0x20
void Xram_F69D_eq_20();

// Returns FLASH[FlashPtr + TableIdx] + HIGH(DiffFactor * (FLASH[FlashPtr + TableIdx + 1] - FLASH[FlashPtr + TableIdx]))
// _62CE:
byte InterpolateTableValue(word FlashPtr, byte TableIdx, byte DiffFactor);

// _5FFB:
word scale10bitADCValue(word V, byte Factor);

// _64A4:
void addWordInXRAMWord(word V, word XramPtr);

// _6498:
void addByteInXRAMWord(byte _V, word XramPtr);

// _C000:
void init_xram_for_serial0();

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

inline void TURN_OFF_IGNITION_COIL_1_4() {
  CLEAR_BIT_IN(P5, 1);
}

inline void TURN_OFF_IGNITION_COIL_2_3() {
  CLEAR_BIT_IN(P5, 0);
}

inline void TURN_OFF_INJECTOR_1() {
  CLEAR_BIT_IN(P4, 0);
}

inline void TURN_OFF_INJECTOR_2() {
  CLEAR_BIT_IN(P4, 1);
}

inline void TURN_OFF_INJECTOR_3() {
  CLEAR_BIT_IN(P4, 2);
}

inline void TURN_OFF_INJECTOR_4() {
  CLEAR_BIT_IN(P4, 3);
}

inline bool checkLO_MC33199() {
  return (P9 & 0x20);
}

inline bool setTxD_and_checkLO_MC33199(bool TxDVal) {
  if (TxDVal)
    SET_BIT_IN(P3, 1);
  else
    CLEAR_BIT_IN(P3, 1);  // TxD @ MC33199 (ISO9141)

  WAIT_MACHINE_CYCLES_BY_2(0x0E);

  return checkLO_MC33199();
}
