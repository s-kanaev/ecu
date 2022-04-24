#pragma once

#include "memory-locations.hpp"

#include <types.hpp>
#include <ram.hpp>
#include <flash.hpp>
#include <xram.hpp>
#include <registers.hpp>
#include <pins.hpp>

#define WAIT_MACHINE_CYCLES_BY_2(x) /* waits for x*2 machine cycles, x = 0 is 0x100 */

#define IGNITION_VOLTAGE_FACTOR BYTE(0x75)

enum class CoilE : byte {
  C_1_4 = 1,
  C_2_3 = 0,

  CoilCount = 2
};

static byte SET_CLEAR_MASKS_PER_COIL[static_cast<byte>(CoilE::CoilCount)] = {
  // 0 = Coil 2/3
  0xFE,
  // 1 = Coil 1/4
  0xFD,
};

static pin COIL_PIN[static_cast<byte>(CoilE::CoilCount)] = {
  // 0 = Coil 2/3
  IGNITION_COIL_23_PIN,
  // 1 = Coil 1/4
  IGNITION_COIL_14_PIN
};

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

void SelectMode();

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

// _6060:
word AbsWordByMSB(word V);

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

inline void START_CHARGING_IGNITION_COIL(CoilE Coil) {
  CLEAR_BIT_IN(P5, static_cast<byte>(Coil));
}

inline void IGNITE_COIL(CoilE Coil) {
  SET_BIT_IN(P5, static_cast<byte>(Coil));
}

inline void START_CHARGING_IGNITION_COIL_1_4() {
  START_CHARGING_IGNITION_COIL(CoilE::C_1_4);
}

inline void START_CHARGING_IGNITION_COIL_2_3() {
  START_CHARGING_IGNITION_COIL(CoilE::C_2_3);
}

inline void TURN_OFF_IGNITION_COIL_1_4() {
  START_CHARGING_IGNITION_COIL_1_4();
}

inline void TURN_OFF_IGNITION_COIL_2_3() {
  START_CHARGING_IGNITION_COIL_2_3();
}

inline void IGNITE_COIL_1_4() {
  IGNITE_COIL(CoilE::C_1_4);
}

inline void IGNITE_COIL_2_3() {
  IGNITE_COIL(CoilE::C_2_3);
}

inline bool IGNITION_COIL_CHARGING(CoilE Coil) {
  return !CHECK_BIT_AT(P5, static_cast<byte>(Coil));
}

inline bool IGNITION_COIL_1_4_CHARGING() {
  return IGNITION_COIL_CHARGING(CoilE::C_1_4);
}

inline bool IGNITION_COIL_2_3_CHARGING() {
  return IGNITION_COIL_CHARGING(CoilE::C_2_3);
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
  return CHECK_BIT_AT(P9, 5);
}

inline bool setTxD_and_checkLO_MC33199(bool TxDVal) {
  if (TxDVal)
    SET_BIT_IN(P3, 1);
  else
    CLEAR_BIT_IN(P3, 1);  // TxD @ MC33199 (ISO9141)

  WAIT_MACHINE_CYCLES_BY_2(0x0E);

  return checkLO_MC33199();
}

// MISCELLANEOUS
bool CAS(byte *Ptr, byte Expected, byte Desired);
bool CHECK_AND_CLEAR_BIT(byte &Ptr, bit Bit);

// Just a fool-proof comparison for equality
template <typename T1, typename T2>
bool equal(const T1 lhs, const T2 rhs) {
  return lhs == rhs;
}
