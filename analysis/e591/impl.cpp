#include "impl.hpp"
#include "memory-locations.hpp"
#include "inlines.hpp"

#include <types.hpp>
#include <binary_ops.hpp>

// #define __E591_IMPLEMENTATION
// #include <memory_locations.hpp>
// #undef __E591_IMPLEMENTATION

#include <pins.hpp>
#include <undefined.hpp>
#include <ports.hpp>

#if __E591_HOST_COMPILATION
byte P1;
byte P2;
byte P3;
byte P4;
byte P5;
byte P6;
byte P7;
byte P8;
byte P9;

byte XRAM[0x10000];
byte FLASH[0x10000];
byte RAM[0x100];
#endif

void Inputs_Part1_CoolantTemp() {
  word CoolantTemp = ADC_10bit(COOLANT_TEMP_PIN);
  SET_MEM_BYTE(XRAM, ADC_COOLANT_TEMP, HIGH(CoolantTemp));
  bool CoolantTempNotInLimits = false;

  if (!FLASH[0x805D]) {

    if (HIGH(CoolantTemp) < FLASH[0x8057]) {
      // coolant_temp_less_than_low_limit
      SET_BIT_IN(RAM[0x23], 2);
      CLEAR_BIT_IN(RAM[0x23], 3);

      CoolantTempNotInLimits = true;
    } else
      if (HIGH(CoolantTemp) > FLASH[0x8058]) {
      // coolant_temp_larger_than_high_limit
      CLEAR_BIT_IN(RAM[0x23], 2);
      SET_BIT_IN(RAM[0x23], 3);

      CoolantTempNotInLimits = true;
    }
  }

  word AdjustedCoolantTemp;

  // CoolantTempNotInLimits = false in both examples
  if (!CoolantTempNotInLimits || FLASH[0x805D]) {
    CLEAR_BIT_IN(RAM[0x23], 2);
    CLEAR_BIT_IN(RAM[0x23], 3);

    // 1. AdjustedCoolantTemp: 0x2F84
    // 2. AdjustedCoolantTemp: 0x0000
    AdjustedCoolantTemp = GetAdcValueFromTableAndAdjustForCalculus(
        RNG_START_IDX(FLASH, COOLANT_TEMPERATURE_TABLE_1), CoolantTemp);
  } else if (CoolantTempNotInLimits) /* CoolantTempNotInLimits && !FLASH[0x805D] */ {
    AdjustedCoolantTemp = COMPOSE_WORD(FLASH[0x8A4B], 0);
  }

  // 1. 0x2F
  // 2. 0x00
  RAM[0x3A] = HIGH(AdjustedCoolantTemp);
  // 1. 0x4C
  // 2. 0x00
  RAM[0x3D] = AdjustTemperature(AdjustedCoolantTemp);
}

void Inputs_Part1_IntakeAirTemp() {
  word IntakeAirTemp = ADC_10bit(INTAKE_AIR_TEMP_PIN);
  SET_MEM_BYTE(XRAM, ADC_INTAKE_AIR_TEMP, HIGH(IntakeAirTemp));

  bool IntakeAirTempOutOfLimits = false;
  bool HasIntakeAirTempSensor = kitting_has_intake_air_temperature_sensor();
  word AdjustedIntakeAirTemp;

  if (HasIntakeAirTempSensor) {
    // Has intake air temperature sensor
    if (!FLASH[0x8060]) {
      if (HIGH(IntakeAirTemp) < GET_MEM_BYTE(FLASH, MINIMUM_INTAKE_AIR_TEMPERATURE)) {
        // intake air temp below minimum
        SET_BIT_IN(RAM[0x24], 4);
        CLEAR_BIT_IN(RAM[0x24], 5);

        IntakeAirTempOutOfLimits = true;
      } else if (GET_MEM_BYTE(FLASH, MAXIMUM_INTAKE_AIR_TEMPERATURE) < HIGH(IntakeAirTemp)) {
        // intake air temp above minimum
        CLEAR_BIT_IN(RAM[0x24], 4);
        SET_BIT_IN(RAM[0x24], 5);

        IntakeAirTempOutOfLimits = true;
      }
    }

    if (FLASH[0x8060] || !IntakeAirTempOutOfLimits) {
      CLEAR_BIT_IN(RAM[0x24], 4);
      CLEAR_BIT_IN(RAM[0x24], 5);

      // TODO Table length ?
      AdjustedIntakeAirTemp = GetAdcValueFromTableAndAdjustForCalculus(
          RNG_START_IDX(FLASH, INTAKE_AIR_TEMPERATURE_TABLE), IntakeAirTemp);
    }
  }

  if (!HasIntakeAirTempSensor || IntakeAirTempOutOfLimits)
    AdjustedIntakeAirTemp = COMPOSE_WORD(FLASH[0x8061], 0);

  RAM[0x3B] = HIGH(AdjustedIntakeAirTemp);
  RAM[0x3E] = AdjustTemperature(AdjustedIntakeAirTemp);
}

void SelectMode() {
  if (checkLO_MC33199())  // test LO @ MC33199 (ISO9141)
    XRAM[0xF7BE] = 3;
  else
    XRAM[0xF7BE] = 0;
}

void Inputs_Part1_COPotentiometer() {
  bool COPotNotInLimits = false;
  bool CantInitCOPot = false;
  byte COPot;

  word AdjustedCOPot;

  if (kitting_has_co_potentiometer_sensor()) {
    // There is a CO Potentiometer sensor
    //_544A:
    COPot = ADC_8bit(CO_POT_PIN);
    SET_MEM_BYTE(XRAM, ADC_CO_POT, COPot);

    if (COPot < FLASH[0x8067]) {
      SET_BIT_IN(RAM[0x23], 4);
      CLEAR_BIT_IN(RAM[0x23], 5);

      COPotNotInLimits = true;
    } else if (COPot > FLASH[0x8068]) {
      CLEAR_BIT_IN(RAM[0x23], 4);
      SET_BIT_IN(RAM[0x23], 5);

      COPotNotInLimits = true;
    }

    //_5478:
    CLEAR_BIT_IN(RAM[0x23], 4);
    CLEAR_BIT_IN(RAM[0x23], 5);

    if (COPot & 0x80)
      COPot = 0xFF;
    else
      COPot *= 2;

    COPot -= 0x50;

    AdjustedCOPot = MultiplySigned(COPot,  0xC0);
  } else if (kitting_has_irom()) {
    // No CO Potentiometer but has an IROM
    CLEAR_BIT_IN(RAM[0x23], 4);
    CLEAR_BIT_IN(RAM[0x23], 5);

    CantInitCOPot = true;
  } else {
    // Neither CO Potentiometer nor IROM is available
    CLEAR_BIT_IN(RAM[0x23], 4);
    CLEAR_BIT_IN(RAM[0x23], 5);

    COPotNotInLimits = true;
  }

  if (COPotNotInLimits)
    AdjustedCOPot = COMPOSE_WORD(FLASH[0x8069], FLASH[0x8069]);

  if (!CHECK_BIT_AT(RAM[0x73], 3)) {
    if (!CantInitCOPot)
      SET_MEM_BYTE(XRAM, ADJUSTED_CO_POT, HIGH(AdjustedCOPot));
    else
      SET_MEM_BYTE(XRAM, ADJUSTED_CO_POT, XRAM[0xFF74]);
  }
}

void Inputs_Part1_IgnitionSwitchVoltage() {
  byte IgnVoltage = ADC_8bit(IGNITION_VOLTAGE_PIN);
  SET_MEM_BYTE(XRAM, ADC_IGNITION_SWITCH_VOLTAGE, IgnVoltage);

  word MultipliedIgnVoltage = WORD(IgnVoltage) * IGNITION_VOLTAGE_FACTOR;
  byte AdjustedIgnVoltage;

  if (CHECK_BIT_AT(MultipliedIgnVoltage, 0x0F)) // not possible as minimum IgnVoltage for it is 0x0119 which doesn't fit a byte
    AdjustedIgnVoltage = 0xFF;
  else
    AdjustedIgnVoltage = HIGH(MultipliedIgnVoltage * 2);

  if (AdjustedIgnVoltage < GET_MEM_BYTE(FLASH, MINIMUM_IGNITION_VOLTAGE))
    SET_BIT_IN(RAM[0x22], 4);
  else
    CLEAR_BIT_IN(RAM[0x22], 4);

  if (GET_MEM_BYTE(FLASH, MAXIMUM_IGNITION_VOLTAGE) < AdjustedIgnVoltage)
    SET_BIT_IN(RAM[0x22], 5);
  else
    CLEAR_BIT_IN(RAM[0x22], 5);

  RAM[0x3C] = AdjustedIgnVoltage;

  if (AdjustedIgnVoltage < 0x36)
    AdjustedIgnVoltage = 0;
  else
    AdjustedIgnVoltage -= 0x36;

  AdjustedIgnVoltage = HIGH(WORD(AdjustedIgnVoltage) * 0x40);

  if (AdjustedIgnVoltage > 0x1F)
    AdjustedIgnVoltage = 0x1F;

  RAM[0x3F] = AdjustedIgnVoltage;
}

void Inputs_Part1() {
  CLEAR_BIT_IN(RAM[0x26], 6);

  // COOLANT TEMPERATURE
  {
    Inputs_Part1_CoolantTemp();
  }

  // INTAKE AIR TEMPERATURE
  {
    Inputs_Part1_IntakeAirTemp();
  }

  // MODE SELECTION
  {
    SelectMode();
  }

  // CO POTENTIOMETER
  {
    Inputs_Part1_COPotentiometer();
  }

  // IGNITION SWITCH VOLTAGE
  {
    Inputs_Part1_IgnitionSwitchVoltage();
  }
}

// example inputs:
// 1. 0x831F, 0x8FC0 for Coolant Temperature = 8 degrees
// 2. 0x831F, 0x7480 for Coolant Temperature = -45 degrees
word GetAdcValueFromTable(word FlashPtr, word ADCValue) {
  // max value in ADCValue = F0:FF
  // max offset = HIGH(ADCValue) >> 4 = F0 >> 4 = F
  // max Factor = (HIGH(ADCValue) << 4) | (LOW(ADCValue) >> 4) = (F0 << 4) | (C0 >> 4) = 0C
  // max Factor = (HIGH(ADCValue) << 4) | (LOW(ADCValue) >> 4) = (EF << 4) | (C0 >> 4) = FC
  // With non nil factor, offset is incremented.
  // Hence, table size is 0x11


  // 1. Offset = 8, Factor = FC
  // 2. Offset = 7, Factor = 0x48
  byte Offset = HIGH(ADCValue) >> 4;
  byte Factor = (HIGH(ADCValue) << 4) | (LOW(ADCValue) >> 4); // (ADCValue >> 4) & 0xFF
  // 1. return 0x4384
  // 1. return 0x0EB8
  return GetValueFromTableImpl(FlashPtr, Offset, Factor, false);
}

// example inputs:
// 1. 0x831F, 0x8FC0 for Coolant Temperature = 8 degrees
// 2. 0x831F, 0x7480 for Coolant Temperature = -45 degrees
word GetAdcValueFromTableAndAdjustForCalculus(word FlashPtr, word ADCValue) {
  // 1. Result = 0x4384
  // 2. Result = 0x0EB8

  word Result = GetAdcValueFromTable(FlashPtr, ADCValue);

  // 1. Returned value: 0x2F84
  // 2. Returned value: 0x0000
  if (HIGH(Result) < 0x14)
    return COMPOSE_WORD(0, 0);

  return COMPOSE_WORD(HIGH(Result) - 0x14, LOW(Result));
}

// FlashPtr - location of table in FLASH
// Offset - ???
// Factor - ???
// Negate - should table data be XOR'ed with 0x80 (negated?)
// Returns - result:
//           T[N] in high byte and zero in low byte if factor eq nil,
//           factor*(T[N+1] - T[N]) if factor isn't nil
//           N = Offset
// _635D:
// Example inputs:
// A. FlashPtr = 0x831F (coolant temperature table), Negate = false
// A.1. Offset = 0x08, Factor = 0xFC
// A.2. Offset = 0x07, Factor = 0x48
word GetValueFromTableImpl(word FlashPtr, byte Offset, byte Factor, bool Negate) {
  if (!Factor)
    return COMPOSE_WORD(FLASH[FlashPtr + Offset], 0);

  // A.1. TableData = FLASH[0x831F + 0x08 + 1] = 0x44
  // A.2. TableData = FLASH[0x831F + 0x07 + 1] = 0x25
  byte TableData = FLASH[FlashPtr + Offset + 1];

  if (Negate)
    TableData ^= 0x80;

  // A.1. Prod1 = 0x44 * 0xFC = 42F0
  // A.2. Prod1 = 0x25 * 0x48 = 0A68
  word Prod1 = TableData * Factor; // A - low byte of result, B - high byte of result

  // A.1. TableData = FLASH[0x831F + 0x08] = 0x25
  // A.1. TableData = FLASH[0x831F + 0x07] = 0x06
  TableData = FLASH[FlashPtr + Offset];

  if (Negate)
    TableData ^= 0x80;

  // A.1. Prod1 = 0x25 * (-0xFC) = 0x25 * (~0xFC + 1) = 0x25 * 0x04 = 0x0094
  // A.1. Prod1 = 0x06 * (-0x48) = 0x06 * (~0x48 + 1) = 0x06 * 0xB8 = 0x0450
  byte NegFactor = NEGATE(Factor);
  word Prod2 = TableData * NegFactor;

  // A.1. Result = 0x42F0 + 0x94 = 0x4384
  // A.1. Result = 0x0A68 + 0x0450 = 0x0EB8
  word Result = Prod1 + Prod2;

  if (Negate) {
    Result ^= 0x8000;
    //CY = CHECK_BIT_AT(Result, 0x0F);
  }

  return Result;
}

byte AdjustTemperature(word TemperatureTableValue) {
  static const word MAX_SUM = 0xFFFF;
  static const word TO_ADD = WORD(0x50);
  static const word DIVIDER = WORD(0xA0);

  byte Result = 0xFF;

  // 1. 0x5014 < MAX_SUM - TO_ADD, true
  // 2. 0x0000 < MAX_SUM - TO_ADD, true
  // 3. 0xC738 < MAX_SUM - TO_ADD, true
  // 1. 0x2F84 <= MAX_SUM - TO_ADD, true
  // 2. 0x0000 <= MAX_SUM - TO_ADD, true
  if (TemperatureTableValue <= MAX_SUM - TO_ADD) {
    // 1. TemperatureTableValue = 0x2F84 + 0x50 = 0x2FD4
    // 2. TemperatureTableValue = 0x0000 + 0x50 = 0x0050
    TemperatureTableValue += TO_ADD;

    word Quot;

    do {
      CLEAR_BIT_IN(RAM[0x27], 2);

      Quot = TemperatureTableValue / DIVIDER;
    } while (CHECK_BIT_AT(RAM[0x27], 2));

    // 1. Quot = 0x2FD4 / 0xA0 = 0x4C
    // 2. Quot = 0x0050 / 0xA0 = 0
    if (!HIGH(Quot))
      Result = LOW(Quot);
  }

  // 1. Returned value: 0x4C
  // 2. Returned value: 0x00
  return Result;
}

// _5FE2:
// M1 is signed
// M2 is unsigned
word MultiplySigned(byte M1, byte M2) {
  if (!CHECK_BIT_AT(M1, 7))
    return M1 * M2;

  M1 = NEGATE(M1);
  word P = M1 * M2;

  M1 = NEGATE(LOW(P));
  M2 = HIGH(P);

  if (!M1) {
    M2 = NEGATE(M2);
  } else {
    M2 = ~M2;
  }

  return COMPOSE_WORD(M2, M1);
}

// _5FFB:
word scale10bitADCValue(word V, byte Factor) {
  // calculus: (WORD(HIGH(V)) * Factor) + LOW(WORD(LOW(V)) * Factor)
  return HIGH_W(QUAD(V) * Factor);
}

// _64A4:
void addWordInXRAMWord(word V, word XramPtr) {
  V += COMPOSE_WORD(XRAM[XramPtr + 1], XRAM[XramPtr]);

  XRAM[XramPtr] = LOW(V);
  XRAM[XramPtr + 1] = HIGH(V);
}

// _6498:
void addByteInXRAMWord(byte _V, word XramPtr) {
  word V = COMPOSE_WORD(XRAM[XramPtr + 1], XRAM[XramPtr]);
  V += _V;

  XRAM[XramPtr] = LOW(V);
  XRAM[XramPtr + 1] = HIGH(V);
}

// Returns FLASH[FlashPtr + TableIdx] + HIGH(DiffFactor * (FLASH[FlashPtr + TableIdx + 1] - FLASH[FlashPtr + TableIdx]))
// _62CE:
byte InterpolateTableValue(word FlashPtr, byte TableIdx, byte DiffFactor) {
  const byte Base = FLASH[FlashPtr + TableIdx];
  const byte Diff = FLASH[FlashPtr + TableIdx + 1] - Base;
  const word Offset = WORD(DiffFactor) * Diff;

  return Base + HIGH(Offset);
}

// _6060:
word AbsWordByMSB(word V) {
  if (CHECK_BIT_AT(V, 15))
    V = COMPOSE_WORD(0 - HIGH(V) - (!!(LOW(V) != 0)), 0 - LOW(V));

  return V;
}

// _C000:
void init_xram_for_serial0() {
  if (CHECK_BIT_AT(RAM[0x2F], 0)) {
    _C006:
    if (CHECK_BIT_AT(RAM[0x2F], 1)) {
      _C07A:
      XRAM[0xF983] = 0xFF;
      XRAM[0xF984] = 0xFF;

      XRAM[0xF989] = 0x02;
      XRAM[0xF98A] = 0;
      XRAM[0xF98B] = 0x14;
      XRAM[0xF98C] = 0;

      XRAM[0xF98D] = 0x14;
      XRAM[0xF98E] = 0;
      XRAM[0xF98F] = 0x88;
      XRAM[0xF990] = 0x13;

      XRAM[0xF991] = 0;
      XRAM[0xF992] = 0;
      XRAM[0xF993] = 0xC8;
      XRAM[0xF994] = 0;

      XRAM[0xF995] = 0;
      XRAM[0xF996] = 0;
      XRAM[0xF997] = 0x14;
      XRAM[0xF998] = 0;

      XRAM[0xF985] = 0x0A;
      XRAM[0xF986] = 0;
      XRAM[0xF987] = 0x02;
      XRAM[0xF988] = 0;
    } else {
      _C009:
      XRAM[0xF981] = 0x17;
      XRAM[0xF982] = 0;
      XRAM[0xF983] = 0x1B;
      XRAM[0xF984] = 0;

      XRAM[0xF989] = 0;
      XRAM[0xF98A] = 0;
      XRAM[0xF98B] = 0x13;
      XRAM[0xF98C] = 0;

      XRAM[0xF991] = 0x13;
      XRAM[0xF992] = 0;
      XRAM[0xF993] = 0x89;
      XRAM[0xF994] = 0x13;

      XRAM[0xF995] = 0x4;
      XRAM[0xF996] = 0;
      XRAM[0xF997] = 0x15;
      XRAM[0xF998] = 0;

      XRAM[0xF98D] = 0x1A;
      XRAM[0xF98E] = 0;
      XRAM[0xF98F] = 0x31;
      XRAM[0xF990] = 0;

      XRAM[0xF987] = 0;
      XRAM[0xF988] = 0;
    }
  } else {
    _C0EB:
    if (CHECK_BIT_AT(RAM[0x2F], 1)) {
      _C15E:
      XRAM[0xF983] = 0xFF;
      XRAM[0xF984] = 0xFF;

      XRAM[0xF989] = 0x02;
      XRAM[0xF98A] = 0;
      XRAM[0xF98B] = 0x14;
      XRAM[0xF98C] = 0;

      XRAM[0xF98D] = 0xC8;
      XRAM[0xF98E] = 0;
      XRAM[0xF98F] = 0x88;
      XRAM[0xF990] = 0x13;

      XRAM[0xF991] = 0x02;
      XRAM[0xF992] = 0;
      XRAM[0xF993] = 0xC8;
      XRAM[0xF994] = 0;

      XRAM[0xF995] = 0;
      XRAM[0xF996] = 0;
      XRAM[0xF997] = 0x14;
      XRAM[0xF998] = 0;

      XRAM[0xF985] = 0x0A;
      XRAM[0xF986] = 0;
      XRAM[0xF987] = 0x02;
      XRAM[0xF988] = 0;
    } else {
      _C0EE:
      XRAM[0xF981] = 0x17;
      XRAM[0xF982] = 0;
      XRAM[0xF983] = 0x1B;
      XRAM[0xF984] = 0;

      XRAM[0xF989] = 0;
      XRAM[0xF98A] = 0;
      XRAM[0xF98B] = 0x14;
      XRAM[0xF98C] = 0;

      XRAM[0xF991] = 0;
      XRAM[0xF992] = 0;
      XRAM[0xF993] = 0x88;
      XRAM[0xF994] = 0x13;

      XRAM[0xF995] = 0;
      XRAM[0xF996] = 0;
      XRAM[0xF997] = 0x14;
      XRAM[0xF998] = 0;

      XRAM[0xF98D] = 0;
      XRAM[0xF98E] = 0;
      XRAM[0xF98F] = 0xF4;
      XRAM[0xF990] = 0x01;

      XRAM[0xF987] = 0;
      XRAM[0xF988] = 0;
    }
  }

_C1CC:
  CLEAR_BIT_IN(RAM[0x2F], 2);
  CLEAR_BIT_IN(RAM[0x2F], 4);
  CLEAR_BIT_IN(RAM[0x2F], 5);
  CLEAR_BIT_IN(RAM[0x2F], 6);

  XRAM[0xFBB3] = 0;
  XRAM[0xF9A3] = 0;
  XRAM[0xF9A1] = 0;

  if (CHECK_BIT_AT(RAM[0x2F], 1)) {
    _C1F5:
    CLEAR_BIT_IN(IEN0, 4); // disable serial0 interrupt
    XRAM[0xF97F] = 0;
    XRAM[0xF980] = 0;
    RAM[0x77] = 0x04;
  } else {
    _C1E4:
    XRAM[0xF97F] = 0;
    XRAM[0xF980] = 0;
    SET_BIT_IN(IEN0, 4); // enable serial0 interrupt
    RAM[0x77] = 0xFF;
  }
}
