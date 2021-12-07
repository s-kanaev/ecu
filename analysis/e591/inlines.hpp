#pragma once

#include "memory-locations.hpp"
#include "impl.hpp"

#include <include/ram.hpp>
#include <include/xram.hpp>
#include <include/flash.hpp>

inline bool CheckCoolantTemperature(byte XramF6BF, byte Flash805D,
                                    byte CoolantTemperature, byte Min,
                                    byte Max) {
  bool TemperatureOK = true;

  if (XramF6BF >= Flash805D) {
    if (CoolantTemperature < Min) {
      TemperatureOK = false;
      SET_BIT_IN(RAM[0x23], 2);
      CLEAR_BIT_IN(RAM[0x23], 3);
    } else if (CoolantTemperature > Max) {
      TemperatureOK = false;
      CLEAR_BIT_IN(RAM[0x23], 2);
      SET_BIT_IN(RAM[0x23], 3);
    }
  }

  return TemperatureOK;
}

inline word InitRam3ATemperature(byte *TableRangeStart, word CoolantTemperature,
                                 byte Threshold) {
  CLEAR_BIT_IN(RAM[0x23], 2);
  CLEAR_BIT_IN(RAM[0x23], 3);
  word AdjustedCoolantTemp = GetAdcValueFromTableAndAdjustForCalculus(
      TableRangeStart - FLASH, CoolantTemperature);
  RAM[0x3A] = HIGH(AdjustedCoolantTemp);
  if (RAM[0x30] >= Threshold)
    SET_BIT_IN(RAM[0x26], 6);
  else
    CLEAR_BIT_IN(RAM[0x26], 6);

  return AdjustedCoolantTemp;
}

inline bool ShouldAdaptThrottlePositionSensorBasedOnXramValue(byte Value) {
  return (0x0B != Value) && (0x15 != Value) &&
         (0x16 != Value) && (0x17 != Value) &&
         (0x1F != Value) && (0x20 != Value) &&
         (0x29 != Value);
}

inline void Xram_F69D_eq_20_CoolantTemperature() {
  word AdjustedCoolantTemp;

  if (!CHECK_BIT_AT(RAM[0x72], 7)) {
    //_2C15:
    //ram_72_bit_7_clear:
    word ScaledADCCoolantTemp = GET_MEM_WORD(XRAM, COOLANT_TEMP_SUM); // R1:R0
    bool TemperatureOK = true;
    bool Ram26Bit6Set = CHECK_BIT_AT(RAM[0x26], 6);

    if (Ram26Bit6Set) {
      //_2C5F:
      //ram_26_bit_6_set:
      if ((TemperatureOK = CheckCoolantTemperature(
          XRAM[0xF6BF], FLASH[0x805D], HIGH(ScaledADCCoolantTemp), FLASH[0x8059], FLASH[0x805A]))) {
        //_2C86:
        //xram_F6BF_less_than_flash_805D_or_coolant_temperature_OK_2:
        AdjustedCoolantTemp = InitRam3ATemperature(
            GET_RNG_START_PTR(FLASH, COOLANT_TEMPERATURE_TABLE_2), ScaledADCCoolantTemp, FLASH[0x805B]);
      }
    } else {
      //_2C20:
      if ((TemperatureOK = CheckCoolantTemperature(
          XRAM[0xF6BF], FLASH[0x805D], HIGH(ScaledADCCoolantTemp), FLASH[0x8057], FLASH[0x8058]))) {
        //_2C4A:
        //xram_F6BF_less_than_flash_805D_or_coolant_temperature_OK:
        AdjustedCoolantTemp = InitRam3ATemperature(
            GET_RNG_START_PTR(FLASH, COOLANT_TEMPERATURE_TABLE_1), ScaledADCCoolantTemp, FLASH[0x805C]);
      }
    }

    if (!TemperatureOK) {
      //_2CB1:
      //_coolant_error_condition_common:
      RAM[0x3A] = FLASH[0x8A4B + ((XRAM[0xF6BF] >> 4) & 0x0F)];
      AdjustedCoolantTemp = COMPOSE_WORD(RAM[0x3A], 0);
    }
  } else {
    //_2C10:
    AdjustedCoolantTemp = COMPOSE_WORD(RAM[0x3A], 0);
  }

  //_2CC4:
  //temperature_init_done:
  RAM[0x3D] = AdjustTemperature(AdjustedCoolantTemp);
}

inline void Xram_F69D_eq_20_IntakeAirTemperature() {
  bool IntakeAirTempOK = true;
  bool HasIntakeAirTemperatureSensor = kitting_has_intake_air_temperature_sensor();
  word AdjustedIntakeAirTemp;

  if (HasIntakeAirTemperatureSensor && !CHECK_BIT_AT(RAM[0x73], 0)) {
    word IntakeAirTemp = GET_MEM_WORD(XRAM, INTAKE_AIR_SUM);

    if (XRAM[0xF6BF] >= FLASH[0x8060]) {
      //_2CF2:
      if (HIGH(IntakeAirTemp) < GET_MEM_BYTE(FLASH, MINIMUM_INTAKE_AIR_TEMPERATURE)) {
        IntakeAirTempOK = false;
        //_2D17:
        //scaled_intake_air_temperature_less_than_minimum_intake_air_temperature:
        SET_BIT_IN(RAM[0x24], 4);
        CLEAR_BIT_IN(RAM[0x24], 5);
      } else if (GET_MEM_BYTE(FLASH, MAXIMUM_INTAKE_AIR_TEMPERATURE) < HIGH(IntakeAirTemp)) {
        IntakeAirTempOK = false;
        //_2D1D:
        //scaled_intake_air_temperature_larger_or_equal_than_maximum_intake_air_temperature:
        CLEAR_BIT_IN(RAM[0x24], 4);
        SET_BIT_IN(RAM[0x24], 5);
      }
    }

    if (IntakeAirTempOK) {
      //_2D09:
      //xram_F6BF_less_than_flash_8060:
      CLEAR_BIT_IN(RAM[0x24], 4);
      CLEAR_BIT_IN(RAM[0x24], 5);
      // R1:R0
      AdjustedIntakeAirTemp = GetAdcValueFromTableAndAdjustForCalculus(
          RNG_START_IDX(FLASH, INTAKE_AIR_TEMPERATURE_TABLE), IntakeAirTemp);
      RAM[0x3B] = HIGH(AdjustedIntakeAirTemp);
    }
  }

  if (!IntakeAirTempOK || !HasIntakeAirTemperatureSensor) {
    //_2D21:
    //no_intake_air_temperature_sensor:
    RAM[0x3B] = FLASH[0x8061];
    AdjustedIntakeAirTemp = COMPOSE_WORD(RAM[0x3B], 0);
  }

  //_2D2A:
  //ram_73_bit_0_set:
  RAM[0x3E] = AdjustTemperature(AdjustedIntakeAirTemp);
}

inline void Xram_F69D_eq_20_CO_Pot() {
  //_2D31:
  word Product;
  bool HasCOPotSensor = kitting_has_co_potentiometer_sensor();
  bool HasIROM = kitting_has_irom();

  if (HasCOPotSensor) {
    //_2D47:
    bool COPotNotInLimits = false;
    word COPot = GET_MEM_WORD(XRAM, CO_POT_SUM);

    if (COPot < 0x1FFF)
      COPot = 0xFFFF;
    else
      COPot <<= 3;

    if (HIGH(COPot) < FLASH[0x8067]) {
      //_2D5F:
      SET_BIT_IN(RAM[0x23], 4);
      CLEAR_BIT_IN(RAM[0x23], 5);
      COPotNotInLimits = true;
    }

    //_2D65:
    //co_pot_not_less_than_minimum_co_pot:

    if (HIGH(COPot) > FLASH[0x8068]) {
      //_2D6F:
      CLEAR_BIT_IN(RAM[0x23], 4);
      SET_BIT_IN(RAM[0x23], 5);
      COPotNotInLimits = true;
    }

    //_2D75:
    //co_pot_less_or_equal_than_maximum_co_pot:

    if (!COPotNotInLimits) {
      //_2D75:
      CLEAR_BIT_IN(RAM[0x23], 4);
      CLEAR_BIT_IN(RAM[0x23], 5);

      byte High = HIGH(COPot);
      if (CHECK_BIT_AT(High, 7))
        High = 0xFF;
      else
        High *= 2;

      High -= 0x50;

      Product = MultiplySigned(High, 0xC0);
    } else {
      Product = COMPOSE_WORD(FLASH[0x8069], FLASH[0x8069]);
    }
  } else if (HasIROM) {
    //_2D9E:
    //pick_CO_pot_from_eeprom:
    CLEAR_BIT_IN(RAM[0x23], 4);
    CLEAR_BIT_IN(RAM[0x23], 5);

    if (!CHECK_BIT_AT(RAM[0x73], 3))
      SET_MEM_BYTE(XRAM, ADJUSTED_CO_POT, XRAM[0xFF74]);
  } else {
    //_2D8A:
    //fallback_CO_pot:
    CLEAR_BIT_IN(RAM[0x23], 4);
    CLEAR_BIT_IN(RAM[0x23], 5);
    Product = COMPOSE_WORD(FLASH[0x8069], FLASH[0x8069]);
  }

  //_2D91:
  //calculate_xram_f681:
  if (HasCOPotSensor || !HasIROM) {
    if (!CHECK_BIT_AT(RAM[0x73], 3))
      SET_MEM_BYTE(XRAM, ADJUSTED_CO_POT, HIGH(Product));
  }

  //_2DAF:
  //xram_f681_initialized:
  if (!CHECK_BIT_AT(RAM[0x73], 4)) {
    if (HasIROM)
      XRAM[0xF682] = XRAM[0xFF75];
    else
      XRAM[0xF682] = FLASH[0x806A];
  }
}

// _695C
void ClearXramF69E_0C_Bytes() {
  for (word XramPtr = WORD_MEM_IDX(XRAM, COOLANT_TEMP_SUM);
       XramPtr < WORD_MEM_IDX(XRAM, COOLANT_TEMP_SUM) + 0x0C; ++XramPtr)
    XRAM[XramPtr] = 0x00;
  XRAM[0xF69D] = 0;
}

inline void Xram_F69D_eq_20_ThrottlePosition() {
  word Throttle = GET_MEM_WORD(XRAM, THROTTLE_POSITION_2);

  if (((FLASH[0x8080] < GET_MEM_BYTE(XRAM, THROTTLE_POSITION_BYTE_2) - GET_MEM_BYTE(XRAM, THROTTLE_POSITION_BYTE_1)) ||
       (Throttle < GET_MEM_WORD(XRAM, THROTTLE_POSITION_SUM))) &&
      kitting_should_adapt_throttle_position_sensor() &&
      ShouldAdaptThrottlePositionSensorBasedOnXramValue(XRAM[0xF7BC]) &&
      (HIGH(Throttle) > FLASH[0x807C])) {
    //_2E4E:
    word Throttle = GET_MEM_WORD(XRAM, THROTTLE_POSITION_2) + COMPOSE_WORD(0, 0x02);
    SET_MEM_WORD(XRAM, THROTTLE_POSITION_2, Throttle);
  } else {
    //_2E02:
    Throttle = (GET_MEM_WORD(XRAM, THROTTLE_POSITION_SUM) & 0xC0) + 0x40;
    SET_MEM_WORD(XRAM, THROTTLE_POSITION_1, Throttle);
    SET_MEM_WORD(XRAM, THROTTLE_POSITION_2, Throttle);
  }

  //_2E75:
  //no_need_to_adapt_throttle_position_sensor:
  Throttle = GET_MEM_WORD(XRAM, THROTTLE_POSITION_SUM);

  if (HIGH(Throttle) < FLASH[0x807A]) {
    //_2E99:
    //xram_f6a7_less_low_limit:
    SET_BIT_IN(RAM[0x24], 0);
    CLEAR_BIT_IN(RAM[0x24], 1);
  } else if (HIGH(Throttle) > FLASH[0x807B]) {
    //_2E9F:
    //xram_f6a7_larger_upper_limit:
    CLEAR_BIT_IN(RAM[0x24], 0);
    SET_BIT_IN(RAM[0x24], 1);
  } else {
    //_2E93:
    CLEAR_BIT_IN(RAM[0x24], 0);
    CLEAR_BIT_IN(RAM[0x24], 1);
  }

  //_2EA3:
  //done_checking_xram_f6a7:
  word Ram49Sum = GET_MEM_WORD(XRAM, RAM_49_SUM);
  word Ram49SumPrev = GET_MEM_WORD(XRAM, RAM_49_SUM_PREV);
  SET_MEM_WORD(XRAM, RAM_49_SUM_PREV, Ram49Sum);

  if (Ram49Sum < Ram49SumPrev)
    Ram49Sum = 0;
  else {
    Ram49Sum -= Ram49SumPrev;

    if (HIGH(Ram49Sum))
      Ram49Sum = COMPOSE_WORD(HIGH(Ram49Sum), 0xFF);
  }

  XRAM[0xF6AC] = LOW(Ram49Sum);
  ClearXramF69E_0C_Bytes();
  SET_BIT_IN(RAM[0x28], 3);
}

