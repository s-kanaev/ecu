#pragma once

#include <types.hpp>
#include <ram.hpp>
#include <flash.hpp>
#include <xram.hpp>
#include <eeprom.hpp>
#include <binary_ops.hpp>
#include <undefined.hpp>
#include <pins.hpp>
#include <registers.hpp>
#include <mask_set.hpp>

#include "e591/memory-locations.hpp"
#include "e591/impl.hpp"
#include "e591/inlines.hpp"

inline byte registerBankMask(byte RS) {
  assert(RS >= 0 && RS < 4);
  return RS << 3;
}

// _08C6:
inline void _087D() {
  RAM[0x30] = 0;
  CLEAR_BIT_IN(RAM[0x25], 2);
  CLEAR_BIT_IN(RAM[0x25], 3);
  RAM[0x44] = RAM[0x45] = 0;
}

// _08C6:
inline void _08C6() {
  SET_BIT_IN(RAM[0x25], 5);
  _087D();
}

// _0823:
inline void _0823() {
  Reg::R1_R0 R1_R0;
  Reg::R3_R2 R3_R2;
  Reg::R5_R4 R5_R4;

  R5_R4 = R1_R0;
  R3_R2 = Reg::CC3::Inst;
  goto set_ram_25_5_and_graceful_finish_ext_int_6; // => _0C03 TODO
}

inline void regularToothCapturedImpl() {
  if (R5 >= 4) {
    // _09BD:
    CLEAR_BIT_IN(RAM[0x25], 2);
    RAM[0x44] = RAM[0x45] = 0;
  }

  // r5_less_than_4:
  // _09C5:
  if (!CHECK_AND_CLEAR_BIT(RAM[0x25], 4) && R7 == RAM[0x36]) {
    // _09D5:
    RAM[0x34] = (RAM[0x33] >= RAM[0x37]) ? (RAM[0x33] - RAM[0x37]) : (RAM[0x33] - RAM[0x37] + 4);
    SET_BIT_IN(RAM[0x27], 0);
    // External Interrupt0 Request flag, schedule vector to external interrupt 0?
    {
      Reg::Set<Reg::TCON>{}.add<Reg::TCON::IE0>();
    }

    Ram36WordT::set(Ram38WordT::get());
    SET_BIT_IN(RAM[0x25], 4);
  }

  // _09E3:
  // ram_25_bit_4_set_or_r7_neq_ram_36:
  if (equal(R6, R0_bank2)) {
    // _09EC:
    SET_BIT_IN(P5, 4);
    Reg::R1_R0::set((QUAD(R5_R4) << 1) * Reg::R1Sel<2>::get() / 256);

    // _0A11:
    Reg::R1_R0::set(R1_R0 + R3_R2);

    if (R5_bank2 < 2) {
      RAM[0x5C] = R5_bank2;
    } else {
      RAM[0x5C] = R5_bank2 - 2;
    }

    switch (RAM[0x5C]) {
      case 0x00: {
        // _0A25:
        byte IgnCoil14ADCV = ADC_8bit(IGNITION_COIL_14_PIN);
        bool IgnCoil14ADCVAboveLimit = false;

        if (IgnCoil14ADCV >= 0x83) {
          // _0A3A:
          RAM[0x89] |= 4;
          IgnCoil14ADCVAboveLimit = true;
        } else if (IgnCoil14ADCV < 0x14) {
          // _0A4E:
          RAM[0x89] |= 8;
        }

        // _0A5B:
        // ign_coil_14_adc_voltage_greater_or_equal_14:
        if (!IgnCoil14ADCVAboveLimit) {
          // _0A5F:
          Reg::COMCLR::Inst = R1_R0;

          // Start charging Ignition Coil 1/4 when COMCLR matches Timer2
          {
            Reg::Set<Reg::CLRMSK>{}.add<Reg::CLRMSK::b1>();
          }
          {
            Reg::Mask<Reg::SETMSK>{}.add<Reg::SETMSK::b1>();
          }
          // Reset COMCLR Register compare interrupt request
          {
            Reg::Mask<Reg::CTCON>{}.add<Reg::CTCON::ICR>();
          }
          // Enable COMCLR Register compare interrupt
          {
            Reg::Set<Reg::IEN0>{}.add<Reg::IEN0::ECR>();
          }

          CLEAR_BIT_IN(RAM[0x2E], 6);
          CLEAR_BIT_IN(RAM[0x2E], 7);
        }
      }
      case 0x01: {
        // _0A76:
        byte IgnCoil23ADCV = ADC_8bit(IGNITION_COIL_23_PIN);
        bool IgnCoil23ADCVAboveLimit = false;

        if (IgnCoil23ADCV >= 0x83) {
          // _0A8B:
          RAM[0x89] |= 1;
          IgnCoil23ADCVAboveLimit = true;
        } else if (IgnCoil23ADCV < 0x14) {
          // _0A9F:
          RAM[0x89] |= 2;
        }

        // _0AAC:
        // ign_coil_23_adc_voltage_greater_or_equal_14:
        if (!IgnCoil23ADCVAboveLimit) {
          // _0AB0:
          Reg::COMCLR::Inst = R1_R0;

          // Start charging Ignition Coil 2/3 when COMCLR matches Timer2
          {
            Reg::Set<Reg::CLRMSK>{}.add<Reg::CLRMSK::b0>();
          }
          {
            Reg::Mask<Reg::SETMSK>{}.add<Reg::SETMSK::b0>();
          }
          // Reset COMCLR Register compare interrupt request
          {
            Reg::Mask<Reg::CTCON>{}.add<Reg::CTCON::ICR>();
          }
          // Enable COMCLR Register compare interrupt
          {
            Reg::Set<Reg::IEN0>{}.add<Reg::IEN0::ECR>();
          }

          CLEAR_BIT_IN(RAM[0x2E], 6);
          CLEAR_BIT_IN(RAM[0x2E], 7);
        }
      }
    }

    //goto r6_bank_3_neq_r0_bank_2_impl; // _0AC5
  }

  // _09E9:
  // r6_bank_3_neq_r0_bank_2:
  // goto r6_bank_3_neq_r0_bank_2_impl // _0AC5
  // _0AC5:
  // r6_bank_3_neq_r0_bank_2_impl:
  if (equal(R6, R2_bank2)) {
    // _0ACD:
    CLEAR_BIT_IN(P5, 4);
    R1_R0 = QUAD(R5_R4) * R3_bank2 / 256;

    // _0AF2:
    R1_R0 += R3_R2;

    switch (R5_bank2) {
      case 0:
      case 2: {
        // _0B04:
        Reg::COMSET::Inst = R1_R0;
        SET_BIT_IN(Reg::SETMSK, 1); // Ignite Coil 1/4 when Timer2 matches COMSET
        break;
      }
      case 1:
      case 3: {
        // _0B0E:
        Reg::COMSET::Inst = R1_R0;
        SET_BIT_IN(Reg::SETMSK, 0); // Ignite Coil 2/3 when Timer2 matches COMSET
        break;
      }
    }

    // _0B15:
    // r5bank2_equals_neither_1_nor_3:
    R6_bank2 = R5_bank2;

    if (++R5_bank2 == 4)
      R5_bank2 = 0;

    // _0B22:
    if (CHECK_BIT_AT(RAM[0x2D], 7) &&
        !CHECK_BIT_AT(RAM[0x2D], 4) &&
        (!!CHECK_BIT_AT(RAM[0x26], 5) != !!CHECK_BIT_AT(RAM[0x25], 0))) {
      // _0B33:
      if (R5_bank2 < 2)
        R5_bank2 += 2
      else /* R5(bank2) >= 2 */
        R5_bank2 -= 2

      // R5_bank2 ^= 0x02; // ???
    }

    // _0B3E:
    R0_bank2 = RAM[0x99 + (R5_bank2 << 2)];
    R1_bank2 = RAM[0x99 + (R5_bank2 << 2) + 1];
    COPY_BIT(RAM[0x26], 5, RAM[0x99 + (R5_bank2 << 2) + 2], 7);
    R2_bank2 = RAM[0x99 + (R5_bank2 << 2) + 2] & 0x7F;
    R3_bank2 = RAM[0x99 + (R5_bank2 << 2) + 3];

    //goto r6_bank_3_neq_r6_bank_2_impl; // _0B58
  }

  // _0ACA:
  // r6_bank_3_neq_r2_bank_2
  // goto r6_bank_3_neq_r6_bank_2_impl; // _0B58
  // r6_bank_3_neq_r6_bank_2_impl:
  // _0B58:
  if (equal(R6, R7_bank1)) {
    // _0B60:
    // r6_eq_r7_bank1:
    if (!CHECK_BIT_AT(RAM[0x2E], 1)) {
      // _0B9A:
      // ram_2e_bit_1_not_set:
      RAM[0x6D] = FLASH[0x0785 + R6_bank2] - 1;
      SET_BIT_AT(RAM[0x27], 1);

      // Request External Interrupt 0
      {
        Reg::Set<Reg::TCON>{}.add<Reg::TCON::IE0>();
      }
    } else {
      // _0B63:
      R1_R0 = (QUAD(R5_R4) << 1) * R4_bank2 / 256;

      // _0B86:
      R1_R0 += R3_R2;
      Reg::CC4::Inst = R1:R0;

      // Reset P1.4/INT2/CC4 interrupt request
      {
        Reg::Mask<Reg::IRCON0>{}.add<Reg::IRCON0::IEX2>();
      }

      // Enable External Interrupt2 / CC4 interrupt
      {
        Reg::Set<Reg::IEN1>{}.add<Reg::IEN1::EX2>();
      }

      // Set alternative input function for P1.4/INT2/CC4
      SET_BIT_IN(P1, 4);

      SET_BIT_AT(RAM[0x25], 7);
    }

    // _0BAF:
    byte Offset = R6_bank2 + 1;

    if (4 == Offset)
      Offset = 0;
    else if (Offset >= 2)
      Offset -= 2;

    Offset <<= 1;

    byte Val = RAM[0xA9 + Offset];
    byte Diff = Val - R0_bank2;

    if (Val < R0_bank2) {
      // _0BC9:
      Val = Diff + 0x3C;
    }

    // _0BCB:
    // ram_a9_plus_offset_shl_1_larger_or_eqal_r0_bank2:
    if (Val < 3) {
      // _0BD4:
      // diff_less_than_3:
      if (R0_bank2 + 3 >= 0x3C)
        R7_bank1 = R0_bank2 + 3 - 0x3C;
      else
        R7_bank1 = R0_bank2 + 3;
    } else {
      R7_bank1 = RAM[0xA9 + Offset];
    }

    // _0BE1:
    R4_bank2 = RAM[0xA9 + Offset + 1];
    R0 = Offset + 1;

    //goto r6_neq_r7_bank1_impl; // _0BE4
  }

  // _0B5D:
  // r6_neq_r7_bank1:
  // goto r6_neq_r7_bank1_impl; // _0BE4
  // r6_neq_r7_bank1_impl:
  // _0BE4:
  if (!CHECK_BIT_AT(R7, 0) && (R7 < 0x1E)) {
    // _0BF0:
    RAM[0x8A + (R7 >> 1)] = ADC_8bit(MAF_PIN);
  }

  //goto set_ram_25_5_and_graceful_finish_ext_int_6; // _0C03
}

// _0897:
// Called iff ExtInt6 is triggered for compare match on CC3
// Uses register bank 3
// R6 - synchronization disc tooth counter ?
// R7 - same?
// R3:R2 - assigned to CC3 if RAM[0x25].1 is set
// R5:R4 - contains delta when to match compare on CC3 yet another time
inline void ExtInt6_CompareMatch() {
  // ram_26_2_is_set:
  Reg::PSW::Inst = registerBankMask(3);

  Reg::R6 R6;
  Reg::R7 R7;
  Reg::R3_R2 R3_R2;
  Reg::R5_R4 R5_R4;

  if (!CHECK_BIT_AT(RAM[0x25], 1)) {
    // Prepare for crankshaft revolution to begin?
    // _088D:
    // ram_25_bit_1_not_set:
    CLEAR_BIT_IN(RAM[0x26], 2);
    // Capture on rising edge at P1.3/INT6/CC3
    {
      Reg::Mask<Reg::CCEN>{}.add<Reg::CCEN::COCAH3, Reg::CCEN::COCAL3>();
    }
    {
      Reg::Set<Reg::CCEN>{}.add<Reg::CCEN, COCAL3>();
    }

    _08C6(); // TODO
    goto graceful_finish_ext_int_6; // _0C05 TODO
    UNREACHABLE;
  } else {
    // _08A3:
    R3_R2 = Reg::CC3::Inst;
    // 0x39 = 57 (dec), maximum for [0..58=60-2) segment
    if (R6 != 0x39) {
      // Lets capture the next tooth timestamp
      // _08B9:
      // r6_neq_39:
      CLEAR_BIT_IN(RAM[0x26], 2);

      // Capture on rising edge at P1.3/INT6/CC3
      {
        Reg::Mask<Reg::CCEN>{}.add<Reg::CCEN::COCAH3, Reg::CCEN::COCAL3>();
      }
      {
        Reg::Set<Reg::CCEN>{}.add<Reg::CCEN, COCAL3>();
      }

      // We think crankshaft has rotated for another tooth
      ++R6;
      ++R7;
    } else /* R6 == 0x39 (57 dec) */ {
      /* Start of missing tooth region, can't capture, only compare */
      // _08AA:
      Reg::CC3::Inst += R5_R4::get();
      {
        Reg::Mask<Reg::IRCON0>{}.add<Reg::IRCON0::IEX6>();
      }

      // We think crankshaft has rotated for another tooth
      ++R6;
      ++R7;
    }

    //goto r7_neq_1e;
    regularToothCapturedImpl(); // _09B7 TODO do the usual thing?
    UNREACHABLE;
  }
  UNREACHABLE;
}

inline void prepareForMissingToothRegion() {
  Reg::R5_R4 R5_R4;
  Reg::R1_R0 R1_R0;
  Reg::R3_R2 R3_R2;

  // Missing tooth region is about to begin. Hence, schedule a compare match
  // within R1_R0 jiffies. Store R1_R0 in R5_R4 for the next event for checks.
  // _091F:
  R5_R4 = R1_R0;
  SET_BIT_IN(RAM[0x26], 2);
  // Compare mode for P1.3/INT6/CC3
  {
    Reg::Mask<Reg::CCEN>{}.add<Reg::CCEN::COCAH3, Reg::CCEN::COCAL3>();
  }
  {
    Reg::Set<Reg::CCEN>{}.add<Reg::CCEN, COCAH3>();
  }

  SET_BIT_IN(P1, 3); // enable alternate function of P1.3

  Reg::CC3::Inst = R3_R2 + R5_R4;

  // IRCON0.IEX6 = 0
  // should be cleared by H/W when processor vectors to interrupt
  // routine
  {
    Reg::Mask<Reg::IRCON0>{}.add<Reg::IRCON0::IEX6>();
  }

  ++R6;
  ++R7;

  //goto r7_neq_1e;
  regularToothCapturedImpl(); // _09B7 TODO do the usual thing?
}

// _0955:
inline void firstToothAfterMissingOnesCaptured() {
  // The first tooth after missing one is captured?
  // _0955:
  R6 = 0;
  R7 = 0;

  if (CHECK_BIT_AT(RAM[0x25], 5)) {
    // _095C:
    location::AsWord<seg::RAM, 0x44>::set(
        R3_R2 - location::AsWord<seg::RAM, 0x46>::get());
  }

  // _0967:
  // ram_25_bit_2_not_set:
  location::AsWord<seg::RAM, 0x46>::set(R3_R2);

  SET_BIT_IN(RAM[0x25], 2);

  if (CHECK_BIT_AT(RAM[0x2D], 7)) {
    // _0974:
    // ram_2d_bit_7_set:
    if (!CHECK_BIT_AT(RAM[0x2E], 0)) {
      // _097B:
      // ram_2e_bit_0_not_set:
      bool CamshaftPositionIRQ =
          !!Reg::Bit<Reg::IRCON0, Reg::IRCON0::IEX5>::get();

      {
        Reg::Mask<Reg::IRCON0>{}.add<IRCON0::IEX5>();
      }
      if (CamshaftPositionIRQ) {
        // TODO Check where it is used
        Reg::Mask<Reg::PSW>{}.add<Reg::PSW::CY>();
      } else {
        // TODO Check where it is used
        Reg::Set<Reg::PSW>{}.add<Reg::PSW::CY>();
      }
    } else {
      // _0977:
      bool CamshaftMarkActive = CHECK_BIT_AT(P1, 2);
      if (CamshaftMarkActive) {
        // TODO Check where it is used
        Reg::Set<Reg::PSW>{}.add<Reg::PSW::CY>();
      } else {
        // TODO Check where it is used
        Reg::Mask<Reg::PSW>{}.add<Reg::PSW::CY>();
      }
    }

    // _0980:
    if (CHECK_AND_CLEAR_BIT(RAM[0x29], 6)) {
      // _09A1:
      // ram_29_bit_6_set:
      if (Reg::Bit<Reg::PSW, Reg::PSW::CY>::get()) {
        SET_BIT_IN(RAM[0x25], 0);
      } else {
        CLEAR_BIT_IN(RAM[0x25], 0);
      }

      CLEAR_BIT_IN(RAM[0x2D], 4);
      goto _09AB;
    } else if (Reg::Bit<Reg::PSW, Reg::PSW::CY>::get()) {
      // _098A:
      // camshaft_reference_mark_active:
      if (!CHECK_BIT_AT(RAM[0x25], 0)) {
        goto ram_25_bit_0_not_set_2; // _09A7 TODO
      } else {
        goto ram_25_bit_0_not_set; // _098D TODO
      }
    } else if (!CHECK_BIT_AT(RAM[0x25], 0)) {
      // _098D:
      // ram_25_bit_0_not_set:
      SET_BIT_IN(RAM[0x2D], 4);

      // _098F:
      byte Ram33 = RAM[0x33];

      if (++Ram33 == 4)
        Ram33 = 0;

      if (!Ram33 || Ram33 > 2) {
        // _09B2:
        // inc_ram_33_should_reset_to_0:
        RAM[0x33] = 0;
        SET_BIT_IN(RAM[0x25], 6);
      } else /* if (Ram33 <= 2) */ {
        // _09AD:
        // ram_33_less_than_2:
        RAM[0x33] = 2;
      }

      //goto r7_neq_1e;
      regularToothCapturedImpl(); // _09B7 TODO
    } else {
      // _09A7:
      // ram_25_bit_0_not_set_2:
      CLEAR_BIT_IN(RAM[0x2D], 4);
      if (CHECK_BIT_AT(RAM[0x25], 0))
        CLEAR_BIT_IN(RAM[0x25], 0);
      else
        SET_BIT_IN(RAM[0x25], 0);

      // _09AB:
      if (!Reg::Bit<Reg::PSW, Reg::PSW::CY>::get()) {
        goto inc_ram_33_should_reset_to_0; // _09B2 TODO
      } else {
        goto ram_33_less_than_2; // _09AD TODO
      }
    }
  } else {
    // _0970:
    CLEAR_BIT_IN(RAM[0x2D], 4);
    goto _098F;
  }
}

// Just a fool-proof comparison for equality
template <typename T1, typename T2>
bool equal(const T1 lhs, const T2 rhs) {
  return lhs == rhs;
}

// _093E:
inline void regularToothCaptured() {
  // Regular tooth captured?
  // _093E:
  R5_R4 = R1_R0;
  ++R6;
  ++R7;

  if (0x1E == R7) {
    R7 = 0;
    ++RAM[0x33];
  }

  // _09B7:
  // r7_neq_1e:
  regularToothCapturedImpl();
}

// _0870:
inline void _0870() {
  // _0870:
  Reg::CLRMSK::Inst = 0;
  Reg::SETMSK::Inst = 0;
  IGNITE_COIL(CoilE::C_1_4);
  IGNITE_COIL(CoilE::C_2_3);
  // Disable COMCLR register compare match Interrupt
  {
    Reg::Mask<Reg::IEN2>{}.add<Reg::IEN2::ECR>();
  }

  _087D();
}

inline void _0912() {
  Reg::R3_R2 R3_R2;
  Reg::R6 R6;

  // _0912:
  word CurrentEventTimestamp = Reg::CC3::Inst;
  R3_R2 = CurrentEventTimestamp;
  COPY_BIT(RAM[0x26], 0, RAM[0x26], 1);
  CLEAR_BIT_IN(RAM[0x26], 1);

  // R6 special values:
  // 0x38 - 56 (dec)
  // 0x3B - 59 (dec)

  if (0x38 == R6) {
    // _091F:
    prepareForMissingToothRegion();
  } else if (R6 < 0x38) {
    regularToothCaptured();
  } else if (0x3B == R6) {
    firstToothAfterMissingOnesCaptured();
  } else /* (R6 > 0x38) && (R6 != 0x3B) */ {
    // _094D:
    // r6_neq_3b:
    SET_BIT_IN(RAM[0x20], 3);
    _0870(); // TODO
  }
}

// should return after this proc
// _086E:
inline void _086E() {
  // _086E:
  // r5_r4_less_than_ram_32_ram_31:
  SET_BIT_IN(RAM[0x20], 0);

  _0870();
  //goto graceful_finish_ext_int_6; // _0C05
}

// _08E1:
// Called iff ExtInt6 is triggered for capture and P1.3 (crankshaft) is active,
// alias high, alias set, alias equal 1.
// Uses register bank 3 (main), register bank2, register bank 1
// Bank 3 (main)
// R1:R0 -
// R3:R2 -
// R5:R4 -
//
// Bank 2
// R0 -
// R1 -
// R2 -
// R3 -
// R4 -
// R5 -
// R6 -
//
// Bank 1
// R7 -
inline void ExtInt6_CaptureEvent() {
  // non_even_zero_cross?:
  using Ram31WordT = location::AsWord<seg::RAM, 0x31>;
  using Ram36WordT = location::AsWord<seg::RAM, 0x36>;
  using Ram38WordT = location::AsWord<seg::RAM, 0x38>;

  Reg::R0Sel<2> R0_bank2;
  Reg::R1Sel<2> R1_bank2;
  Reg::R2Sel<2> R2_bank2;
  Reg::R3Sel<2> R3_bank2;
  Reg::R4Sel<2> R4_bank2;
  Reg::R5Sel<2> R5_bank2;
  Reg::R6Sel<2> R6_bank2;
  Reg::R7Sel<1> R7_bank1;

  Reg::R1_R0 R1_R0;
  Reg::R3_R2 R3_R2;
  Reg::R5_R4 R5_R4;

  Reg::PSW::Inst = registerBankMask(3);
  if (!CHECK_BIT_AT(RAM[0x25], 1)) {
    _08C6(); // TODO
    //goto graceful_finish_ext_int_6;
    return;
  }

  // _08ED:
  word LastEventTimestap = R3_R2;
  word TimeSinceLastEvent = Reg::CC3::Inst - LastEventTimestap;
  R1_R0 = TimeSinceLastEvent;

  if (RAM[0x30] != 4) {
    // _08CB:
    // ram_30_neq_4:
    if (RAM[0x30] < 2) {
      // _08D3:
      {
        Reg::Set<Reg::PSW> S;
        S.add<Reg::PSW::CY>(); // TODO where is it used?
      }
      ++RAM[0x30];

      _0823(); // TODO
    } else {
      // _08D0:
      // goto _0791;
      // _0791:
      word R5_R4x2 = R5_R4 << 1;
      RAM[0x31] = LOW(R5_R4x2);

      if (!CHECK_BIT_AT(R5_R4, 15)) {
        // _079D:
        // r5_r4_shl_1_bit_15_not_set:
        RAM[0x32] = HIGH(R5_R4x2);

        Ram31WordT::set(R1_R0 - R5_R4x2);
        {
          // TODO Find where it's used and pass it as a bool parameter
          Reg::Set<Reg::PSW>{}.add<Reg::PSW::F0>();
        }

        if (R1_R0 < R5_R4x2) {
          // _07AE:
          {
            // TODO Find where it's used and pass it as a bool parameter
            Reg::Mask<Reg::PSW>{}.add<Reg::PSW::F0>();
          }

          Ram31WordT::set(NEGATE(Ram31WordT::get()));
        }
        // _07BE:
        // r1_r0_was_larger_than_r5_r4_shl_1:
        word Ram31W = Ram31WordT::get();
        Ram31WordT::set(Ram31W - R5_R4);

        if (Ram31W >= R5_R4) {
          // _07EA:
          // ram_32_ram_31_was_greater_or_equal_than_r5_r4:
          if (!R5) {
            // _07FB:
            Ram31WordT::set(Ram31W - (R5_R4 >> 1));

            if (Ram31W < (R5_R4 >> 1)) {
              goto no_overflow_on_ram32_ram31_plus_half_r5_r4; // _0820 TODO
            } else {
              // _0806
              if (Reg::Bit<Reg::PSW, Reg::PSW::F0>{}.get()) {
                // _080E:
                // psw_f0_set:
                if (RAM[0x30] != 3) {
                  goto no_overflow_on_ram32_ram31_plus_half_r5_r4; // => _0820 TODO
                } else {
                  // _0813:
                  R3_R2 = Reg::CC3::Inst;
                  ++RAM[0x30];
                  CLEAR_BIT_IN(RAM[0x26], 1);
                  CLEAR_BIT_IN(RAM[0x26], 0);
                  goto _0955; // TODO
                }
                UNREACHABLE;
              } else {
                // _0809:
                RAM[0x30] = 3;
                goto _0823; // TODO
              }
              UNREACHABLE
            }
            UNREACHABLE
          } else {
            // _07ED:
            quad Sum = QUAD(Ram31W) + (R5_R4 >> 1);
            Ram31WordT::set(LOW_W(Sum));

            if (Sum <= 0xFFFF) {
              goto no_overflow_on_ram32_ram31_plus_half_r5_r4; // => _0820 TODO
            } else {
              goto _0806; // TODO
            }
            UNREACHABLE
          }
          UNREACHABLE;
        } else /* _07CB: */ if (!R5) {
          // _07DE:
          // r5_eq_0:
          quad Sum = QUAD(Ram31W) + (R5_R4 >> 1);
          Ram31WordT::set(LOW_W(Sum));

          if (Sum <= 0xFFFF) {
            goto no_overflow_on_ram32_ram31_plus_half_r5_r4; // => _0820 TODO
          } else {
            goto _0806; // TODO
          }
          UNREACHABLE;
        } else /* R5 != 0 */ {
          // _07CE:
          quad Sum = QUAD(Ram31W) + (R5_R4 >> 1);
          Ram31WordT::set(LOW_W(Sum));

          if (Sum <= 0xFFFF) {
            // _0820:
            // no_overflow_on_ram32_ram31_plus_half_r5_r4:
            RAM[0x30] = 2;

            // _0823:
            _0823();
          } else {
            goto _0806; // TODO
          }
          UNREACHABLE;
        }
        UNREACHABLE;
      } else {
        // _079A:
        goto no_overflow_on_ram32_ram31_plus_half_r5_r4; // => _0820 TODO
      }
      UNREACHABLE
    }
  } else /* _08F8: */ if (!R1 && !R5) {
    // RAM[0x30] equals 4 here
    // _0902:
    // both_r1_and_r5_eq_0:

    if (R1_R0 < R5_R4 / 2) {
      // Malfunction of crankshaft position sensor or rapid acceleration?
      // _084B:
      // TODO
    } else if (R1_R0 < 3 * R5_R4 / 2) {
      // Crankshaft has rotated for yet another tooth. We captured the tooth.
      // _0912:
      // TODO
    } else /* if (R1_R0 >= 3 * R5_R4 / 2) */ {
      // The first tooth after the missing ones is captured?
      // _086E:
      _086E();
      return;
    }
  } else {
    // RAM[0x30] == 4 here
    // _082E:
    // either_r1_or_r5_neq_0:

    if (R1_R0 >= (R5_R4 * 2) {
      // The first tooth after the missing ones is captured?
      _086E();
      return;
    } else if ((R1_R0 >= R5_R4) || (R1_R0 >= (R5_R4 / 2)) {
      // Crankshaft has rotated for yet another tooth. We captured the tooth.
      // _0912:
      // TODO
    } else /* if (R1_R0 < (R5_R4 / 2)) */ {
      // Malfunction of crankshaft position sensor or rapid acceleration?
      // _084B:
      if (CHECK_BIT_AT(RAM[0x26], 1)) {
        // _086A:
        // ram_26_1_set:
        SET_BIT_IN(RAM[0x20], 1);
        goto _0870; // TODO
      } else if (CHECK_BIT_AT(RAM[0x26], 0)) {
        // _0866:
        // ram_26_0_set:
        SET_BIT_IN(RAM[0x20], 2)
        goto _0870; // TODO
      } else {
        // _0851:
        SET_BIT_IN(RAM[0x26], 1);
        //goto graceful_finish_ext_int_6; // => _0C05 TODO
        return;
      }
    }
  }
}

// _08D8:
void ExternalInterrupt6() {
  if (CHECK_BIT_AT(RAM[0x26], 2)) {
    ExtInt6_CompareMatch();
  } else if (CHECK_BIT_AT(P1, 3)) {
    // _08E1:
    // non_even_zero_cross?:
    ExtInt6_CaptureEvent();
  } else {
    // _0C0B:
    // ret_from_ext_int_6:
    return;
  }

  // _0C05:
  // graceful_finish_ext_int_6:
  // 'graceful' here means 'POP all PUSH-ed registers'
  return;
}
