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

inline void StopTimer0() {
  CLEAR_BIT_IN(TCON, 4);
}

inline void StartTimer0() {
  SET_BIT_IN(TCON, 4);
}

inline void DisableSerial0Int() {
  Reg::Mask<Reg::IEN0> M;
  M.add<Reg::IEN0::ES0>();
}

inline void EnableSerial0Int() {
  Reg::Set<Reg::IEN0> M;
  M.add<Reg::IEN0::ES0>();
}

inline void EnableSerial0Receiver() {
  Reg::Set<Reg::S0CON> M;
  M.add<Reg::S0CON::REN0>();
}

inline void DisableSerial0Receiver() {
  Reg::Mask<Reg::S0CON> M;
  M.add<Reg::S0CON::REN0>();
}

inline bool checkSerial0TxD() {
  return CHECK_BIT_AT(P3, 1);
}

inline bool checkSerial0RxD() {
  return CHECK_BIT_AT(P3, 0);
}

#define BYTE_PAIR_GEQ(rh, lh) \
  ((XRAM[(rh)] >= XRAM[(lh)]) && (XRAM[(rh) - 1] >= XRAM[(lh) - 1]))

inline void nullifyXramF97F_F980_switchRam77To4() {
  // _0F29:
  // ram_2f_bit_4_or_bit_5_set:
  XRAM[0xF97F] = XRAM[0xF980] = 0;
  RAM[0x77] = 4;
}

inline void nullifyXram_F9A0_F9A1_F97F_F980_switchRam77To4() {
  // _0F1F:
  XRAM[0xF9A0] = XRAM[0xF9A1] = 0;
  nullifyXramF97F_F980_switchRam77To4();
}

inline void _0F1C() {
  // _0F1C:
  DisableSerial0Int(); // Disable Serial0 Interrupt
  nullifyXram_F9A0_F9A1_F97F_F980_switchRam77To4();
}

inline void switchRam77ToFC() {
  RAM[0x77] = 0xFC;
}

inline void _1192() {
  // _1192:
  XRAM[0xF97F] = XRAM[0xF980] = 0;
  // Clear Serail0 Receiver Interrupt request flag
  {
    Reg::Mask<Reg::S0CON> M;
    M.add<Reg::S0CON::RI0>();
  }

  EnableSerial0Receiver();
  RAM[0x77] = 0xFF;

  CLEAR_BIT_IN(RAM[0x2F], 1);
  init_xram_for_serial0();
  S0RELH_S0RELL = 0:0;
}

inline void _11E7() {
  // _11E7:
  // ram_2f_bit_1_not_set_4:
  EnableSerial0Int();
  RAM[0x77] = 0x06;
}

inline void _11B3() {
  // _11B3:
  // ram_2f_bit_2_set:
  XRAM[0xF97F] = XRAM[0xF980] = 0;
  if (!XRAM[0xF991] && !XRAM[0xF992]) {
    // _11D2:
    // xram_f991_and_xram_f992_eq_0:
    if (CHECK_BIT_AT(RAM[0x2F], 1) &&
        (XRAM[0xF983] || XRAM[0xF984])) {
      // _11E1:
      EnableSerial0Int();
      RAM[0x77] = 0xFF;
    }
    _11E7();
  } else {
    // _11C9:
    DisableSerial0Int();
    RAM[0x77] = 0x05;
  }
}

inline void switchRam77ToFF() {
  // _0EAF:
  XRAM[0xF97F] = XRAM[0xF980] = 0;
  EnableSerial0Int(); // Enable Serial0 interrupt
  RAM[0x77] = 0xFF;
}

inline void switchRam77To2() {
  // _124A:
  // xram_f987_and_xram_f988_eq_0:
  XRAM[0xF97F] = XRAM[0xF980] = 0;
  {
    Reg::Mask<Reg::S0CON> M;
    M.add<Reg::S0CON::TB80>();
  }
  // Set Serial0 Transmitter interrupt flag?
  {
    Reg::Set<Reg::S0CON> S;
    S.add<Reg::S0CON::TI0>();
  }
  EnableSerial0Int();
  RAM[0x77] = 2;
}

inline void _11F3() {
  // _11F3:
  if (CHECK_BIT_AT(RAM[0x2F], 1)) {
    // _11F9:
    // ram_2f_bit_1_not_set_5:
    XRAM[0xF99F] = XRAM[0xF9A3];
    XRAM[0xF99E] = XRAM[0xF9A2];

    DPTR[7] = 0xFAA8;
    if (!XRAM[0xF98D] && !XRAM[0xF98E]) {
      switchRam77To2();
    } else {
      // _123A:
      XRAM[0xF97F] = XRAM[0xF980] = 0;
      RAM[0x77] = 3;
    }
  } else {
    // _11F6:
    nullifyXram_F9A0_F9A1_F97F_F980_switchRam77To4();
  }
}

inline void _11F0() {
  // _11F0:
  DisableSerial0Receiver();

  _11F3();
}

// _12FF:
inline void finishTimer0OverflowInterrupt() {
  // _12FF:
  const byte Ram6AInc = ++RAM[0x6A];
  if (Ram6AInc == 0x3E)
    RAM[0x6A] = 0;

  if (Ram6AInc >= RAM[0x6B])
    SET_BIT_IN(P4, 4); // start scavenging absorber
  else
    CLEAR_BIT_IN(P4, 4); // stop scavenging absorber

  if (++RAM[0x68] >= RAM[0x69])
    SET_BIT_IN(P9, 1); // turn on fuel metering device?

  // _131B:
  // ram_68_less_ram_69:
  if (0x7D == RAM[0x68]) {
    RAM[0x68] = 0;
    CLEAR_BIT_IN(P9, 1);
  }

  // _1324:
  // ram_68_neq_7d:

  if (Reg::Bit<Reg::TCON, Reg::TCON::TF0>{}.get())
    SET_BIT_IN(RAM[0x20], 4);

  // _1333:
  // FINISH Timer0 Overflow Interrupt
}

inline void clearRam29Bit1() {
  // _12FD:
  // flash_8752_is_nil:
  CLEAR_BIT_IN(RAM[0x29], 1);
}

inline void defaultRam4E() {
  // _12F3:
  RAM[0x4E] = (FLASH[0x8751] & 0xF0) >> 4;
}

// _12C9:
inline void either_ignition_coil_just_discharged() {
  // _12C9:
  if (--RAM[0x4F]) {
    defaultRam4E();
    clearRam29Bit1();
  } else /* RAM[0x4F] started being equal 0 */ {
    switch (RAM[0x4D]) {
      case 0:
      case 1:
      default:
        // ram_4d_not_eq_1_3:
        break;
    }
  }
}

// _12DA:
inline void either_ignition_coil_started_charging_sub() {
  // _12DA:
  byte Ram4EUpdateVal = (FLASH[0x8A5B + RAM[0x3F]] & 0xF0) >> 4;
  if (!Ram4EUpdateVal)
    ++Ram4EUpdateVal;
  RAM[0x4E] = Ram4EUpdateVal;
}

// _12EA:
inline byte clrmsk_setmsk_updated_sub() {
  // _12EA:
  byte Ram4F = FLASH[0x8752];
  RAM[0x4F] = Ram4F;
  return Ram4F;
}

template <CoilE Coil, bool IgniteOrUpdateMasks>
inline void processIgnCoilForTimer0Overflow() {
  if (IGNITION_COIL_CHARGING(Coil)) {
    if constexpr (IgniteOrUpdateMasks) {
      IGNITE_COIL(Coil);
      either_ignition_coil_just_discharged();
    } else {
      //goto either_ignition_coil_is_charging; // => _1290
    }
  } else {
    if constexpr (IgniteOrUpdateMasks) {
      START_CHARGING_IGNITION_COIL(Coil);
      //goto either_ignition_coil_started_charging; // => _12C6
      //goto either_ignition_coil_started_charging_sub; // => _12DA
      //GOTO_x_THEN_12FF(either_ignition_coil_started_charging_sub);
      either_ignition_coil_started_charging_sub();
    } else {
      // _1285:
      CLRMSK &= SET_CLEAR_MASKS_PER_COIL[static_cast<byte>(Coil)];
      SETMSK &= SET_CLEAR_MASKS_PER_COIL[static_cast<byte>(Coil)];
      //goto clrmsk_setmsk_updated; // => _1293
      //goto clrmsk_setmsk_updated_sub; // => _12EA
      if (clrmsk_setmsk_updated_sub()) {
        defaultRam4E();
      }

      clearRam29Bit1();
    }
  }
}

/// \tparam IgniteOrUpdateMasks false to only update set/clr masks
///                             true to perform ignition/charging
template <bool IgniteOrUpdateMasks>
inline void processRam4DForTimer0Overflow() {
  switch (RAM[0x4D]) {
    case 0: {
      processIgnCoilForTimer0Overflow<CoilE::C_1_4, IgniteOrUpdateMasks>();
      break;
    }
    case 1: {
      processIgnCoilForTimer0Overflow<CoilE::C_2_3, IgniteOrUpdateMasks>();
      break;
    }
    default: {
      //goto flash_8752_is_nil; // => _12FD
      clearRam29Bit1();
      break;
    }
  }
}

inline void ram77EqFF() {
  // _0DFD:
  if (CHECK_BIT_AT(RAM[0x2F], 1)) {
    // _0E0D:
    // ram_2f_bit_1_set:
    if (BYTE_PAIR_GEQ(0xF980, 0xF994)) {
      // _0E31:
      // xram_f97f_geq_xram_f993:
      _0F1C();
    }
  } else {
    XRAM[0xF97F] = XRAM[0xF980] = 0;
  }
}

inline void ram77EqFE() {
  // _0E3C:
  // ram_77_eq_fe:
  if (CHECK_BIT_AT(S0CON, 0)) {
    // _0EAC:
    // receive_request_on_serial0:
    CLEAR_BIT_IN(S0CON, 0); // Clear RI0 flag (serial recieve request)

    switchRam77ToFF();
  } else /* _0E41 */ if (CHECK_BIT_AT(P3, 0)) {
    // _0E6A:
    // RxD_eq_1:
    if (BYTE_PAIR_GEQ(0xF980, 0xF982)) {
      // _0E8E:
      // xram_f97f_geq_xram_f981:
      XRAM[0xF97F] = XRAM[0xF980] = 0;

      if (CHECK_BIT_AT(RAM[0x2F], 1)) {
        // _0E9E:
        // ram_2f_1_is_set:
        SET_BIT_IN(RAM[0x2F], 0);
        init_xram_for_serial0();
        S0RELH_S0RELL = 0xFFD0;
      }

      //goto ram_2f_bit_1_not_set_4;
      _11E7();
    } else {
      switchRam77ToFF();
    }
  } else {
    // _0E44:
    if (BYTE_PAIR_GEQ(0xF980, 0xF984)) {
      // _0E68:
      // xram_f97f_geq_xram_f983:
      switchRam77ToFF();
    }
  }
}

inline void ram77EqFD() {
  // _102B:
  if (!checkSerial0TxD()) {
    // _1040:
    // serial0_txd_not_set:
    if (checkSerial0RxD()) {
      // _107C:
      // serial0_rxd_set:
      if (CHECK_BIT_AT(RAM[0x2F], 1)) {
        // _1082:
        // ram_2f_bit_2_set_2:
        SET_BIT_IN(P3, 1);
        EnableSerial0Receiver();
        nullifyXram_F9A0_F9A1_F97F_F980_switchRam77To4();
      } else {
        // _108A:
        // ram_2f_bit_1_not_set_3:
        XRAM[0xF97F] = XRAM[0xF980] = 0;
      }
    } else /* _1043: */ if (BYTE_PAIR_GEQ(0xF980, 0xF986)) {
      // _1067:
      // xram_f97f_geq_xram_f985:
      SET_BIT_IN(P3, 1); // Set TxD @ MC33199
      EnableSerial0Receiver();
      XRAM[0x7F] = XRAM[0xF980] = 0;

      switchRam77ToFC();
    }
  } else {
    // _102E:
    DisableSerial0Receiver();
    CLEAR_BIT_IN(P3, 1);
    XRAM[0xF97F] = XRAM[0xF980] = 0;
  }
}

inline void ram77EqFC() {
  // _109A:
  if (BYTE_PAIR_GEQ(0xF980, 0xF988)) {
    // _10BE:
    // xram_f97f_geq_xram_f987:
    XRAM[0xF99F]:XRAM[0xF99E] = XRAM[0xF9A3]:XRAM[0xF9A2];
    DPTR[7] = 0xFAA8;
    switchRam77To2();
  }
}

inline void ram77Eq08() {
  // _0F96:
  if (BYTE_PAIR_GEQ(0xF980, 0xF998)) {
    // _0FBA:
    // xram_f97f_geq_xram_f997:
    DisableSerial0Int();
    if (!CHECK_BIT_AT(RAM[0x2F], 2)) {
      // _0FC3:
      // ram_2f_bit_2_not_set:
      _1192();
    } else {
      // _0FC0: goto ram_2f_bit_2_set; // => _11B3
      _11B3();
    }
  }
}

inline void ram77Eq07() {
  // _0F60:
  if (BYTE_PAIR_GEQ(0xF980, 0xF996)) {
    // _0F84:
    // xram_f97f_geq_xram_f995:
    {
      Reg::Mask<Reg::S0CON> M;
      M.add<Reg::S0CON::RI0>();
    }
    {
      Reg::Set<Reg::S0CON> S;
      S.add<Reg::S0CON::REN0>();
    }
    EnableSerial0Int();
    RAM[0x77] = 8;
  }
}

inline void ram77Eq06() {
  // _0EF2:
  if (BYTE_PAIR_GEQ(0xF980, 0xF994)) {
    // _0F19:
    // xram_f97f_geq_xram_f993_2:
    if (!CHECK_BIT_AT(RAM[0x2F], 1)) {
      // _0F19:
      // ram_2f_bit_1_not_set:
      if (CHECK_BIT_AT(RAM[0x2F], 0)) {
        // _0F4F:
        // ram_2f_bit_0_set:
        DisableSerial0Int(); // Disable Serial0 Interrupt
        if (CHECK_BIT_AT(RAM[0x2F], 4) ||
            CHECK_BIT_AT(RAM[0x2F], 5)) {
          nullifyXramF97F_F980_switchRam77To4();
        } else {
          // _0F58:
          CLEAR_BIT_IN(RAM[0x2F], 2);

          _1192();
        }
      } else {
        // _0F3C:
        XRAM[0xF97F] = XRAM[0xF980] = 0;
        EnableSerial0Int(); // Enable Serial0 Interrupt
        RAM[0x77] = 0xFF;
      }
    } else {
      _0F1C();
    }
  }
}

inline void ram77Eq05() {
  // _0EC5:
  if (BYTE_PAIR_GEQ(0xF980, 0xF991)) {
    // _0EE9:
    // xram_f97f_geq_xram_f991:
    CLEAR_BIT_IN(S0CON, 0); // Clear Serial0 Receieve request
    SET_BIT_IN(S0CON, 4); // Enable serial0 reception
    //goto ram_2f_bit_1_not_set_4; // => _11E7
    _11E7();
  }
}

inline void ram77Eq04() {
  // _0FCE:
  // ram_77_eq_4:
  if (!CHECK_BIT_AT(Reg::S0CON, Reg::S0CON::RI0)) {
    // _0FE3:
    // no_serial0_receive_request:
    if (BYTE_PAIR_GEQ(0xF980, 0xF990)) {
      // _1007:
      // xram_f97f_geq_xram_f98f:
      XRAM[0xF97F] = XRAM[0xF980] = 0;
      if (!CHECK_BIT_AT(RAM[0x2F], 1)) {
        // _101F:
        // ram_2f_bit_1_not_set_2:
        if (!CHECK_BIT_AT(RAM[0x2F], 0)) {
          // _1022:
          _11B3();
        }
      } else {
        // _1014:
        CLEAR_BIT_IN(RAM[0x2F], 0);
        if (CHECK_BIT_AT(RAM[0x2F], 0)) {
          // _101C:
          // ram_2f_bit_0_set_2:
        } else {
          // _1019:
        }
      }
    }
  } else {
    // _0FD3:
    {
      Mask<Reg::S0CON> M;
      M.add<Reg::S0CON::RI0>();
    }
    XRAM[0xF97F] = XRAM[0xF980] = 0;
  }
}

inline void ram77Eq03() {
  // _10DF:
  if (Reg::Bit<Reg::S0CON, Reg::S0CON::RI0>().get()) {
    // _10E7:
    // no_serial0_receive_request_2:
    if (BYTE_PAIR_GEQ(0xF980, 0xF98E)) {
      // _110B:
      // xram_f97f_geq_xram_f98d:
      XRAM[0xF97F] = XRAM[0xF980] = 0;
      if (XRAM[0xF987] || XRAM[0xF988]) {
        // _1124:
        // xram_f987_or_xram_f988_not_null:
        if (CHECK_BIT_AT(RAM[0x2F], 1)) {
          // _112A:
          // ram_2f_bit_1_set_2:
          RAM[0x77] = 0xFD;
        } else {
          // _1127:
          switchRam77ToFC();
        }
      } else {
        // _124A:
        // xram_f987_and_xram_f988_eq_0:
        switchRam77To2();
      }
    }
  } else {
    // _10E4:
    _11F0();
  }
}

inline void ram77Eq02() {
  // _1133:
  if (BYTE_PAIR_GEQ(0xF980, 0xF98C)) {
    // _1157:
    // _xram_f97f_geq_xram_f98b:
    {
      Mask<Reg::IEN0> M;
      M.add<Reg::IEN0::ES0>();
    }

    _11F3();
  }
}

inline void ram77Eq01() {
  // _1160:
  if (Reg::Bit<Reg::S0CON, Reg::S0CON::RI0>().get()) {
    // _1168:
    // no_serial0_receive_request_3:
    if (BYTE_PAIR_GEQ(0xF980, 0xF98A)) {
      // _118C:
      // xram_f97f_geq_xram_f989:
      switchRam77To2();
    }
  } else {
    // _1165:
    _11F0();
  }
}

inline void processRam77ForTimer0OverflowAndSelectNewRam77Value() {
  byte Ram77 = RAM[0x77];
  // _0DF8:
  // no_overflow_on_inc_xram_f97f_or_f980:
  switch (Ram77) {
    case 0xFF: {
        ram77EqFF();
        break;
      }
    case 0xFE: {
        ram77EqFE();
        break;
      }
    case 0xFD: {
        ram77EqFD();
        break;
      }
    case 0xFC: {
        ram77EqFC();
        break;
      }
    case 0x08: {
        ram77Eq08();
        break;
      }
    case 0x07: {
        ram77Eq07();
        break;
      }
    case 0x06: {
        ram77Eq06();
        break;
      }
    case 0x05: {
        ram77Eq05();
        break;
      }
    case 0x04: {
        ram77Eq04();
        break;
      }
    case 0x03: {
        ram77Eq03();
        break;
      }
    case 0x02: {
        ram77Eq02();
        break;
      }
    case 0x01: {
        ram77Eq01();
        break;
      }
    default:
      break;
  }
}

// _0DB6:
void Timer0OverflowInterrupt() {
  StopTimer0();

  //S0RELH_S0RELL is S0RELH:S0RELL

  // TH_TL0 is TH0:TL0
  {
    static const word ToAdd = 0xFAD0;
    if (0xFFFF - TH_TL0 < ToAdd)
      // 0DD0:
      TH_TL0 = 0xFACB;
    else
      TH_TL0 += ToAdd;
  }

  // 0DD6:
  // dont_reload_timer0:
  StartTimer0();

  if (--RAM[0x35] == 0) {
    RAM[0x35] = 0x14;
    SET_BIT_IN(RAM[0x28], 0);
  }

  // 0DE0:
  if (!(++XRAM[0xF97F]) && !(++XRAM[0xF980])) {
    // __0DEE:
    XRAM[0xF97F] = 0xFF;
    XRAM[0xF980] = 0xFF;
  }

  // TODO Decompile and re-organize
  processRam77ForTimer0OverflowAndSelectNewRam77Value();

  // TODO Decompile and re-organize
  // _1263:
  if (CHECK_BIT_AT(RAM[0x29], 1)) {
    // _1269:
    // ram_29_bit_1_set:
    if (CHECK_BIT_AT(RAM[0x2B], 7)) {
      // _126F:
      // ram_2b_bit_7_set:
      processRam4DForTimer0Overflow<false>();
    } else {
      // _12FD:
      // flash_8752_is_nil:
      clearRam29Bit1();
    }
  } else /* _1299: */ if (RAM[0x4F]) {
    // 12A0:
    // ram_4f_neq_nil:
    if (--RAM[0x4E]) {
      // _1296:
      // ram_4e_neq_0:
      // INTENTIONALY EMPTY
    } else {
      // RAM[0x4E] became 0
      // _12A3:
      processRam4DForTimer0Overflow<true>();
    }
  } else {
    // _129D:
    // INTENTIONALY EMPTY
  }

  finishTimer0OverflowInterrupt();
}


