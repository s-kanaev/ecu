#pragma once

#include "types.hpp"
#include "ram.hpp"
#include "mem-seg-helpers.hpp"
//#include "mask_set.hpp"
#include "binary_ops.hpp"

#include <cassert>

namespace Reg {
  namespace detail {
    template <typename _Reg, typename _Reg::BitEnumT _Bit>
    struct BitVal;
  } // namespace detail

  template <word _Loc, typename _Type, bool _Mapped>
  struct Register;
} // namespace Reg

///////////////////////////////////////////////////////////////////////////////

#define DEFINE_REGISTER(name, address, type, init, mapped, ...)                \
namespace Reg {                                                                \
  template <>                                                                  \
  struct Register<address, type, mapped> {                                     \
    using Type = type;                                                         \
    static constexpr word Loc = address;                                       \
    static constexpr bool Mapped = mapped;                                     \
    static constexpr Type Init = init;                                         \
    enum BitEnumT : Type {                                                     \
      __VA_ARGS__                                                              \
    };                                                                         \
    static Type &Inst;                                                         \
    template <BitEnumT _Bit>                                                   \
    static constexpr byte bitN() {                                             \
      return (byte)(_Bit);                                                     \
    }                                                                          \
  };                                                                           \
  using name = Register<address, type, mapped>;                                \
}

#define DEFINE_REG_BIT(reg, bit)                                               \
namespace Reg { namespace detail {                                             \
  template <>                                                                  \
  struct BitVal<reg, reg::bit> {                                               \
    static constexpr typename reg::Type value = 1 << (reg::Type)(reg::bit);    \
  };                                                                           \
}}

#define DEFINE_REGISTER8(name, addr, init, b7, b6, b5, b4, b3, b2, b1, b0)     \
DEFINE_REGISTER(name, addr, byte, init, false, b7, b6, b5, b4, b3, b2, b1, b0);\
  DEFINE_REG_BIT(name, b7);                                                    \
  DEFINE_REG_BIT(name, b6);                                                    \
  DEFINE_REG_BIT(name, b5);                                                    \
  DEFINE_REG_BIT(name, b4);                                                    \
  DEFINE_REG_BIT(name, b3);                                                    \
  DEFINE_REG_BIT(name, b2);                                                    \
  DEFINE_REG_BIT(name, b1);                                                    \
  DEFINE_REG_BIT(name, b0);

#define DEFINE_REGISTER8_NB(name, addr, init)                                  \
DEFINE_REGISTER(name, addr, byte, init, false, b7, b6, b5, b4, b3, b2, b1, b0);\
  DEFINE_REG_BIT(name, b7);                                                    \
  DEFINE_REG_BIT(name, b6);                                                    \
  DEFINE_REG_BIT(name, b5);                                                    \
  DEFINE_REG_BIT(name, b4);                                                    \
  DEFINE_REG_BIT(name, b3);                                                    \
  DEFINE_REG_BIT(name, b2);                                                    \
  DEFINE_REG_BIT(name, b1);                                                    \
  DEFINE_REG_BIT(name, b0);

#define DEFINE_REGISTER8_MAP(name, addr, init, b7, b6, b5, b4, b3, b2, b1, b0) \
DEFINE_REGISTER(name, addr, byte, init, true, b7, b6, b5, b4, b3, b2, b1, b0); \
  DEFINE_REG_BIT(name, b7);                                                    \
  DEFINE_REG_BIT(name, b6);                                                    \
  DEFINE_REG_BIT(name, b5);                                                    \
  DEFINE_REG_BIT(name, b4);                                                    \
  DEFINE_REG_BIT(name, b3);                                                    \
  DEFINE_REG_BIT(name, b2);                                                    \
  DEFINE_REG_BIT(name, b1);                                                    \
  DEFINE_REG_BIT(name, b0);

#define DEFINE_REGISTER16_NB(name, addr, init)                                 \
DEFINE_REGISTER(name, addr, word, init, false, bf, be, bd, bc, bb, ba, b9, b8, \
                b7, b6, b5, b4, b3, b2, b1, b0);\
  DEFINE_REG_BIT(name, bf);                                                    \
  DEFINE_REG_BIT(name, be);                                                    \
  DEFINE_REG_BIT(name, bd);                                                    \
  DEFINE_REG_BIT(name, bc);                                                    \
  DEFINE_REG_BIT(name, bb);                                                    \
  DEFINE_REG_BIT(name, ba);                                                    \
  DEFINE_REG_BIT(name, b9);                                                    \
  DEFINE_REG_BIT(name, b8);                                                    \
  DEFINE_REG_BIT(name, b7);                                                    \
  DEFINE_REG_BIT(name, b6);                                                    \
  DEFINE_REG_BIT(name, b5);                                                    \
  DEFINE_REG_BIT(name, b4);                                                    \
  DEFINE_REG_BIT(name, b3);                                                    \
  DEFINE_REG_BIT(name, b2);                                                    \
  DEFINE_REG_BIT(name, b1);                                                    \
  DEFINE_REG_BIT(name, b0);

DEFINE_REGISTER8(SYSCON, 0xB1, 0, CLKP, PMOD, b5, RMAP, b3, b2, XMAP1, XMAP0);
DEFINE_REGISTER8(IP1, 0xB9, 0, PDIR, b6, b5, b4, b3, b2, b1, b0);
DEFINE_REGISTER8(IEN0, 0xA8, 0, EAL, WDT, ET2, ES0, ET1, EX1, ET0, EX0);
DEFINE_REGISTER8(IEN1, 0xB8, 0, EXEN2, SWDT, EX6, EX5, EX4, EX3, EX2, EADC);
DEFINE_REGISTER8(IEN2, 0x9A, 0, b7, b6, ECR, ECS, ECT, ECMP, b1, ES1);
DEFINE_REGISTER8(S0CON, 0x98, 0, SM0, SM1, SM20, REN0, TB80, RB80, TI0, RI0);
DEFINE_REGISTER8(TCON, 0x88, 0, TF1, TR1, TF0, TR0, IE1, IT1, IE0, IT0);

DEFINE_REGISTER8_NB(DPL, 0x82, 0);
DEFINE_REGISTER8_NB(DPH, 0x83, 0);
DEFINE_REGISTER16_NB(DPTR, 0x82, 0);

DEFINE_REGISTER8_NB(DPSEL, 0x92, 0);
DEFINE_REGISTER8(PSW, 0xD0, 0, CY, AC, F0, RS1, RS0, OV, F1, P);

DEFINE_REGISTER8_NB(CCH3, 0xC7, 0);
DEFINE_REGISTER8_NB(CCL3, 0xC6, 0);
DEFINE_REGISTER16_NB(CC3, 0xC6, 0);

DEFINE_REGISTER8_NB(CCH4, 0xCF, 0);
DEFINE_REGISTER8_NB(CCL4, 0xCE, 0);
DEFINE_REGISTER16_NB(CC4, 0xCE, 0);

DEFINE_REGISTER8(IRCON0, 0xC0, 0, EXF2, TF2, IEX6, IEX5,
                                  IEX4, IEX3, IEX2, IADC);

DEFINE_REGISTER8(CCEN, 0xC1, 0, COCAH3, COCAL3, COCAH2, COCAL2,
                                COCAH1, COCAL1, COCAH0, COCAL0);

DEFINE_REGISTER8_NB(COMCLRL, 0xA3, 0);
DEFINE_REGISTER8_NB(COMCLRH, 0xA4, 0);
DEFINE_REGISTER16_NB(COMCLR, 0xA3, 0);

DEFINE_REGISTER8_NB(SETMSK, 0xA5, 0);
DEFINE_REGISTER8_NB(CLRMSK, 0xA6, 0);

DEFINE_REGISTER8(CTCON, 0xE1, 0x40, T2PS1, CTP, ICR, ICS,
                                    CTF, CLK2, CLK1, CLK0);
DEFINE_REGISTER8(CT1CON, 0xE2, 0x40, b7, CT1P, b5, b4,
                                     CT1F, CLK12, CLK11, CLK10);

// DPTR
namespace Reg {
// TODO rework this class to use RAM position of DPTR and DPSEL
class DPTR_t {
  static constexpr unsigned int COUNT = 8;
  static word MValue[COUNT];// = {0};

public:
  DPTR_t(const DPTR_t &) = delete;
  DPTR_t(DPTR_t &&) = delete;

  DPTR_t &operator=(const DPTR_t &) = delete;
  DPTR_t &operator=(DPTR_t &&) = delete;

  explicit DPTR_t() {}

  // Change DPSEL
  static void select(unsigned int Selector) {
    assert(Selector < COUNT);
    DPSEL::Inst = Selector;
  }

  static word &get() {
    return DPTR::Inst;
  }
};
} // namespace Reg

using DPTR_t = Reg::DPTR_t;

extern DPTR_t DPTR;

// GPRs
namespace Reg {
namespace impl {
template <int _Index>
struct GPR {
  static constexpr int Index = _Index;
  static_assert(Index >= 0 && Index < 8, "Index should be equal to [0..7]");

  static byte &get() {
    return RAM[(PSW::Inst & 0x18) + Index];
  }
};

template <int _RS, int _Index>
struct GPRSel {
  static constexpr int Index = _Index;
  static constexpr int RS = _RS;
  static_assert(Index >= 0 && Index < 8, "Index should be in range [0..7]");
  static_assert(RS >= 0 && RS < 4, "Register bank selector should be in range [0..3]");

  static byte &get() {
    return RAM[(RS << 0x03) + Index];
  }
};

template <int _BaseIndex>
struct GPRWord {
  static constexpr int BaseIndex = _BaseIndex;
  static_assert(BaseIndex % 2 == 0, "BaseIndex must be even");
  using LowByte = GPR<BaseIndex>;
  using HighByte = GPR<BaseIndex + 1>;

  static word get() {
    return COMPOSE_WORD(HighByte::get(), LowByte::get());
  }

  static void set(word W) {
    HighByte::get() = HIGH(W);
    LowByte::get() = LOW(W);
  }
};

template <int _RS, int _BaseIndex>
struct GPRWordSel {
  static constexpr int BaseIndex = _BaseIndex;
  static constexpr int RS = _RS;
  static_assert(BaseIndex % 2 == 0, "BaseIndex must be even");
  using LowByte = GPRSel<RS, BaseIndex>;
  using HighByte = GPRSel<RS, BaseIndex + 1>;

  static word get() {
    return COMPOSE_WORD(HighByte::get(), LowByte::get());
  }

  static void set(word W) {
    HighByte::get() = HIGH(W);
    LowByte::get() = LOW(W);
  }
};
} // namespace impl

template <int _RS, int _Index>
struct GPRSel;

template <int _Index>
struct GPR {
  static constexpr int Index = _Index;
  using Impl = impl::GPR<Index>;
  using This = GPR<Index>;

  operator byte &() {
    return Impl::get();
  }

  This &operator=(byte V) {
    Impl::get() = V;
    return *this;
  }

  template <int OtherIndex>
  This &operator=(GPR<OtherIndex> &V) {
    Impl::get() = V;
    return *this;
  }

  template <int RS, int OtherIndex>
  This &operator=(GPRSel<RS, OtherIndex> &V) {
    Impl::get() = V;
    return *this;
  }

  This &operator+=(byte V) {
    Impl::get() = Impl::get() + V;
    return *this;
  }
};

template <int _RS, int _Index>
struct GPRSel {
  static constexpr int RS = _RS;
  static constexpr int Index = _Index;
  using Impl = impl::GPRSel<RS, Index>;
  using This = GPRSel<RS, Index>;

  operator byte &() {
    return Impl::get();
  }

  This &operator=(byte V) {
    Impl::get() = V;
    return *this;
  }

  This &operator+=(byte V) {
    Impl::get() = Impl::get() + V;
    return *this;
  }
};

template <int _RS, int _BaseIndex>
struct GPRWordSel;

template <int _BaseIndex>
struct GPRWord {
  static constexpr int BaseIndex = _BaseIndex;
  using Impl = impl::GPRWord<BaseIndex>;
  using This = GPRWord<BaseIndex>;

  operator word() {
    return Impl::get();
  }

  This &operator=(word V) {
    Impl::set(V);
    return *this;
  }

  This &operator+=(word V) {
    Impl::set(Impl::get() + V);
    return *this;
  }
};

template <int _RS, int _BaseIndex>
struct GPRWordSel {
  static constexpr int RS = _RS;
  static constexpr int BaseIndex = _BaseIndex;
  using Impl = impl::GPRWordSel<RS, BaseIndex>;
  using This = GPRWordSel<RS, BaseIndex>;

  operator word() {
    return Impl::get();
  }

  This &operator=(word V) {
    Impl::set(V);
    return *this;
  }

  This &operator+=(word V) {
    Impl::set(Impl::get() + V);
    return *this;
  }
};


using R0 = GPR<0>;
using R1 = GPR<1>;
using R2 = GPR<2>;
using R3 = GPR<3>;
using R4 = GPR<4>;
using R5 = GPR<5>;
using R6 = GPR<6>;
using R7 = GPR<7>;

template <int _RS>
using R0Sel = GPRSel<_RS, 0>;
template <int _RS>
using R1Sel = GPRSel<_RS, 1>;
template <int _RS>
using R2Sel = GPRSel<_RS, 2>;
template <int _RS>
using R3Sel = GPRSel<_RS, 3>;
template <int _RS>
using R4Sel = GPRSel<_RS, 4>;
template <int _RS>
using R5Sel = GPRSel<_RS, 5>;
template <int _RS>
using R6Sel = GPRSel<_RS, 6>;
template <int _RS>
using R7Sel = GPRSel<_RS, 7>;

using R1_R0 = GPRWord<0>;
using R3_R2 = GPRWord<2>;
using R5_R4 = GPRWord<4>;
using R7_R6 = GPRWord<6>;

template <int _RS>
using R1_R0Sel = GPRWordSel<_RS, 0>;
template <int _RS>
using R3_R2Sel = GPRWordSel<_RS, 2>;
template <int _RS>
using R5_R4Sel = GPRWordSel<_RS, 4>;
template <int _RS>
using R7_R6Sel = GPRWordSel<_RS, 6>;
} // namespace Reg

// ports
extern byte P1;
extern byte P2;
extern byte P3;
extern byte P4;
extern byte P5;
extern byte P6;
extern byte P7;
extern byte P8;
extern byte P9;
