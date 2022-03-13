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
    static constexpr Type &Inst = *static_cast<Type *>(&RAM[address]);         \
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
DEFINE_REGISTER8(S0CON, 0x98, 0, SM0, SM1, SM20, REN0, TB80, RB80, TI0, RI0);
DEFINE_REGISTER8(TCON, 0x88, 0, TF1, TR1, TF0, TR0, IE1, IT1, IE0, IT0);
DEFINE_REGISTER8_NB(DPL, 0x82, 0);
DEFINE_REGISTER8_NB(DPH, 0x83, 0);
DEFINE_REGISTER16_NB(DPTR, 0x82, 0);
DEFINE_REGISTER8_NB(DPSEL, 0x92, 0);


namespace Reg {
class DPTR_t {
  static constexpr unsigned int COUNT = 8;
  word MValue[COUNT] = {0};
  int MSelector = 0;

  word &get() {
    return MValue[MSelector];
  }

public:
  DPTR_t(const DPTR_t &) = delete;
  DPTR_t(DPTR_t &&) = delete;

  DPTR_t &operator=(const DPTR_t &) = delete;
  DPTR_t &operator=(DPTR_t &&) = delete;

  explicit DPTR_t() {}

  void select(unsigned int Selector) {
    assert(Selector < COUNT);
    MSelector = Selector;
  }

  word &operator[](unsigned int Selector) {
    select(Selector);
    DPL::Inst = LOW(get());
    DPH::Inst = HIGH(get());
    return get();
  }

  DPTR_t &operator=(word Val) {
    get() = Val;
    return *this;
  }

  operator word&() { return get(); }

  operator word() { return get(); }
  operator int() { return get(); }
  operator size_t() { return get(); }
};
} // namespace Reg

using DPTR_t = Reg::DPTR_t;

extern DPTR_t DPTR;

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
