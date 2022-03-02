#pragma once

#include <types.hpp>
#include <ram.hpp>

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
    enum BitEnumT : byte {                                                     \
      __VA_ARGS__                                                              \
    };                                                                         \
    static constexpr Type &Inst = RAM[address];                                \
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
    static constexpr typename reg::Type value = 1 << (byte)(reg::bit);         \
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

DEFINE_REGISTER8(SYSCON, 0xB1, 0, CLKP, PMOD, b5, RMAP, b3, b2, XMAP1, XMAP0);
DEFINE_REGISTER8(IP1, 0xB9, 0, PDIR, b6, b5, b4, b3, b2, b1, b0);
DEFINE_REGISTER8(IEN0, 0xA8, 0, EAL, WDT, ET2, ES0, ET1, EX1, ET0, EX0);
DEFINE_REGISTER8(S0CON, 0x98, 0, SM0, SM1, SM20, REN0, TB80, RB80, TI0, RI0);

///////////////////////////////////////////////////////////////////////////////

namespace Reg {
  template <typename _Reg>
  class Bits {
  public:
    using RegT = _Reg;
    using ValT = typename RegT::Type;
    using BitEnumT = typename RegT::BitEnumT;
    static constexpr size_t Amount = (1 << sizeof(ValT));

  private:
    ValT MMask;

  public:
    Bits() : MMask{0} {}

    Bits(const Bits &) = delete;
    Bits(Bits &&) = delete;

    Bits &operator=(const Bits &) = delete;
    Bits &operator=(Bits &&) = delete;

    template <BitEnumT Bit>
    Bits &OR() {
      MMask |= detail::BitVal<RegT, Bit>::value;
      return *this;
    }

    ValT get() const {
      return RegT::Inst & MMask;
    }
  };

  template <typename _Reg, typename _Reg::BitEnumT _Bit>
  class Bit {
  public:
    using RegT = _Reg;
    using BitsT = Bits<RegT>;
    using ValT = typename BitsT::ValT;
    using BitEnumT = typename RegT::BitEnumT;

  private:
    Bits<RegT> MB;

  public:
    Bit() : MB() { MB.template OR<_Bit>(); }

    Bit(const Bit &) = delete;
    Bit(Bit &&) = delete;

    Bit &operator=(const Bit &) = delete;
    Bit &operator=(Bit &&) = delete;

    template <BitEnumT Bit>
    Bits<RegT> &OR() {
      return MB.template OR<Bit>();
    }

    ValT get() const {
      return MB.get();
    }
  };

  template <typename _Reg, bool _MaskOrSet>
  class MaskSet {
  public:
    using RegT = _Reg;
    using ValT = typename RegT::Type;
    using BitEnumT = typename RegT::BitEnumT;

    MaskSet() = default;

    MaskSet(const MaskSet &) = delete;
    MaskSet(MaskSet &&) = delete;

    MaskSet &operator=(const MaskSet &) = delete;
    MaskSet &operator=(MaskSet &&) = delete;

    ~MaskSet() {
      if constexpr (RegT::Mapped) {
        SYSCON::Inst |= detail::BitVal<SYSCON, SYSCON::BitEnumT::RMAP>::value;
      }

      if (_MaskOrSet) {
        RegT::Inst &= ~(MBitSet.get());
      } else {
        RegT::Inst |= MBitSet.get();
      }
    }

  private:
    template <BitEnumT FirstBit, BitEnumT ...OtherBits>
    struct Helper {
      static const void add(Bits<RegT> &BitSet) {
        BitSet.template OR<FirstBit>();
        Helper<OtherBits...>::add(BitSet);
      }
    };

    template <BitEnumT Bit>
    struct Helper<Bit> {
      static const void add(Bits<RegT> &BitSet) {
        BitSet.template OR<Bit>();
      }
    };

  public:
    template <BitEnumT ...Bits>
    const void add() {
      Helper<Bits...>::add(MBitSet);
    }

  private:
    Bits<RegT> MBitSet;
  };

  template <typename _Reg>
  using Mask = MaskSet<_Reg, true>;

  template <typename _Reg>
  using Set = MaskSet<_Reg, false>;

} // namespace Reg
