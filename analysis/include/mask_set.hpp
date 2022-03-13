#pragma once

#include <types.hpp>
#include <ram.hpp>
#include <registers.hpp>

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

      if constexpr (_MaskOrSet) {
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
