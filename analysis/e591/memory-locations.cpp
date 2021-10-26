#include "memory-locations.hpp"
#include <types.hpp>

#include <map>
#include <sstream>

namespace seg {
  byte *XRAM::get() { return ::XRAM; }
  byte *FLASH::get() { return ::FLASH; }
  byte *RAM::get() { return ::RAM; }

#if __E591_HOST_COMPILATION
# define SEG_NAME(x) const char *x::name() { return #x; }
  SEG_NAME(XRAM);
  SEG_NAME(FLASH);
  SEG_NAME(RAM);
# undef SEG_NAME
#endif // __E591_HOST_COMPILATION
} // namespace seg

namespace location {
#if __E591_HOST_COMPILATION
  struct Registrar {
    struct Range {
      size_t Start = 0;
      size_t End = 0;
      std::string N = "N / A";
    };

    using Ranges = std::vector<Range>;
    static std::map<::byte *, Ranges> RangesPerSegment;

    // Should verify that no range instersection takes place
    template <typename Segment, typename Tag>
    static void range(int Start, int End) {
      rangeImpl<Segment, Tag>(Start, End);
    }

    template <typename Segment, typename Tag>
    static void byte(int Addr) {
      rangeImpl<Segment, Tag>(Addr, Addr);
    }

    template <typename Segment, typename Tag>
    static void word(int Addr) {
      rangeImpl<Segment, Tag>(Addr, Addr + 1);
    }

    template <typename Segment>
    static bool checkRegisteredAccess(std::size_t Idx) {
      Ranges &Rs = RangesPerSegment[Segment::get()];

      for (const Range &R : Rs)
        if ((R.Start <= Idx) && (Idx <= R.End))
          return true;

      return false;
    }

    static void dump(std::stringstream &SS) {
#define MAP_EL(x) { seg::x::get(), seg::x::name() }
      static const std::map<::byte *, const char *> SegmentMap = {
        MAP_EL(XRAM),
        MAP_EL(FLASH),
        MAP_EL(RAM)
      };
#undef MAP_EL

      std::hex(SS);

      SS << "==============================================================\n";
      SS << "Segments reference:\n";
      SS << "  FLASH: " << reinterpret_cast<void *>(seg::FLASH::get()) << "\n";
      SS << "  XRAM: " << reinterpret_cast<void *>(seg::XRAM::get()) << "\n";
      SS << "  RAM: " << reinterpret_cast<void *>(seg::RAM::get()) << "\n";
      SS << "\n";
      SS << "Registrar dump start\n";
      for (const auto &SegRanges : RangesPerSegment) {
        SS << "  Seg [" << SegmentMap.at(SegRanges.first) << "]:\n";

        for (const auto &R : SegRanges.second)
          SS << "    [" << R.N << "]: " << R.Start << " .. " << R.End
             << ", Length: " << (R.End - R.Start) << "\n";
      }
      SS << "Registrar dump end\n";
      SS << "==============================================================\n";
    }

  private:
    // Returns true if range intersection found
    static bool range(::byte *SegmentStart,
                      size_t Start, size_t End,
                      Range &OtherRange) {
      Ranges &Rs = RangesPerSegment[SegmentStart];

      for (const Range &R : Rs) {
        if (!(R.Start > End || Start > R.End)) {
          OtherRange = R;
          return true;
        }
      }

      return false;
    }

    template <typename Segment, typename Tag>
    static void rangeImpl(size_t Start, size_t End) {
      Range OtherRange;
      if (range(Segment::get(), Start, End, OtherRange)) {
        fprintf(stderr, "Intersection for ranges in segment %s: "
                "Attempt to add range %s [%zu .. %zu] intersecting with %s "
                "[%zu .. %zu]\n",
                Segment::name(),
                typeid(Tag).name(), Start, End,
                OtherRange.N.c_str(), OtherRange.Start, OtherRange.End);
        abort();
      }

      Range R{Start, End, typeid(Tag).name()};
      RangesPerSegment[Segment::get()].push_back(R);
    }
  };

  std::map<::byte *, Registrar::Ranges> Registrar::RangesPerSegment;

  void dumpRegisteredMemoryLocations (std::stringstream& SS) {
    Registrar::dump(SS);
  }

#endif  // __E591_HOST_COMPILATION
} // namespace location

namespace seg {
  template <typename _Seg>
  struct Segment {
    using SelfSeg = _Seg;

    static byte &get(std::size_t Idx) {
#if __E591_HOST_COMPILATION
      if (!::location::Registrar::checkRegisteredAccess<SelfSeg>(Idx)) {
        fprintf(stderr, "Attempt to access unregistered range with Idx = %zu"
                " in segment %s\n",
                Idx,
                SelfSeg::name());
      }
#endif

      return SelfSeg::get()[Idx];
    }
  };
} // namespace seg


////////////////////////////////////////////////////////////////////////////////

#if __E591_HOST_COMPILATION
# define MEM_LOC_CTOR(type, rtype, segment, tag, ...)         \
  type<segment, tag>::type() {  \
    Registrar::rtype<Segment, Tag>(__VA_ARGS__);              \
  }
# define REGISTER_MEM_LOC(type, segment, tag)                 \
  location::type<seg::segment, location::tags::MEM_TAG(tag)>  \
      segment ## _ ## tag;
#else // __E591_HOST_COMPILATION
# define MEM_LOC_CTOR(type, rtype, segment, tag, ...)
# define REGISTER_MEM_LOC(type, segment, tag)
#endif

#define DEFINE_MEMORY_RANGE_METHODS(segment, tag)                       \
namespace location {                                                    \
  MEM_LOC_CTOR(Range, range, seg::segment,                              \
               location::tags::MEM_TAG(tag), Start, End)                \
  byte *Range<seg::segment, location::tags::MEM_TAG(tag)>::getStart() { \
    return &seg::Segment<Segment>::get(Start);                          \
  }                                                                     \
  byte *Range<seg::segment, location::tags::MEM_TAG(tag)>::begin() {    \
    return &seg::Segment<Segment>::get(Start);                          \
  }                                                                     \
  byte *Range<seg::segment, location::tags::MEM_TAG(tag)>::getEnd() {   \
    return &seg::Segment<Segment>::get(End);                            \
  }                                                                     \
  byte *Range<seg::segment, location::tags::MEM_TAG(tag)>::end() {      \
    return (&seg::Segment<Segment>::get(End)) + 1;                      \
  }                                                                     \
}                                                                       \
REGISTER_MEM_LOC(Range, segment, tag);

#define DEFINE_MEMORY_BYTE_METHODS(segment, tag)                              \
namespace location {                                                          \
  MEM_LOC_CTOR(Byte, byte, seg::segment,                                      \
               location::tags::MEM_TAG(tag), Offset)                          \
                                                                              \
  byte *Byte<seg::segment, location::tags::MEM_TAG(tag)>::getStart() {        \
    return &seg::Segment<Segment>::get(Offset);                               \
  }                                                                           \
                                                                              \
  byte *Byte<seg::segment, location::tags::MEM_TAG(tag)>::begin() {           \
    return &seg::Segment<Segment>::get(Offset);                               \
  }                                                                           \
                                                                              \
  byte *Byte<seg::segment, location::tags::MEM_TAG(tag)>::getEnd() {          \
    return &seg::Segment<Segment>::get(Offset);                               \
  }                                                                           \
                                                                              \
  byte *Byte<seg::segment, location::tags::MEM_TAG(tag)>::end() {             \
    return (&seg::Segment<Segment>::get(Offset)) + 1;                         \
  }                                                                           \
                                                                              \
  byte Byte<seg::segment, location::tags::MEM_TAG(tag)>::get() {              \
    return seg::Segment<Segment>::get(Offset);                                \
  }                                                                           \
                                                                              \
  void Byte<seg::segment, location::tags::MEM_TAG(tag)>::set(byte Value) {    \
    seg::Segment<Segment>::get(Offset) = Value;                               \
  }                                                                           \
}                                                                             \
REGISTER_MEM_LOC(Byte, segment, tag);

#define DEFINE_MEMORY_WORD_METHODS(segment, tag)                            \
namespace location {                                                        \
  MEM_LOC_CTOR(Word, word, seg::segment,                                    \
               location::tags::MEM_TAG(tag), Offset);                       \
                                                                            \
  word *Word<seg::segment, location::tags::MEM_TAG(tag)>::getStart() {      \
    return WORD_PTR(&seg::Segment<Segment>::get(Offset));                   \
  }                                                                         \
                                                                            \
  word *Word<seg::segment, location::tags::MEM_TAG(tag)>::begin() {         \
    return WORD_PTR(&seg::Segment<Segment>::get(Offset));                   \
  }                                                                         \
                                                                            \
  word *Word<seg::segment, location::tags::MEM_TAG(tag)>::getEnd() {        \
    return WORD_PTR(&seg::Segment<Segment>::get(Offset + 1));               \
  }                                                                         \
                                                                            \
  word *Word<seg::segment, location::tags::MEM_TAG(tag)>::end() {           \
    return WORD_PTR((&seg::Segment<Segment>::get(Offset)) + Size);          \
  }                                                                         \
                                                                            \
  word Word<seg::segment, location::tags::MEM_TAG(tag)>::get() {            \
    return COMPOSE_WORD(seg::Segment<Segment>::get(Offset + 1),             \
                        seg::Segment<Segment>::get(Offset));                \
  }                                                                         \
                                                                            \
  void Word<seg::segment, location::tags::MEM_TAG(tag)>::set(word Value) {  \
    seg::Segment<Segment>::get(Offset) = LOW(Value);                        \
    seg::Segment<Segment>::get(Offset + 1) = HIGH(Value);                   \
  }                                                                         \
}                                                                           \
REGISTER_MEM_LOC(Word, segment, tag);

////////////////////////////////////////////////////////////////////////////////
// XRAM
DEFINE_MEMORY_RANGE_METHODS(XRAM, SUM_OF_EGO_CALIBRATION);
DEFINE_MEMORY_RANGE_METHODS(XRAM, EGO_CALIBRATION);

DEFINE_MEMORY_WORD_METHODS(XRAM, CHECKSUM);

DEFINE_MEMORY_BYTE_METHODS(XRAM, ADJUSTED_CO_POT);

DEFINE_MEMORY_BYTE_METHODS(XRAM, ADJUSTED_COOLANT_TEMP);

DEFINE_MEMORY_BYTE_METHODS(XRAM, ADC_THROTTLE_POSITION);
DEFINE_MEMORY_BYTE_METHODS(XRAM, ADC_COOLANT_TEMP);
DEFINE_MEMORY_BYTE_METHODS(XRAM, ADC_INTAKE_AIR_TEMP);
DEFINE_MEMORY_BYTE_METHODS(XRAM, ADC_IGNITION_SWITCH_VOLTAGE);
DEFINE_MEMORY_BYTE_METHODS(XRAM, ADC_CO_POT);

DEFINE_MEMORY_WORD_METHODS(XRAM, IGNITION_SW_VOLTAGE_SUM);
DEFINE_MEMORY_WORD_METHODS(XRAM, COOLANT_TEMP_SUM);
DEFINE_MEMORY_WORD_METHODS(XRAM, INTAKE_AIR_SUM);
DEFINE_MEMORY_WORD_METHODS(XRAM, CO_POT_SUM);
DEFINE_MEMORY_WORD_METHODS(XRAM, THROTTLE_POSITION_SUM);
DEFINE_MEMORY_WORD_METHODS(XRAM, RAM_49_SUM);
DEFINE_MEMORY_WORD_METHODS(XRAM, RAM_49_SUM_PREV);

DEFINE_MEMORY_WORD_METHODS(XRAM, THROTTLE_POSITION_LESS_THRESHOLD);

DEFINE_MEMORY_WORD_METHODS(XRAM, THROTTLE_POSITION_THRESHOLD);

DEFINE_MEMORY_BYTE_METHODS(XRAM, THROTTLE_POSITION_BYTE_1);
DEFINE_MEMORY_BYTE_METHODS(XRAM, THROTTLE_POSITION_BYTE_2);

DEFINE_MEMORY_WORD_METHODS(XRAM, THROTTLE_POSITION_1);
DEFINE_MEMORY_WORD_METHODS(XRAM, THROTTLE_POSITION_2);

////////////////////////////////////////////////////////////////////////////////
// FLASH
DEFINE_MEMORY_RANGE_METHODS(FLASH, COOLANT_TEMPERATURE_TABLE_1);
DEFINE_MEMORY_RANGE_METHODS(FLASH, COOLANT_TEMPERATURE_TABLE_2);
DEFINE_MEMORY_RANGE_METHODS(FLASH, INTAKE_AIR_TEMPERATURE_TABLE);

DEFINE_MEMORY_BYTE_METHODS(FLASH, MINIMUM_INTAKE_AIR_TEMPERATURE);
DEFINE_MEMORY_BYTE_METHODS(FLASH, MAXIMUM_INTAKE_AIR_TEMPERATURE);

DEFINE_MEMORY_RANGE_METHODS(FLASH, KITTING);

DEFINE_MEMORY_BYTE_METHODS(FLASH, MINIMUM_IGNITION_VOLTAGE);
DEFINE_MEMORY_BYTE_METHODS(FLASH, MAXIMUM_IGNITION_VOLTAGE);

////////////////////////////////////////////////////////////////////////////////
// RAM


