#pragma once

#include "defines.hpp"
#include "types.hpp"
#include "ram.hpp"
#include "flash.hpp"
#include "xram.hpp"
#include "binary_ops.hpp"

#include <vector>
#include <map>
#include <string>
#include <typeinfo>
#include <cstdio>

// Namespace for describing memory segments
namespace seg {
  // Method get return start of memory segment

  struct XRAM {
    static byte *get();
#if __E591_HOST_COMPILATION
    static const char *name();
#endif  // __E591_HOST_COMPILATION
  };

  struct FLASH {
    static byte *get();
#if __E591_HOST_COMPILATION
    static const char *name();
#endif  // __E591_HOST_COMPILATION
  };

  struct RAM {
    static byte *get();
#if __E591_HOST_COMPILATION
    static const char *name();
#endif  // __E591_HOST_COMPILATION
  };
} // namespace seg

// namespace for helpers on describing regions in memory segments
namespace location {
#if __E591_HOST_COMPILATION
  void dumpRegisteredMemoryLocations(std::stringstream &SS);
#endif  // __E591_HOST_COMPILATION

template <typename _Segment, typename _Tag>
  struct Range;

  template <typename _Segment, typename _Tag>
  struct InvalidRange;

  template <typename _Segment, typename _Tag>
  struct Byte;

  template <typename _Segment, typename _Tag>
  struct Word;
} // namespace location

#define MEM_TAG(tag_name) tag_name ## _ ## tag

#define DEFINE_MEM_TAG(t_name)              \
namespace location { namespace tags {       \
  struct MEM_TAG(t_name) {};                \
}}

#if __E591_HOST_COMPILATION
# define DECLARE_MEM_LOC_CTOR(type) \
    type();
#else // __E591_HOST_COMPILATION
# define DECLARE_MEM_LOC_CTOR(type)
#endif // !__E591_HOST_COMPILATION

// Define range (a table, usually) in segment [wstart..end]
// (inclusive on each end) which is referred to with tag
#define DECLARE_MEMORY_RANGE(segment, tag, _start, _end)      \
DEFINE_MEM_TAG(tag);                                          \
namespace location {                                          \
  template <>                                                 \
  struct Range<seg::segment, location::tags::MEM_TAG(tag)> {  \
    using Segment = seg::segment;                             \
    using Tag = location::tags::MEM_TAG(tag);                 \
                                                              \
    static constexpr size_t Start = (_start);                 \
    static constexpr size_t End = (_end);                     \
    static constexpr size_t Length = End - Start + 1;         \
                                                              \
    DECLARE_MEM_LOC_CTOR(Range);                              \
                                                              \
    static byte *getStart();                                  \
                                                              \
    static byte *begin();                                     \
                                                              \
    static byte *getEnd();                                    \
                                                              \
    static byte *end();                                       \
  };                                                          \
}


#define RNG_START_IDX(segment, tag) \
  location::Range<seg::segment, location::tags::MEM_TAG(tag)>::Start

#define RNG_END_IDX(segment, tag) \
  location::Range<seg::segment, location::tags::MEM_TAG(tag)>::End

#define RNG_LENGTH(segment, tag) \
  location::Range<seg::segment, location::tags::MEM_TAG(tag)>::Length

#define GET_RNG_START_PTR(segment, tag) \
  location::Range<seg::segment, location::tags::MEM_TAG(tag)>::getStart()

#define GET_RNG_END_PTR(segment, tag) \
  location::Range<seg::segment, location::tags::MEM_TAG(tag)>::getEnd()

#define GET_RNG_BEGIN(segment, tag) \
  location::Range<seg::segment, location::tags::MEM_TAG(tag)>::begin()

#define GET_RNG_END(segment, tag) \
  location::Range<seg::segment, location::tags::MEM_TAG(tag)>::end()


// Define a byte in memory segment which. The byte is
// referred to with a tag
#define DECLARE_MEMORY_BYTE(segment, tag, position)         \
DEFINE_MEM_TAG(tag);                                        \
namespace location {                                        \
  template <>                                               \
  struct Byte<seg::segment, location::tags::MEM_TAG(tag)> { \
    using Segment = seg::segment;                           \
    using Tag = location::tags::MEM_TAG(tag);               \
                                                            \
    static constexpr int Offset = (position);               \
    static constexpr int Size = 1;                          \
                                                            \
    DECLARE_MEM_LOC_CTOR(Byte);                             \
                                                            \
    static byte *getStart();                                \
                                                            \
    static byte *begin();                                   \
                                                            \
    static byte *getEnd();                                  \
                                                            \
    static byte *end();                                     \
                                                            \
    static byte get();                                      \
                                                            \
    static void set(byte Value);                            \
  };                                                        \
}

#define GET_MEM_BYTE_PTR(segment, name) \
  location::Byte<seg::segment, location::tags::MEM_TAG(name)>::getStart()

#define GET_MEM_BYTE(segment, name) \
  location::Byte<seg::segment, location::tags::MEM_TAG(name)>::get()

#define SET_MEM_BYTE(segment, name, value)                    \
do {                                                          \
  location::Byte<seg::segment, location::tags::MEM_TAG(name)> \
      ::set((value));                                         \
} while (0)


// Define a word in memory segment which. The word is
// referred to with a tag. The word is stored with
// LSB endiannes i.e. least significant byte is stored
// at the lowest address and most significant byte is
// stored at the highest address.
#define DECLARE_MEMORY_WORD(segment, tag, position)         \
DEFINE_MEM_TAG(tag);                                        \
namespace location {                                        \
  template <>                                               \
  struct Word<seg::segment, location::tags::MEM_TAG(tag)> { \
    using Segment = seg::segment;                           \
    using Tag = location::tags::MEM_TAG(tag);               \
                                                            \
    static constexpr int Offset = (position);               \
    static constexpr int Size = 2;                          \
                                                            \
    DECLARE_MEM_LOC_CTOR(Word);                             \
                                                            \
    static word *getStart();                                \
                                                            \
    static word *begin();                                   \
                                                            \
    static word *getEnd();                                  \
                                                            \
    static word *end();                                     \
                                                            \
    static word get();                                      \
                                                            \
    static void set(word Value);                            \
  };                                                        \
}

#define GET_MEM_WORD_PTR(segment, name) \
  location::Word<seg::segment, location::tags::MEM_TAG(name)>::getStart()

#define GET_MEM_WORD(segment, name) \
  location::Word<seg::segment, location::tags::MEM_TAG(name)>::get()

#define SET_MEM_WORD(segment, name, value)                    \
do {                                                          \
  location::Word<seg::segment, location::tags::MEM_TAG(name)> \
      ::set((value))                                          \
} while (0)

////////////////////////////////////////////////////////////
// XRAM
DECLARE_MEMORY_RANGE(XRAM, SUM_OF_EGO_CALIBRATION, 0xF400, 0xF4FF);
DECLARE_MEMORY_RANGE(XRAM, EGO_CALIBRATION, 0xF500, 0xF5FF);

DECLARE_MEMORY_WORD(XRAM, CHECKSUM, 0xF658);

DECLARE_MEMORY_BYTE(XRAM, ADJUSTED_CO_POT, 0xF681);

DECLARE_MEMORY_BYTE(XRAM, ADJUSTED_COOLANT_TEMP, 0xF683);

DECLARE_MEMORY_BYTE(XRAM, ADC_THROTTLE_POSITION, 0xF685);
DECLARE_MEMORY_BYTE(XRAM, ADC_COOLANT_TEMP, 0xF686);
DECLARE_MEMORY_BYTE(XRAM, ADC_INTAKE_AIR_TEMP, 0xF687);
DECLARE_MEMORY_BYTE(XRAM, ADC_IGNITION_SWITCH_VOLTAGE, 0xF688);
DECLARE_MEMORY_BYTE(XRAM, ADC_CO_POT, 0xF689);

DECLARE_MEMORY_WORD(XRAM, IGNITION_SW_VOLTAGE_SUM, 0xF69B);
DECLARE_MEMORY_WORD(XRAM, COOLANT_TEMP_SUM, 0xF69E);
DECLARE_MEMORY_WORD(XRAM, INTAKE_AIR_SUM, 0xF6A0);
DECLARE_MEMORY_WORD(XRAM, CO_POT_SUM, 0xF6A4);
DECLARE_MEMORY_WORD(XRAM, THROTTLE_POSITION_SUM, 0xF6A6);
DECLARE_MEMORY_WORD(XRAM, RAM_49_SUM, 0xF6A8);
DECLARE_MEMORY_WORD(XRAM, RAM_49_SUM_PREV, 0xF6AA);

DECLARE_MEMORY_WORD(XRAM, THROTTLE_POSITION_LESS_THRESHOLD, 0xF6AD);

DECLARE_MEMORY_WORD(XRAM, THROTTLE_POSITION_THRESHOLD, 0xF6AF);

DECLARE_MEMORY_BYTE(XRAM, THROTTLE_POSITION_BYTE_1, 0xF6B3);
DECLARE_MEMORY_BYTE(XRAM, THROTTLE_POSITION_BYTE_2, 0xF6B4);

DECLARE_MEMORY_WORD(XRAM, THROTTLE_POSITION_1, 0xF6B5);
DECLARE_MEMORY_WORD(XRAM, THROTTLE_POSITION_2, 0xF6B7);

////////////////////////////////////////////////////////////
// FLASH
DECLARE_MEMORY_RANGE(FLASH, COOLANT_TEMPERATURE_TABLE_1, 0x831F, 0x832F);
DECLARE_MEMORY_RANGE(FLASH, COOLANT_TEMPERATURE_TABLE_2, 0x8330, 0x8340);
DECLARE_MEMORY_RANGE(FLASH, INTAKE_AIR_TEMPERATURE_TABLE, 0x8341, 0x8351);

DECLARE_MEMORY_BYTE(FLASH, MINIMUM_INTAKE_AIR_TEMPERATURE, 0x805E);
DECLARE_MEMORY_BYTE(FLASH, MAXIMUM_INTAKE_AIR_TEMPERATURE, 0x805F);

DECLARE_MEMORY_RANGE(FLASH, KITTING, 0x873F, 0x8744);

DECLARE_MEMORY_BYTE(FLASH, MINIMUM_IGNITION_VOLTAGE, 0x8062);
DECLARE_MEMORY_BYTE(FLASH, MAXIMUM_IGNITION_VOLTAGE, 0x8063);
////////////////////////////////////////////////////////////
// RAM

