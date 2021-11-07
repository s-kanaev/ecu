#pragma once

#include "defines.hpp"

#include <cstdint>
#include <string>
#include <vector>
#include <map>

/*
TYPES:
 - byte - a byte
 - pin - a pin
*/

typedef uint8_t byte;
typedef uint16_t word;
typedef uint32_t quad;
typedef int8_t sbyte;
typedef int16_t sword;
typedef int32_t squad;
typedef int pin;

enum Pin : pin {
  P0_0,
  P0_1,
  P0_2,
  P0_3,
  P0_4,
  P0_5,
  P0_6,
  P0_7,

  P1_0,
  P1_1,
  P1_2,
  P1_3,
  P1_4,
  P1_5,
  P1_6,
  P1_7,

  P2_0,
  P2_1,
  P2_2,
  P2_3,
  P2_4,
  P2_5,
  P2_6,
  P2_7,

  P3_0,
  P3_1,
  P3_2,
  P3_3,
  P3_4,
  P3_5,
  P3_6,
  P3_7,

  P4_0,
  P4_1,
  P4_2,
  P4_3,
  P4_4,
  P4_5,
  P4_6,
  P4_7,

  P5_0,
  P5_1,
  P5_2,
  P5_3,
  P5_4,
  P5_5,
  P5_6,
  P5_7,

  P6_0,
  P6_1,
  P6_2,
  P6_3,
  P6_4,
  P6_5,
  P6_6,
  P6_7,

  P7_0,
  P7_1,
  P7_2,
  P7_3,
  P7_4,
  P7_5,
  P7_6,
  P7_7,

  P8_0,
  P8_1,
  P8_2,
  P8_3,
  P8_4,
  P8_5,
  P8_6,

  P9_0,
  P9_1,
  P9_2,
  P9_3,
  P9_4,
  P9_5,
  P9_6,
  P9_7,
};

#define PACKED __attribute__((__packed__))

struct TableEntryT {
  const byte Val;
  const byte TableIdx;
  const byte InterpolationFraction;

  TableEntryT(byte V)
    : Val{V},
      TableIdx{static_cast<byte>((Val & 0xF0) >> 4)},
      InterpolationFraction{static_cast<byte>((Val & 0x0F))}
  {}
};

struct OffsetAndFactorT {
  const byte Val;
  const byte Offset;
  const byte Factor;

  OffsetAndFactorT(byte V)
    : Val{V},
      Offset{static_cast<byte>((Val & 0xF8) >> 3)},
      Factor{static_cast<byte>((Val & 0x07) << 5)}
  {}
};

#define BYTE(x) (byte)(x)
#define WORD(x) (word)(x)
#define QUAD(x) (quad)(x)

#define BYTE_PTR(x) (byte *)(x)
#define WORD_PTR(x) (word *)(x)
#define QUAD_PTR(x) (quad *)(x)
