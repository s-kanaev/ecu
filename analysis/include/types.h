#pragma once

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

#define PACKED __attribute__((__packed__))
struct PACKED TableEntryS {
  byte TableIdx : 4;
  byte InterpolationFraction : 4; // Used for interpolation / profiling.
};

union TableEntryU {
  TableEntryS TE;
  byte ByteVal;
};

typedef union TableEntryU TableEntryT;

#define BYTE(x) (byte)(x)
#define WORD(x) (word)(x)
#define QUAD(x) (quad)(x)

