#pragma once

#include "ram.h"
#include "flash.h"
#include "xram.h"

#define DEFINE_MEMORY_RANGE(segment, name, start, end)                          \
  static const int segment ## _ ## name ## _START_PTR = (start);                \
  static const int segment ## _ ## name ## _END_PTR = (end);                    \
  static const int segment ## _ ## name ## _RANGE_LENGTH = (end) - (start) + 1; \
  static inline byte* get_ ## segment ## _ ## name ## _start() {                \
    return &segment[segment ## _ ## name ## _START_PTR];                        \
  };                                                                            \
  static inline byte* get_ ## segment ## _ ## name ## _start() {                \
    return &segment[segment ## _ ## name ## _END_PTR];                          \
  };

#define RNG_START(segment, name) \
  segment ## _ ## name ## _START_PTR
#define RNG_END(segment, name) \
  segment ## _ ## name ## _END_PTR
#define RNG_LENGTH(segment, name) \
  segment ## _ ## name ## _RANGE_LENGTH
#define GET_RNG_START(segment, name) \
  get_ ## segment ## _ ## name ## _start()
#define GET_RNG_END(segment, name) \
  get_ ## segment ## _ ## name ## _end()


#define DEFINE_MEMORY_LOCATION(segment, name, loc)        \
  static const int segment ## _ ## name ## _PTR = (loc);  \

#define MEM_LOC(segment, name) \
  segment ## _ ## name ## _PTR
#define GET_MEM_LOC(segment, name) \
  &segment[segment ## _ ## name ## _PTR]


DEFINE_RANGE(XRAM, SUM_OF_EGO_CALIBRATION, 0xF400, 0xF4FF);
DEFINE_RANGE(XRAM, EGO_CALIBRATION, 0xF500, 0xF5FF);

DEFINE_MEMORY_LOCATION(XRAM, CHECKSUM_LOW, 0xF658);
DEFINE_MEMORY_LOCATION(XRAM, CHECKSUM_HIGH, 0xF659);


