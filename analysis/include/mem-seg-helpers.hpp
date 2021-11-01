#pragma once

#include "binary_ops.hpp"

#include <array>

#define DEF_ARRAY(type, from, to, name) \
std::array<type, (to) - (from) + 1> name

#define DEF_WORD_AT_BYTE_OFFSET(seg, offset, name, val) \
do {                                                    \
  seg[offset] = LOW(val);                               \
  seg[offset + 1] = HIGH(val);                          \
} while(0)

template <typename T, size_t _Count>
void copy(T *Start, std::array<T, _Count> &Orig) {
  for (size_t Idx = 0; Idx < Orig.size(); ++Idx)
    Start[Idx] = Orig[Idx];
}


namespace seg {
  struct XRAM;
  struct FLASH;
  struct RAM;

template <typename _Seg>
  struct Segment {
    using SelfSeg = _Seg;

    static byte &get(std::size_t Idx);
  };
}
