#pragma once

#include "defines.hpp"
#include "types.hpp"

// Namespace for describing memory segments
namespace seg {
// TODO rework segments classes
//   template <class _Seg>
//   class Segment {
//
//   };

  // Method get return start of memory segment
  struct XRAM {
//     template <typename T>
//     static T *get();
    static byte *get();

#if __E591_HOST_COMPILATION
    static const char *name();
#endif  // __E591_HOST_COMPILATION
  };

  struct FLASH {
//     template <typename T>
//     static T *get();
    static byte *get();

#if __E591_HOST_COMPILATION
    static const char *name();
#endif  // __E591_HOST_COMPILATION
  };

  struct RAM {
//     template <typename T>
//     static T *get();
    static byte *get();

#if __E591_HOST_COMPILATION
    static const char *name();
#endif  // __E591_HOST_COMPILATION
  };

  struct EEPROM {
//     template <typename T>
//     static T *get();
    static byte *get();

#if __E591_HOST_COMPILATION
    static const char *name();
#endif  // __E591_HOST_COMPILATION
  };
} // namespace seg

