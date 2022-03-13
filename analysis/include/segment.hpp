#pragma once

#include "defines.hpp"
#include "types.hpp"

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

