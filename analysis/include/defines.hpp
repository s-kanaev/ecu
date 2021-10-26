#pragma once

//#define __COMPILE_FOR_TESTS 1

#define UNUSED(x) (void)(x)

#if __COMPILE_FOR_TESTS != 0
# define __E591_TESTS 1
#else
# define __E591_TESTS 0
#endif

#define __E591_HOST_COMPILATION __E591_TESTS
