
add_library(impl_host impl.cpp memory-locations.cpp)
target_compile_definitions(impl_host PUBLIC -D__COMPILE_FOR_TESTS=1)
target_compile_options(impl_host PUBLIC -Wall -Werror)

