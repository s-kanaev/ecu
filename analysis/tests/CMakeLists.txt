
add_executable(test_ranges test_ranges.cpp impl.cpp common.cpp)
target_compile_definitions(test_ranges PUBLIC -D__COMPILE_FOR_TESTS=1)
target_link_libraries(test_ranges impl_host)
