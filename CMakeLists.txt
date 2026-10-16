cmake_minimum_required(VERSION 3.16)
project(proto_drivers VERSION 1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

option(PROTO_BUILD_TESTS "Build GoogleTest unit tests" ON)
option(PROTO_COVERAGE "Instrument for gcov/lcov coverage" OFF)
option(PROTO_SANITIZE "Build tests with ASan + UBSan" OFF)

# ----------------------------------------------------------------- library ---
add_library(proto
    src/crc.cpp
    src/i2c.cpp
    src/spi.cpp
    src/can.cpp
    src/isotp.cpp
    src/uart.cpp
    src/lin.cpp
    src/onewire.cpp
    src/modbus.cpp
)
target_include_directories(proto PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_compile_options(proto PRIVATE -Wall -Wextra -Wpedantic -Wshadow)

if(PROTO_COVERAGE)
    target_compile_options(proto PRIVATE --coverage -O0)
    target_link_options(proto PUBLIC --coverage)
endif()
if(PROTO_SANITIZE)
    target_compile_options(proto PUBLIC -fsanitize=address,undefined -fno-omit-frame-pointer)
    target_link_options(proto PUBLIC -fsanitize=address,undefined)
endif()

# ------------------------------------------------------------------- tests ---
if(PROTO_BUILD_TESTS)
    enable_testing()
    find_package(GTest CONFIG QUIET)
    if(NOT GTest_FOUND)
        include(FetchContent)
        FetchContent_Declare(googletest
            GIT_REPOSITORY https://github.com/google/googletest.git
            GIT_TAG v1.15.2
            GIT_SHALLOW TRUE)
        set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
        set(INSTALL_GTEST OFF CACHE BOOL "" FORCE)
        FetchContent_MakeAvailable(googletest)
    endif()

    add_executable(proto_tests
        tests/test_crc.cpp
        tests/test_i2c.cpp
        tests/test_spi.cpp
        tests/test_can.cpp
        tests/test_isotp.cpp
        tests/test_uart.cpp
        tests/test_lin.cpp
        tests/test_onewire.cpp
        tests/test_modbus.cpp
    )
    target_link_libraries(proto_tests PRIVATE proto GTest::gmock_main)
    target_compile_options(proto_tests PRIVATE -Wall -Wextra)

    include(GoogleTest)
    gtest_discover_tests(proto_tests)
endif()
