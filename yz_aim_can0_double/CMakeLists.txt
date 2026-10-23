cmake_minimum_required(VERSION 3.14)
project(yz_aim_can0 VERSION 1.2.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

add_compile_options(-Wall -Wextra -Wpedantic -Wno-unused-parameter)

find_package(Threads REQUIRED)

# 只用板载 CAN 控制器（SocketCAN）的维护工具，不链接任何 USB-CAN 厂商库。
add_executable(yz_aim_can0
    src/yz_aim_can0.cpp
    src/yz_aim.cpp
    src/frame_trace.cpp
    src/motion_log.cpp
    src/socketcan_transport.cpp
    src/can_link_query.cpp
)
target_include_directories(yz_aim_can0 PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_SOURCE_DIR}/src
)
target_link_libraries(yz_aim_can0 PRIVATE Threads::Threads)
set_target_properties(yz_aim_can0 PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin
)
