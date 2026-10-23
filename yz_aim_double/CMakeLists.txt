cmake_minimum_required(VERSION 3.14)
project(yz_aim_double VERSION 1.2.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

add_compile_options(-Wall -Wextra -Wpedantic -Wno-unused-parameter)

find_package(Threads REQUIRED)

# USB-CAN双轴：运行时加载LYS库；复用板载版的完整命令和控制循环。
add_executable(yz_aim_double
    src/yz_aim_can0.cpp
    src/yz_aim.cpp
    src/lys_transport.cpp
    src/shared_can_bus.cpp
    src/frame_trace.cpp
    src/motion_log.cpp
    src/socketcan_transport.cpp
    src/can_link_query.cpp
)
target_include_directories(yz_aim_double PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_SOURCE_DIR}/src
)
target_compile_definitions(yz_aim_double PRIVATE YZ_AIM_USB_DOUBLE)
target_link_libraries(yz_aim_double PRIVATE Threads::Threads ${CMAKE_DL_LIBS})
set_target_properties(yz_aim_double PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin
)
