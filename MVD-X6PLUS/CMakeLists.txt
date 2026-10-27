cmake_minimum_required(VERSION 3.16)
project(mvd_x6_test LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "构建类型" FORCE)
endif()

find_package(Threads REQUIRED)

add_library(mvd_core STATIC
  src/json_mini.cpp
  src/protocol.cpp
  src/bus.cpp
  src/fan.cpp
  src/runtime.cpp
  src/report.cpp
)
target_include_directories(mvd_core PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_compile_options(mvd_core PRIVATE -Wall -Wextra)
target_link_libraries(mvd_core PUBLIC Threads::Threads)

add_executable(mvd_x6_cli src/main.cpp)
target_link_libraries(mvd_x6_cli PRIVATE mvd_core)
target_compile_options(mvd_x6_cli PRIVATE -Wall -Wextra)
