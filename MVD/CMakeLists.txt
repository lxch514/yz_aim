cmake_minimum_required(VERSION 3.16)
project(mvd_test LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "构建类型" FORCE)
endif()

add_library(mvd_core STATIC
  src/json_mini.cpp
  src/protocol.cpp
  src/bus.cpp
  src/runtime.cpp
  src/report.cpp
)
target_include_directories(mvd_core PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_compile_options(mvd_core PRIVATE -Wall -Wextra)

add_executable(mvd_cli src/main.cpp)
target_link_libraries(mvd_cli PRIVATE mvd_core)
target_compile_options(mvd_cli PRIVATE -Wall -Wextra)
