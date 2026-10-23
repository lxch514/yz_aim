cmake_minimum_required(VERSION 3.14)
project(yz_aim VERSION 1.5.0 LANGUAGES CXX)
include(CTest)
add_library(yz_aim_driver STATIC src/yz_aim.cpp src/lys_transport.cpp src/frame_trace.cpp)
target_compile_features(yz_aim_driver PUBLIC cxx_std_17)
target_include_directories(yz_aim_driver PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(yz_aim_driver PUBLIC pthread)
target_link_libraries(yz_aim_driver PUBLIC ${CMAKE_DL_LIBS})
foreach(test_target yz_aim_protocol_test yz_aim_driver_test yz_aim_sdo_timeout_test)
    add_executable(${test_target} test/${test_target}.cpp)
    target_link_libraries(${test_target} PRIVATE yz_aim_driver)
    if(BUILD_TESTING)
        add_test(NAME ${test_target} COMMAND ${test_target})
    endif()
endforeach()

add_library(yz_aim_fake_lys SHARED test/fake_lys_sdk.cpp)
set_target_properties(yz_aim_fake_lys PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/test-fixtures)
add_executable(yz_aim_transport_test test/yz_aim_transport_test.cpp)
target_link_libraries(yz_aim_transport_test PRIVATE yz_aim_driver)
add_dependencies(yz_aim_transport_test yz_aim_fake_lys)
if(BUILD_TESTING)
    add_test(NAME yz_aim_transport_test COMMAND yz_aim_transport_test $<TARGET_FILE:yz_aim_fake_lys>)
endif()

# Standalone command-line front end: depends only on include/ and src/, so it
# builds from this folder alone without the controller repository or FTXUI.
find_package(Threads REQUIRED)
add_executable(yz_aim_cli tools/yz_aim_cli.cpp)
target_link_libraries(yz_aim_cli PRIVATE yz_aim_driver Threads::Threads)
set_target_properties(yz_aim_cli PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/bin)

# Maintenance UI is isolated from the controller and builds only when the
# shared FTXUI display layer is enabled.
if(TARGET maintenance_tui)
    find_package(Threads REQUIRED)
    add_library(yz_aim_tool_ui STATIC tools/yz_aim_tui.cpp)
    target_include_directories(yz_aim_tool_ui PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/tools)
    target_link_libraries(yz_aim_tool_ui PUBLIC yz_aim_driver maintenance_tui Threads::Threads)
    foreach(tool_target yz_aim_setup yz_aim_test)
        set(tool_source tools/${tool_target}.cpp)
        if(tool_target STREQUAL "yz_aim_test")
            set(tool_source test/yz_aim_test.cpp)
        endif()
        add_executable(${tool_target} ${tool_source})
        target_link_libraries(${tool_target} PRIVATE yz_aim_tool_ui)
    endforeach()
    add_executable(yz_aim_tui_policy_test test/yz_aim_tui_policy_test.cpp)
    target_link_libraries(yz_aim_tui_policy_test PRIVATE yz_aim_tool_ui)
    if(BUILD_TESTING)
        add_test(NAME yz_aim_tui_policy_test COMMAND yz_aim_tui_policy_test)
        find_package(Python3 COMPONENTS Interpreter QUIET)
        if(Python3_Interpreter_FOUND)
            add_test(NAME yz_aim_tui_virtual
                COMMAND ${Python3_EXECUTABLE}
                    ${CMAKE_CURRENT_SOURCE_DIR}/test/test_yz_aim_tui_virtual.py
                    --setup $<TARGET_FILE:yz_aim_setup>
                    --test $<TARGET_FILE:yz_aim_test>)
            set_tests_properties(yz_aim_tui_virtual PROPERTIES
                LABELS integration TIMEOUT 120)
        endif()
    endif()
endif()
