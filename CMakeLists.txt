# Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
# SPDX-License-Identifier: Apache-2.0
cmake_minimum_required(VERSION 3.16)
project(manipulation VERSION 0.2.0 LANGUAGES CXX)
include(GNUInstallDirs)
foreach(directory IN ITEMS CMAKE_INSTALL_BINDIR CMAKE_INSTALL_LIBDIR CMAKE_INSTALL_DATADIR CMAKE_INSTALL_LIBEXECDIR)
    if(IS_ABSOLUTE "${${directory}}")
        message(FATAL_ERROR "${directory} must be relative for a relocatable installation")
    endif()
endforeach()
include(CTest)
find_package(Threads REQUIRED)
find_package(yaml-cpp REQUIRED)
option(MANIPULATION_ENABLE_NERO "Build the Nero/Revo2 driver (vendor SDK is used at run time only)" ON)
option(MANIPULATION_ENABLE_SO101 "Build the SO101 arm and gripper driver" ON)
set(MANIPULATION_MOTOR_SOURCE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../../peripherals/motor" CACHE PATH
    "Motor source tree for the SO101 driver; otherwise use an installed motor package")
if(MANIPULATION_ENABLE_SO101)
    if(EXISTS "${MANIPULATION_MOTOR_SOURCE_DIR}/CMakeLists.txt")
        # Only its Feetech driver; not installed from here.
        set(SROBOTIS_PERIPHERALS_MOTOR_ENABLED_DRIVERS "drv_uart_feetech" CACHE STRING
            "Enabled motor drivers (semicolon-separated). Empty means auto-detect." FORCE)
        set(BUILD_TESTS OFF CACHE BOOL "Build test programs" FORCE)
        add_subdirectory("${MANIPULATION_MOTOR_SOURCE_DIR}" motor EXCLUDE_FROM_ALL)
        set(MANIPULATION_MOTOR_TARGET motor)
    else()
        find_package(motor CONFIG REQUIRED)
        set(MANIPULATION_MOTOR_TARGET components::motor)
    endif()
endif()
add_compile_options(-Wall -Wextra -Werror)  # After the motor component: these apply to this component's code.
find_package(OpenCV QUIET COMPONENTS core calib3d)
option(MANIPULATION_ENABLE_CALIBRATION "Build the offline OpenCV hand-eye solver (default: when OpenCV is found)"
    ${OpenCV_FOUND})
# Pure-Python modules go to PREFIX/python, which build/envsetup.sh puts on PYTHONPATH.
set(MANIPULATION_PYTHON_DIR "python")
set(MANIPULATION_LOCK_DIRECTORY "/run/lock/spacemit-manipulation" CACHE STRING
    "Host-wide device lease directory shared by every program")
if(NOT IS_ABSOLUTE "${MANIPULATION_LOCK_DIRECTORY}")
    message(FATAL_ERROR "MANIPULATION_LOCK_DIRECTORY must be absolute")
endif()

# Contracts: device interfaces, sessions, leases and shared types.
add_library(manipulation_contracts core/types.cpp core/arm.cpp core/device.cpp core/resource_paths.cpp)
target_compile_features(manipulation_contracts PUBLIC cxx_std_17)
target_include_directories(manipulation_contracts
    PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include> $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
    PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/core)
target_compile_definitions(manipulation_contracts PRIVATE
    MANIPULATION_LOCK_DIRECTORY="${MANIPULATION_LOCK_DIRECTORY}")
set_target_properties(manipulation_contracts PROPERTIES POSITION_INDEPENDENT_CODE ON)

# Drivers: each module appends sources and a registration function; no central model switch.
include(cmake/driver_modules.cmake)
set(MANIPULATION_EXTRA_DRIVER_DIRS "" CACHE STRING "Additional driver module directories")
add_library(manipulation_drivers STATIC)
target_link_libraries(manipulation_drivers PUBLIC manipulation_contracts)
set_target_properties(manipulation_drivers PROPERTIES POSITION_INDEPENDENT_CODE ON)
set_property(GLOBAL PROPERTY MANIPULATION_DEVICE_REGISTRATIONS "")
add_subdirectory(drivers/mock)
if(MANIPULATION_ENABLE_NERO)
    add_subdirectory(drivers/arms/nero)
endif()
if(MANIPULATION_ENABLE_SO101)
    add_subdirectory(drivers/arms/so101)
endif()
foreach(module IN LISTS MANIPULATION_EXTRA_DRIVER_DIRS)
    string(MD5 module_key "${module}")
    add_subdirectory("${module}" "extra_drivers/${module_key}")
endforeach()
get_property(registrations GLOBAL PROPERTY MANIPULATION_DEVICE_REGISTRATIONS)
set(registry_source "#include \"manipulation/device.h\"\nnamespace spacemit::manipulation {\n")
foreach(symbol IN LISTS registrations)
    string(APPEND registry_source "void ${symbol}(DeviceRegistry*);\n")
endforeach()
string(APPEND registry_source "DeviceRegistry MakeBuiltinDeviceRegistry() { DeviceRegistry registry;\n")
foreach(symbol IN LISTS registrations)
    string(APPEND registry_source "${symbol}(&registry);\n")
endforeach()
string(APPEND registry_source "return registry; }\n}\n")
file(GENERATE OUTPUT "${CMAKE_CURRENT_BINARY_DIR}/builtin_registry.cpp" CONTENT "${registry_source}")
target_sources(manipulation_drivers PRIVATE "${CMAKE_CURRENT_BINARY_DIR}/builtin_registry.cpp")

# Capabilities and tools.
add_library(manipulation_motion runtime/device_plan.cpp)
target_link_libraries(manipulation_motion PUBLIC manipulation_contracts PRIVATE yaml-cpp Threads::Threads)
set_target_properties(manipulation_motion PROPERTIES POSITION_INDEPENDENT_CODE ON)
add_executable(manipulation-device capabilities/motion/cli/device_cli.cpp
    capabilities/motion/cli/device_service.cpp capabilities/diagnostics/arm_check.cpp)
target_include_directories(manipulation-device PRIVATE capabilities/diagnostics)
target_link_libraries(manipulation-device PRIVATE manipulation_motion manipulation_drivers yaml-cpp Threads::Threads)
file(RELATIVE_PATH MANIPULATION_LIB_FROM_BIN "/${CMAKE_INSTALL_BINDIR}" "/${CMAKE_INSTALL_LIBDIR}")
set_target_properties(manipulation-device PROPERTIES INSTALL_RPATH "$ORIGIN/${MANIPULATION_LIB_FROM_BIN}")

if(MANIPULATION_ENABLE_CALIBRATION)
    find_package(OpenCV REQUIRED COMPONENTS core calib3d)
    add_library(manipulation_calibration capabilities/calibration/src/calibration.cpp)
    target_compile_features(manipulation_calibration PUBLIC cxx_std_17)
    target_include_directories(manipulation_calibration PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}> PRIVATE ${OpenCV_INCLUDE_DIRS})
    target_link_libraries(manipulation_calibration PRIVATE ${OpenCV_LIBS})
    add_executable(manipulation-calibration capabilities/calibration/cli/calibration_cli.cpp)
    target_link_libraries(manipulation-calibration PRIVATE manipulation_calibration yaml-cpp)
    install(TARGETS manipulation_calibration manipulation-calibration EXPORT manipulationTargets
        ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR} LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
        RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})
endif()

# Installation. Every runtime path is relative to the prefix, so the tree can be moved.
install(TARGETS manipulation_contracts manipulation_drivers manipulation_motion manipulation-device
    EXPORT manipulationTargets
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})
install(DIRECTORY include/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
install(DIRECTORY bindings/python/spacemit_manipulation
    DESTINATION ${MANIPULATION_PYTHON_DIR} PATTERN "__pycache__" EXCLUDE)
install(FILES capabilities/kinematics/python/urdf_fk.py capabilities/kinematics/python/urdf_ik.py
    capabilities/calibration/cli/workbench.py
    capabilities/calibration/cli/collect.py capabilities/calibration/cli/hand_eye_cli.py
    capabilities/calibration/python/sampling.py capabilities/calibration/python/charuco.py
    capabilities/calibration/python/charuco_sampler.py capabilities/diagnostics/cli/operator_cli.py
    DESTINATION ${MANIPULATION_PYTHON_DIR}/spacemit_manipulation)
install(DIRECTORY descriptions/ DESTINATION ${CMAKE_INSTALL_DATADIR}/manipulation/descriptions)
install(DIRECTORY examples/ DESTINATION ${CMAKE_INSTALL_DATADIR}/manipulation/examples)
install(FILES tools/build_manifest.py tools/check_environment.py tools/configure_device.py
    DESTINATION ${CMAKE_INSTALL_DATADIR}/manipulation/tools)
configure_file(tools/layout.json.in ${CMAKE_CURRENT_BINARY_DIR}/.spacemit-manipulation.json @ONLY)
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/.spacemit-manipulation.json DESTINATION .)
file(RELATIVE_PATH COMMAND_PREFIX_FROM_BINDIR "/${CMAKE_INSTALL_BINDIR}" "/")
foreach(command IN ITEMS manipulation hand-eye)
    set(COMMAND_NAME ${command})
    if(command STREQUAL "manipulation")
        set(COMMAND_MODULE spacemit_manipulation.operator_cli)
    else()
        set(COMMAND_MODULE spacemit_manipulation.hand_eye_cli)
    endif()
    configure_file(tools/command.in ${CMAKE_CURRENT_BINARY_DIR}/${command} @ONLY)
    install(PROGRAMS ${CMAKE_CURRENT_BINARY_DIR}/${command} DESTINATION ${CMAKE_INSTALL_BINDIR})
endforeach()
install(FILES LICENSE NOTICE DESTINATION ${CMAKE_INSTALL_DATADIR}/manipulation)
install(EXPORT manipulationTargets NAMESPACE spacemit:: DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/manipulation)
configure_file(cmake/manipulation-config.cmake ${CMAKE_CURRENT_BINARY_DIR}/manipulation-config.cmake @ONLY)
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/manipulation-config.cmake DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/manipulation)

if(BUILD_TESTING)
    find_package(Python3 REQUIRED COMPONENTS Interpreter)
    set(test_environment "PYTHONDONTWRITEBYTECODE=1;MANIPULATION_DEVICE=$<TARGET_FILE:manipulation-device>"
        "MANIPULATION_TEST_NERO=$<BOOL:${MANIPULATION_ENABLE_NERO}>"
        "MANIPULATION_TEST_SO101=$<BOOL:${MANIPULATION_ENABLE_SO101}>"
        "MANIPULATION_LOCK_DIRECTORY=${CMAKE_CURRENT_BINARY_DIR}/test-locks")
    add_executable(manipulation_arm_test tests/arm_test.cpp)
    target_include_directories(manipulation_arm_test PRIVATE core)
    target_link_libraries(manipulation_arm_test PRIVATE manipulation_contracts)
    add_test(NAME manipulation_arm_contract COMMAND manipulation_arm_test)
    add_executable(manipulation_device_test tests/device_test.cpp)
    target_link_libraries(manipulation_device_test PRIVATE manipulation_motion manipulation_drivers)
    add_test(NAME manipulation_device_embedded_contract COMMAND manipulation_device_test)
    set_tests_properties(manipulation_arm_contract manipulation_device_embedded_contract PROPERTIES
        ENVIRONMENT "${test_environment}")
    add_test(NAME manipulation_device_contract COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/tests/test_device.py)
    add_test(NAME manipulation_tools_contract COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/tests/test_tools.py)
    add_test(NAME manipulation_kinematics_contract
        COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/tests/test_kinematics.py)
    set_tests_properties(manipulation_device_contract manipulation_tools_contract manipulation_kinematics_contract PROPERTIES
        TIMEOUT 90 ENVIRONMENT "${test_environment}")
    add_executable(manipulation_control_api_test tests/control_api_test.cpp)
    target_link_libraries(manipulation_control_api_test PRIVATE manipulation_motion manipulation_drivers)
    add_test(NAME manipulation_control_api_contract COMMAND manipulation_control_api_test)
    add_test(NAME manipulation_control_cli_contract
        COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/tests/test_control_api.py)
    set_tests_properties(manipulation_control_api_contract manipulation_control_cli_contract PROPERTIES
        TIMEOUT 90 ENVIRONMENT "${test_environment}")
    if(MANIPULATION_ENABLE_SO101)
        add_test(NAME manipulation_so101_contract
            COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/tests/test_so101.py)
        add_test(NAME manipulation_so101_calibration_contract
            COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/tests/test_so101_calibration.py)
        set_tests_properties(manipulation_so101_contract manipulation_so101_calibration_contract PROPERTIES
            TIMEOUT 90 ENVIRONMENT "${test_environment};MANIPULATION_FEETECH_BUS=$<TARGET_FILE:manipulation_feetech_bus>")
    endif()
    if(MANIPULATION_ENABLE_NERO)
        add_executable(manipulation_recovery_test tests/recovery_test.cpp)
        target_link_libraries(manipulation_recovery_test PRIVATE manipulation_motion manipulation_drivers)
        add_test(NAME manipulation_nero_worker_contract
            COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/tests/test_nero.py)
        set_tests_properties(manipulation_nero_worker_contract PROPERTIES TIMEOUT 120
            ENVIRONMENT "${test_environment};MANIPULATION_RECOVERY=$<TARGET_FILE:manipulation_recovery_test>")
    endif()
    if(MANIPULATION_ENABLE_CALIBRATION)
        add_executable(manipulation_calibration_test tests/calibration_test.cpp)
        target_include_directories(manipulation_calibration_test PRIVATE ${OpenCV_INCLUDE_DIRS})
        target_link_libraries(manipulation_calibration_test PRIVATE manipulation_calibration ${OpenCV_LIBS})
        add_test(NAME manipulation_calibration_contract COMMAND manipulation_calibration_test)
        add_test(NAME manipulation_charuco_contract
            COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/tests/test_charuco.py)
        set_tests_properties(manipulation_charuco_contract PROPERTIES TIMEOUT 60 ENVIRONMENT "PYTHONDONTWRITEBYTECODE=1")
    endif()
endif()
