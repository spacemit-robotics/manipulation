# Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
# SPDX-License-Identifier: Apache-2.0
cmake_minimum_required(VERSION 3.16)
project(manipulation VERSION 0.2.0 LANGUAGES CXX)
include(GNUInstallDirs)
foreach(directory IN ITEMS CMAKE_INSTALL_BINDIR CMAKE_INSTALL_LIBDIR CMAKE_INSTALL_DATADIR
    CMAKE_INSTALL_LIBEXECDIR)
    if(IS_ABSOLUTE "${${directory}}")
        message(FATAL_ERROR "${directory} must be relative for a relocatable installation")
    endif()
endforeach()
include(CTest)
find_package(Threads REQUIRED)
find_package(yaml-cpp REQUIRED)
option(MANIPULATION_ENABLE_NERO "Build the Nero/Revo2 driver (vendor SDK is used at run time only)"
    ON)
option(MANIPULATION_ENABLE_SO101 "Build the SO101 arm and gripper driver" ON)
option(MANIPULATION_ENABLE_KINEMATICS_EIGEN
    "Build the built-in Eigen FK/IK backend (no external kinematics library)" ON)
option(MANIPULATION_ENABLE_KINEMATICS_PINOCCHIO "Build the offline Pinocchio FK/IK backend" OFF)
option(MANIPULATION_ENABLE_MUJOCO "Build the local MuJoCo physics simulation driver" OFF)
option(MANIPULATION_ENABLE_MUJOCO_REMOTE "Build the TCP MuJoCo client without a local engine" OFF)
option(MANIPULATION_ENABLE_SIMULATION_RENDERER "Build the simulation viewer and RGB-D renderer" OFF)
if(MANIPULATION_ENABLE_SIMULATION_RENDERER AND NOT MANIPULATION_ENABLE_MUJOCO)
    message(FATAL_ERROR "The simulation renderer requires MANIPULATION_ENABLE_MUJOCO=ON")
endif()
set(_manipulation_mujoco_source "${CMAKE_CURRENT_SOURCE_DIR}/simulation/mujoco")
if(DEFINED MANIPULATION_MUJOCO_SOURCE_DIR)
    get_filename_component(_configured_mujoco_source "${MANIPULATION_MUJOCO_SOURCE_DIR}"
        ABSOLUTE BASE_DIR "${CMAKE_CURRENT_SOURCE_DIR}")
    get_filename_component(_sdk_mujoco_source "${CMAKE_CURRENT_SOURCE_DIR}/../../simulation/mujoco"
        ABSOLUTE)
    # Upgrade the SDK default in existing build trees while retaining explicit external sources.
    if(_configured_mujoco_source STREQUAL _sdk_mujoco_source)
        set(MANIPULATION_MUJOCO_SOURCE_DIR "${_manipulation_mujoco_source}" CACHE PATH
            "MuJoCo physics source; otherwise use an installed mujoco_world package" FORCE)
    endif()
endif()
set(MANIPULATION_MUJOCO_SOURCE_DIR "${_manipulation_mujoco_source}" CACHE PATH
    "MuJoCo physics source; otherwise use an installed mujoco_world package")
if(MANIPULATION_ENABLE_MUJOCO)
    if(EXISTS "${MANIPULATION_MUJOCO_SOURCE_DIR}/CMakeLists.txt")
        get_filename_component(_active_mujoco_source "${MANIPULATION_MUJOCO_SOURCE_DIR}"
            ABSOLUTE BASE_DIR "${CMAKE_CURRENT_SOURCE_DIR}")
        if(NOT _active_mujoco_source STREQUAL _manipulation_mujoco_source)
            # External providers may also offer a humanoid simulator; only use their physics API.
            set(MUJOCO_BUILD_LEGACY_SIMULATOR OFF)
        endif()
        set(MUJOCO_BUILD_RENDERER ${MANIPULATION_ENABLE_SIMULATION_RENDERER})
        add_subdirectory("${MANIPULATION_MUJOCO_SOURCE_DIR}" mujoco_world)
    else()
        find_package(mujoco_world CONFIG REQUIRED)
    endif()
endif()
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
# After the motor component: these apply to this component's code.
add_compile_options(-Wall -Wextra -Werror)
find_package(OpenCV QUIET COMPONENTS core calib3d)
option(MANIPULATION_ENABLE_CALIBRATION
    "Build the offline OpenCV hand-eye solver (default: when OpenCV is found)"
    ${OpenCV_FOUND})
# Pure-Python modules go to PREFIX/python, which build/envsetup.sh puts on PYTHONPATH.
set(MANIPULATION_PYTHON_DIR "python")
set(MANIPULATION_LOCK_DIRECTORY "/run/lock/spacemit-manipulation" CACHE STRING
    "Host-wide device lease directory shared by every program")
if(NOT IS_ABSOLUTE "${MANIPULATION_LOCK_DIRECTORY}")
    message(FATAL_ERROR "MANIPULATION_LOCK_DIRECTORY must be absolute")
endif()

# Contracts: device interfaces, sessions, leases and shared types.
add_library(manipulation_contracts core/types.cpp core/arm.cpp core/device.cpp
    core/resource_paths.cpp)
target_compile_features(manipulation_contracts PUBLIC cxx_std_17)
target_include_directories(manipulation_contracts
    PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
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
if(MANIPULATION_ENABLE_MUJOCO OR MANIPULATION_ENABLE_MUJOCO_REMOTE)
    add_subdirectory(drivers/simulation/mujoco)
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
string(APPEND registry_source
    "DeviceRegistry MakeBuiltinDeviceRegistry() { DeviceRegistry registry;\n")
foreach(symbol IN LISTS registrations)
    string(APPEND registry_source "${symbol}(&registry);\n")
endforeach()
string(APPEND registry_source "return registry; }\n}\n")
file(GENERATE OUTPUT "${CMAKE_CURRENT_BINARY_DIR}/builtin_registry.cpp" CONTENT
    "${registry_source}")
target_sources(manipulation_drivers PRIVATE "${CMAKE_CURRENT_BINARY_DIR}/builtin_registry.cpp")

# Capabilities and tools.
add_library(manipulation_kinematics capabilities/kinematics/kinematics.cpp)
target_compile_features(manipulation_kinematics PUBLIC cxx_std_17)
target_include_directories(manipulation_kinematics PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>)
set_target_properties(manipulation_kinematics PROPERTIES POSITION_INDEPENDENT_CODE ON)
set(kinematics_source "#include \"manipulation/kinematics.h\"\nnamespace spacemit::manipulation {\n")
if(MANIPULATION_ENABLE_KINEMATICS_EIGEN)
    # Header-only Eigen keeps this backend free of any external kinematics library.
    find_package(Eigen3 QUIET CONFIG)
    if(NOT Eigen3_FOUND)
        find_path(MANIPULATION_EIGEN_INCLUDE_DIR Eigen/Core
            HINTS /usr/include/eigen3 /usr/local/include/eigen3)
        if(NOT MANIPULATION_EIGEN_INCLUDE_DIR)
            message(FATAL_ERROR
                "MANIPULATION_ENABLE_KINEMATICS_EIGEN=ON requires Eigen3 headers; "
                "install libeigen3-dev or set MANIPULATION_ENABLE_KINEMATICS_EIGEN=OFF")
        endif()
    endif()
    target_sources(manipulation_kinematics PRIVATE capabilities/kinematics/eigen_kinematics.cpp)
    # Eigen is header-only and used only by this translation unit, so take the include path
    # instead of linking Eigen3::Eigen. A static library re-exports PRIVATE link libraries as
    # INTERFACE_LINK_LIBRARIES, which would force every consumer of manipulation_kinematics to
    # call find_package(Eigen3) even though no Eigen symbol crosses the library boundary.
    if(Eigen3_FOUND)
        target_include_directories(manipulation_kinematics PRIVATE ${EIGEN3_INCLUDE_DIR})
    else()
        target_include_directories(manipulation_kinematics
            PRIVATE ${MANIPULATION_EIGEN_INCLUDE_DIR})
    endif()
    string(APPEND kinematics_source "void RegisterEigenKinematics(KinematicsRegistry*);\n")
endif()
if(MANIPULATION_ENABLE_KINEMATICS_PINOCCHIO)
    find_package(pinocchio CONFIG REQUIRED)
    target_sources(manipulation_kinematics PRIVATE capabilities/kinematics/pinocchio_kinematics.cpp)
    target_link_libraries(manipulation_kinematics PRIVATE pinocchio::pinocchio)
    string(APPEND kinematics_source "void RegisterPinocchioKinematics(KinematicsRegistry*);\n")
endif()
string(APPEND kinematics_source
    "KinematicsRegistry MakeBuiltinKinematicsRegistry() { KinematicsRegistry registry;\n")
if(MANIPULATION_ENABLE_KINEMATICS_EIGEN)
    string(APPEND kinematics_source "RegisterEigenKinematics(&registry);\n")
endif()
if(MANIPULATION_ENABLE_KINEMATICS_PINOCCHIO)
    string(APPEND kinematics_source "RegisterPinocchioKinematics(&registry);\n")
endif()
string(APPEND kinematics_source "return registry; }\n}\n")
file(GENERATE OUTPUT "${CMAKE_CURRENT_BINARY_DIR}/builtin_kinematics.cpp"
    CONTENT "${kinematics_source}")
target_sources(manipulation_kinematics PRIVATE
    "${CMAKE_CURRENT_BINARY_DIR}/builtin_kinematics.cpp")

add_library(manipulation_motion runtime/device_plan.cpp runtime/device_composition.cpp
    capabilities/capability_registry.cpp capabilities/motion/registered_capabilities.cpp)
target_link_libraries(manipulation_motion PUBLIC manipulation_contracts PRIVATE yaml-cpp
    Threads::Threads)
set_target_properties(manipulation_motion PROPERTIES POSITION_INDEPENDENT_CODE ON)
set(MANIPULATION_EXTRA_CAPABILITY_DIRS "" CACHE STRING "Additional capability module directories")
set_property(GLOBAL PROPERTY MANIPULATION_CAPABILITY_REGISTRATIONS "")
manipulation_register_capability(RegisterMotionCapabilities)
foreach(module IN LISTS MANIPULATION_EXTRA_CAPABILITY_DIRS)
    string(MD5 module_key "${module}")
    add_subdirectory("${module}" "extra_capabilities/${module_key}")
endforeach()
get_property(capability_registrations GLOBAL PROPERTY MANIPULATION_CAPABILITY_REGISTRATIONS)
set(capability_source
    "#include \"manipulation/capability.h\"\nnamespace spacemit::manipulation {\n")
foreach(symbol IN LISTS capability_registrations)
    string(APPEND capability_source "void ${symbol}(CapabilityRegistry*);\n")
endforeach()
string(APPEND capability_source
    "CapabilityRegistry MakeBuiltinCapabilityRegistry() { CapabilityRegistry registry;\n")
foreach(symbol IN LISTS capability_registrations)
    string(APPEND capability_source "${symbol}(&registry);\n")
endforeach()
string(APPEND capability_source "return registry; }\n}\n")
file(GENERATE OUTPUT "${CMAKE_CURRENT_BINARY_DIR}/builtin_capabilities.cpp"
    CONTENT "${capability_source}")
target_sources(manipulation_motion PRIVATE "${CMAKE_CURRENT_BINARY_DIR}/builtin_capabilities.cpp")
add_executable(manipulation-device capabilities/motion/cli/device_cli.cpp
    capabilities/motion/cli/device_service.cpp capabilities/diagnostics/arm_check.cpp)
target_include_directories(manipulation-device PRIVATE capabilities/diagnostics)
target_link_libraries(manipulation-device PRIVATE manipulation_motion manipulation_drivers yaml-cpp
    Threads::Threads)
file(RELATIVE_PATH MANIPULATION_LIB_FROM_BIN "/${CMAKE_INSTALL_BINDIR}" "/${CMAKE_INSTALL_LIBDIR}")
set_target_properties(manipulation-device PROPERTIES INSTALL_RPATH
    "$ORIGIN/${MANIPULATION_LIB_FROM_BIN}")

if(MANIPULATION_ENABLE_MUJOCO OR MANIPULATION_ENABLE_MUJOCO_REMOTE)
    add_executable(manipulation-simulation-camera
        capabilities/simulation/observer_client.cpp capabilities/simulation/camera_cli.cpp)
    target_include_directories(manipulation-simulation-camera PRIVATE drivers/simulation/mujoco)
    target_link_libraries(manipulation-simulation-camera PRIVATE manipulation_drivers yaml-cpp
        Threads::Threads)
    set_target_properties(manipulation-simulation-camera PROPERTIES INSTALL_RPATH
        "$ORIGIN/${MANIPULATION_LIB_FROM_BIN}")
    install(TARGETS manipulation-simulation-camera RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})
endif()

if(MANIPULATION_ENABLE_MUJOCO)
    add_executable(manipulation-simulation capabilities/simulation/server.cpp
        capabilities/simulation/simulation_server.cpp)
    target_include_directories(manipulation-simulation PRIVATE drivers/simulation/mujoco)
    target_link_libraries(manipulation-simulation PRIVATE manipulation_motion manipulation_drivers
        mujoco_sim::world yaml-cpp Threads::Threads)
    if(MANIPULATION_ENABLE_SIMULATION_RENDERER)
        target_sources(manipulation-simulation PRIVATE capabilities/simulation/renderer_bridge.cpp)
        target_link_libraries(manipulation-simulation PRIVATE mujoco_sim::render)
        target_compile_definitions(manipulation-simulation PRIVATE
            MANIPULATION_HAS_SIMULATION_RENDERER=1)
    endif()
    set_target_properties(manipulation-simulation PROPERTIES INSTALL_RPATH
        "$ORIGIN/${MANIPULATION_LIB_FROM_BIN}")
    install(TARGETS manipulation-simulation RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})
endif()

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
    manipulation_kinematics
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
    capabilities/calibration/python/calibration_geometry.py
    capabilities/calibration/python/hand_eye.py
    capabilities/calibration/python/calibration_session.py
    DESTINATION ${MANIPULATION_PYTHON_DIR}/spacemit_manipulation)
install(DIRECTORY descriptions/ DESTINATION ${CMAKE_INSTALL_DATADIR}/manipulation/descriptions)
install(DIRECTORY examples/ DESTINATION ${CMAKE_INSTALL_DATADIR}/manipulation/examples)
install(FILES tools/build_manifest.py tools/check_environment.py tools/configure_device.py
    tools/prepare_simulation_assets.py
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
install(EXPORT manipulationTargets NAMESPACE spacemit:: DESTINATION
    ${CMAKE_INSTALL_LIBDIR}/cmake/manipulation)
configure_file(cmake/manipulation-config.cmake ${CMAKE_CURRENT_BINARY_DIR}/manipulation-config.cmake
    @ONLY)
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/manipulation-config.cmake DESTINATION
    ${CMAKE_INSTALL_LIBDIR}/cmake/manipulation)

if(BUILD_TESTING)
    find_package(Python3 REQUIRED COMPONENTS Interpreter)
    add_executable(manipulation_kinematics_test tests/kinematics_test.cpp)
    target_link_libraries(manipulation_kinematics_test PRIVATE manipulation_kinematics)
    if(MANIPULATION_ENABLE_KINEMATICS_PINOCCHIO OR MANIPULATION_ENABLE_KINEMATICS_EIGEN)
        add_test(NAME manipulation_kinematics_cpp_contract COMMAND manipulation_kinematics_test
            ${CMAKE_CURRENT_SOURCE_DIR}/tests/fixtures/kinematics_chain.urdf
            ${CMAKE_CURRENT_SOURCE_DIR}/descriptions/robots/so101/so101.urdf)
    else()
        add_test(NAME manipulation_kinematics_cpp_contract COMMAND manipulation_kinematics_test)
    endif()
    add_test(NAME manipulation_simulation_assets COMMAND ${Python3_EXECUTABLE}
        ${CMAKE_CURRENT_SOURCE_DIR}/tests/test_simulation_assets.py)
    set_tests_properties(manipulation_simulation_assets PROPERTIES TIMEOUT 30
        ENVIRONMENT "PYTHONDONTWRITEBYTECODE=1")
    set(test_environment
        "PYTHONDONTWRITEBYTECODE=1;MANIPULATION_DEVICE=$<TARGET_FILE:manipulation-device>"
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
    add_executable(manipulation_capability_test tests/capability_test.cpp)
    target_link_libraries(manipulation_capability_test PRIVATE manipulation_motion
        manipulation_drivers yaml-cpp)
    add_test(NAME manipulation_capability_contract COMMAND manipulation_capability_test)
    add_executable(manipulation_composition_test tests/composition_test.cpp)
    target_link_libraries(manipulation_composition_test PRIVATE manipulation_contracts
        manipulation_drivers)
    add_test(NAME manipulation_composition_contract COMMAND manipulation_composition_test)
    add_test(NAME manipulation_composition_cli_contract COMMAND ${Python3_EXECUTABLE}
        ${CMAKE_CURRENT_SOURCE_DIR}/tests/test_composition.py)
    add_test(NAME manipulation_capability_client_contract COMMAND ${Python3_EXECUTABLE}
        ${CMAKE_CURRENT_SOURCE_DIR}/tests/test_capability_client.py)
    set_tests_properties(manipulation_arm_contract manipulation_device_embedded_contract
        manipulation_capability_contract PROPERTIES
        ENVIRONMENT "${test_environment}")
    set_tests_properties(manipulation_composition_contract manipulation_composition_cli_contract
        manipulation_capability_client_contract
        PROPERTIES TIMEOUT 120 ENVIRONMENT "${test_environment}")
    add_test(NAME manipulation_device_contract COMMAND ${Python3_EXECUTABLE}
        ${CMAKE_CURRENT_SOURCE_DIR}/tests/test_device.py)
    add_test(NAME manipulation_tools_contract COMMAND ${Python3_EXECUTABLE}
        ${CMAKE_CURRENT_SOURCE_DIR}/tests/test_tools.py)
    add_test(NAME manipulation_kinematics_contract
        COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/tests/test_kinematics.py)
    set_tests_properties(manipulation_device_contract manipulation_tools_contract
        manipulation_kinematics_contract PROPERTIES
        TIMEOUT 90 ENVIRONMENT "${test_environment}")
    add_executable(manipulation_control_api_test tests/control_api_test.cpp)
    target_link_libraries(manipulation_control_api_test PRIVATE manipulation_motion
        manipulation_drivers)
    add_test(NAME manipulation_control_api_contract COMMAND manipulation_control_api_test)
    add_test(NAME manipulation_control_cli_contract
        COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/tests/test_control_api.py)
    set_tests_properties(manipulation_control_api_contract manipulation_control_cli_contract
        PROPERTIES
        TIMEOUT 90 ENVIRONMENT "${test_environment}")
    if(MANIPULATION_ENABLE_MUJOCO)
        add_executable(manipulation_mujoco_test tests/mujoco_test.cpp)
        target_link_libraries(manipulation_mujoco_test PRIVATE manipulation_motion
            manipulation_drivers)
        add_test(NAME manipulation_mujoco_contract COMMAND manipulation_mujoco_test
            ${CMAKE_CURRENT_SOURCE_DIR}/tests/fixtures/mujoco)
        add_test(NAME manipulation_mujoco_client_contract COMMAND ${Python3_EXECUTABLE}
            ${CMAKE_CURRENT_SOURCE_DIR}/tests/test_mujoco.py)
        add_test(NAME manipulation_mujoco_remote_contract COMMAND ${Python3_EXECUTABLE}
            ${CMAKE_CURRENT_SOURCE_DIR}/tests/test_mujoco_remote.py)
        add_executable(manipulation_mujoco_runtime_test tests/mujoco_runtime_test.cpp)
        target_link_libraries(manipulation_mujoco_runtime_test PRIVATE manipulation_motion
            manipulation_drivers mujoco_sim::world)
        add_test(NAME manipulation_mujoco_runtime_contract COMMAND manipulation_mujoco_runtime_test
            ${CMAKE_CURRENT_SOURCE_DIR}/tests/fixtures/mujoco)
        set_tests_properties(manipulation_mujoco_runtime_contract PROPERTIES TIMEOUT 30
            ENVIRONMENT "${test_environment}")
        set_tests_properties(manipulation_mujoco_contract manipulation_mujoco_client_contract
            PROPERTIES TIMEOUT 120 ENVIRONMENT "${test_environment}")
        set_tests_properties(manipulation_mujoco_remote_contract PROPERTIES TIMEOUT 150
            ENVIRONMENT "${test_environment};MANIPULATION_SIMULATION=$<TARGET_FILE:manipulation-simulation>")
        if(MANIPULATION_ENABLE_SIMULATION_RENDERER AND NOT MUJOCO_RENDER_USE_GL4ES)
            find_program(MANIPULATION_XVFB_RUN xvfb-run)
            if(MANIPULATION_XVFB_RUN)
                add_test(NAME manipulation_simulation_render_contract
                    COMMAND ${MANIPULATION_XVFB_RUN} -a ${Python3_EXECUTABLE}
                        ${CMAKE_CURRENT_SOURCE_DIR}/tests/test_simulation_render.py)
                set_tests_properties(manipulation_simulation_render_contract PROPERTIES TIMEOUT 60
                    ENVIRONMENT "${test_environment};MANIPULATION_SIMULATION=$<TARGET_FILE:manipulation-simulation>;MANIPULATION_SIMULATION_CAMERA=$<TARGET_FILE:manipulation-simulation-camera>")
            endif()
        endif()
    endif()
    if(MANIPULATION_ENABLE_MUJOCO OR MANIPULATION_ENABLE_MUJOCO_REMOTE)
        add_test(NAME manipulation_simulation_driver_contract COMMAND ${Python3_EXECUTABLE}
            ${CMAKE_CURRENT_SOURCE_DIR}/tests/test_simulation_drivers.py)
        set_tests_properties(manipulation_simulation_driver_contract PROPERTIES TIMEOUT 30
            ENVIRONMENT "${test_environment};MANIPULATION_TEST_MUJOCO=$<BOOL:${MANIPULATION_ENABLE_MUJOCO}>")
        add_test(NAME manipulation_simulation_camera_contract COMMAND ${Python3_EXECUTABLE}
            ${CMAKE_CURRENT_SOURCE_DIR}/tests/test_simulation_camera.py)
        set_tests_properties(manipulation_simulation_camera_contract PROPERTIES TIMEOUT 60
            ENVIRONMENT "PYTHONDONTWRITEBYTECODE=1;MANIPULATION_SIMULATION_CAMERA=$<TARGET_FILE:manipulation-simulation-camera>")
        add_test(NAME manipulation_mujoco_transport_contract COMMAND ${Python3_EXECUTABLE}
            ${CMAKE_CURRENT_SOURCE_DIR}/tests/test_mujoco_transport.py)
        set_tests_properties(manipulation_mujoco_transport_contract PROPERTIES TIMEOUT 120
            ENVIRONMENT "${test_environment}")
    endif()
    if(MANIPULATION_ENABLE_SO101)
        add_test(NAME manipulation_so101_contract
            COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/tests/test_so101.py)
        add_test(NAME manipulation_so101_calibration_contract
            COMMAND ${Python3_EXECUTABLE}
                ${CMAKE_CURRENT_SOURCE_DIR}/tests/test_so101_calibration.py)
        set(so101_environment "${test_environment}"
            "MANIPULATION_FEETECH_BUS=$<TARGET_FILE:manipulation_feetech_bus>")
        set_tests_properties(manipulation_so101_contract manipulation_so101_calibration_contract
            manipulation_composition_cli_contract
            PROPERTIES TIMEOUT 90 ENVIRONMENT "${so101_environment}")
    endif()
    if(MANIPULATION_ENABLE_NERO)
        add_executable(manipulation_recovery_test tests/recovery_test.cpp)
        target_link_libraries(manipulation_recovery_test PRIVATE manipulation_motion
            manipulation_drivers)
        add_test(NAME manipulation_nero_worker_contract
            COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/tests/test_nero.py)
        set(nero_environment "${test_environment}"
            "MANIPULATION_RECOVERY=$<TARGET_FILE:manipulation_recovery_test>")
        set_tests_properties(manipulation_nero_worker_contract PROPERTIES TIMEOUT 120
            ENVIRONMENT "${nero_environment}")
    endif()
    if(MANIPULATION_ENABLE_CALIBRATION)
        add_test(NAME manipulation_calibration_session_contract COMMAND ${Python3_EXECUTABLE}
            ${CMAKE_CURRENT_SOURCE_DIR}/tests/test_calibration_session.py)
        set_tests_properties(manipulation_calibration_session_contract PROPERTIES TIMEOUT 120
            ENVIRONMENT "PYTHONDONTWRITEBYTECODE=1;PYTHONNOUSERSITE=1")
        add_executable(manipulation_calibration_test tests/calibration_test.cpp)
        target_include_directories(manipulation_calibration_test PRIVATE ${OpenCV_INCLUDE_DIRS})
        target_link_libraries(manipulation_calibration_test PRIVATE manipulation_calibration
            ${OpenCV_LIBS})
        add_test(NAME manipulation_calibration_contract COMMAND manipulation_calibration_test)
        add_test(NAME manipulation_charuco_contract
            COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/tests/test_charuco.py)
        set_tests_properties(manipulation_charuco_contract PROPERTIES TIMEOUT 60 ENVIRONMENT
            "PYTHONDONTWRITEBYTECODE=1")
    endif()
endif()
