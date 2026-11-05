# Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
# SPDX-License-Identifier: Apache-2.0
# Shared so that so101-calibrate can load it; it carries its own copy of the motor driver.
add_library(manipulation_feetech_bus SHARED ../../common/so101_session/feetech_bus.cpp)
target_compile_features(manipulation_feetech_bus PRIVATE cxx_std_17)
target_link_libraries(manipulation_feetech_bus PRIVATE ${MANIPULATION_MOTOR_TARGET})
# The driver registers itself from a static constructor.
target_link_options(manipulation_feetech_bus PRIVATE
    "LINKER:--whole-archive,$<TARGET_FILE:${MANIPULATION_MOTOR_TARGET}>,--no-whole-archive"
    "LINKER:--exclude-libs,libmotor.a")
set_target_properties(manipulation_feetech_bus PROPERTIES VERSION 1.0.0 SOVERSION 1)
install(TARGETS manipulation_feetech_bus EXPORT manipulationTargets
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR})
target_sources(manipulation_drivers PRIVATE so101_arm.cpp
    ../../common/so101_session/so101_session.cpp ../../common/so101_session/device_driver.cpp
    ../../effectors/so101_gripper/so101_gripper.cpp)
target_include_directories(manipulation_drivers PRIVATE ../../common/so101_session)
target_link_libraries(manipulation_drivers PRIVATE manipulation_feetech_bus yaml-cpp)
manipulation_register_device(RegisterSo101Device)
install(PROGRAMS tools/calibrate.py DESTINATION ${CMAKE_INSTALL_BINDIR} RENAME so101-calibrate)
