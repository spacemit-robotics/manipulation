# Standalone registration module; selected with MANIPULATION_EXTRA_DRIVER_DIRS.
target_sources(manipulation_drivers PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/bench_driver.cpp)
manipulation_register_device(RegisterBenchDevice)
