target_sources(manipulation_drivers PRIVATE nero_arm.cpp
    ../../common/agx_session/worker_channel.cpp ../../common/agx_session/device_driver.cpp
    ../../effectors/revo2/revo2.cpp)
target_include_directories(manipulation_drivers PRIVATE ../../common/agx_session)
target_link_libraries(manipulation_drivers PRIVATE yaml-cpp)
manipulation_register_device(RegisterAgxDevice)
install(FILES ../../common/agx_session/agx_worker.py
    DESTINATION ${CMAKE_INSTALL_LIBEXECDIR}/manipulation)
