target_sources(manipulation_drivers PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/device_driver.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/mapping_contract.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/wire_protocol.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/remote_session.cpp)
target_link_libraries(manipulation_drivers PRIVATE yaml-cpp Threads::Threads)
if(MANIPULATION_ENABLE_MUJOCO)
    target_sources(manipulation_drivers PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/mujoco_session.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/mujoco_runtime.cpp)
    target_compile_definitions(manipulation_drivers PRIVATE MANIPULATION_HAS_MUJOCO_LOCAL=1)
    target_link_libraries(manipulation_drivers PRIVATE mujoco_sim::world)
else()
    target_compile_definitions(manipulation_drivers PRIVATE MANIPULATION_HAS_MUJOCO_LOCAL=0)
endif()
manipulation_register_device(RegisterMujocoDevice)
