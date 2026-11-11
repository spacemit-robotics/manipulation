# Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
# SPDX-License-Identifier: Apache-2.0
cmake_minimum_required(VERSION 3.16)
project(manipulation_mujoco VERSION 1.0.0 LANGUAGES CXX)

set(CMAKE_POSITION_INDEPENDENT_CODE ON)
set(CMAKE_BUILD_RPATH_USE_ORIGIN ON)
if(NOT CMAKE_SYSTEM_PROCESSOR MATCHES "x86_64|amd64|AMD64|riscv64|riscv")
    message(FATAL_ERROR "MuJoCo physics is not supported on ${CMAKE_SYSTEM_PROCESSOR}")
endif()

include(cmake/FindMuJoCo.cmake)
include(CTest)
include(GNUInstallDirs)
include(CMakePackageConfigHelpers)
option(MUJOCO_BUILD_RENDERER "Build snapshot RGB-D and viewer support" OFF)

add_library(mujoco_world SHARED src/mujoco_world.cpp)
add_library(mujoco_sim::world ALIAS mujoco_world)
set_target_properties(mujoco_world PROPERTIES
    EXPORT_NAME world
    INSTALL_RPATH "\$ORIGIN"
)
target_compile_features(mujoco_world PUBLIC cxx_std_17)
target_include_directories(mujoco_world
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
    PRIVATE ${MUJOCO_INCLUDE_DIR}
)
target_link_libraries(mujoco_world PRIVATE ${MUJOCO_LIB})

if(BUILD_TESTING)
    add_executable(test_mujoco_world tests/test_mujoco_world.cpp)
    target_link_libraries(test_mujoco_world PRIVATE mujoco_world)
    add_test(NAME mujoco_world
        COMMAND test_mujoco_world ${CMAKE_CURRENT_SOURCE_DIR}/tests)
    set_tests_properties(mujoco_world PROPERTIES TIMEOUT 20)
endif()

install(TARGETS mujoco_world EXPORT mujoco_world-targets
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)
install(FILES include/mujoco_world.h include/mujoco_native_snapshot.h
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
install(EXPORT mujoco_world-targets
    FILE mujoco_world-targets.cmake
    NAMESPACE mujoco_sim::
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/mujoco_world
)
configure_package_config_file(cmake/mujoco_world-config.cmake.in
    ${CMAKE_CURRENT_BINARY_DIR}/mujoco_world-config.cmake
    INSTALL_DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/mujoco_world
)
write_basic_package_version_file(
    ${CMAKE_CURRENT_BINARY_DIR}/mujoco_world-config-version.cmake
    VERSION 1.0.0 COMPATIBILITY SameMajorVersion
)
install(FILES
    ${CMAKE_CURRENT_BINARY_DIR}/mujoco_world-config.cmake
    ${CMAKE_CURRENT_BINARY_DIR}/mujoco_world-config-version.cmake
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/mujoco_world
)

# Both libraries resolve this vendored runtime through their relative RPATH.
get_filename_component(_mujoco_library_dir "${MUJOCO_LIB}" DIRECTORY)
file(GLOB _mujoco_so_files "${_mujoco_library_dir}/libmujoco.so*")
install(FILES ${_mujoco_so_files} DESTINATION ${CMAKE_INSTALL_LIBDIR})
install(FILES LICENSE NOTICE DESTINATION ${CMAKE_INSTALL_DATADIR}/licenses/mujoco_world)
get_filename_component(_mujoco_root "${MUJOCO_INCLUDE_DIR}" DIRECTORY)
find_file(MUJOCO_THIRD_PARTY_NOTICES
    NAMES THIRD_PARTY_NOTICES
    HINTS "${_mujoco_root}" "${_mujoco_root}/share/mujoco"
    NO_DEFAULT_PATH
)
if(NOT EXISTS "${MUJOCO_THIRD_PARTY_NOTICES}")
    # The SDK's RISC-V 3.4.0 binary bundle omits the release-generated notices.
    file(STRINGS "${MUJOCO_INCLUDE_DIR}/mujoco/mujoco.h" _mujoco_header_version
        REGEX "^#define mjVERSION_HEADER 340$")
    if(_mujoco_header_version)
        set(MUJOCO_THIRD_PARTY_NOTICES
            "${CMAKE_CURRENT_SOURCE_DIR}/licenses/mujoco-3.4.0/THIRD_PARTY_NOTICES")
    endif()
endif()
if(NOT EXISTS "${MUJOCO_THIRD_PARTY_NOTICES}")
    message(FATAL_ERROR
        "Bundling libmujoco requires THIRD_PARTY_NOTICES; set MUJOCO_THIRD_PARTY_NOTICES")
endif()
install(FILES "${MUJOCO_THIRD_PARTY_NOTICES}"
    DESTINATION ${CMAKE_INSTALL_DATADIR}/licenses/mujoco_world/mujoco)

if(BUILD_TESTING)
    add_executable(test_mujoco_snapshot tests/test_mujoco_snapshot.cpp)
    target_include_directories(test_mujoco_snapshot PRIVATE ${MUJOCO_INCLUDE_DIR})
    target_link_libraries(test_mujoco_snapshot PRIVATE mujoco_world ${MUJOCO_LIB})
    add_test(NAME mujoco_snapshot
        COMMAND test_mujoco_snapshot ${CMAKE_CURRENT_SOURCE_DIR}/tests)
    set_tests_properties(mujoco_snapshot PROPERTIES TIMEOUT 20)
endif()

if(MUJOCO_BUILD_RENDERER)
    set(_render_backend_default GLFW)
    set(_render_gl4es_default OFF)
    if(CMAKE_SYSTEM_PROCESSOR MATCHES "riscv64|riscv")
        set(_render_backend_default X11)
        set(_render_gl4es_default ON)
    endif()
    set(MUJOCO_RENDER_BACKEND "${_render_backend_default}" CACHE STRING "GLFW or X11")
    set_property(CACHE MUJOCO_RENDER_BACKEND PROPERTY STRINGS GLFW X11)
    option(MUJOCO_RENDER_USE_GL4ES "Use gl4es with the native X11/GLX renderer" ${_render_gl4es_default})
    if(NOT MUJOCO_RENDER_BACKEND MATCHES "^(GLFW|X11)$")
        message(FATAL_ERROR "MUJOCO_RENDER_BACKEND must be GLFW or X11")
    endif()
    if(MUJOCO_RENDER_BACKEND STREQUAL "GLFW" AND MUJOCO_RENDER_USE_GL4ES)
        message(FATAL_ERROR "GLFW is incompatible with gl4es; select MUJOCO_RENDER_BACKEND=X11")
    endif()
    add_library(mujoco_render SHARED src/mujoco_render.cpp src/mujoco_render_context.cpp)
    add_library(mujoco_sim::render ALIAS mujoco_render)
    set_target_properties(mujoco_render PROPERTIES EXPORT_NAME render INSTALL_RPATH "\$ORIGIN")
    target_compile_features(mujoco_render PUBLIC cxx_std_17)
    target_compile_definitions(mujoco_render INTERFACE
        MUJOCO_RENDER_USES_GL4ES=$<BOOL:${MUJOCO_RENDER_USE_GL4ES}>)
    target_include_directories(mujoco_render
        PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
            $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
        PRIVATE ${MUJOCO_INCLUDE_DIR})
    target_link_libraries(mujoco_render PUBLIC mujoco_world PRIVATE ${MUJOCO_LIB})
    if(MUJOCO_RENDER_BACKEND STREQUAL "X11")
        find_package(X11 REQUIRED)
        target_compile_definitions(mujoco_render PRIVATE MUJOCO_RENDER_CONTEXT_X11=1)
        target_include_directories(mujoco_render PRIVATE ${X11_INCLUDE_DIR})
        target_link_libraries(mujoco_render PRIVATE ${X11_LIBRARIES})
    else()
        find_package(glfw3 REQUIRED)
        target_link_libraries(mujoco_render PRIVATE glfw)
    endif()
    if(MUJOCO_RENDER_USE_GL4ES)
        include(cmake/FindGL4ES.cmake)
        target_compile_definitions(mujoco_render PRIVATE MUJOCO_RENDER_GL4ES=1)
        target_include_directories(mujoco_render PRIVATE ${GL4ES_INCLUDE_DIR})
        target_link_libraries(mujoco_render PRIVATE ${GL4ES_LIB})
        set_property(TARGET mujoco_render APPEND PROPERTY
            INSTALL_RPATH "\$ORIGIN/mujoco_world/gl4es")
        _gl4es_install_runtime("${CMAKE_INSTALL_LIBDIR}/mujoco_world/gl4es")
        find_file(MUJOCO_RENDER_GL4ES_LICENSE NAMES LICENSE
            HINTS "${GL4ES_ROOT}" "${GL4ES_ROOT}/share/licenses/gl4es" NO_DEFAULT_PATH)
        if(NOT EXISTS "${MUJOCO_RENDER_GL4ES_LICENSE}")
            message(FATAL_ERROR "Bundling gl4es requires MUJOCO_RENDER_GL4ES_LICENSE")
        endif()
        install(FILES "${MUJOCO_RENDER_GL4ES_LICENSE}"
            DESTINATION ${CMAKE_INSTALL_DATADIR}/licenses/mujoco_world/gl4es)
    else()
        find_package(OpenGL REQUIRED)
        target_link_libraries(mujoco_render PRIVATE OpenGL::GL)
    endif()
    install(TARGETS mujoco_render EXPORT mujoco_world-targets
        LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
        ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
        RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})
    install(FILES include/mujoco_render.h DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
    if(BUILD_TESTING)
        find_package(Threads REQUIRED)
        add_executable(test_mujoco_rgbd tests/test_mujoco_rgbd.cpp)
        target_link_libraries(test_mujoco_rgbd PRIVATE mujoco_render Threads::Threads)
        find_program(MUJOCO_XVFB_RUN xvfb-run)
        if(MUJOCO_XVFB_RUN)
            add_test(NAME mujoco_rgbd COMMAND ${MUJOCO_XVFB_RUN} -a
                $<TARGET_FILE:test_mujoco_rgbd> ${CMAKE_CURRENT_SOURCE_DIR}/tests)
            set_tests_properties(mujoco_rgbd PROPERTIES TIMEOUT 60)
        else()
            message(STATUS "MuJoCo RGB-D test built but not registered: xvfb-run unavailable")
        endif()
    endif()
endif()
