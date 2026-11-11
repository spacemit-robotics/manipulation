# FindGL4ES.cmake
#
# 查找 gl4es 头文件和库；找不到时自动 git clone 固定 commit + 打补丁 + 编译到 ~/.cache/thirdparty/。
#
# 输入变量（可选）:
#   GL4ES_ROOT  — gl4es 根目录（CMake 变量或同名环境变量）
#
# 输出变量:
#   GL4ES_ROOT         — gl4es 根目录
#   GL4ES_INCLUDE_DIR  — 头文件目录（含 GL/gl.h、GL/glx.h）
#   GL4ES_LIB          — libGL.so 路径
#
# 查找优先级（先检测、再 fetch+build 兜底）:
#   1. -DGL4ES_ROOT=...
#   2. 环境变量 GL4ES_ROOT
#   3. ~/.cache/thirdparty/gl4es/（之前 fetch+build 留下的）
#   4. 兜底：触发 fetch_thirdparty 拉源码 + apply 补丁 + cmake/make，再次 find

set(_GL4ES_REPO "https://github.com/ptitSeb/gl4es.git")
set(_GL4ES_COMMIT "e6bb082b495820b308d34b9e1338bc87bfa8e2fa")  # 固定依赖版本
set(_GL4ES_PATCH "${CMAKE_CURRENT_LIST_DIR}/gl4es-imgtec-stencil.patch")

function(_gl4es_find_in_hints out_inc out_lib)
    find_path(_g4_inc
        NAMES GL/gl.h
        HINTS ${ARGN}
        PATH_SUFFIXES include
        NO_DEFAULT_PATH
    )
    find_library(_g4_lib
        NAMES GL
        HINTS ${ARGN}
        PATH_SUFFIXES lib
        NO_DEFAULT_PATH
    )
    set(${out_inc} "${_g4_inc}" PARENT_SCOPE)
    set(${out_lib} "${_g4_lib}" PARENT_SCOPE)
endfunction()

function(_gl4es_install_runtime destination)
    file(GLOB _gl4es_shared_libraries "${GL4ES_ROOT}/lib/libGL.so*")
    install(FILES ${_gl4es_shared_libraries} DESTINATION "${destination}")
    foreach(_library IN LISTS _gl4es_shared_libraries)
        if(NOT IS_SYMLINK "${_library}")
            get_filename_component(_name "${_library}" NAME)
            if(IS_ABSOLUTE "${destination}")
                set(_installed "\$ENV{DESTDIR}${destination}/${_name}")
            else()
                set(_installed "\$ENV{DESTDIR}\${CMAKE_INSTALL_PREFIX}/${destination}/${_name}")
            endif()
            # Only sanitize the installed vendor copy; SDK targets retain their $ORIGIN paths.
            install(CODE "file(RPATH_REMOVE FILE \"${_installed}\")")
        endif()
    endforeach()
endfunction()

# ---- 步骤 1：组装 hints（含 cache 路径） ----
set(_gl4es_hints "")
if(DEFINED GL4ES_ROOT)
    list(APPEND _gl4es_hints "${GL4ES_ROOT}")
endif()
if(DEFINED ENV{GL4ES_ROOT})
    list(APPEND _gl4es_hints "$ENV{GL4ES_ROOT}")
endif()
if(DEFINED ENV{HOME})
    list(APPEND _gl4es_hints "$ENV{HOME}/.cache/thirdparty/gl4es")
endif()

# ---- 步骤 2：先 find 一次（命中预装 / 已 fetch+build 的 cache） ----
_gl4es_find_in_hints(GL4ES_INCLUDE_DIR GL4ES_LIB ${_gl4es_hints})

# ---- 步骤 3：找不到 → git clone 固定 commit + 打补丁 + 编译 ----
if(NOT GL4ES_INCLUDE_DIR OR NOT GL4ES_LIB)
    include("${CMAKE_CURRENT_LIST_DIR}/FetchThirdParty.cmake")
    fetch_thirdparty(
        NAME gl4es
        GIT_REPO "${_GL4ES_REPO}"
        GIT_COMMIT "${_GL4ES_COMMIT}"
        OUT_SOURCE_DIR _gl4es_src
    )

    if(NOT EXISTS "${_gl4es_src}/lib/libGL.so.1")
        # IMGTEC stencil 补丁（幂等：reverse --check 成功说明已应用，跳过）
        execute_process(
            COMMAND git apply --reverse --check "${_GL4ES_PATCH}"
            WORKING_DIRECTORY "${_gl4es_src}"
            RESULT_VARIABLE _g4_applied
            OUTPUT_QUIET ERROR_QUIET
        )
        if(NOT _g4_applied EQUAL 0)
            execute_process(
                COMMAND git apply "${_GL4ES_PATCH}"
                WORKING_DIRECTORY "${_gl4es_src}"
                RESULT_VARIABLE _g4_patch_res
            )
            if(NOT _g4_patch_res EQUAL 0)
                message(FATAL_ERROR "gl4es: 应用补丁失败：${_GL4ES_PATCH}")
            endif()
        endif()

        # 编译（全默认选项，gl4es 把产物输出到 <src>/lib/libGL.so.1）
        include(ProcessorCount)
        processorcount(_g4_nproc)
        if(_g4_nproc EQUAL 0)
            set(_g4_nproc 4)
        endif()
        message(STATUS "gl4es: 编译中（${_gl4es_src}，-j${_g4_nproc}）...")
        execute_process(
            COMMAND ${CMAKE_COMMAND} -S "${_gl4es_src}" -B "${_gl4es_src}/build" -DCMAKE_BUILD_TYPE=Release
            RESULT_VARIABLE _g4_cfg
        )
        if(_g4_cfg EQUAL 0)
            execute_process(
                COMMAND ${CMAKE_COMMAND} --build "${_gl4es_src}/build" --parallel ${_g4_nproc}
                RESULT_VARIABLE _g4_bld
            )
        endif()
        if(NOT _g4_cfg EQUAL 0 OR NOT _g4_bld EQUAL 0)
            message(FATAL_ERROR "gl4es: 编译失败（configure=${_g4_cfg} build=${_g4_bld}）")
        endif()
    endif()

    # gl4es 的 CMake 只产出带 SONAME 的实体 libGL.so.1，不产出 libGL.so 开发符号链接；
    # find_library(NAMES GL) 与链接需要 libGL.so，补一个相对符号链接。
    if(EXISTS "${_gl4es_src}/lib/libGL.so.1" AND NOT EXISTS "${_gl4es_src}/lib/libGL.so")
        execute_process(COMMAND ${CMAKE_COMMAND} -E create_symlink libGL.so.1 "${_gl4es_src}/lib/libGL.so")
    endif()

    unset(GL4ES_INCLUDE_DIR CACHE)
    unset(GL4ES_LIB CACHE)
    _gl4es_find_in_hints(GL4ES_INCLUDE_DIR GL4ES_LIB "${_gl4es_src}" ${_gl4es_hints})
endif()

# ---- 步骤 4：仍找不到 → fatal ----
if(NOT GL4ES_INCLUDE_DIR OR NOT GL4ES_LIB)
    message(FATAL_ERROR
        "gl4es 未找到 / 编译失败。\n"
        "  手动构建：git clone ${_GL4ES_REPO} && cd gl4es && git checkout ${_GL4ES_COMMIT}\n"
        "    && git apply ${_GL4ES_PATCH} && cmake -S . -B build -DCMAKE_BUILD_TYPE=Release && cmake --build build\n"
        "  或 -DGL4ES_ROOT=/path/to/gl4es 指向已编好的目录")
endif()

# GL4ES_ROOT = 含 include/ 与 lib/ 的根目录（供安装私有 gl4es 运行库用）
get_filename_component(GL4ES_ROOT "${GL4ES_INCLUDE_DIR}" DIRECTORY)

message(STATUS "gl4es: ${GL4ES_LIB}")
message(STATUS "  includes: ${GL4ES_INCLUDE_DIR}")
