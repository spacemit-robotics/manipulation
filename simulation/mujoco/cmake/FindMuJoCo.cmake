# FindMuJoCo.cmake
#
# 查找 MuJoCo 头文件和库；找不到时自动拉取 prebuilt release 到 ~/.cache/thirdparty/。
#
# 输入变量（可选）:
#   MUJOCO_DIR  — MuJoCo 安装根目录（CMake 变量或同名环境变量）
#
# 输出变量:
#   MUJOCO_INCLUDE_DIR  — 头文件目录（含 mujoco/mujoco.h）
#   MUJOCO_LIB          — libmujoco 路径
#
# 查找优先级（先检测、再 fetch 兜底）:
#   1. -DMUJOCO_DIR=...
#   2. 环境变量 MUJOCO_DIR
#   3. /usr/local、/opt/mujoco
#   4. ~/.cache/thirdparty/mujoco/mujoco-<版本>/（之前 fetch 留下的；版本随架构而定）
#   5. 兜底：触发 fetch_thirdparty 拉取并解压到 cache，再次 find（x86_64 取官方包，riscv64 取 SpaceMIT archive 包）

# 预编译包版本统一 3.4.0，来源按架构区分（包内布局一致：include/mujoco + lib/libmujoco.so）：
#   x86_64  → MuJoCo 官方 GitHub release
#   riscv64 → SpaceMIT bianbu26 prebuilt_libs（官方无 riscv64 release）
set(_MJ_VERSION "3.4.0")
set(_MJ_RELEASE "mujoco-${_MJ_VERSION}")
if(CMAKE_SYSTEM_PROCESSOR MATCHES "riscv64|riscv")
    set(_MJ_URL "https://archive.spacemit.com/ros2/prebuilt_libs/bianbu26/opt/ext/mujoco/${_MJ_RELEASE}.tar.gz")
else()
    set(_MJ_URL "https://github.com/google-deepmind/mujoco/releases/download/${_MJ_VERSION}/${_MJ_RELEASE}-linux-x86_64.tar.gz")
endif()

function(_mujoco_find_in_hints out_inc out_lib)
    find_path(_mj_inc
        NAMES mujoco/mujoco.h
        HINTS ${ARGN}
        PATH_SUFFIXES include
        NO_DEFAULT_PATH
    )
    find_library(_mj_lib
        NAMES mujoco
        HINTS ${ARGN}
        PATH_SUFFIXES lib
        NO_DEFAULT_PATH
    )
    set(${out_inc} "${_mj_inc}" PARENT_SCOPE)
    set(${out_lib} "${_mj_lib}" PARENT_SCOPE)
endfunction()

# ---- 步骤 1：组装 hints（含 cache 路径） ----
set(_mujoco_hints "")
if(DEFINED MUJOCO_DIR)
    list(APPEND _mujoco_hints "${MUJOCO_DIR}")
endif()
if(DEFINED ENV{MUJOCO_DIR})
    list(APPEND _mujoco_hints "$ENV{MUJOCO_DIR}")
endif()
list(APPEND _mujoco_hints /usr/local /opt/mujoco)
if(DEFINED ENV{HOME})
    list(APPEND _mujoco_hints "$ENV{HOME}/.cache/thirdparty/mujoco/${_MJ_RELEASE}")
endif()

# ---- 步骤 2：先 find 一次（命中预装 / 已 fetch 的 cache） ----
_mujoco_find_in_hints(MUJOCO_INCLUDE_DIR MUJOCO_LIB ${_mujoco_hints})

# ---- 步骤 3：找不到 → 触发自动 fetch（按架构拉对应预编译包，见顶部 _MJ_URL） ----
if(NOT MUJOCO_INCLUDE_DIR OR NOT MUJOCO_LIB)
    include("${CMAKE_CURRENT_LIST_DIR}/FetchThirdParty.cmake")
    fetch_thirdparty(
        NAME mujoco
        ARCHIVE_URL "${_MJ_URL}"
        ARCHIVE_SUBDIR "${_MJ_RELEASE}"
        OUT_SOURCE_DIR _mj_fetched_dir
    )
    unset(MUJOCO_INCLUDE_DIR CACHE)
    unset(MUJOCO_LIB CACHE)
    _mujoco_find_in_hints(MUJOCO_INCLUDE_DIR MUJOCO_LIB "${_mj_fetched_dir}" ${_mujoco_hints})
endif()

# ---- 步骤 4：仍找不到 → fatal ----
if(NOT MUJOCO_INCLUDE_DIR OR NOT MUJOCO_LIB)
    message(FATAL_ERROR
        "MuJoCo not found.\n"
        "  网络不通时手动下载 ${_MJ_URL}\n"
        "  解压后 export MUJOCO_DIR=/path/to/${_MJ_RELEASE}\n"
        "  或编译时显式指定: cmake .. -DMUJOCO_DIR=/path/to/${_MJ_RELEASE}")
endif()

message(STATUS "MuJoCo: ${MUJOCO_LIB}")
message(STATUS "  includes: ${MUJOCO_INCLUDE_DIR}")
