# Component-owned gl4es dependency. GL4ES_ROOT (CMake, then environment) may
# select a previously built dependency, including a relocated copy. Otherwise
# fetch into SROBOTIS_THIRDPARTY_CACHE (or the FetchThirdParty default cache).
# Every accepted library is bound to the pinned source and patches by a build
# receipt containing their fingerprint and the actual library's SHA-256.

set(_GL4ES_REPO "https://github.com/ptitSeb/gl4es.git")
set(_GL4ES_COMMIT "e6bb082b495820b308d34b9e1338bc87bfa8e2fa")
set(_GL4ES_PATCHES
    "${CMAKE_CURRENT_LIST_DIR}/gl4es-imgtec-stencil.patch"
    "${CMAKE_CURRENT_LIST_DIR}/gl4es-empty-texture-mipmap.patch"
    "${CMAKE_CURRENT_LIST_DIR}/gl4es-buffer-queries.patch")
set(_gl4es_identity "${_GL4ES_COMMIT};${CMAKE_SYSTEM_NAME};${CMAKE_SYSTEM_PROCESSOR}")
foreach(_patch IN LISTS _GL4ES_PATCHES)
    file(SHA256 "${_patch}" _patch_hash)
    string(APPEND _gl4es_identity ";${_patch_hash}")
endforeach()
string(SHA256 _GL4ES_FINGERPRINT "${_gl4es_identity}")
set(_GL4ES_RECEIPT ".manipulation-gl4es-build")

function(_gl4es_find root out_inc out_lib)
    # Old FindGL4ES versions cached these private names across root changes.
    unset(_g4_inc CACHE)
    unset(_g4_lib CACHE)
    set(_g4_inc "_g4_inc-NOTFOUND")
    set(_g4_lib "_g4_lib-NOTFOUND")
    find_path(_g4_inc NAMES GL/gl.h HINTS "${root}"
        PATH_SUFFIXES include NO_DEFAULT_PATH)
    find_library(_g4_lib NAMES GL HINTS "${root}"
        PATH_SUFFIXES lib NO_DEFAULT_PATH)
    set(${out_inc} "${_g4_inc}" PARENT_SCOPE)
    set(${out_lib} "${_g4_lib}" PARENT_SCOPE)
    unset(_g4_inc CACHE)
    unset(_g4_lib CACHE)
endfunction()

function(_gl4es_verified root library out_valid)
    set(_valid FALSE)
    if(EXISTS "${library}" AND EXISTS "${root}/${_GL4ES_RECEIPT}")
        file(READ "${root}/${_GL4ES_RECEIPT}" _receipt)
        file(SHA256 "${library}" _library_hash)
        if(_receipt STREQUAL "${_GL4ES_FINGERPRINT}\n${_library_hash}\n")
            set(_valid TRUE)
        endif()
    endif()
    set(${out_valid} "${_valid}" PARENT_SCOPE)
endfunction()

function(_gl4es_apply_patches source)
    foreach(_patch IN LISTS _GL4ES_PATCHES)
        execute_process(COMMAND git apply --reverse --check "${_patch}"
            WORKING_DIRECTORY "${source}" RESULT_VARIABLE _applied
            OUTPUT_QUIET ERROR_QUIET)
        if(NOT _applied EQUAL 0)
            execute_process(COMMAND git apply "${_patch}"
                WORKING_DIRECTORY "${source}" RESULT_VARIABLE _result)
            if(NOT _result EQUAL 0)
                message(FATAL_ERROR "gl4es: failed to apply ${_patch} in ${source}")
            endif()
        endif()
    endforeach()
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

set(_gl4es_root "")
if(DEFINED GL4ES_ROOT AND NOT GL4ES_ROOT STREQUAL "")
    set(_gl4es_root "${GL4ES_ROOT}")
elseif(DEFINED ENV{GL4ES_ROOT} AND NOT "$ENV{GL4ES_ROOT}" STREQUAL "")
    set(_gl4es_root "$ENV{GL4ES_ROOT}")
endif()
unset(GL4ES_INCLUDE_DIR CACHE)
unset(GL4ES_LIB CACHE)

if(NOT _gl4es_root STREQUAL "")
    _gl4es_find("${_gl4es_root}" GL4ES_INCLUDE_DIR GL4ES_LIB)
    _gl4es_verified("${_gl4es_root}" "${GL4ES_LIB}" _verified)
    if(NOT GL4ES_INCLUDE_DIR OR NOT _verified)
        message(FATAL_ERROR
            "GL4ES_ROOT=${_gl4es_root} is not a verified manipulation gl4es build.\n"
            "The build receipt must match the required patches and the library SHA-256.\n"
            "Run `unset GL4ES_ROOT` to clear the environment override, then reconfigure:\n"
            "  cmake -U GL4ES_ROOT -S <source> -B <build> <options>.\n"
            "Set SROBOTIS_THIRDPARTY_CACHE to choose a custom cache location.\n"
            "GL4ES_ROOT may then select that complete build directory or a relocated copy;\n"
            "preserve ${_GL4ES_RECEIPT}, include/ and lib/ together. The supplied root was not modified.")
    endif()
else()
    include("${CMAKE_CURRENT_LIST_DIR}/FetchThirdParty.cmake")
    # Never reuse or modify the shared humanoid gl4es cache.
    fetch_thirdparty(NAME "manipulation-gl4es-${_GL4ES_FINGERPRINT}"
        GIT_REPO "${_GL4ES_REPO}" GIT_COMMIT "${_GL4ES_COMMIT}"
        OUT_SOURCE_DIR _gl4es_root)
    _gl4es_apply_patches("${_gl4es_root}")
    _gl4es_verified("${_gl4es_root}" "${_gl4es_root}/lib/libGL.so.1" _verified)
    if(NOT _verified)
        file(REMOVE "${_gl4es_root}/${_GL4ES_RECEIPT}")
        # A replaced library may be newer than all objects; force an actual relink
        # before recording its provenance. Explicit GL4ES_ROOT is never modified.
        file(REMOVE "${_gl4es_root}/lib/libGL.so.1")
        include(ProcessorCount)
        processorcount(_nproc)
        if(_nproc EQUAL 0)
            set(_nproc 4)
        endif()
        message(STATUS "gl4es: building patched dependency at ${_gl4es_root}")
        execute_process(COMMAND "${CMAKE_COMMAND}" -S "${_gl4es_root}"
            -B "${_gl4es_root}/build" -DCMAKE_BUILD_TYPE=Release
            RESULT_VARIABLE _configured)
        set(_built "not run")
        if(_configured EQUAL 0)
            execute_process(COMMAND "${CMAKE_COMMAND}" --build "${_gl4es_root}/build"
                --parallel "${_nproc}" RESULT_VARIABLE _built)
        endif()
        if(NOT _configured EQUAL 0 OR NOT _built EQUAL 0
                OR NOT EXISTS "${_gl4es_root}/lib/libGL.so.1")
            message(FATAL_ERROR
                "gl4es: build failed (configure=${_configured}, build=${_built})")
        endif()
        file(SHA256 "${_gl4es_root}/lib/libGL.so.1" _library_hash)
        file(WRITE "${_gl4es_root}/${_GL4ES_RECEIPT}"
            "${_GL4ES_FINGERPRINT}\n${_library_hash}\n")
    endif()
    # Upstream produces only the SONAME file; linking also needs libGL.so.
    if(NOT EXISTS "${_gl4es_root}/lib/libGL.so")
        execute_process(COMMAND "${CMAKE_COMMAND}" -E create_symlink
            libGL.so.1 "${_gl4es_root}/lib/libGL.so" RESULT_VARIABLE _linked)
        if(NOT _linked EQUAL 0)
            message(FATAL_ERROR "gl4es: cannot create libGL.so in ${_gl4es_root}/lib")
        endif()
    endif()
    _gl4es_find("${_gl4es_root}" GL4ES_INCLUDE_DIR GL4ES_LIB)
    _gl4es_verified("${_gl4es_root}" "${GL4ES_LIB}" _verified)
    if(NOT GL4ES_INCLUDE_DIR OR NOT _verified)
        message(FATAL_ERROR "gl4es: patched headers/library unavailable in ${_gl4es_root}")
    endif()
endif()

set(GL4ES_ROOT "${_gl4es_root}")
message(STATUS "gl4es: ${GL4ES_LIB}")
message(STATUS "  includes: ${GL4ES_INCLUDE_DIR}")
