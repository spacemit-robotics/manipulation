# Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
# SPDX-License-Identifier: Apache-2.0
# Module registration creates explicit linker references, including static Release builds.
# The registration function must be defined in namespace spacemit::manipulation.
function(manipulation_register_device symbol)
    set_property(GLOBAL APPEND PROPERTY MANIPULATION_DEVICE_REGISTRATIONS "${symbol}")
endfunction()

# Capability modules append sources to manipulation_motion and register one entry point.
function(manipulation_register_capability symbol)
    if(NOT symbol MATCHES "^[A-Za-z_][A-Za-z0-9_]*$")
        message(FATAL_ERROR "Invalid capability registration symbol: ${symbol}")
    endif()
    get_property(registrations GLOBAL PROPERTY MANIPULATION_CAPABILITY_REGISTRATIONS)
    if(symbol IN_LIST registrations)
        message(FATAL_ERROR "Duplicate capability registration symbol: ${symbol}")
    endif()
    set_property(GLOBAL APPEND PROPERTY MANIPULATION_CAPABILITY_REGISTRATIONS "${symbol}")
endfunction()
