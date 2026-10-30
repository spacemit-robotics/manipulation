# Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
# SPDX-License-Identifier: Apache-2.0
# Module registration creates explicit linker references, including static Release builds.
# The registration function must be defined in namespace spacemit::manipulation.
function(manipulation_register_device symbol)
    set_property(GLOBAL APPEND PROPERTY MANIPULATION_DEVICE_REGISTRATIONS "${symbol}")
endfunction()
