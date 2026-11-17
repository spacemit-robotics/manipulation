# Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
# SPDX-License-Identifier: Apache-2.0
# An external capability module; no runtime parser or driver changes are needed.
target_sources(manipulation_motion PRIVATE "${CMAKE_CURRENT_LIST_DIR}/capability_probe.cpp")
manipulation_register_capability(RegisterProbeCapability)
