// Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
// SPDX-License-Identifier: Apache-2.0
// Configuration-relative resource resolution without opening a device.
#ifndef MANIPULATION_RESOURCE_PATHS_H_
#define MANIPULATION_RESOURCE_PATHS_H_

#include <string>
#include <vector>

#include "manipulation/types.h"

namespace spacemit::manipulation {
// Resolve declared resource paths against directory. Bare executable names use PATH.
// Missing options and absolute paths are retained for normal driver validation.
DriverOptions ResolveResourceOptions(const DriverOptions& options, const std::string& directory,
                                     const std::vector<std::string>& resources,
                                     const std::vector<std::string>& executables);
}  // namespace spacemit::manipulation
#endif  // MANIPULATION_RESOURCE_PATHS_H_
