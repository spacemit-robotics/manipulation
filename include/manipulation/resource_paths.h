// Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
// SPDX-License-Identifier: Apache-2.0
/**
 * @file resource_paths.h
 * @brief Configuration-relative resource resolution, without opening a device.
 */
#ifndef MANIPULATION_RESOURCE_PATHS_H_
#define MANIPULATION_RESOURCE_PATHS_H_

#include <string>
#include <vector>

#include "manipulation/types.h"

namespace spacemit::manipulation {
/**
 * @brief Resolve declared resource paths against a directory.
 *
 * Missing options and absolute paths are kept for the driver's own validation.
 *
 * @param options Driver options from a device configuration.
 * @param directory Directory relative paths are resolved against, normally the configuration's.
 * @param resources Option names holding file or directory paths.
 * @param executables Option names holding programs; bare names are kept for a PATH search.
 * @return Options with the declared paths resolved.
 */
DriverOptions ResolveResourceOptions(const DriverOptions& options, const std::string& directory,
                                     const std::vector<std::string>& resources,
                                     const std::vector<std::string>& executables);
}  // namespace spacemit::manipulation
#endif  // MANIPULATION_RESOURCE_PATHS_H_
