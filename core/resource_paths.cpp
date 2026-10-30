// Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
// SPDX-License-Identifier: Apache-2.0
// Normalize resources from their owning configuration, not the caller's cwd.
#include "manipulation/resource_paths.h"

#include <filesystem>  // NOLINT(build/c++17): target explicitly requires C++17.

namespace spacemit::manipulation {
DriverOptions ResolveResourceOptions(const DriverOptions& options, const std::string& directory,
    const std::vector<std::string>& resources, const std::vector<std::string>& executables) {
    auto result = options;
    const auto base = std::filesystem::absolute(directory);
    for (const auto& key : resources) {
        auto found = result.find(key);
        if (found != result.end() && !found->second.empty()) {
            found->second = (base / found->second).lexically_normal().string();
        }
    }
    for (const auto& key : executables) {
        auto found = result.find(key);
        if (found != result.end() && std::filesystem::path(found->second).has_parent_path()) {
            found->second = (base / found->second).lexically_normal().string();
        }
    }
    return result;
}
}  // namespace spacemit::manipulation
