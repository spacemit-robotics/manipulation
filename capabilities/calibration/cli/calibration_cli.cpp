// Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
// SPDX-License-Identifier: Apache-2.0
// Offline solver: stdout report only; never overwrites an active extrinsic.
#include "manipulation/calibration.h"
#include <yaml-cpp/yaml.h>
#include <filesystem>  // NOLINT(build/c++17): target explicitly requires C++17.
#include <iomanip>
#include <iostream>
#include <set>
#include <stdexcept>
namespace {
void Keys(const YAML::Node& node, const std::set<std::string>& keys) {
    if (!node.IsMap()) throw std::runtime_error("expected map");
    std::set<std::string> seen;
    for (const auto& pair : node) {
        auto key = pair.first.as<std::string>();
        if (!keys.count(key) || !seen.insert(key).second) throw std::runtime_error("unknown/duplicate key");
    }
}
spacemit::manipulation::Transform Pose(const YAML::Node& node) {
    const auto values = node.as<std::vector<double>>();
    if (values.size() != 16) throw std::runtime_error("transform requires 16 row-major entries");
    spacemit::manipulation::Transform result;
    std::copy(values.begin(), values.end(), result.begin());
    return result;
}
}  // namespace
int main(int argc, char** argv) {
    namespace m = spacemit::manipulation;
    try {
        const std::string usage = "manipulation-calibration solve DATASET [--max-error-mm MM] [--max-error-deg DEG]";
        if (argc < 3 || argc % 2 == 0 || std::string(argv[1]) != "solve") throw std::runtime_error(usage);
        double max_translation_m = m::kDefaultMaxTranslationM, max_rotation_rad = m::kDefaultMaxRotationRad;
        for (int i = 3; i < argc; i += 2) {
            const std::string option = argv[i];
            const double value = std::stod(argv[i + 1]);
            if (option == "--max-error-mm") max_translation_m = value / 1000;
            else if (option == "--max-error-deg") max_rotation_rad = value * 3.14159265358979323846 / 180;
            else throw std::runtime_error(usage);
        }
        if (std::filesystem::file_size(argv[2]) > 2 * 1024 * 1024) throw std::runtime_error("dataset exceeds 2 MiB");
        const auto documents = YAML::LoadAllFromFile(argv[2]);
        if (documents.size() != 1) throw std::runtime_error("requires one document");
        const auto root = documents.front();
        Keys(root, {"schema_version", "setup", "samples"});
        if (root["schema_version"].as<int>() != 1 || root["setup"].as<std::string>() != "eye_to_hand" ||
            !root["samples"].IsSequence()) throw std::runtime_error("requires v1 eye_to_hand samples");
        std::vector<m::CalibrationSample> samples;
        for (const auto& sample : root["samples"]) {
            Keys(sample, {"T_base_flange", "T_camera_target"});
            samples.push_back({Pose(sample["T_base_flange"]), Pose(sample["T_camera_target"])});
        }
        const auto result = m::SolveEyeToHand(samples, max_translation_m, max_rotation_rad);
        std::cout << std::setprecision(17) << "{\"schema_version\":1,\"setup\":\"eye_to_hand\",\"T_base_camera\":[";
        for (std::size_t i = 0; i < 16; ++i) { if (i) std::cout << ','; std::cout << result.base_from_camera[i]; }
        std::cout << "],\"quality_passed\":" << (result.quality_passed ? "true" : "false")
            << ",\"holdout_translation_m\":" << result.holdout_translation_m
            << ",\"holdout_rotation_rad\":" << result.holdout_rotation_rad
            << ",\"maximum_translation_m\":" << result.maximum_translation_m
            << ",\"maximum_rotation_rad\":" << result.maximum_rotation_rad
            << ",\"threshold_translation_m\":" << result.threshold_translation_m
            << ",\"threshold_rotation_rad\":" << result.threshold_rotation_rad
            << ",\"sample_count\":" << samples.size() << "}\n";
        return result.quality_passed ? 0 : 1;
    } catch (const std::exception& error) { std::cerr << error.what() << '\n'; return 2; }
}
