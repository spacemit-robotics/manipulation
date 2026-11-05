// Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
// SPDX-License-Identifier: Apache-2.0
// Offline eye-to-hand calibration. Row-major rigid transforms, translations in metres.
#ifndef MANIPULATION_CALIBRATION_H_
#define MANIPULATION_CALIBRATION_H_
#include <array>
#include <vector>
namespace spacemit::manipulation {
using Transform = std::array<double, 16>;
struct CalibrationSample {
    Transform base_from_flange;
    Transform camera_from_target;
};
struct CalibrationResult {
    Transform base_from_camera;
    double holdout_translation_m = 0;
    double holdout_rotation_rad = 0;
    double maximum_translation_m = 0;
    double maximum_rotation_rad = 0;
    double threshold_translation_m = 0;  // Quality gate actually applied.
    double threshold_rotation_rad = 0;
    bool quality_passed = false;
};
constexpr double kDefaultMaxTranslationM = 0.005;
constexpr double kDefaultMaxRotationRad = 0.03490658503988659;  // 2 degrees.
constexpr double kCeilingMaxTranslationM = 0.015;               // Looser gates are refused.
constexpr double kCeilingMaxRotationRad = 0.08726646259971647;  // 5 degrees.
// Requires >=12 samples and rotational excitation about nonparallel axes.
// Every fourth sample is reserved for holdout; no device is accessed or configured.
// Both holdout and all-sample residuals must be within the thresholds to pass.
CalibrationResult SolveEyeToHand(const std::vector<CalibrationSample>& samples,
    double max_translation_m = kDefaultMaxTranslationM, double max_rotation_rad = kDefaultMaxRotationRad);
void ValidateTransform(const Transform& transform);
}  // namespace spacemit::manipulation
#endif  // MANIPULATION_CALIBRATION_H_
