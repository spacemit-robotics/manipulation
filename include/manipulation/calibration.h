// Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
// SPDX-License-Identifier: Apache-2.0
/**
 * @file calibration.h
 * @brief Offline eye-to-hand calibration (requires MANIPULATION_ENABLE_CALIBRATION).
 *
 * Transforms are row-major 4x4 rigid transforms with translations in metres. No device is accessed
 * or configured.
 */
#ifndef MANIPULATION_CALIBRATION_H_
#define MANIPULATION_CALIBRATION_H_
#include <array>
#include <vector>

namespace spacemit::manipulation {
/// Row-major 4x4 rigid transform; translation in metres.
using Transform = std::array<double, 16>;

/// One stationary observation of the board fixed to the flange.
struct CalibrationSample {
    /// Flange pose in the arm base frame.
    Transform base_from_flange;
    /// Board pose in the camera frame.
    Transform camera_from_target;
};

/// Solved camera pose and its quality.
struct CalibrationResult {
    /// Camera pose in the arm base frame.
    Transform base_from_camera;
    /// Largest translation residual of the held-out samples, m.
    double holdout_translation_m = 0;
    /// Largest rotation residual of the held-out samples, rad.
    double holdout_rotation_rad = 0;
    /// Largest translation residual of all samples, m.
    double maximum_translation_m = 0;
    /// Largest rotation residual of all samples, rad.
    double maximum_rotation_rad = 0;
    /// Translation threshold actually applied, m.
    double threshold_translation_m = 0;
    /// Rotation threshold actually applied, rad.
    double threshold_rotation_rad = 0;
    /// True when held-out and all-sample residuals are within both thresholds.
    bool quality_passed = false;
};

/// Default translation threshold, m.
constexpr double kDefaultMaxTranslationM = 0.005;
/// Default rotation threshold: 2 degrees.
constexpr double kDefaultMaxRotationRad = 0.03490658503988659;
/// Loosest accepted translation threshold, m; looser thresholds are refused.
constexpr double kCeilingMaxTranslationM = 0.015;
/// Loosest accepted rotation threshold: 5 degrees.
constexpr double kCeilingMaxRotationRad = 0.08726646259971647;

/**
 * @brief Solve the camera pose in the arm base frame from stationary samples.
 *
 * Every fourth sample is held out for validation. Both held-out and all-sample residuals must be
 * within the thresholds to pass.
 *
 * @param samples 12 to 1000 samples with rotation about nonparallel axes.
 * @param max_translation_m Translation threshold, m; at most kCeilingMaxTranslationM.
 * @param max_rotation_rad Rotation threshold, rad; at most kCeilingMaxRotationRad.
 * @return The solved pose with its residuals; check quality_passed before use.
 * @throws std::runtime_error If the samples are too few, invalid or not excited enough.
 */
CalibrationResult SolveEyeToHand(const std::vector<CalibrationSample>& samples,
                                 double max_translation_m = kDefaultMaxTranslationM,
                                 double max_rotation_rad = kDefaultMaxRotationRad);

/**
 * @brief Check that a transform is finite, homogeneous and rigid.
 * @throws std::runtime_error If it is not.
 */
void ValidateTransform(const Transform& transform);
}  // namespace spacemit::manipulation
#endif  // MANIPULATION_CALIBRATION_H_
