// Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
// SPDX-License-Identifier: Apache-2.0
// Park eye-to-hand solve with independent holdout and degeneracy rejection.
#include "manipulation/calibration.h"
#include <opencv2/calib3d.hpp>
#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace spacemit::manipulation {
namespace {
cv::Mat Matrix(const Transform& input) {
    return cv::Mat(4, 4, CV_64F, const_cast<double*>(input.data())).clone();
}

Transform Array(const cv::Mat& input) {
    Transform result{};
    std::copy(input.ptr<double>(), input.ptr<double>() + 16, result.begin());
    return result;
}

cv::Rect Rotation() {
    return {0, 0, 3, 3};
}

cv::Rect Translation() {
    return {3, 0, 1, 3};
}

cv::Mat Inverse(const cv::Mat& pose) {
    cv::Mat result = cv::Mat::eye(4, 4, CV_64F);
    cv::Mat rotation = pose(Rotation()).t();
    rotation.copyTo(result(Rotation()));
    cv::Mat translation = -rotation * pose(Translation());
    translation.copyTo(result(Translation()));
    return result;
}

double Angle(const cv::Mat& rotation) {
    cv::Mat vector;
    cv::Rodrigues(rotation, vector);
    return cv::norm(vector);
}
}  // namespace

void ValidateTransform(const Transform& transform) {
    for (double value : transform)
        if (!std::isfinite(value)) throw std::runtime_error("nonfinite transform");
    if (std::abs(transform[12]) + std::abs(transform[13]) + std::abs(transform[14]) +
                std::abs(transform[15] - 1) >
        1e-8)
        throw std::runtime_error("invalid homogeneous row");
    const auto pose = Matrix(transform);
    const auto rotation = pose(Rotation());
    if (cv::norm(rotation.t() * rotation - cv::Mat::eye(3, 3, CV_64F)) > 1e-6 ||
        std::abs(cv::determinant(rotation) - 1) > 1e-6)
        throw std::runtime_error("nonrigid transform");
}

CalibrationResult SolveEyeToHand(const std::vector<CalibrationSample>& samples,
                                 double max_translation_m, double max_rotation_rad) {
    if (samples.size() < 12 || samples.size() > 1000)
        throw std::runtime_error("requires 12..1000 samples");
    if (!(max_translation_m > 0 && max_translation_m <= kCeilingMaxTranslationM &&
          max_rotation_rad > 0 && max_rotation_rad <= kCeilingMaxRotationRad)) {
        throw std::runtime_error("quality thresholds must be within (0, 15 mm] and (0, 5 deg]");
    }
    std::vector<cv::Mat> rotations, translations, targets, offsets, axes;
    for (std::size_t i = 0; i < samples.size(); ++i) {
        ValidateTransform(samples[i].base_from_flange);
        ValidateTransform(samples[i].camera_from_target);
        if (i % 4 == 0) continue;
        const auto arm = Inverse(Matrix(samples[i].base_from_flange));
        const auto target = Matrix(samples[i].camera_from_target);
        rotations.push_back(arm(Rotation()).clone());
        translations.push_back(arm(Translation()).clone());
        targets.push_back(target(Rotation()).clone());
        offsets.push_back(target(Translation()).clone());
        cv::Mat axis;
        cv::Rodrigues(rotations.front().t() * rotations.back(), axis);
        if (cv::norm(axis) > 0.15) axes.push_back(axis / cv::norm(axis));
    }
    bool excited = false;
    for (const auto& first : axes)
        for (const auto& second : axes) {
            excited = excited || cv::norm(first.cross(second)) > 0.2;
        }
    if (!excited) throw std::runtime_error("degenerate rotational excitation");
    cv::Mat rotation, translation;
    cv::calibrateHandEye(rotations, translations, targets, offsets, rotation, translation,
                         cv::CALIB_HAND_EYE_PARK);
    cv::Mat camera = cv::Mat::eye(4, 4, CV_64F);
    rotation.copyTo(camera(Rotation()));
    translation.copyTo(camera(Translation()));
    CalibrationResult result;
    result.base_from_camera = Array(camera);
    ValidateTransform(result.base_from_camera);
    cv::Mat mean_rotation = cv::Mat::zeros(3, 3, CV_64F),
            mean_translation = cv::Mat::zeros(3, 1, CV_64F);
    unsigned count = 0;
    for (std::size_t i = 0; i < samples.size(); ++i)
        if (i % 4 != 0) {
            cv::Mat fixed = Inverse(Matrix(samples[i].base_from_flange)) * camera *
                            Matrix(samples[i].camera_from_target);
            mean_rotation += fixed(Rotation());
            mean_translation += fixed(Translation());
            ++count;
        }
    cv::SVD svd(mean_rotation);
    cv::Mat correction = cv::Mat::eye(3, 3, CV_64F);
    correction.at<double>(2, 2) = cv::determinant(svd.u * svd.vt);
    mean_rotation = svd.u * correction * svd.vt;
    mean_translation /= count;
    for (std::size_t i = 0; i < samples.size(); ++i) {
        cv::Mat fixed = Inverse(Matrix(samples[i].base_from_flange)) * camera *
                        Matrix(samples[i].camera_from_target);
        const double position = cv::norm(fixed(Translation()) - mean_translation);
        const double angle = Angle(mean_rotation.t() * fixed(Rotation()));
        if (!std::isfinite(position) || !std::isfinite(angle))
            throw std::runtime_error("nonfinite residual");
        result.maximum_translation_m = std::max(result.maximum_translation_m, position);
        result.maximum_rotation_rad = std::max(result.maximum_rotation_rad, angle);
        if (i % 4 == 0) {
            result.holdout_translation_m = std::max(result.holdout_translation_m, position);
            result.holdout_rotation_rad = std::max(result.holdout_rotation_rad, angle);
        }
    }
    result.threshold_translation_m = max_translation_m;
    result.threshold_rotation_rad = max_rotation_rad;
    result.quality_passed = result.maximum_translation_m <= max_translation_m &&
                            result.maximum_rotation_rad <= max_rotation_rad;
    return result;
}
}  // namespace spacemit::manipulation
