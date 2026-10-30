// Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
// SPDX-License-Identifier: Apache-2.0
// Exact synthetic hand-eye data, independent holdout and degenerate pose rejection.
#include "manipulation/calibration.h"
#include <opencv2/calib3d.hpp>
#include <cmath>
#include <iostream>
#include <stdexcept>
namespace m = spacemit::manipulation;
namespace {
m::Transform Array(const cv::Mat& matrix) {
    m::Transform result;
    std::copy(matrix.ptr<double>(), matrix.ptr<double>() + 16, result.begin());
    return result;
}
cv::Mat Pose(double x, double y, double z, double angle) {
    cv::Mat result = cv::Mat::eye(4, 4, CV_64F), rotation;
    cv::Rodrigues(cv::Vec3d(x, y, z) * angle, rotation);
    rotation.copyTo(result(cv::Rect(0, 0, 3, 3)));
    result.at<double>(0, 3) = x * 0.1;
    result.at<double>(1, 3) = y * 0.2;
    result.at<double>(2, 3) = z * 0.3;
    return result;
}
}  // namespace
int main() {
    try {
        const auto camera = Pose(1, -1, 1, 0.3), target = Pose(0, 0, 1, 0.2);
        std::vector<m::CalibrationSample> samples;
        for (unsigned i = 0; i < 20; ++i) {
            auto arm = Pose(std::sin(i * 0.7), std::cos(i * 0.9), std::sin(i * 0.4), 0.8);
            cv::Mat observation = camera.inv() * arm * target;
            samples.push_back({Array(arm), Array(observation)});
        }
        auto result = m::SolveEyeToHand(samples);
        if (!result.quality_passed) throw std::runtime_error("exact synthetic calibration failed");
        auto expected = Array(camera);
        for (unsigned i = 0; i < 16; ++i) if (std::abs(result.base_from_camera[i] - expected[i]) > 1e-8) {
            throw std::runtime_error("wrong eye-to-hand transform direction");
        }
        samples[0].camera_from_target[3] += 0.03;
        if (m::SolveEyeToHand(samples).quality_passed) throw std::runtime_error("holdout corruption accepted");
        samples[0].camera_from_target[3] -= 0.03;
        samples[1].camera_from_target[3] += 0.03;
        auto corrupted = m::SolveEyeToHand(samples);
        if (corrupted.quality_passed || corrupted.maximum_translation_m <= 0.005) {
            throw std::runtime_error("training corruption accepted");
        }
        if (corrupted.threshold_translation_m != m::kDefaultMaxTranslationM) {
            throw std::runtime_error("applied threshold not reported");
        }
        bool ceiling = false;
        try { m::SolveEyeToHand(samples, 0.016, m::kDefaultMaxRotationRad); } catch (const std::runtime_error&) {
            ceiling = true;
        }
        if (!ceiling) throw std::runtime_error("threshold above the 15 mm ceiling accepted");
        for (auto& sample : samples) sample = samples[1];
        bool rejected = false;
        try { m::SolveEyeToHand(samples); } catch (const std::runtime_error&) { rejected = true; }
        if (!rejected) throw std::runtime_error("degenerate data accepted");
        std::cout << "calibration contracts passed\n";
        return 0;
    } catch (const std::exception& error) { std::cerr << error.what() << '\n'; return 1; }
}
