// Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
// SPDX-License-Identifier: Apache-2.0
#include "manipulation/kinematics.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <filesystem>  // NOLINT(build/c++17): this target requires C++17.
#include <set>
#include <stdexcept>

// Instantiate this private backend against the current Eigen/Boost headers. Imported explicit
// instantiations can have a different template ABI even when the Pinocchio version matches.
#ifdef PINOCCHIO_ENABLE_TEMPLATE_INSTANTIATION
#undef PINOCCHIO_ENABLE_TEMPLATE_INSTANTIATION
#endif

#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/spatial/explog.hpp>

namespace spacemit::manipulation {
namespace {

class PinocchioKinematics final : public Kinematics {
public:
    explicit PinocchioKinematics(const KinematicsConfig& config) : names_(config.joint_names) {
        if (config.base_link.empty() || config.tip_link.empty() || names_.empty() ||
                names_.size() > 64 || config.base_link == config.tip_link)
            throw std::invalid_argument("kinematics needs base, tip and an explicit joint order");
        if (!std::filesystem::is_regular_file(config.urdf_path) ||
                std::filesystem::file_size(config.urdf_path) > 2 * 1024 * 1024)
            throw std::invalid_argument("URDF must be a regular file no larger than 2 MiB");
        pinocchio::urdf::buildModel(config.urdf_path, model_);
        if (!model_.existFrame(config.base_link, pinocchio::BODY) ||
                !model_.existFrame(config.tip_link, pinocchio::BODY))
            throw std::invalid_argument("kinematics base or tip link is absent from URDF");
        base_ = model_.getFrameId(config.base_link, pinocchio::BODY);
        tip_ = model_.getFrameId(config.tip_link, pinocchio::BODY);
        auto frame = tip_;
        while (frame != base_) {
            if (frame == 0) throw std::invalid_argument("tip is not a descendant of the base link");
            frame = model_.frames[frame].parentFrame;
        }
        const auto base_joint = model_.frames[base_].parentJoint;
        std::set<pinocchio::JointIndex> chain;
        std::vector<std::string> chain_names;
        auto joint = model_.frames[tip_].parentJoint;
        while (joint != base_joint) {
            if (joint == 0)
                throw std::invalid_argument("tip is not a descendant of the base joint");
            chain.insert(joint);
            chain_names.push_back(model_.names[joint]);
            joint = model_.parents[joint];
        }
        std::reverse(chain_names.begin(), chain_names.end());
        if (chain_names != names_)
            throw std::invalid_argument("joint names must follow the complete base-to-tip chain");
        for (const auto& name : names_) {
            if (!model_.existJointName(name))
                throw std::invalid_argument("unknown kinematic joint: " + name);
            const auto id = model_.getJointId(name);
            const auto& selected = model_.joints[id];
            if (!chain.erase(id) || selected.nq() != 1 || selected.nv() != 1 ||
                    selected.shortname().rfind("JointModelR", 0) != 0)
                throw std::invalid_argument("expected distinct bounded revolute chain joints");
            q_indices_.push_back(selected.idx_q());
            v_indices_.push_back(selected.idx_v());
            const double lower = model_.lowerPositionLimit[selected.idx_q()];
            const double upper = model_.upperPositionLimit[selected.idx_q()];
            if (!std::isfinite(lower) || !std::isfinite(upper) || lower >= upper)
                throw std::invalid_argument("kinematic joint must have finite ordered limits");
            lower_.push_back(lower);
            upper_.push_back(upper);
        }
        data_ = std::make_unique<pinocchio::Data>(model_);
    }

    KinematicPose Forward(const std::vector<double>& joints_rad) override {
        const auto pose = Evaluate(Configuration(joints_rad));
        const Eigen::Quaterniond quaternion(pose.rotation());
        return {pose.translation().x(), pose.translation().y(), pose.translation().z(),
                quaternion.w(), quaternion.x(), quaternion.y(), quaternion.z()};
    }

    std::optional<std::vector<double>> Inverse(const KinematicPose& target,
            const std::vector<double>& seed_rad, const InverseOptions& options) override {
        Eigen::Quaterniond orientation(target.qw, target.qx, target.qy, target.qz);
        const Eigen::Vector3d translation(target.x, target.y, target.z);
        const double orientation_norm = orientation.norm();
        if (!orientation.coeffs().allFinite() || !std::isfinite(orientation_norm) ||
                orientation_norm < 1e-12 ||
                !translation.allFinite() || !std::isfinite(options.timeout_s) ||
                options.timeout_s <= 0 || options.timeout_s > 60 ||
                !std::isfinite(options.epsilon) || options.epsilon <= 0 ||
                !std::isfinite(options.position_weight) || options.position_weight < 0 ||
                options.position_weight > 1 || options.max_iterations < 1 ||
                options.max_iterations > 100000)
            throw std::invalid_argument("invalid inverse kinematics target or search bounds");
        const pinocchio::SE3 desired(orientation.normalized().toRotationMatrix(), translation);
        Eigen::VectorXd q = Configuration(seed_rad);
        Clamp(&q);
        using SearchClock = std::chrono::steady_clock;
        const auto started = SearchClock::now();
        const double position_weight = options.position_weight;
        const double rotation_weight = 1 - position_weight;
        double step_size = 0.5;
        double previous_error = 1e100;
        int stagnant = 0;
        for (int iteration = 0; iteration < options.max_iterations; ++iteration) {
            if (std::chrono::duration<double>(SearchClock::now() - started).count() >
                    options.timeout_s)
                break;
            const auto current = Evaluate(q);
            Eigen::Matrix<double, 6, 1> error;
            error.head<3>() = position_weight * (desired.translation() - current.translation());
            // Both error and Jacobian are expressed in the configured base, not the tool frame.
            error.tail<3>() = rotation_weight *
                    pinocchio::log3(desired.rotation() * current.rotation().transpose());
            const double norm = error.norm();
            if (norm < options.epsilon) {
                if (std::chrono::duration<double>(SearchClock::now() - started).count() >
                        options.timeout_s)
                    return std::nullopt;
                return Extract(q);
            }
            if (iteration > 10 && norm > previous_error * 1.5)
                step_size = std::max(0.01, step_size * 0.5);
            stagnant = std::abs(norm - previous_error) < 1e-8 ? stagnant + 1 : 0;
            if (stagnant > 10) break;
            previous_error = norm;
            pinocchio::Data::Matrix6x full(6, model_.nv);
            full.setZero();
            pinocchio::computeFrameJacobian(model_, *data_, q, tip_,
                    pinocchio::LOCAL_WORLD_ALIGNED, full);
            Eigen::MatrixXd jacobian(6, names_.size());
            const auto world_to_base = data_->oMf[base_].rotation().transpose().eval();
            for (size_t i = 0; i < names_.size(); ++i) {
                jacobian.col(i).head<3>() =
                        position_weight * world_to_base * full.col(v_indices_[i]).head<3>();
                jacobian.col(i).tail<3>() =
                        rotation_weight * world_to_base * full.col(v_indices_[i]).tail<3>();
            }
            Eigen::Matrix<double, 6, 6> normal = jacobian * jacobian.transpose();
            normal.diagonal().array() += 1e-4;
            Eigen::VectorXd delta = jacobian.transpose() * normal.ldlt().solve(error);
            if (!delta.allFinite()) return std::nullopt;
            if (delta.norm() > 0.5) delta *= 0.5 / delta.norm();
            for (size_t i = 0; i < names_.size(); ++i) q[q_indices_[i]] += step_size * delta[i];
            Clamp(&q);
        }
        return std::nullopt;
    }

    const std::vector<std::string>& JointNames() const override { return names_; }
    const std::vector<double>& LowerLimits() const override { return lower_; }
    const std::vector<double>& UpperLimits() const override { return upper_; }

private:
    Eigen::VectorXd Configuration(const std::vector<double>& joints) const {
        if (joints.size() != names_.size())
            throw std::invalid_argument("kinematics joint count differs from the configured order");
        Eigen::VectorXd q = pinocchio::neutral(model_);
        for (size_t i = 0; i < joints.size(); ++i) {
            if (!std::isfinite(joints[i])) throw std::invalid_argument("nonfinite joint position");
            q[q_indices_[i]] = joints[i];
        }
        return q;
    }

    pinocchio::SE3 Evaluate(const Eigen::VectorXd& q) {
        pinocchio::forwardKinematics(model_, *data_, q);
        pinocchio::updateFramePlacements(model_, *data_);
        return data_->oMf[base_].actInv(data_->oMf[tip_]);
    }

    void Clamp(Eigen::VectorXd* q) const {
        for (size_t i = 0; i < names_.size(); ++i)
            (*q)[q_indices_[i]] = std::clamp((*q)[q_indices_[i]], lower_[i], upper_[i]);
    }

    std::vector<double> Extract(const Eigen::VectorXd& q) const {
        std::vector<double> result;
        for (const int index : q_indices_) result.push_back(q[index]);
        return result;
    }

    pinocchio::Model model_;
    std::unique_ptr<pinocchio::Data> data_;
    pinocchio::FrameIndex base_ = 0;
    pinocchio::FrameIndex tip_ = 0;
    std::vector<std::string> names_;
    std::vector<int> q_indices_;
    std::vector<int> v_indices_;
    std::vector<double> lower_;
    std::vector<double> upper_;
};

}  // namespace

void RegisterPinocchioKinematics(KinematicsRegistry* registry) {
    registry->Register("pinocchio", [](const KinematicsConfig& config) {
        return std::make_unique<PinocchioKinematics>(config);
    });
}

}  // namespace spacemit::manipulation
