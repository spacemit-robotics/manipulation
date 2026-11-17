// Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
// SPDX-License-Identifier: Apache-2.0
/** @file eigen_kinematics.cpp
 * @brief Dependency-free FK/IK backend: minimal URDF reader, transform-chain FK and a damped
 * least-squares search. Needs only Eigen headers; no URDF parser library and no vendor SDK.
 *
 * Behaviour mirrors the optional Pinocchio backend so a configuration can switch backends
 * without changing results: bounded revolute chains in an explicit base-to-tip joint order, a
 * weighted error expressed in the configured base frame, and the same step annealing schedule.
 */

#include "manipulation/kinematics.h"

#include <Eigen/Cholesky>
#include <Eigen/Core>
#include <Eigen/Geometry>

#include <algorithm>
#include <cctype>
#include <chrono>
#include <cmath>
#include <cstring>
#include <filesystem>  // NOLINT(build/c++17): this target requires C++17.
#include <fstream>
#include <set>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace spacemit::manipulation {
namespace {

// ---------------------------------------------------------------------------------------------
// Minimal XML reader
//
// URDF kinematics only needs elements, attributes, comments and the XML declaration. Keeping the
// reader local avoids a urdfdom/tinyxml2 dependency for a format this component only reads.
// ---------------------------------------------------------------------------------------------

struct XmlNode {
    std::string name;
    std::vector<std::pair<std::string, std::string>> attributes;
    std::vector<XmlNode> children;

    const std::string* Attribute(const std::string& key) const {
        for (const auto& entry : attributes)
            if (entry.first == key) return &entry.second;
        return nullptr;
    }

    const XmlNode* Child(const std::string& key) const {
        for (const auto& child : children)
            if (child.name == key) return &child;
        return nullptr;
    }
};

class XmlReader {
public:
    explicit XmlReader(std::string text) : text_(std::move(text)) {}

    XmlNode Parse() {
        SkipMisc();
        if (pos_ >= text_.size() || text_[pos_] != '<') Fail("expected a root element");
        XmlNode root = ParseElement();
        SkipMisc();
        if (pos_ != text_.size()) Fail("trailing content after the root element");
        return root;
    }

private:
    [[noreturn]] void Fail(const std::string& reason) const {
        throw std::invalid_argument("URDF XML: " + reason + " near offset " + std::to_string(pos_));
    }

    bool StartsWith(const char* token) const {
        const std::size_t length = std::strlen(token);
        return text_.size() - pos_ >= length && text_.compare(pos_, length, token) == 0;
    }

    void SkipWhitespace() {
        while (pos_ < text_.size() && std::isspace(static_cast<unsigned char>(text_[pos_]))) ++pos_;
    }

    /// Skips whitespace, the XML declaration, comments and doctype declarations.
    void SkipMisc() {
        while (true) {
            SkipWhitespace();
            if (StartsWith("<?")) {
                const auto end = text_.find("?>", pos_);
                if (end == std::string::npos) Fail("unterminated declaration");
                pos_ = end + 2;
            } else if (StartsWith("<!--")) {
                const auto end = text_.find("-->", pos_);
                if (end == std::string::npos) Fail("unterminated comment");
                pos_ = end + 3;
            } else if (StartsWith("<!")) {
                const auto end = text_.find('>', pos_);
                if (end == std::string::npos) Fail("unterminated declaration");
                pos_ = end + 1;
            } else {
                return;
            }
        }
    }

    std::string ReadName() {
        const std::size_t begin = pos_;
        while (pos_ < text_.size()) {
            const char current = text_[pos_];
            if (std::isspace(static_cast<unsigned char>(current)) || current == '>' ||
                    current == '/' || current == '=')
                break;
            ++pos_;
        }
        if (pos_ == begin) Fail("expected a name");
        return text_.substr(begin, pos_ - begin);
    }

    std::string ReadValue() {
        SkipWhitespace();
        if (pos_ >= text_.size() || (text_[pos_] != '"' && text_[pos_] != '\''))
            Fail("expected a quoted attribute value");
        const char quote = text_[pos_++];
        std::string value;
        while (pos_ < text_.size() && text_[pos_] != quote) {
            if (text_[pos_] == '&') {
                const auto end = text_.find(';', pos_);
                if (end == std::string::npos || end - pos_ > 8) Fail("malformed entity");
                const auto entity = text_.substr(pos_ + 1, end - pos_ - 1);
                if (entity == "amp") value += '&';
                else if (entity == "lt") value += '<';
                else if (entity == "gt") value += '>';
                else if (entity == "quot") value += '"';
                else if (entity == "apos") value += '\'';
                else Fail("unsupported entity");
                pos_ = end + 1;
                continue;
            }
            value += text_[pos_++];
        }
        if (pos_ >= text_.size()) Fail("unterminated attribute value");
        ++pos_;  // closing quote
        return value;
    }

    XmlNode ParseElement() {
        ++pos_;  // '<'
        XmlNode node;
        node.name = ReadName();
        while (true) {
            SkipWhitespace();
            if (pos_ >= text_.size()) Fail("unterminated start tag");
            if (StartsWith("/>")) {
                pos_ += 2;
                return node;
            }
            if (text_[pos_] == '>') {
                ++pos_;
                break;
            }
            const std::string key = ReadName();
            SkipWhitespace();
            if (pos_ >= text_.size() || text_[pos_] != '=') Fail("expected '=' after attribute name");
            ++pos_;
            node.attributes.emplace_back(key, ReadValue());
        }
        while (true) {
            if (pos_ >= text_.size()) Fail("unterminated element '" + node.name + "'");
            if (StartsWith("</")) {
                pos_ += 2;
                const std::string closing = ReadName();
                if (closing != node.name) Fail("mismatched closing tag '" + closing + "'");
                SkipWhitespace();
                if (pos_ >= text_.size() || text_[pos_] != '>') Fail("unterminated closing tag");
                ++pos_;
                return node;
            }
            if (StartsWith("<!--")) {
                const auto end = text_.find("-->", pos_);
                if (end == std::string::npos) Fail("unterminated comment");
                pos_ = end + 3;
                continue;
            }
            if (text_[pos_] == '<') {
                node.children.push_back(ParseElement());
                continue;
            }
            // Character data carries no kinematics; skip to the next tag.
            const auto next = text_.find('<', pos_);
            if (next == std::string::npos) Fail("unterminated element '" + node.name + "'");
            pos_ = next;
        }
    }

    std::string text_;
    std::size_t pos_ = 0;
};

std::string ReadTextFile(const std::string& path) {
    const auto size = std::filesystem::file_size(path);
    if (size > 2 * 1024 * 1024) throw std::invalid_argument("URDF must not exceed 2 MiB");
    std::string text(static_cast<std::size_t>(size), '\0');
    if (size == 0) return text;
    std::ifstream stream(path, std::ios::binary);
    if (!stream) throw std::invalid_argument("URDF cannot be opened: " + path);
    stream.read(text.data(), static_cast<std::streamsize>(size));
    if (!stream) throw std::invalid_argument("URDF cannot be read: " + path);
    return text;
}

std::vector<double> ParseTriple(const std::string& text, const std::string& field) {
    std::vector<double> values;
    std::size_t begin = 0;
    while (begin <= text.size()) {
        const auto end = text.find_first_of(" \t\r\n", begin);
        const auto token =
                text.substr(begin, end == std::string::npos ? std::string::npos : end - begin);
        if (!token.empty()) {
            try {
                values.push_back(std::stod(token));
            } catch (const std::exception&) {
                throw std::invalid_argument("URDF has a non-numeric " + field);
            }
        }
        if (end == std::string::npos) break;
        begin = end + 1;
    }
    if (values.size() != 3 || !std::isfinite(values[0]) || !std::isfinite(values[1]) ||
            !std::isfinite(values[2]))
        throw std::invalid_argument("URDF has an invalid " + field);
    return values;
}

// ---------------------------------------------------------------------------------------------
// Rotations
// ---------------------------------------------------------------------------------------------

Eigen::Vector3d ToVector3(const std::vector<double>& values) {
    return Eigen::Vector3d(values[0], values[1], values[2]);
}

/// URDF roll-pitch-yaw is a fixed-axis sequence: Rz(yaw) * Ry(pitch) * Rx(roll).
Eigen::Matrix3d RotationFromRpy(const Eigen::Vector3d& rpy) {
    return (Eigen::AngleAxisd(rpy.z(), Eigen::Vector3d::UnitZ()) *
            Eigen::AngleAxisd(rpy.y(), Eigen::Vector3d::UnitY()) *
            Eigen::AngleAxisd(rpy.x(), Eigen::Vector3d::UnitX()))
            .toRotationMatrix();
}

Eigen::Vector3d NormalizedAxis(const Eigen::Vector3d& axis) {
    const double norm = axis.norm();
    if (!std::isfinite(norm) || norm < 1e-12)
        throw std::invalid_argument("URDF has an invalid rotation axis");
    return axis / norm;
}

Eigen::Matrix3d RotationFromAxis(const Eigen::Vector3d& axis, double angle) {
    return Eigen::AngleAxisd(angle, NormalizedAxis(axis)).toRotationMatrix();
}

/// SO(3) logarithm: the rotation vector of a rotation matrix, the counterpart of pinocchio::log3.
Eigen::Vector3d LogSo3(const Eigen::Matrix3d& rotation) {
    if (!rotation.allFinite()) throw std::invalid_argument("nonfinite rotation");
    const double cosine = std::clamp((rotation.trace() - 1.0) * 0.5, -1.0, 1.0);
    const double angle = std::acos(cosine);
    const Eigen::Vector3d vee(rotation(2, 1) - rotation(1, 2),
            rotation(0, 2) - rotation(2, 0), rotation(1, 0) - rotation(0, 1));
    if (angle < 1e-8) return 0.5 * vee;
    constexpr double kPi = 3.14159265358979323846;
    if (kPi - angle < 1e-6) {
        // Near pi the skew part vanishes; recover the axis from the symmetric part instead.
        const Eigen::Matrix3d symmetric = (rotation + Eigen::Matrix3d::Identity()) * 0.5;
        Eigen::Index column = 0;
        if (symmetric(1, 1) > symmetric(column, column)) column = 1;
        if (symmetric(2, 2) > symmetric(column, column)) column = 2;
        Eigen::Vector3d axis = symmetric.col(column);
        const double norm = axis.norm();
        if (!std::isfinite(norm) || norm < 1e-12) throw std::invalid_argument("degenerate rotation");
        axis /= norm;
        if (axis.dot(vee) < 0) axis = -axis;
        return angle * axis;
    }
    return (angle / (2.0 * std::sin(angle))) * vee;
}

// ---------------------------------------------------------------------------------------------
// Model
// ---------------------------------------------------------------------------------------------

struct JointSpec {
    std::string name;
    std::string type;
    std::string parent_link;
    std::string child_link;
    Eigen::Vector3d origin_translation = Eigen::Vector3d::Zero();
    Eigen::Vector3d origin_rpy = Eigen::Vector3d::Zero();
    Eigen::Vector3d axis = Eigen::Vector3d::UnitX();
    double lower = 0;
    double upper = 0;
};

JointSpec ParseJoint(const XmlNode& node, std::set<std::string>* links) {
    JointSpec joint;
    const auto* name = node.Attribute("name");
    const auto* type = node.Attribute("type");
    if (!name || name->empty() || !type)
        throw std::invalid_argument("URDF joint needs a name and a type");
    joint.name = *name;
    joint.type = *type;
    const XmlNode* parent = node.Child("parent");
    const XmlNode* child = node.Child("child");
    if (!parent || !child) throw std::invalid_argument("URDF joint lacks parent or child");
    const auto* parent_link = parent->Attribute("link");
    const auto* child_link = child->Attribute("link");
    if (!parent_link || !child_link) throw std::invalid_argument("URDF joint link attribute missing");
    joint.parent_link = *parent_link;
    joint.child_link = *child_link;
    links->insert(joint.parent_link);
    links->insert(joint.child_link);

    if (const XmlNode* origin = node.Child("origin")) {
        if (const auto* xyz = origin->Attribute("xyz"))
            joint.origin_translation = ToVector3(ParseTriple(*xyz, "joint origin"));
        if (const auto* rpy = origin->Attribute("rpy"))
            joint.origin_rpy = ToVector3(ParseTriple(*rpy, "joint rpy"));
    }
    if (const XmlNode* axis = node.Child("axis")) {
        if (const auto* xyz = axis->Attribute("xyz"))
            joint.axis = ToVector3(ParseTriple(*xyz, "joint axis"));
        // A fixed joint carries no motion, and URDF exporters commonly leave xyz="0 0 0" there,
        // so the axis is only required for joints that actually rotate.
        if (joint.type != "fixed") NormalizedAxis(joint.axis);
    }
    if (const XmlNode* limit = node.Child("limit")) {
        const auto* lower = limit->Attribute("lower");
        const auto* upper = limit->Attribute("upper");
        if (lower && upper) {
            try {
                joint.lower = std::stod(*lower);
                joint.upper = std::stod(*upper);
            } catch (const std::exception&) {
                throw std::invalid_argument("URDF has a non-numeric joint limit");
            }
        }
    }
    return joint;
}

class EigenKinematics final : public Kinematics {
public:
    explicit EigenKinematics(const KinematicsConfig& config) : names_(config.joint_names) {
        if (config.base_link.empty() || config.tip_link.empty() || names_.empty() ||
                names_.size() > 64 || config.base_link == config.tip_link)
            throw std::invalid_argument("kinematics needs base, tip and an explicit joint order");
        if (!std::filesystem::is_regular_file(config.urdf_path) ||
                std::filesystem::file_size(config.urdf_path) > 2 * 1024 * 1024)
            throw std::invalid_argument("URDF must be a regular file no larger than 2 MiB");
        XmlReader reader(ReadTextFile(config.urdf_path));
        const XmlNode robot = reader.Parse();
        if (robot.name != "robot") throw std::invalid_argument("URDF root element must be 'robot'");

        std::set<std::string> links;
        for (const auto& child : robot.children) {
            if (child.name != "joint") continue;
            joints_.push_back(ParseJoint(child, &links));
        }
        if (links.count(config.base_link) == 0 || links.count(config.tip_link) == 0)
            throw std::invalid_argument("kinematics base or tip link is absent from URDF");

        BuildChain(config.base_link, config.tip_link);
        for (const auto index : chain_) {
            const auto& joint = joints_[index];
            if (joint.type == "fixed") continue;
            if (joint.type != "revolute")
                throw std::invalid_argument("expected distinct bounded revolute chain joints");
            if (!std::isfinite(joint.lower) || !std::isfinite(joint.upper) ||
                    joint.lower >= joint.upper)
                throw std::invalid_argument("kinematic joint must have finite ordered limits");
            active_.push_back(index);
            lower_.push_back(joint.lower);
            upper_.push_back(joint.upper);
        }
        if (active_.size() != names_.size() || !JointOrderMatchesChain())
            throw std::invalid_argument("joint names must follow the complete base-to-tip chain");
    }

    KinematicPose Forward(const std::vector<double>& joints_rad) override {
        const auto pose = Evaluate(joints_rad, nullptr);
        const Eigen::Quaterniond quaternion(pose.rotation());
        return {pose.translation().x(), pose.translation().y(), pose.translation().z(),
                quaternion.w(), quaternion.x(), quaternion.y(), quaternion.z()};
    }

    std::optional<std::vector<double>> Inverse(const KinematicPose& target,
            const std::vector<double>& seed_rad, const InverseOptions& options) override {
        const Eigen::Quaterniond orientation(target.qw, target.qx, target.qy, target.qz);
        const Eigen::Vector3d translation(target.x, target.y, target.z);
        const double orientation_norm = orientation.norm();
        if (!orientation.coeffs().allFinite() || !std::isfinite(orientation_norm) ||
                orientation_norm < 1e-12 || !translation.allFinite() ||
                !std::isfinite(options.timeout_s) || options.timeout_s <= 0 ||
                options.timeout_s > 60 || !std::isfinite(options.epsilon) ||
                options.epsilon <= 0 || !std::isfinite(options.position_weight) ||
                options.position_weight < 0 || options.position_weight > 1 ||
                options.max_iterations < 1 || options.max_iterations > 100000)
            throw std::invalid_argument("invalid inverse kinematics target or search bounds");
        const Eigen::Matrix3d desired_rotation = orientation.normalized().toRotationMatrix();
        std::vector<double> q = ClampAll(seed_rad);
        using SearchClock = std::chrono::steady_clock;
        const auto started = SearchClock::now();
        const auto elapsed = [&started] {
            return std::chrono::duration<double>(SearchClock::now() - started).count();
        };
        const double position_weight = options.position_weight;
        const double rotation_weight = 1 - position_weight;
        double step_size = 0.5;
        double previous_error = 1e100;
        int stagnant = 0;
        for (int iteration = 0; iteration < options.max_iterations; ++iteration) {
            if (elapsed() > options.timeout_s) break;
            std::vector<Frame> frames;
            const auto current = Evaluate(q, &frames);
            Eigen::Matrix<double, 6, 1> error;
            error.head<3>() = position_weight * (translation - current.translation());
            // Both error and Jacobian are expressed in the configured base, not the tool frame.
            error.tail<3>() =
                    rotation_weight * LogSo3(desired_rotation * current.rotation().transpose());
            const double norm = error.norm();
            if (norm < options.epsilon) {
                // A request that already expired must not report success.
                if (elapsed() > options.timeout_s) return std::nullopt;
                return ClampAll(q);
            }
            if (iteration > 10 && norm > previous_error * 1.5)
                step_size = std::max(0.01, step_size * 0.5);
            stagnant = std::abs(norm - previous_error) < 1e-8 ? stagnant + 1 : 0;
            if (stagnant > 10) break;
            previous_error = norm;

            Eigen::MatrixXd jacobian(6, names_.size());
            const Eigen::Vector3d tip = current.translation();
            for (std::size_t i = 0; i < names_.size(); ++i) {
                // Revolute column: z x (tip - joint) for translation and z for rotation.
                const auto& frame = frames[i];
                jacobian.col(static_cast<Eigen::Index>(i)).head<3>() =
                        position_weight * frame.axis.cross(tip - frame.origin);
                jacobian.col(static_cast<Eigen::Index>(i)).tail<3>() =
                        rotation_weight * frame.axis;
            }
            Eigen::Matrix<double, 6, 6> normal = jacobian * jacobian.transpose();
            normal.diagonal().array() += 1e-4;
            const Eigen::VectorXd delta = jacobian.transpose() * normal.ldlt().solve(error);
            if (!delta.allFinite()) return std::nullopt;
            const double delta_norm = delta.norm();
            const double scale = delta_norm > 0.5 ? 0.5 / delta_norm : 1.0;
            for (std::size_t i = 0; i < names_.size(); ++i)
                q[i] += step_size * scale * delta[static_cast<Eigen::Index>(i)];
            q = ClampAll(q);
        }
        return std::nullopt;
    }

    const std::vector<std::string>& JointNames() const override { return names_; }
    const std::vector<double>& LowerLimits() const override { return lower_; }
    const std::vector<double>& UpperLimits() const override { return upper_; }

private:
    /// Per active joint: its origin and unit axis in the configured base frame.
    struct Frame {
        Eigen::Vector3d origin = Eigen::Vector3d::Zero();
        Eigen::Vector3d axis = Eigen::Vector3d::UnitX();
    };

    /// Walks tip -> base so the chain holds exactly one path, independent of document order.
    void BuildChain(const std::string& base, const std::string& tip) {
        std::set<std::string> children;
        for (const auto& joint : joints_) {
            if (!children.insert(joint.child_link).second)
                throw std::invalid_argument("URDF has multiple parents for one link");
        }
        std::vector<std::size_t> reversed;
        std::set<std::string> visited;
        std::string current = tip;
        while (current != base) {
            if (!visited.insert(current).second) throw std::invalid_argument("URDF chain is cyclic");
            std::size_t match = joints_.size();
            for (std::size_t index = 0; index < joints_.size(); ++index)
                if (joints_[index].child_link == current) {
                    match = index;
                    break;
                }
            if (match == joints_.size())
                throw std::invalid_argument("tip is not a descendant of the base link");
            reversed.push_back(match);
            current = joints_[match].parent_link;
        }
        chain_.assign(reversed.rbegin(), reversed.rend());
    }

    bool JointOrderMatchesChain() const {
        for (std::size_t i = 0; i < active_.size(); ++i)
            if (joints_[active_[i]].name != names_[i]) return false;
        return true;
    }

    Eigen::Isometry3d Evaluate(const std::vector<double>& joints_rad,
            std::vector<Frame>* frames) const {
        if (joints_rad.size() != names_.size())
            throw std::invalid_argument("kinematics joint count differs from the configured order");
        for (const double value : joints_rad)
            if (!std::isfinite(value)) throw std::invalid_argument("nonfinite joint position");
        // Starting from identity makes the result relative to the base link: the chain walk stops
        // at the base, so any joint above it is never applied.
        Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
        std::size_t active = 0;
        if (frames) frames->clear();
        for (const auto index : chain_) {
            const auto& joint = joints_[index];
            Eigen::Isometry3d offset = Eigen::Isometry3d::Identity();
            offset.linear() = RotationFromRpy(joint.origin_rpy);
            offset.translation() = joint.origin_translation;
            pose = pose * offset;
            if (joint.type == "fixed") continue;
            if (frames) {
                frames->push_back({pose.translation(),
                        NormalizedAxis(pose.linear() * joint.axis)});
            }
            Eigen::Isometry3d motion = Eigen::Isometry3d::Identity();
            motion.linear() = RotationFromAxis(joint.axis, joints_rad[active]);
            pose = pose * motion;
            ++active;
        }
        return pose;
    }

    std::vector<double> ClampAll(const std::vector<double>& joints) const {
        if (joints.size() != names_.size())
            throw std::invalid_argument("kinematics joint count differs from the configured order");
        std::vector<double> result = joints;
        for (std::size_t i = 0; i < result.size(); ++i) {
            if (!std::isfinite(result[i]))
                throw std::invalid_argument("nonfinite joint position");
            result[i] = std::clamp(result[i], lower_[i], upper_[i]);
        }
        return result;
    }

    std::vector<JointSpec> joints_;
    std::vector<std::size_t> chain_;
    std::vector<std::size_t> active_;
    std::vector<std::string> names_;
    std::vector<double> lower_;
    std::vector<double> upper_;
};

}  // namespace

void RegisterEigenKinematics(KinematicsRegistry* registry) {
    registry->Register("eigen", [](const KinematicsConfig& config) {
        return std::make_unique<EigenKinematics>(config);
    });
}

}  // namespace spacemit::manipulation
