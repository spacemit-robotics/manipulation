/*
 * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
 * SPDX-License-Identifier: Apache-2.0
 *
 * @file mujoco_world.cpp
 * @brief Independent headless MuJoCo model ownership and named control mapping.
 */
#include "mujoco_world.h"
#include "mujoco_native_snapshot.h"

#include <mujoco/mujoco.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <set>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace mujoco_sim {
namespace {

void ValidateRange(double minimum, double maximum, const std::string& description) {
    if (!std::isfinite(minimum) || !std::isfinite(maximum) || minimum > maximum) {
        throw std::runtime_error("invalid " + description + " range");
    }
}

bool AllFinite(const mjtNum* values, int count) {
    for (int index = 0; index < count; ++index) {
        if (!std::isfinite(values[index])) {
            return false;
        }
    }
    return true;
}

void RefreshGeometry(const mjModel* model, mjData* data) {
    // Refresh post-integration geometry without control callbacks or stepping.
    mj_fwdPosition(model, data);
    for (int warning = 0; warning < mjNWARNING; ++warning) {
        if (data->warning[warning].number != 0) {
            throw std::runtime_error("MuJoCo snapshot geometry warning");
        }
    }
    if (!AllFinite(data->xpos, 3 * model->nbody) || !AllFinite(data->xquat, 4 * model->nbody) ||
            !AllFinite(data->cam_xpos, 3 * model->ncam)) {
        throw std::runtime_error("nonfinite MuJoCo snapshot geometry");
    }
}

bool BodiesTouch(const mjModel* model, const mjData* data, const std::string& body1,
        const std::string& body2, bool include_descendants) {
    if (body1.empty() || body2.empty() || body1.find('\0') != std::string::npos ||
            body2.find('\0') != std::string::npos) {
        throw std::invalid_argument("invalid contact body name");
    }
    const int first = mj_name2id(model, mjOBJ_BODY, body1.c_str());
    const int second = mj_name2id(model, mjOBJ_BODY, body2.c_str());
    if (first < 0 || second < 0 || first == second) {
        throw std::invalid_argument("contact bodies must exist and differ");
    }
    const auto matches = [&](int actual, int expected) {
        while (actual > 0 && actual != expected && include_descendants) {
            actual = model->body_parentid[actual];
        }
        return actual == expected;
    };
    for (int contact = 0; contact < data->ncon; ++contact) {
        const auto& pair = data->contact[contact];
        if (pair.dist > 0.0 || pair.geom[0] < 0 || pair.geom[1] < 0) continue;
        const int left = model->geom_bodyid[pair.geom[0]];
        const int right = model->geom_bodyid[pair.geom[1]];
        if ((matches(left, first) && matches(right, second)) ||
                (matches(left, second) && matches(right, first))) {
            return true;
        }
    }
    return false;
}

struct EqualityState {
    EqualityState() = default;

    explicit EqualityState(const mjModel* model) {
        if (model->neq > 0) {
            object1.assign(model->eq_obj1id, model->eq_obj1id + model->neq);
            object2.assign(model->eq_obj2id, model->eq_obj2id + model->neq);
            parameters.assign(model->eq_data, model->eq_data + model->neq * mjNEQDATA);
        }
    }

    void Apply(mjModel* model) const {
        std::copy(object1.begin(), object1.end(), model->eq_obj1id);
        std::copy(object2.begin(), object2.end(), model->eq_obj2id);
        std::copy(parameters.begin(), parameters.end(), model->eq_data);
    }

    std::vector<int> object1;
    std::vector<int> object2;
    std::vector<mjtNum> parameters;
};

}  // namespace

class WorldSnapshot::Impl {
public:
    explicit Impl(const mjModel* source) : equality(source) {}

    std::shared_ptr<const mjModel> model;
    std::unique_ptr<mjData, decltype(&mj_deleteData)> data{nullptr, mj_deleteData};
    EqualityState equality;
    std::uint64_t sample_id = 0;
};

WorldSnapshot::WorldSnapshot() = default;
WorldSnapshot::~WorldSnapshot() = default;
WorldSnapshot::WorldSnapshot(const WorldSnapshot&) = default;
WorldSnapshot& WorldSnapshot::operator=(const WorldSnapshot&) = default;

bool WorldSnapshot::IsValid() const {
    return impl_ != nullptr;
}

std::uint64_t WorldSnapshot::SampleId() const {
    if (!impl_) throw std::invalid_argument("empty MuJoCo snapshot");
    return impl_->sample_id;
}

double WorldSnapshot::Time() const {
    if (!impl_) throw std::invalid_argument("empty MuJoCo snapshot");
    return impl_->data->time;
}

JointInfo WorldSnapshot::Joint(const std::string& name) const {
    if (!impl_) throw std::invalid_argument("empty MuJoCo snapshot");
    if (name.empty() || name.find('\0') != std::string::npos) {
        throw std::invalid_argument("invalid MuJoCo joint name");
    }
    const auto* model = impl_->model.get();
    const int joint = mj_name2id(model, mjOBJ_JOINT, name.c_str());
    if (joint < 0 ||
            (model->jnt_type[joint] != mjJNT_HINGE && model->jnt_type[joint] != mjJNT_SLIDE)) {
        throw std::invalid_argument("snapshot scalar joint not found: " + name);
    }
    JointInfo result;
    result.type = JointType::kHinge;
    if (model->jnt_type[joint] == mjJNT_SLIDE) result.type = JointType::kSlide;
    result.position = impl_->data->qpos[model->jnt_qposadr[joint]];
    result.velocity = impl_->data->qvel[model->jnt_dofadr[joint]];
    result.limited = model->jnt_limited[joint] != 0;
    result.minimum = model->jnt_range[2 * joint];
    result.maximum = model->jnt_range[2 * joint + 1];
    return result;
}

class NativeSnapshot::Impl {
public:
    explicit Impl(const WorldSnapshot& snapshot) { Update(snapshot); }

    void Update(const WorldSnapshot& snapshot) {
        if (!snapshot.impl_) throw std::invalid_argument("empty MuJoCo snapshot");
        std::unique_ptr<mjModel, decltype(&mj_deleteModel)> next_model{nullptr, mj_deleteModel};
        mjModel* selected_model = model.get();
        if (source_model != snapshot.impl_->model) {
            next_model.reset(mj_copyModel(nullptr, snapshot.impl_->model.get()));
            if (!next_model) throw std::runtime_error("cannot copy native MuJoCo model");
            selected_model = next_model.get();
        }
        // Equality edits are small per-snapshot state, not a reason to recopy
        // mesh/texture assets. Restore the previous private model if refresh fails.
        const EqualityState previous(selected_model);
        snapshot.impl_->equality.Apply(selected_model);
        try {
            std::unique_ptr<mjData, decltype(&mj_deleteData)> next_data{
                    mj_copyData(nullptr, selected_model, snapshot.impl_->data.get()),
                    mj_deleteData};
            if (!next_data) throw std::runtime_error("cannot allocate native MuJoCo snapshot");
            RefreshGeometry(selected_model, next_data.get());
            data = std::move(next_data);
        } catch (...) {
            previous.Apply(selected_model);
            throw;
        }
        if (next_model) model = std::move(next_model);
        source_model = snapshot.impl_->model;
    }

    std::shared_ptr<const mjModel> source_model;
    std::unique_ptr<mjModel, decltype(&mj_deleteModel)> model{nullptr, mj_deleteModel};
    std::unique_ptr<mjData, decltype(&mj_deleteData)> data{nullptr, mj_deleteData};
};

NativeSnapshot::NativeSnapshot(const WorldSnapshot& snapshot)
    : impl_(std::make_unique<Impl>(snapshot)) {}

NativeSnapshot::~NativeSnapshot() = default;

void NativeSnapshot::Update(const WorldSnapshot& snapshot) {
    impl_->Update(snapshot);
}

const mjModel_* NativeSnapshot::Model() const {
    return impl_->model.get();
}

const mjData_* NativeSnapshot::Data() const {
    return impl_->data.get();
}

bool WorldSnapshot::TouchingBodies(
        const std::string& body1, const std::string& body2, bool include_descendants) const {
    const NativeSnapshot native(*this);
    return BodiesTouch(native.Model(), native.Data(), body1, body2, include_descendants);
}

class World::Impl {
public:
    explicit Impl(const std::string& xml_path) {
        const int runtime_version = mj_version();
        if (runtime_version != mjVERSION_HEADER) {
            throw std::runtime_error("MuJoCo runtime/header version mismatch: runtime=" +
                    std::to_string(runtime_version) +
                    ", header=" + std::to_string(mjVERSION_HEADER));
        }
        if (xml_path.empty()) {
            throw std::invalid_argument("MuJoCo XML path is empty");
        }
        char error[1024] = {};
        model_.reset(mj_loadXML(xml_path.c_str(), nullptr, error, sizeof(error)));
        if (!model_) {
            throw std::runtime_error("cannot load MuJoCo model: " + std::string(error));
        }
        if (!std::isfinite(model_->opt.timestep) || model_->opt.timestep <= 0.0) {
            throw std::runtime_error("MuJoCo timestep must be positive and finite");
        }
        for (int joint = 0; joint < model_->njnt; ++joint) {
            if (model_->jnt_limited[joint]) {
                ValidateRange(
                        model_->jnt_range[2 * joint], model_->jnt_range[2 * joint + 1], "joint");
            }
        }
        for (int actuator = 0; actuator < model_->nu; ++actuator) {
            if (model_->actuator_ctrllimited[actuator]) {
                ValidateRange(model_->actuator_ctrlrange[2 * actuator],
                        model_->actuator_ctrlrange[2 * actuator + 1], "actuator control");
            }
        }
        data_.reset(mj_makeData(model_.get()));
        if (!data_) {
            throw std::runtime_error("cannot allocate MuJoCo data");
        }
        geometry_data_.reset(mj_makeData(model_.get()));
        if (!geometry_data_) throw std::runtime_error("cannot allocate MuJoCo geometry scratch");
        original_equality_ = EqualityState(model_.get());
        Reset("");
        // Clone model assets before a caller starts its realtime stepping loop.
        // Weld edits live in small snapshot equality records, never in this base.
        SnapshotModel();
    }

    void RequireHealthy() const {
        if (faulted_) {
            throw std::runtime_error("MuJoCo world is faulted; call Reset before use");
        }
    }

    int Find(int object_type, const std::string& name) const {
        if (name.empty()) {
            throw std::invalid_argument("MuJoCo object name is empty");
        }
        const int identifier = mj_name2id(model_.get(), object_type, name.c_str());
        if (identifier < 0) {
            throw std::invalid_argument("MuJoCo object not found: " + name);
        }
        return identifier;
    }

    void ValidateState() {
        for (int warning = 0; warning < mjNWARNING; ++warning) {
            if (data_->warning[warning].number != 0) {
                faulted_ = true;
                throw std::runtime_error("MuJoCo physics warning " + std::to_string(warning) +
                        ", info=" + std::to_string(data_->warning[warning].lastinfo));
            }
        }
        if (!std::isfinite(data_->time) || !AllFinite(data_->qpos, model_->nq) ||
                !AllFinite(data_->qvel, model_->nv) || !AllFinite(data_->qacc, model_->nv) ||
                !AllFinite(data_->act, model_->na) || !AllFinite(data_->ctrl, model_->nu)) {
            faulted_ = true;
            throw std::runtime_error("MuJoCo produced nonfinite physics state");
        }
    }

    void ReserveSamples(std::size_t count) const {
        if (count > std::numeric_limits<std::uint64_t>::max() - sample_id_) {
            throw std::overflow_error("MuJoCo sample identifier exhausted");
        }
    }

    void Reset(const std::string& keyframe) {
        int key = -1;
        if (!keyframe.empty()) {
            key = Find(mjOBJ_KEY, keyframe);
        }
        ReserveSamples(1);
        original_equality_.Apply(model_.get());
        if (key < 0) {
            mj_resetData(model_.get(), data_.get());
        } else {
            mj_resetDataKeyframe(model_.get(), data_.get(), key);
        }
        mj_forward(model_.get(), data_.get());
        ValidateState();
        faulted_ = false;
        ++sample_id_;
    }

    JointInfo Joint(const std::string& name) const {
        RequireHealthy();
        const int joint = Find(mjOBJ_JOINT, name);
        const int type = model_->jnt_type[joint];
        if (type != mjJNT_HINGE && type != mjJNT_SLIDE) {
            throw std::invalid_argument("MuJoCo joint is not scalar: " + name);
        }
        JointInfo result;
        result.type = JointType::kHinge;
        if (type == mjJNT_SLIDE) {
            result.type = JointType::kSlide;
        }
        result.position = data_->qpos[model_->jnt_qposadr[joint]];
        result.velocity = data_->qvel[model_->jnt_dofadr[joint]];
        result.limited = model_->jnt_limited[joint] != 0;
        result.minimum = model_->jnt_range[2 * joint];
        result.maximum = model_->jnt_range[2 * joint + 1];
        return result;
    }

    std::vector<std::string> TransmissionJoints(int actuator) const {
        std::vector<std::pair<int, double>> joints;
        const int transmission = model_->actuator_trntype[actuator];
        const int target = model_->actuator_trnid[2 * actuator];
        if (transmission == mjTRN_JOINT) {
            joints.emplace_back(target, 1.0);
        } else if (transmission == mjTRN_TENDON) {
            if (target < 0 || target >= model_->ntendon) {
                return {};
            }
            const int start = model_->tendon_adr[target];
            const int count = model_->tendon_num[target];
            if (start < 0 || count <= 0 || start > model_->nwrap - count) {
                return {};
            }
            for (int index = start; index < start + count; ++index) {
                const double coefficient = model_->wrap_prm[index];
                if (model_->wrap_type[index] != mjWRAP_JOINT || !std::isfinite(coefficient) ||
                        coefficient == 0.0) {
                    return {};
                }
                const int joint = model_->wrap_objid[index];
                const auto existing = std::find_if(joints.begin(), joints.end(),
                        [joint](const auto& entry) { return entry.first == joint; });
                if (existing == joints.end()) {
                    joints.emplace_back(joint, coefficient);
                } else {
                    existing->second += coefficient;
                }
            }
        } else {
            return {};
        }
        std::vector<std::string> names;
        for (const auto& joint : joints) {
            if (joint.first < 0 || joint.first >= model_->njnt || !std::isfinite(joint.second) ||
                    joint.second == 0.0) {
                return {};
            }
            const int type = model_->jnt_type[joint.first];
            const char* name = mj_id2name(model_.get(), mjOBJ_JOINT, joint.first);
            if ((type != mjJNT_HINGE && type != mjJNT_SLIDE) || name == nullptr || *name == '\0') {
                return {};
            }
            names.emplace_back(name);
        }
        return names;
    }

    ActuatorInfo Actuator(const std::string& name) const {
        RequireHealthy();
        const int actuator = Find(mjOBJ_ACTUATOR, name);
        ActuatorInfo result;
        result.control = data_->ctrl[actuator];
        const int group = model_->actuator_group[actuator];
        const unsigned int disabled_groups = static_cast<unsigned int>(model_->opt.disableactuator);
        const bool group_disabled = group >= 0 &&
                group < std::numeric_limits<unsigned int>::digits &&
                (disabled_groups & (1U << group)) != 0;
        result.enabled = (model_->opt.disableflags & mjDSBL_ACTUATION) == 0 && !group_disabled;
        result.control_limited = model_->actuator_ctrllimited[actuator] != 0;
        result.minimum = model_->actuator_ctrlrange[2 * actuator];
        result.maximum = model_->actuator_ctrlrange[2 * actuator + 1];
        result.gain = model_->actuator_gainprm[actuator * mjNGAIN];
        result.position_bias = model_->actuator_biasprm[actuator * mjNBIAS + 1];
        result.velocity_bias = model_->actuator_biasprm[actuator * mjNBIAS + 2];
        const double constant_bias = model_->actuator_biasprm[actuator * mjNBIAS];
        const double tolerance = 1e-10 * std::max(1.0, std::abs(result.gain));
        result.affine_position_control = model_->actuator_dyntype[actuator] == mjDYN_NONE &&
                model_->actuator_gaintype[actuator] == mjGAIN_FIXED &&
                model_->actuator_biastype[actuator] == mjBIAS_AFFINE &&
                std::isfinite(result.gain) && std::isfinite(result.position_bias) &&
                std::isfinite(result.velocity_bias) && result.gain > 0.0 &&
                result.position_bias < 0.0 && result.velocity_bias <= 0.0 &&
                std::abs(constant_bias) <= tolerance;
        result.position_control = result.affine_position_control &&
                std::abs(result.position_bias + result.gain) <= tolerance;
        result.joint_transmission = model_->actuator_trntype[actuator] == mjTRN_JOINT;
        if (result.joint_transmission) {
            const int joint = model_->actuator_trnid[2 * actuator];
            const char* joint_name = mj_id2name(model_.get(), mjOBJ_JOINT, joint);
            if (joint_name != nullptr) {
                result.joint_name = joint_name;
            }
        }
        result.gear = model_->actuator_gear[6 * actuator];
        if (std::isfinite(result.gear) && result.gear != 0.0) {
            result.controlled_joints = TransmissionJoints(actuator);
        }
        result.transmission_supported = !result.controlled_joints.empty();
        return result;
    }

    void SetControls(const std::vector<std::pair<std::string, double>>& controls) {
        RequireHealthy();
        std::set<int> seen;
        std::vector<std::pair<int, double>> validated;
        validated.reserve(controls.size());
        for (const auto& entry : controls) {
            const int actuator = Find(mjOBJ_ACTUATOR, entry.first);
            if (!seen.insert(actuator).second || !std::isfinite(entry.second)) {
                throw std::invalid_argument("duplicate or nonfinite control: " + entry.first);
            }
            if (model_->actuator_ctrllimited[actuator] &&
                    (entry.second < model_->actuator_ctrlrange[2 * actuator] ||
                            entry.second > model_->actuator_ctrlrange[2 * actuator + 1])) {
                throw std::invalid_argument("control outside model range: " + entry.first);
            }
            validated.emplace_back(actuator, entry.second);
        }
        for (const auto& entry : validated) {
            data_->ctrl[entry.first] = entry.second;
        }
    }

    void Step(std::size_t count) {
        RequireHealthy();
        ReserveSamples(count);
        for (std::size_t step = 0; step < count; ++step) {
            const double previous_time = data_->time;
            mj_step(model_.get(), data_.get());
            ValidateState();
            const double expected_time = previous_time + model_->opt.timestep;
            const double tolerance = 1e-10 * std::max(1.0, std::abs(expected_time));
            if (data_->time <= previous_time || std::abs(data_->time - expected_time) > tolerance) {
                faulted_ = true;
                throw std::runtime_error("MuJoCo unexpectedly reset or changed simulation time");
            }
            ++sample_id_;
        }
    }

    std::shared_ptr<const mjModel> SnapshotModel() const {
        if (!snapshot_model_) {
            snapshot_model_ = std::shared_ptr<const mjModel>(
                    mj_copyModel(nullptr, model_.get()), mj_deleteModel);
            if (!snapshot_model_) throw std::runtime_error("cannot copy MuJoCo snapshot model");
        }
        return snapshot_model_;
    }

    const mjData* Geometry() const {
        RequireHealthy();
        if (geometry_sample_ != sample_id_) {
            mj_copyData(geometry_data_.get(), model_.get(), data_.get());
            RefreshGeometry(model_.get(), geometry_data_.get());
            geometry_sample_ = sample_id_;
        }
        return geometry_data_.get();
    }

    int Weld(const std::string& name) const {
        RequireHealthy();
        if (name.find('\0') != std::string::npos) {
            throw std::invalid_argument("invalid weld name");
        }
        const int equality = Find(mjOBJ_EQUALITY, name);
        if (model_->eq_type[equality] != mjEQ_WELD || model_->eq_objtype[equality] != mjOBJ_BODY) {
            throw std::invalid_argument("equality is not a body-based weld: " + name);
        }
        return equality;
    }

    std::unique_ptr<mjModel, decltype(&mj_deleteModel)> model_{nullptr, mj_deleteModel};
    std::unique_ptr<mjData, decltype(&mj_deleteData)> data_{nullptr, mj_deleteData};
    mutable std::unique_ptr<mjData, decltype(&mj_deleteData)> geometry_data_{
            nullptr, mj_deleteData};
    mutable std::uint64_t geometry_sample_ = 0;
    std::uint64_t sample_id_ = 0;
    bool faulted_ = false;
    mutable std::shared_ptr<const mjModel> snapshot_model_;
    EqualityState original_equality_;
};

World::World(const std::string& xml_path) : impl_(std::make_unique<Impl>(xml_path)) {}

World::~World() = default;

void World::Reset(const std::string& keyframe) {
    impl_->Reset(keyframe);
}

JointInfo World::Joint(const std::string& name) const {
    return impl_->Joint(name);
}

ActuatorInfo World::Actuator(const std::string& name) const {
    return impl_->Actuator(name);
}

void World::SetControls(const std::vector<std::pair<std::string, double>>& controls) {
    impl_->SetControls(controls);
}

void World::Step(std::size_t count) {
    impl_->Step(count);
}

double World::Time() const {
    impl_->RequireHealthy();
    return impl_->data_->time;
}

double World::Timestep() const {
    return impl_->model_->opt.timestep;
}

std::uint64_t World::SampleId() const {
    impl_->RequireHealthy();
    return impl_->sample_id_;
}

WorldSnapshot World::Snapshot() const {
    impl_->RequireHealthy();
    auto state = std::make_shared<WorldSnapshot::Impl>(impl_->model_.get());
    state->model = impl_->SnapshotModel();
    state->data.reset(mj_copyData(nullptr, state->model.get(), impl_->data_.get()));
    if (!state->data) throw std::runtime_error("cannot copy MuJoCo snapshot data");
    state->sample_id = impl_->sample_id_;
    WorldSnapshot snapshot;
    snapshot.impl_ = std::move(state);
    return snapshot;
}

bool World::TouchingBodies(
        const std::string& body1, const std::string& body2, bool include_descendants) const {
    return BodiesTouch(impl_->model_.get(), impl_->Geometry(), body1, body2, include_descendants);
}

void World::AttachWeld(
        const std::string& equality, const std::string& body1, const std::string& body2) {
    const int weld = impl_->Weld(equality);
    if (body1.find('\0') != std::string::npos || body2.find('\0') != std::string::npos) {
        throw std::invalid_argument("invalid weld body name");
    }
    const int first = impl_->Find(mjOBJ_BODY, body1);
    const int second = impl_->Find(mjOBJ_BODY, body2);
    if (first == second) throw std::invalid_argument("weld bodies must differ");
    impl_->ReserveSamples(1);
    const auto* data = impl_->Geometry();
    std::array<mjtNum, mjNEQDATA> parameters{};
    mjtNum inverse_position[3];
    mjtNum inverse_rotation[4];
    mju_negPose(
            inverse_position, inverse_rotation, data->xpos + 3 * first, data->xquat + 4 * first);
    mju_mulPose(parameters.data() + 3, parameters.data() + 6, inverse_position, inverse_rotation,
            data->xpos + 3 * second, data->xquat + 4 * second);
    parameters[10] = impl_->model_->eq_data[weld * mjNEQDATA + 10];
    if (!AllFinite(parameters.data(), mjNEQDATA) || parameters[10] <= 0.0) {
        throw std::invalid_argument("weld pose/torque scale must be finite and valid");
    }
    // No throwing operations below: validation and all allocations precede writes.
    impl_->model_->eq_obj1id[weld] = first;
    impl_->model_->eq_obj2id[weld] = second;
    std::copy(parameters.begin(), parameters.end(), impl_->model_->eq_data + weld * mjNEQDATA);
    impl_->data_->eq_active[weld] = 1;
    ++impl_->sample_id_;
}

void World::ReleaseWeld(const std::string& equality) {
    const int weld = impl_->Weld(equality);
    impl_->ReserveSamples(1);
    impl_->data_->eq_active[weld] = 0;
    ++impl_->sample_id_;
}

}  // namespace mujoco_sim
