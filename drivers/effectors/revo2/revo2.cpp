// Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
// SPDX-License-Identifier: Apache-2.0
// Normalized right-hand Revo2 adapter over the same owned worker as Nero.
#include "agx_session.h"
#include <iomanip>
#include <sstream>
#include <utility>
namespace spacemit::manipulation::agx {
namespace {
class Revo2 final : public EndEffector {
public:
    Revo2(bool feedback, std::shared_ptr<SharedSession> connection)
        : feedback_(feedback), connection_(std::move(connection)) {}
    EffectorCapabilities GetCapabilities() const override { return {false, 2.55}; }
    void Connect() override {
        connection_->Acquire();
        try {
            // Without position/status frames the finger-current frame is the liveness evidence.
            connection_->Call("effector_connect", std::string(",\"feedback\":\"") +
                (feedback_ ? "position" : "none") + "\"");
            connected_ = true;
        } catch (...) { connection_->Release(); throw; }
    }
    void Disconnect() noexcept override {
        if (connected_) connection_->Release();
        connected_ = false;
    }
    EffectorState Read() override {
        const auto node = connection_->Call("effector_read");
        EffectorState result;
        result.positions = node["positions"].as<std::vector<double>>();
        result.sample_id = node["sample_id"].as<std::uint64_t>();
        result.received_at = Clock::time_point(std::chrono::duration_cast<Clock::duration>(
            std::chrono::nanoseconds(node["received_ns"].as<std::int64_t>())));
        result.valid = node["valid"].as<bool>();
        result.moving = node["moving"].as<bool>();
        result.fault = node["fault"].as<bool>();
        result.side = node["side"].as<std::string>();
        return result;
    }
    void Submit(const EffectorPoint& point) override {
        std::ostringstream fields;
        fields << std::setprecision(17) << ",\"duration_s\":" << point.duration_s << ",\"positions\":[";
        for (std::size_t i = 0; i < point.positions.size(); ++i) {
            if (i) fields << ',';
            fields << point.positions[i];
        }
        fields << ']';
        connection_->Call("effector_submit", fields.str());
    }
private:
    bool feedback_;
    std::shared_ptr<SharedSession> connection_;
    bool connected_ = false;
};
}  // namespace
std::unique_ptr<EndEffector> MakeEffector(const EffectorConfig& config, std::shared_ptr<SharedSession> connection) {
    return std::make_unique<Revo2>(config.feedback == EffectorFeedback::kPosition, std::move(connection));
}
}  // namespace spacemit::manipulation::agx
