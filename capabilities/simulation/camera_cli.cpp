// Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
// SPDX-License-Identifier: Apache-2.0
#include "observer_client.h"

#include <filesystem>  // NOLINT(build/c++17): target explicitly requires C++17.
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <string>

namespace {
constexpr char kUsage[] =
        "Usage: manipulation-simulation-camera --output DIRECTORY "
        "[--camera NAME] [--host 127.0.0.1] [--port 9100] "
        "[--width 640] [--height 480]\n"
        "Writes rgb.ppm, depth.f32 (little-endian metres, top-left), "
        "metadata.json. DIRECTORY must not exist. Observer only.\n";

unsigned Integer(const std::string& value) {
    if (value.empty() || value.find_first_not_of("0123456789") != std::string::npos) {
        throw std::invalid_argument("expected a positive integer");
    }
    const auto parsed = std::stoul(value);
    if (parsed == 0 || parsed > 65535) throw std::invalid_argument("integer out of range");
    return static_cast<unsigned>(parsed);
}

void Save(const std::filesystem::path& path, const std::string& bytes) {
    std::ofstream stream(path, std::ios::binary);
    stream.exceptions(std::ios::badbit | std::ios::failbit);
    stream.write(bytes.data(), static_cast<std::streamsize>(bytes.size()));
    stream.close();
}
}  // namespace

int main(int argc, char** argv) {
    try {
        namespace m = spacemit::manipulation;
        std::string host = "127.0.0.1";
        unsigned port = 9100;
        std::filesystem::path destination;
        m::SimulationCaptureRequest request;
        request.camera = "overview";
        for (int index = 1; index < argc; ++index) {
            const std::string key = argv[index];
            if (key == "--help") {
                std::cout << kUsage;
                return 0;
            }
            if (index + 1 >= argc) throw std::invalid_argument("missing value for " + key);
            const std::string value = argv[++index];
            if (key == "--host")
                host = value;
            else if (key == "--port")
                port = Integer(value);
            else if (key == "--camera")
                request.camera = value;
            else if (key == "--width")
                request.width = static_cast<int>(Integer(value));
            else if (key == "--height")
                request.height = static_cast<int>(Integer(value));
            else if (key == "--output")
                destination = value;
            else
                throw std::invalid_argument("unknown argument: " + key);
        }
        if (destination.empty()) throw std::invalid_argument("--output DIRECTORY is required");
        if (std::filesystem::exists(destination)) {
            throw std::runtime_error("output directory already exists: " + destination.string());
        }
        const auto frame = m::CaptureSimulationCamera(host, port, request);
        // No file is written until the entire bounded frame has passed validation.
        if (!std::filesystem::create_directory(destination)) {
            throw std::runtime_error("could not create output directory");
        }
        const auto ppm = "P6\n" + std::to_string(frame.width) + " " + std::to_string(frame.height) +
                "\n255\n" + frame.rgb;
        Save(destination / "rgb.ppm", ppm);
        Save(destination / "depth.f32", frame.depth);
        Save(destination / "metadata.json", frame.metadata + "\n");
        std::cout << destination.string() << '\n';
        return 0;
    } catch (const std::exception& error) {
        std::cerr << error.what() << '\n';
        return 1;
    }
}
