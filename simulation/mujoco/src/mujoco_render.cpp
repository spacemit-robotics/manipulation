/*
 * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
 * SPDX-License-Identifier: Apache-2.0
 *
 * @file mujoco_render.cpp
 * @brief Main-thread graphics contexts rendering detached world snapshots only.
 */
#include "mujoco_render.h"
#include "mujoco_native_snapshot.h"
#include "mujoco_render_context.h"
#include "mujoco_world.h"

#if defined(MUJOCO_RENDER_GL4ES)
#define GL_GLEXT_PROTOTYPES
#endif
#include <GL/gl.h>
#include <mujoco/mujoco.h>
#include <sys/syscall.h>
#include <unistd.h>

#include <algorithm>
#include <cmath>
#include <exception>
#include <sstream>
#include <stdexcept>
#include <utility>

namespace mujoco_sim {
namespace {

constexpr int kMaxDimension = 4096;
constexpr int kMaxPixels = 4194304;
constexpr int kMaxGeometries = 10000;

void RequireMainThread() {
    detail::RequireRenderThread();
}

void ValidateDimensions(int width, int height) {
    if (width < 1 || height < 1 || width > kMaxDimension || height > kMaxDimension ||
            width > kMaxPixels / height) {
        throw std::invalid_argument("MuJoCo image dimensions exceed supported limits");
    }
}

void ValidateName(const std::string& name) {
    if (name.empty() || name.size() > 255 || name.find('\0') != std::string::npos) {
        throw std::invalid_argument("invalid MuJoCo camera/window name");
    }
}

void CheckGraphics(const char* operation, GLenum error = glGetError()) {
    if (error != GL_NO_ERROR) {
        std::ostringstream message;
        message << "MuJoCo OpenGL error 0x" << std::hex << error << " during " << operation;
        throw std::runtime_error(message.str());
    }
}

#if defined(MUJOCO_RENDER_GL4ES)
void ConfigureDepthReadback(mjrContext* context) {
    // GLES floating-point depth reads require floating-point storage. Match
    // mjr_resizeOffscreen so the initial framebuffer has the same format.
    GLint previous_renderbuffer = 0;
    glGetIntegerv(GL_RENDERBUFFER_BINDING, &previous_renderbuffer);
    CheckGraphics("RGB-D renderbuffer state");
    glBindRenderbuffer(GL_RENDERBUFFER, context->offDepthStencil);
    glRenderbufferStorage(GL_RENDERBUFFER, GL_DEPTH32F_STENCIL8,
            context->offWidth, context->offHeight);
    const GLenum allocation_error = glGetError();
    glBindRenderbuffer(GL_RENDERBUFFER, previous_renderbuffer);
    CheckGraphics("RGB-D floating-point depth allocation", allocation_error);
    CheckGraphics("RGB-D renderbuffer restoration");

    const int previous_buffer = context->currentBuffer;
    mjr_setBuffer(mjFB_OFFSCREEN, context);
    const GLenum status = glCheckFramebufferStatus(GL_FRAMEBUFFER);
    const GLenum framebuffer_error = glGetError();
    mjr_setBuffer(previous_buffer, context);
    CheckGraphics("RGB-D framebuffer validation", framebuffer_error);
    CheckGraphics("RGB-D framebuffer restoration");
    if (status != GL_FRAMEBUFFER_COMPLETE) {
        throw std::runtime_error("MuJoCo floating-point depth framebuffer incomplete");
    }
}
#endif

class Graphics {
public:
    Graphics(bool visible, int width, int height, const std::string& title) {
        RequireMainThread();
        ValidateDimensions(width, height);
        ValidateName(title);
        window = std::make_unique<detail::RenderContext>(visible, width, height, title);
        mjv_defaultScene(&scene);
        mjr_defaultContext(&context);
        mjv_defaultOption(&option);
        mjv_defaultCamera(&camera);
    }

    ~Graphics() {
        // Context teardown on another thread violates the ownership contract.
        if (static_cast<pid_t>(syscall(SYS_gettid)) != getpid()) std::terminate();
        window->MakeCurrent();
        ClearModel();
    }

    void ClearModel(bool clear_native = true) {
        if (ready) {
            mjr_freeContext(&context);
            mjv_freeScene(&scene);
            ready = false;
        }
        data.reset();
        if (clear_native) native.reset();
        graphics_model = nullptr;
    }

    void Load(const WorldSnapshot& snapshot, bool depth_readback = false) {
#if !defined(MUJOCO_RENDER_GL4ES)
        static_cast<void>(depth_readback);
#endif
        RequireMainThread();
        window->MakeCurrent();
        if (native) {
            native->Update(snapshot);
        } else {
            native = std::make_unique<NativeSnapshot>(snapshot);
        }
        if (graphics_model != native->Model()) {
            const auto* model = native->Model();
            ValidateDimensions(model->vis.global.offwidth, model->vis.global.offheight);
            ClearModel(false);
            data.reset(mj_makeData(native->Model()));
            if (!data) throw std::runtime_error("cannot allocate renderer data");
            mjv_makeScene(native->Model(), &scene, kMaxGeometries);
            // Mark resources owned before GL allocation so failures still clean up.
            ready = true;
            // Rendering preferences must not rewrite the model shared by the
            // world/snapshots. Only this temporary shallow descriptor is edited.
            mjModel context_model = *native->Model();
#if defined(MUJOCO_RENDER_GL4ES)
            context_model.vis.quality.offsamples = 0;
            context_model.vis.quality.shadowsize = 0;
            scene.flags[mjRND_SHADOW] = 0;
            scene.flags[mjRND_REFLECTION] = 0;
#endif
            mjr_makeContext(&context_model, &context, mjFONTSCALE_100);
            mjv_defaultFreeCamera(native->Model(), &camera);
            context.readDepthMap = mjDEPTH_ZERONEAR;
            CheckGraphics("render context initialization");
#if defined(MUJOCO_RENDER_GL4ES)
            if (depth_readback) ConfigureDepthReadback(&context);
#endif
            graphics_model = native->Model();
        }
        mj_copyData(data.get(), native->Model(), native->Data());
    }

    void UpdateScene() {
        mjv_updateScene(native->Model(), data.get(), &option, nullptr, &camera, mjCAT_ALL, &scene);
        if (scene.status != 0 || data->warning[mjWARN_VGEOMFULL].number != 0) {
            throw std::runtime_error("MuJoCo render scene capacity exceeded");
        }
    }

    std::unique_ptr<detail::RenderContext> window;
    mjvScene scene{};
    mjrContext context{};
    mjvOption option{};
    mjvCamera camera{};
    bool ready = false;
    const mjModel* graphics_model = nullptr;
    std::unique_ptr<NativeSnapshot> native;
    std::unique_ptr<mjData, decltype(&mj_deleteData)> data{nullptr, mj_deleteData};
};

void ValidateCamera(const CameraSpec& camera) {
    ValidateName(camera.name);
    ValidateDimensions(camera.width, camera.height);
    if (!std::isfinite(camera.min_depth_m) || !std::isfinite(camera.max_depth_m) ||
            camera.min_depth_m <= 0.0 || camera.max_depth_m <= camera.min_depth_m) {
        throw std::invalid_argument("invalid MuJoCo metric depth range");
    }
}

}  // namespace

class Renderer::Impl {
public:
    Graphics graphics{false, 1, 1, "MuJoCo RGB-D"};

    RgbdFrame Capture(const WorldSnapshot& snapshot, const CameraSpec& spec) {
        RequireMainThread();
        ValidateCamera(spec);
        graphics.Load(snapshot, true);
        const auto* model = graphics.native->Model();
        const int camera = mj_name2id(model, mjOBJ_CAMERA, spec.name.c_str());
        if (camera < 0) throw std::invalid_argument("MuJoCo camera not found: " + spec.name);
        if (model->cam_orthographic[camera] || model->cam_sensorsize[2 * camera] != 0.0f ||
                model->cam_sensorsize[2 * camera + 1] != 0.0f) {
            throw std::invalid_argument("only fovy-based perspective cameras are supported");
        }
        const double fovy = model->cam_fovy[camera];
        const double near = model->vis.map.znear * model->stat.extent;
        const double far = model->vis.map.zfar * model->stat.extent;
        if (!std::isfinite(fovy) || fovy <= 0.0 || fovy >= 180.0 || !std::isfinite(near) ||
                !std::isfinite(far) || near <= 0.0 || far <= near) {
            throw std::invalid_argument("invalid MuJoCo camera projection");
        }
        GLint maximum = 0;
        glGetIntegerv(GL_MAX_RENDERBUFFER_SIZE, &maximum);
        if (spec.width > maximum || spec.height > maximum) {
            throw std::invalid_argument("image exceeds OpenGL renderbuffer limit");
        }
        graphics.camera.type = mjCAMERA_FIXED;
        graphics.camera.fixedcamid = camera;
        graphics.UpdateScene();
        mjr_resizeOffscreen(spec.width, spec.height, &graphics.context);
        CheckGraphics("RGB-D framebuffer resize");
        mjr_setBuffer(mjFB_OFFSCREEN, &graphics.context);
        if (graphics.context.currentBuffer != mjFB_OFFSCREEN) {
            throw std::runtime_error("MuJoCo offscreen framebuffer unavailable");
        }
        const mjrRect viewport{0, 0, spec.width, spec.height};
        const std::size_t pixels = static_cast<std::size_t>(spec.width) * spec.height;
        std::vector<std::uint8_t> bottom_rgb(3 * pixels);
        std::vector<float> bottom_depth(pixels);
        mjr_render(viewport, &graphics.scene, &graphics.context);
        CheckGraphics("RGB-D rendering");
        mjr_readPixels(bottom_rgb.data(), bottom_depth.data(), viewport, &graphics.context);
        CheckGraphics("RGB-D readback");
        RgbdFrame frame;
        frame.width = spec.width;
        frame.height = spec.height;
        frame.rgb.resize(3 * pixels);
        frame.depth_m.resize(pixels);
        frame.sample_id = snapshot.SampleId();
        frame.sim_time = snapshot.Time();
        const double focal = 0.5 * spec.height / std::tan(fovy * std::acos(-1.0) / 360.0);
        frame.intrinsics = {focal, 0.0, (spec.width - 1) * 0.5, 0.0, focal, (spec.height - 1) * 0.5,
                0.0, 0.0, 1.0};
        const auto* native_data = graphics.native->Data();
        for (int row = 0; row < 3; ++row) {
            for (int column = 0; column < 3; ++column) {
                const double sign = column == 0 ? 1.0 : -1.0;
                frame.world_from_camera[4 * row + column] =
                        sign * native_data->cam_xmat[9 * camera + 3 * row + column];
            }
            frame.world_from_camera[4 * row + 3] = native_data->cam_xpos[3 * camera + row];
        }
        frame.world_from_camera[15] = 1.0;
        for (int row = 0; row < spec.height; ++row) {
            const std::size_t destination = static_cast<std::size_t>(row) * spec.width;
            const std::size_t source = static_cast<std::size_t>(spec.height - 1 - row) * spec.width;
            std::copy_n(bottom_rgb.data() + 3 * source, 3 * spec.width,
                    frame.rgb.data() + 3 * destination);
            for (int column = 0; column < spec.width; ++column) {
                const double buffer = bottom_depth[source + column];
                const double metric = near * far / (far - buffer * (far - near));
                if (std::isfinite(buffer) && buffer >= 0.0 && buffer < 1.0 &&
                        std::isfinite(metric) && metric >= spec.min_depth_m &&
                        metric <= spec.max_depth_m) {
                    frame.depth_m[destination + column] = static_cast<float>(metric);
                }
            }
        }
        return frame;
    }
};

Renderer::Renderer() : impl_(std::make_unique<Impl>()) {}

Renderer::~Renderer() = default;

RgbdFrame Renderer::Capture(const WorldSnapshot& snapshot, const CameraSpec& camera) {
    return impl_->Capture(snapshot, camera);
}

class Viewer::Impl {
public:
    explicit Impl(const ViewerOptions& options)
        : graphics(true, options.width, options.height, options.title) {
        graphics.window->SetMouseCallback([this](detail::MouseAction action, double dx, double dy) {
            if (!graphics.native) return;
            int motion = mjMOUSE_ROTATE_V;
            if (action == detail::MouseAction::kMove) motion = mjMOUSE_MOVE_V;
            if (action == detail::MouseAction::kZoom) motion = mjMOUSE_ZOOM;
            mjv_moveCamera(
                    graphics.native->Model(), motion, dx, dy, &graphics.scene, &graphics.camera);
        });
    }

    Graphics graphics;
};

Viewer::Viewer(const ViewerOptions& options) : impl_(std::make_unique<Impl>(options)) {}

Viewer::~Viewer() = default;

void Viewer::Draw(const WorldSnapshot& snapshot) {
    impl_->graphics.Load(snapshot);
    auto& graphics = impl_->graphics;
    graphics.window->PollEvents();
    const auto [width, height] = graphics.window->Size();
    if (width <= 0 || height <= 0) return;
    ValidateDimensions(width, height);
    graphics.UpdateScene();
    mjr_setBuffer(mjFB_WINDOW, &graphics.context);
    const mjrRect viewport{0, 0, width, height};
    mjr_render(viewport, &graphics.scene, &graphics.context);
    CheckGraphics("viewer rendering");
    graphics.window->Swap();
}

bool Viewer::ShouldClose() const {
    RequireMainThread();
    return impl_->graphics.window->ShouldClose();
}

}  // namespace mujoco_sim
