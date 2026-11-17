/*
 * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
 * SPDX-License-Identifier: Apache-2.0
 *
 * @file mujoco_render_context.cpp
 * @brief GLFW or native X11/GLX context ownership and orbit input.
 */
#include "mujoco_render_context.h"

#if defined(MUJOCO_RENDER_CONTEXT_X11)
#include <GL/glx.h>
#include <X11/Xlib.h>
#include <X11/keysym.h>
#else
#include <GLFW/glfw3.h>
#endif
#include <sys/syscall.h>
#include <unistd.h>

#include <exception>
#include <stdexcept>
#include <utility>

namespace mujoco_sim::detail {
namespace {

#if !defined(MUJOCO_RENDER_CONTEXT_X11)
int glfw_users = 0;
#endif

}  // namespace

void RequireRenderThread() {
    if (static_cast<pid_t>(syscall(SYS_gettid)) != getpid()) {
        throw std::logic_error("MuJoCo graphics must run on the process main thread");
    }
}

class RenderContext::Impl {
public:
    Impl(bool visible, int width, int height, const std::string& title)
        : width_(width), height_(height) {
        RequireRenderThread();
        try {
            Init(visible, title);
        } catch (...) {
            Cleanup();
            throw;
        }
    }

    ~Impl() {
        if (static_cast<pid_t>(syscall(SYS_gettid)) != getpid()) std::terminate();
        Cleanup();
    }

    void Move(double x, double y, bool left, bool right) {
        const double dx = (x - last_x_) / height_;
        const double dy = (y - last_y_) / height_;
        last_x_ = x;
        last_y_ = y;
        if (!mouse_ || (!left && !right)) return;
        const auto action = right ? MouseAction::kMove : MouseAction::kRotate;
        mouse_(action, dx, dy);
    }

    void Zoom(double offset) {
        if (mouse_) mouse_(MouseAction::kZoom, 0.0, -0.05 * offset);
    }

#if defined(MUJOCO_RENDER_CONTEXT_X11)
    void Init(bool visible, const std::string& title) {
        // All new renderer Xlib work is main-thread only. Keep Xlib compatible
        // with applications which have other display connections on workers.
        if (!XInitThreads()) throw std::runtime_error("cannot initialize Xlib threading");
        display_ = XOpenDisplay(nullptr);
        if (!display_) throw std::runtime_error("X11 display unavailable; check DISPLAY");
        int attributes[] = {
                GLX_RGBA, GLX_DEPTH_SIZE, 24, GLX_STENCIL_SIZE, 8, GLX_DOUBLEBUFFER, None};
        XVisualInfo* visual = glXChooseVisual(display_, DefaultScreen(display_), attributes);
        if (!visual) throw std::runtime_error("no GLX visual with depth/stencil buffers");
        const Window root = DefaultRootWindow(display_);
        colormap_ = XCreateColormap(display_, root, visual->visual, AllocNone);
        XSetWindowAttributes settings{};
        settings.colormap = colormap_;
        settings.event_mask = ExposureMask | KeyPressMask | ButtonPressMask | ButtonReleaseMask |
                PointerMotionMask | StructureNotifyMask;
        window_ = XCreateWindow(display_, root, 0, 0, width_, height_, 0, visual->depth,
                InputOutput, visual->visual, CWColormap | CWEventMask, &settings);
        context_ = glXCreateContext(display_, visual, nullptr, GL_TRUE);
        XFree(visual);
        if (!window_ || !context_) throw std::runtime_error("cannot create X11/GLX context");
        XStoreName(display_, window_, title.c_str());
        close_message_ = XInternAtom(display_, "WM_DELETE_WINDOW", False);
        XSetWMProtocols(display_, window_, &close_message_, 1);
        if (visible) XMapWindow(display_, window_);
        XFlush(display_);
        MakeCurrent();
    }

    void Cleanup() noexcept {
        if (display_ && context_) {
            glXMakeCurrent(display_, None, nullptr);
            glXDestroyContext(display_, context_);
        }
        if (display_ && window_) XDestroyWindow(display_, window_);
        if (display_ && colormap_) XFreeColormap(display_, colormap_);
        if (display_) XCloseDisplay(display_);
        display_ = nullptr;
        context_ = nullptr;
        window_ = 0;
        colormap_ = 0;
    }

    void MakeCurrent() {
        if (!glXMakeCurrent(display_, window_, context_)) {
            throw std::runtime_error("cannot bind X11/GLX context");
        }
    }

    void Swap() { glXSwapBuffers(display_, window_); }

    void PollEvents() {
        while (XPending(display_)) {
            XEvent event;
            XNextEvent(display_, &event);
            if (event.type == ClientMessage &&
                    static_cast<Atom>(event.xclient.data.l[0]) == close_message_) {
                close_ = true;
            } else if (event.type == DestroyNotify) {
                close_ = true;
            } else if (event.type == ConfigureNotify) {
                width_ = event.xconfigure.width;
                height_ = event.xconfigure.height;
            } else if (event.type == KeyPress && XLookupKeysym(&event.xkey, 0) == XK_Escape) {
                close_ = true;
            } else if (event.type == MotionNotify && height_ > 0) {
                Move(event.xmotion.x, event.xmotion.y, event.xmotion.state & Button1Mask,
                        event.xmotion.state & Button3Mask);
            } else if (event.type == ButtonPress) {
                last_x_ = event.xbutton.x;
                last_y_ = event.xbutton.y;
                if (event.xbutton.button == Button4) Zoom(1.0);
                if (event.xbutton.button == Button5) Zoom(-1.0);
            }
        }
    }

    std::pair<int, int> Size() const {
        XWindowAttributes attributes{};
        if (!XGetWindowAttributes(display_, window_, &attributes)) {
            throw std::runtime_error("cannot read X11 window size");
        }
        return {attributes.width, attributes.height};
    }

    bool ShouldClose() const { return close_; }

    Display* display_ = nullptr;
    Window window_ = 0;
    Colormap colormap_ = 0;
    GLXContext context_ = nullptr;
    Atom close_message_ = 0;
#else
    void Init(bool visible, const std::string& title) {
        if (glfw_users == 0 && !glfwInit()) {
            throw std::runtime_error("GLFW initialization failed; a working display is required");
        }
        ++glfw_users;
        glfw_active_ = true;
        glfwDefaultWindowHints();
        glfwWindowHint(GLFW_VISIBLE, visible ? GLFW_TRUE : GLFW_FALSE);
        glfwWindowHint(GLFW_DEPTH_BITS, 24);
        glfwWindowHint(GLFW_STENCIL_BITS, 8);
        glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 2);
        glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 1);
        window_ = glfwCreateWindow(width_, height_, title.c_str(), nullptr, nullptr);
        glfwDefaultWindowHints();
        if (!window_) throw std::runtime_error("cannot create GLFW compatibility context");
        MakeCurrent();
        glfwSwapInterval(0);
        glfwSetWindowUserPointer(window_, this);
        glfwSetCursorPosCallback(window_, [](GLFWwindow* window, double x, double y) {
            auto* self = static_cast<Impl*>(glfwGetWindowUserPointer(window));
            glfwGetWindowSize(window, &self->width_, &self->height_);
            if (self->height_ > 0) {
                self->Move(x, y, glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS,
                        glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_RIGHT) == GLFW_PRESS);
            }
        });
        glfwSetScrollCallback(window_, [](GLFWwindow* window, double, double offset) {
            static_cast<Impl*>(glfwGetWindowUserPointer(window))->Zoom(offset);
        });
    }

    void Cleanup() noexcept {
        if (window_) glfwDestroyWindow(window_);
        window_ = nullptr;
        if (glfw_active_ && --glfw_users == 0) glfwTerminate();
        glfw_active_ = false;
    }

    void MakeCurrent() { glfwMakeContextCurrent(window_); }

    void Swap() { glfwSwapBuffers(window_); }

    void PollEvents() { glfwPollEvents(); }

    std::pair<int, int> Size() const {
        int width = 0;
        int height = 0;
        glfwGetFramebufferSize(window_, &width, &height);
        return {width, height};
    }

    bool ShouldClose() const { return glfwWindowShouldClose(window_) != 0; }

    GLFWwindow* window_ = nullptr;
    bool glfw_active_ = false;
#endif
    int width_ = 0;
    int height_ = 0;
    bool close_ = false;
    double last_x_ = 0.0;
    double last_y_ = 0.0;
    MouseCallback mouse_;
};

RenderContext::RenderContext(bool visible, int width, int height, const std::string& title)
    : impl_(std::make_unique<Impl>(visible, width, height, title)) {}

RenderContext::~RenderContext() = default;

void RenderContext::MakeCurrent() {
    RequireRenderThread();
    impl_->MakeCurrent();
}

void RenderContext::Swap() {
    RequireRenderThread();
    impl_->Swap();
}

void RenderContext::PollEvents() {
    RequireRenderThread();
    impl_->PollEvents();
}

std::pair<int, int> RenderContext::Size() const {
    RequireRenderThread();
    return impl_->Size();
}

bool RenderContext::ShouldClose() const {
    RequireRenderThread();
    return impl_->ShouldClose();
}

void RenderContext::SetMouseCallback(MouseCallback callback) {
    RequireRenderThread();
    impl_->mouse_ = std::move(callback);
}

}  // namespace mujoco_sim::detail
