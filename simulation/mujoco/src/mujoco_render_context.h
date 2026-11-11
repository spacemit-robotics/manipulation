/*
 * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
 * SPDX-License-Identifier: Apache-2.0
 *
 * @file mujoco_render_context.h
 * @brief Internal main-thread window/context abstraction, without physics state.
 */
#ifndef MUJOCO_RENDER_CONTEXT_H
#define MUJOCO_RENDER_CONTEXT_H

#include <functional>
#include <memory>
#include <string>
#include <utility>

namespace mujoco_sim::detail {

enum class MouseAction { kRotate, kMove, kZoom };

/** All operations and destruction must occur on the process main thread. */
class RenderContext {
public:
    using MouseCallback = std::function<void(MouseAction, double, double)>;
    RenderContext(bool visible, int width, int height, const std::string& title);
    ~RenderContext();
    RenderContext(const RenderContext&) = delete;
    RenderContext& operator=(const RenderContext&) = delete;
    void MakeCurrent();
    void Swap();
    void PollEvents();
    std::pair<int, int> Size() const;
    bool ShouldClose() const;
    void SetMouseCallback(MouseCallback callback);

private:
    class Impl;
    std::unique_ptr<Impl> impl_;
};

void RequireRenderThread();

}  // namespace mujoco_sim::detail

#endif  // MUJOCO_RENDER_CONTEXT_H
