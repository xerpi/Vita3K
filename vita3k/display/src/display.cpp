// Vita3K emulator project
// Copyright (C) 2026 Vita3K team
//
// This program is free software; you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation; either version 2 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License along
// with this program; if not, write to the Free Software Foundation, Inc.,
// 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.

#include <display/functions.h>

#include <dialog/state.h>
#include <display/state.h>
#include <emuenv/state.h>
#include <kernel/state.h>
#include <renderer/state.h>

#include <chrono>
#include <motion/functions.h>
#include <optional>
#include <touch/functions.h>

// Code heavily influenced by PPSSSPP's SceDisplay.cpp

static constexpr int TARGET_FPS = 60;
static constexpr int64_t TARGET_MICRO_PER_FRAME = 1000000LL / TARGET_FPS;
// how many cycles do we need to see before we start predicting the next frame
static constexpr int predict_threshold = 3;
static constexpr int max_expected_swapchain_size = 6;

// Notifies the vblank callbacks in ascending UID order. Each callback is looked up under the
// display lock and notified without it, because a notify may take the lock of the object its thread
// waits on. The pass stops at the highest UID registered when it starts. A callback registered
// during the pass is notified if its UID is above the current one and within that limit. A callback
// unregistered before the pass reaches it is skipped. One unregistered after it was looked up still
// gets its notify.
static void notify_vblank_callbacks(DisplayState &display) {
    // The highest UID this pass notifies, and the UID of the last callback looked up
    std::optional<SceUID> end;
    std::optional<SceUID> last;
    while (true) {
        CallbackPtr cb;
        {
            const std::lock_guard<std::mutex> guard(display.mutex);
            const auto &callbacks = display.vblank_callbacks;
            if (!end) {
                if (callbacks.empty())
                    return;
                end = callbacks.rbegin()->first;
            }
            const auto it = last ? callbacks.upper_bound(*last) : callbacks.begin();
            if (it == callbacks.end() || it->first > *end)
                return;
            last = it->first;
            cb = it->second;
        }
        cb->notify(cb->get_notifier_id(), 0);
    }
}

static void vblank_sync_thread(EmuEnvState &emuenv) {
    DisplayState &display = emuenv.display;

    while (!display.abort.load()) {
        {
            const std::lock_guard<std::mutex> guard(display.mutex);

            {
                const std::lock_guard<std::mutex> guard_info(display.display_info_mutex);
                ++display.vblank_count;

                // in this case, even though no new game frames are being rendered, we still need to update the screen
                if (emuenv.kernel.is_threads_paused() || (emuenv.common_dialog.status == SCE_COMMON_DIALOG_STATUS_RUNNING))
                    // only display the UI/common dialog at 30 fps
                    // this is necessary so that the command buffer processing doesn't get starved
                    // with vsync enabled and a screen with a refresh rate of 60Hz or less
                    if (display.vblank_count % 2 == 0)
                        emuenv.renderer->should_display = true;
            }

            // maybe we should also use a mutex for this part, but it shouldn't be an issue
            touch_vsync_update(emuenv);
            refresh_motion(emuenv.motion, emuenv.ctrl);
        }
        // Notify the callbacks before ending the vblank waits
        notify_vblank_callbacks(display);
        {
            const std::lock_guard<std::mutex> guard(display.mutex);
            // End the waits whose vblank came
            auto &waiters = display.vblank_waiters;
            for (auto it = waiters.begin(); it != waiters.end();) {
                auto &node = *it++;
                if (*node.request.target <= display.vblank_count)
                    waiters.complete(node);
            }
        }
        const auto time_ms = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
        const auto time_left = TARGET_MICRO_PER_FRAME - (time_ms % TARGET_MICRO_PER_FRAME);
        std::this_thread::sleep_for(std::chrono::microseconds(time_left));
    }
}

void start_sync_thread(EmuEnvState &emuenv) {
    emuenv.display.vblank_thread = std::make_unique<std::thread>(vblank_sync_thread, std::ref(emuenv));
}

// Waits for the vblank the request asks for. Nothing runs after the wait, so a thread exiting during it
// needs no handling here.
static void wait_for_vblank(DisplayState &display, ThreadState &thread, VBlankRequest request, bool is_cb) {
    // A vblank wait shows in thread info as an event wait with no object
    WaitOptions options(nullptr, is_cb);
    QueuedThreadWait wait(display, display.vblank_waiters, std::unique_lock{ display.mutex }, thread, { .type = SCE_KERNEL_WAITTYPE_EVENT }, request, options);
    VBlankRequest &vblank = wait.node.request;
    // The display is never deleted
    std::ignore = wait.run(FinalCallbackCheck::after_wait, [&]() -> std::optional<SceInt32> {
        // A since_setbuf wait counts from the last sceDisplaySetFrameBuf. Other waits count from the
        // vblank where the thread's last display wait ended, and wait for at least one vblank.
        if (!vblank.target)
            vblank.target = vblank.since_setbuf ? display.last_setframe_vblank_count + vblank.vcount : thread.next_display_wait(display.vblank_count + 1, vblank.vcount);
        if (*vblank.target > display.vblank_count)
            return std::nullopt;
        return SCE_KERNEL_OK;
    });
}

void wait_vblank(DisplayState &display, const ThreadStatePtr &wait_thread, uint64_t target_vcount, bool is_cb) {
    if (wait_thread)
        wait_for_vblank(display, *wait_thread, VBlankRequest{ .target = target_vcount }, is_cb);
}

void wait_display_vblank(DisplayState &display, const ThreadStatePtr &wait_thread, int vcount, bool since_setbuf, bool is_cb) {
    if (wait_thread)
        wait_for_vblank(display, *wait_thread, VBlankRequest{ .vcount = vcount, .since_setbuf = since_setbuf }, is_cb);
}

static void reset_swapchain_cycle(DisplayState &display, Address sync_object) {
    display.predicted_frames.resize(1);
    display.predicted_frames[0].sync_object = sync_object;
    display.predicted_frame_position = 0;
    display.predicted_cycles_seen = 0;
}

DisplayFrameInfo *predict_next_image(EmuEnvState &emuenv, Address sync_object) {
    auto &display = emuenv.display;
    std::lock_guard<std::mutex> lock(display.display_info_mutex);

    if (display.predicted_cycles_seen >= predict_threshold) {
        // just check that the next sync_object in line is the one we expect
        display.predicted_frame_position = (display.predicted_frame_position + 1) % display.predicted_frames.size();
        if (display.predicted_frames[display.predicted_frame_position].sync_object != sync_object)
            // bad, this isn't what we expect
            reset_swapchain_cycle(display, sync_object);

    } else if (display.predicted_cycles_seen >= 1) {
        display.predicted_frame_position = (display.predicted_frame_position + 1) % display.predicted_frames.size();

        if (display.predicted_frame_position == 0)
            display.predicted_cycles_seen++;

        if (display.predicted_frames[display.predicted_frame_position].sync_object != sync_object)
            // bad, this isn't what we expect
            reset_swapchain_cycle(display, sync_object);
    } else {
        // check if we have a cycle
        bool has_cycle = false;
        for (int idx = 0; idx < display.predicted_frames.size(); idx++) {
            if (display.predicted_frames[idx].sync_object == sync_object) {
                // we found a cycle
                has_cycle = true;
                display.predicted_frames.erase(display.predicted_frames.begin(), display.predicted_frames.begin() + idx);
                display.predicted_frame_position = 0;
                display.predicted_cycles_seen = 1;
                break;
            }
        }

        if (!has_cycle) {
            // predicted_frame_position is initialized to -1, so this is fine
            display.predicted_frame_position++;
            if (display.predicted_frame_position == display.predicted_frames.size()) {
                // keep the last max_expected_swapchain_size frames for the swapchain cycle
                if (display.predicted_frames.size() == max_expected_swapchain_size) {
                    display.predicted_frame_position = 0;
                } else {
                    display.predicted_frames.resize(display.predicted_frames.size() + 1);
                }
            }
            display.predicted_frames[display.predicted_frame_position].sync_object = sync_object;
        }
    }

    bool predict = display.predicted_cycles_seen >= predict_threshold;
    DisplayFrameInfo *frame = nullptr;
    if (predict) {
        // set the next framebuffer image here
        frame = new DisplayFrameInfo;
        *frame = display.predicted_frames[display.predicted_frame_position].frame_info;
    }

    return frame;
}

void update_prediction(EmuEnvState &emuenv, DisplayFrameInfo &frame) {
    auto &display = emuenv.display;
    std::lock_guard<std::mutex> lock(display.display_info_mutex);
    Address sync_object = display.current_sync_object;

    if (!display.predicting) {
        display.next_rendered_frame = frame;
        emuenv.renderer->should_display = true;
    }

    for (auto &pred_frame : display.predicted_frames) {
        if (pred_frame.sync_object != sync_object)
            continue;

        if (memcmp(&pred_frame.frame_info, &frame, sizeof(DisplayFrameInfo)) == 0)
            // we got what we expected, fine
            return;

        pred_frame.frame_info = frame;
        break;
    }

    if (display.predicting) {
        LOG_TRACE("Mispredicted the next swapchain image");
        display.next_rendered_frame = frame;
        emuenv.renderer->should_display = true;
    }

    // let predict_next_image reset the cycle if necessary
    display.predicted_cycles_seen = std::min(display.predicted_cycles_seen, 1U);
}

void DisplayState::deinit() {
    abort = true;
    if (vblank_thread && vblank_thread->joinable())
        vblank_thread->join();

    vblank_thread.reset();
    abort = false;

    {
        const std::lock_guard<std::mutex> guard(mutex);
        vblank_callbacks.clear();
    }

    {
        const std::lock_guard<std::mutex> guard(display_info_mutex);
        sce_frame = {};
        next_rendered_frame = {};
    }

    predicted_frames.clear();
    predicted_frame_position = static_cast<uint32_t>(-1);
    predicted_cycles_seen = 0;
    predicting = false;
    current_sync_object = 0;

    vblank_count = 0;
    last_setframe_vblank_count = 0;

    fps_hack = false;
    // pretty sure we set this on game boot
    fullscreen = false;
}