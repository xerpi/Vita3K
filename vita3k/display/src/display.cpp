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
#include <touch/functions.h>

// Code heavily influenced by PPSSSPP's SceDisplay.cpp

static constexpr int TARGET_FPS = 60;
static constexpr int64_t TARGET_MICRO_PER_FRAME = 1000000LL / TARGET_FPS;
// how many cycles do we need to see before we start predicting the next frame
static constexpr int predict_threshold = 3;
static constexpr int max_expected_swapchain_size = 6;

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

            // Notify Vblank callback in each VBLANK start
            for (auto &[_, cb] : display.vblank_callbacks)
                cb->event_notify(cb->get_notifier_id());

            // The waits whose vblank came end here, unless a callback notify ended them first
            display.vblank_waiters.remove_if([&](auto &waiter) {
                if (waiter.entry.target_vcount > display.vblank_count)
                    return false;
                waiter.thread->end_wait(waiter.generation, SCE_KERNEL_OK);
                return true;
            });
        }
        const auto time_ms = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
        const auto time_left = TARGET_MICRO_PER_FRAME - (time_ms % TARGET_MICRO_PER_FRAME);
        std::this_thread::sleep_for(std::chrono::microseconds(time_left));
    }
}

void start_sync_thread(EmuEnvState &emuenv) {
    emuenv.display.vblank_thread = std::make_unique<std::thread>(vblank_sync_thread, std::ref(emuenv));
}

void wait_vblank(DisplayState &display, const ThreadStatePtr &wait_thread, const uint64_t target_vcount, const bool is_cb) {
    if (!wait_thread) {
        return;
    }

    // One queue entry for the whole call
    WaitQueue<VBlankWaitEntry>::Waiter waiter{ .thread = wait_thread, .entry = { target_vcount } };
    const WaitTarget target{ .type = SCE_KERNEL_WAITTYPE_EVENT, .callbacks = is_cb };
    while (true) {
        std::unique_lock<std::mutex> lock(display.mutex);
        if (target_vcount <= display.vblank_count)
            return;
        // With no deadline only a pending callback keeps the wait from registering
        if (const std::expected<uint64_t, SceInt32> registered = wait_thread->register_wait(target, Deadline::max(), nullptr)) {
            waiter.generation = *registered;
            display.vblank_waiters.push(waiter);
            // The vblank thread and a callback notify end the wait, and the thread's end ends it here
            WaitResult state = wait_thread->check_wait();
            while (state && *state != SCE_KERNEL_OK) {
                lock.unlock();
                (void)wait_thread->park(Deadline::max());
                lock.lock();
                state = wait_thread->check_wait();
            }
            if (!state)
                wait_thread->end_wait(waiter.generation, state);
            // A notify ends the wait without the queue's lock, so the waiter may still be queued
            display.vblank_waiters.remove(waiter);
            const ThreadState::WaitEnd end = wait_thread->unregister_wait(waiter.generation);
            // Nothing runs after the wait, so a thread exiting during it needs no handling here
            if (!end.result)
                return;
            if (*end.result != SCE_KERNEL_ERROR_NOTIFY_CALLBACK) {
                // The callbacks notified after the vblank ended the wait run before it returns
                lock.unlock();
                if (is_cb && wait_thread->has_pending_callbacks())
                    (void)wait_thread->run_callbacks();
                return;
            }
        }
        // The callbacks run with no lock held, before the wait looks at the vblank count again
        lock.unlock();
        if (const WaitResult ran = wait_thread->run_callbacks(); !ran)
            return;
    }
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