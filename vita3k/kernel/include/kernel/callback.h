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

#pragma once

#include <kernel/types.h>
#include <mutex>

#define SCE_UID_INVALID_UID (SceUID)(0xFFFFFFFF)

struct KernelState;
struct ThreadState;
typedef std::shared_ptr<ThreadState> ThreadStatePtr;

// A guest-installed callback. Pending notifications are coalesced (count +
// last notifier_id + last notify_arg). The owning thread drains them at
// callback-aware boundaries (process_callbacks).
struct Callback {
    Callback(SceUID thread_id, std::string name, Ptr<SceKernelCallbackFunction> cb_func, Ptr<void> pCommon)
        : thread_id(thread_id)
        , name(std::move(name))
        , cb_func(cb_func)
        , userdata(pCommon) {}

    // Immutable, set at construction. No locking needed.
    SceUID get_owner_thread_id() const { return thread_id; }
    const std::string &get_name() const { return name; }
    Ptr<SceKernelCallbackFunction> get_callback_function() const { return cb_func; }
    Ptr<void> get_user_common_ptr() const { return userdata; }

    // Snapshot of the mutable state for sceKernelGetCallbackInfo /
    // sceKernelGetCallbackCount.
    struct Info {
        SceUID notifier_id;
        SceInt32 notify_arg;
        uint32_t num_notifications;
    };
    Info info();

    // Coalesce a notification. notifier_id == SCE_UID_INVALID_UID for direct
    // (non-event) notifications from sceKernelNotifyCallback. Wakes the owning
    // thread via callbacks_pending + unpark.
    void notify(KernelState &kernel, SceUID notifier_id, SceInt32 notify_arg);

    // Discard pending notifications. Does not interrupt an in-progress run.
    void cancel();

    enum class ExecuteResult {
        not_pending, // No notifications coalesced, nothing ran.
        handled, // Ran, function returned 0.
        delete_self, // Ran, function returned non-zero (auto-delete contract).
    };
    // Drain coalesced notifications by invoking the guest function on the
    // owning thread. Resets the pending state regardless of the return value.
    ExecuteResult execute(ThreadState &thread);

private:
    const SceUID thread_id;
    const std::string name;
    const Ptr<SceKernelCallbackFunction> cb_func;
    const Ptr<void> userdata;

    std::mutex mutex;
    uint32_t num_notifications = 0;
    SceInt32 notification_arg = 0;
    SceUID notifier_id = SCE_UID_INVALID_UID;
};

typedef std::shared_ptr<Callback> CallbackPtr;
