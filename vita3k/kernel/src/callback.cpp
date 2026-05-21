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

#include <kernel/callback.h>
#include <kernel/state.h>
#include <kernel/thread/thread_state.h>

#include <mutex>
#include <vector>

uint32_t ThreadState::process_callbacks() {
    {
        std::lock_guard lock(mutex);
        callbacks_pending = false;
    }

    // Walk a snapshot: dispatched guest code may create/delete callbacks.
    std::vector<CallbackPtr> snapshot;
    {
        std::lock_guard lock(kernel.mutex);
        snapshot.assign(callbacks.begin(), callbacks.end());
    }

    uint32_t num_dispatched = 0;
    for (const CallbackPtr &cb : snapshot) {
        const Callback::ExecuteResult res = cb->execute(*this);
        if (res == Callback::ExecuteResult::not_pending)
            continue;
        ++num_dispatched;
        if (res == Callback::ExecuteResult::delete_self) {
            std::lock_guard lock(kernel.mutex);
            std::erase(callbacks, cb);
            std::erase_if(kernel.callbacks, [&](const auto &kv) { return kv.second == cb; });
        }
        std::lock_guard lock(mutex);
        if (exit_request || destroy_requested)
            break;
    }
    return num_dispatched;
}

Callback::Info Callback::info() {
    std::lock_guard lock(mutex);
    return Info{
        .notifier_id = notifier_id,
        .notify_arg = notification_arg,
        .num_notifications = num_notifications,
    };
}

void Callback::notify(KernelState &kernel, SceUID notifier_id, SceInt32 notify_arg) {
    {
        std::lock_guard lock(mutex);
        this->notifier_id = notifier_id;
        this->notification_arg = notify_arg;
        ++this->num_notifications;
    }

    const ThreadStatePtr thread = kernel.get_thread(this->thread_id);
    if (!thread)
        return;

    {
        std::lock_guard lock(thread->mutex);
        thread->callbacks_pending = true;
    }
    thread->unpark();
}

void Callback::cancel() {
    std::lock_guard lock(mutex);
    num_notifications = 0;
    notifier_id = SCE_UID_INVALID_UID;
}

Callback::ExecuteResult Callback::execute(ThreadState &thread) {
    SceUID snap_notifier;
    uint32_t snap_num;
    SceInt32 snap_arg;
    {
        std::lock_guard lock(mutex);
        if (num_notifications == 0)
            return ExecuteResult::not_pending;
        snap_notifier = notifier_id;
        snap_num = num_notifications;
        snap_arg = notification_arg;
        num_notifications = 0;
        notifier_id = SCE_UID_INVALID_UID;
    }

    const std::vector<uint32_t> args = {
        static_cast<uint32_t>(snap_notifier),
        snap_num,
        static_cast<uint32_t>(snap_arg),
        userdata.address(),
    };
    const uint32_t ret = thread.call_guest(cb_func.address(), RegisterArgs{ args });
    return ret != 0 ? ExecuteResult::delete_self : ExecuteResult::handled;
}
