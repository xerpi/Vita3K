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

#include <kernel/kernel_object.h>
#include <kernel/types.h>

#include <atomic>
#include <memory>
#include <optional>
#include <string>

struct ThreadState;
using ThreadStatePtr = std::shared_ptr<ThreadState>;

// Lock order: a callback's lock, then its owner thread's mutex. Code that holds a thread's mutex
// must not take the lock of a callback.
class Callback final : public WithUidClass<KernelObject, UidClass::callback> {
public:
    // What the callback function receives about the notifications since it last ran
    struct Notification {
        SceUID notifier_id;
        uint32_t count;
        SceInt32 arg;
    };

    Callback(const ThreadStatePtr &owner, const std::string &name, Ptr<SceKernelCallbackFunction> cb_func, Ptr<void> pCommon);

    // Notifies directly, not from an event
    void direct_notify(SceInt32 notify_arg);
    // Cancels every notification sent to this callback
    void cancel();

    // UID of the event that notified the callback last
    SceUID get_notifier_id();
    // notifyArg from the last time the callback was notified
    SceInt32 get_notify_arg();
    // Number of times the callback has been notified since it last ran
    uint32_t get_num_notifications();

    // Takes the pending notifications for a run, or nothing if there are none or it's to be deleted
    std::optional<Notification> start_run();
    // Ends a run, marking the callback to be deleted if delete_it, and returns whether it should be deleted now
    bool end_run(bool delete_it);
    // Marks the callback to be deleted. Returns false if it already was.
    bool mark_for_delete();
    // Whether a run of the callback is going on
    bool running();

    const SceUID thread_id; // UID of the thread that created this callback
    const std::string name;
    const Ptr<SceKernelCallbackFunction> cb_func; // Function to execute when the callback should run
    const Ptr<void> userdata; // User-provided data - passed as pCommon

private:
    // Drops the pending notifications, so the callback never runs again
    void on_delete(SceUID deleter) override;
    // Clears the notifications, with the lock held
    void reset();

    const std::weak_ptr<ThreadState> owner; // Thread that created this callback, woken when it is notified

    uint32_t num_notifications = 0; // Number of times this callback has been notified - reset every time it is run
    SceInt32 notification_arg = 0; // User-specified argument passed by sceKernelNotifyCallback
    // UID of the last notifier, SCE_UID_INVALID_UID for a direct notify, 0 when nothing is pending
    SceUID notifier_id = 0;
    // The runs of this callback going on, more than one when it runs inside its own run
    int runs = 0;
    // The callback was deleted or returned nonzero while it ran, and goes once its last run ends
    bool delete_pending = false;
};

using CallbackPtr = std::shared_ptr<Callback>;

// A registered thread event handler, run on the threads of target for the event types in mask
class ThreadEventHandler final : public WithUidClass<KernelObject, UidClass::thread_event> {
public:
    ThreadEventHandler(SceUID target, SceUInt32 mask, Address handler, Address common);

    // Whether the handler runs on the thread for an event of type
    bool runs_for(SceUID thread_id, SceUInt32 type) const;

    // A thread UID, or SCE_KERNEL_THREAD_ID_PROCESS_ALL_ID for every thread of the process
    const SceUID target;
    const SceUInt32 mask;
    const Address handler;
    const Address common;
    // Set by a run that returned nonzero, which unregisters it. Its UID stays with its class.
    std::atomic<bool> released = false;
};
