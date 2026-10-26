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

// The waits on a waitable's queue. Apart from waitable.h, since they need the whole ThreadState,
// which holds waitables of its own.

#include <kernel/thread/thread_state.h>
#include <kernel/waitable.h>

#include <algorithm>
#include <chrono>
#include <expected>
#include <optional>
#include <tuple>

// Runs thread's pending callbacks if callbacks is set, releasing the lock while they run. Returns
// WAIT_DELETE if object was deleted meanwhile.
template <typename Object>
[[nodiscard]] WaitResult run_pending_callbacks(const Object &object, std::unique_lock<std::mutex> &lock, const ThreadStatePtr &thread, bool callbacks) {
    if (!callbacks || !thread->has_pending_callbacks())
        return SCE_KERNEL_OK;
    lock.unlock();
    WaitResult r = thread->run_callbacks();
    lock.lock();
    if (r && object.was_deleted())
        r = SCE_KERNEL_ERROR_WAIT_DELETE;
    return r;
}

// Waits on queue, one of object's. Each pass runs the pending callbacks, calls take, then registers
// the wait with object's endpoint and sleeps, and a callback notify starts a new pass. take returns
// the result once the thread got what it waits for, or nullopt to keep waiting, and is passed the
// result of the last pass (SCE_KERNEL_OK the first time). Exactly one party ends each registration,
// the first to claim it (ThreadState::register_wait), and the wait's own timeout and exit go through
// object's depart_locked() like a notify's. The claim writes the time left to timeout, and a deadline
// gone before the wait is queued writes 0. finish gets the result once the wait ended, before the
// callbacks notified during it run, unless the thread exits or the callbacks at a pass's start fail.
// callbacks_check decides whether those callbacks run before the wait returns. lock holds object's
// lock.
template <typename Object, typename Entry, typename Take, typename Finish>
[[nodiscard]] WaitResult wait_loop(Object &object, WaitQueue<Entry> &queue, std::unique_lock<std::mutex> &lock, const ThreadStatePtr &thread, WaitTarget target, Entry entry, SceUInt32 *timeout, CallbacksCheck callbacks_check, Take take, Finish finish) {
    Waitable &waitable = object;
    // One waiter for the whole call, so what its entry got while queued stays across passes
    typename WaitQueue<Entry>::Waiter waiter{ .thread = thread, .entry = std::move(entry) };
    const Deadline deadline = deadline_from(timeout);
    // The result of the last pass, then the final result
    WaitResult result = SCE_KERNEL_OK;
    while (true) {
        if (const WaitResult r = run_pending_callbacks(object, lock, thread, target.callbacks); !r || *r != SCE_KERNEL_OK)
            return r;
        if (const std::optional<SceInt32> taken = take(waiter.entry, *result)) {
            result = *taken;
            if (callbacks_check == CallbacksCheck::after_take_or_wait)
                break;
            finish(*result);
            return result;
        }
        const std::expected<uint64_t, SceInt32> registered = thread->register_wait(target, deadline, timeout, object.endpoint());
        if (!registered) {
            // A callback is already pending, so the next pass runs it without sleeping, and its
            // take gets NOTIFY_CALLBACK as the last result
            if (registered.error() == SCE_KERNEL_ERROR_NOTIFY_CALLBACK) {
                result = SCE_KERNEL_ERROR_NOTIFY_CALLBACK;
                continue;
            }
            if (timeout)
                *timeout = 0;
            result = registered.error();
            break;
        }
        waiter.priority = thread->get_priority();
        waiter.generation = *registered;
        queue.push(waiter);
        // Each look reads the registration under the lock before anything else, so the waitable is
        // brought up to date only for a wait that still waits on a thread not exiting
        WaitResult state;
        while (true) {
            state = thread->check_wait();
            if (!state || *state == SCE_KERNEL_OK)
                break;
            const Deadline now = std::chrono::steady_clock::now();
            // What was due by the deadline comes first, in the same hold as the timeout's claim
            if (now >= deadline) {
                waitable.recheck_locked(*thread, deadline);
                break;
            }
            const Deadline look = waitable.recheck_locked(*thread, now);
            lock.unlock();
            // A recheck asked with wake(), the wait's own look or the deadline ends a park early,
            // and only the next look decides
            std::ignore = thread->park(std::min(look, deadline));
            lock.lock();
        }
        // The thread's exit or the deadline ends the wait, unless a claim already did
        if ((!state || *state == SCE_KERNEL_ERROR_WAIT_TIMEOUT) && waitable.depart_locked(*thread, waiter.generation, state ? Departure::timeout : Departure::exit))
            waitable.serve_locked();
        // A notify ends a wait with no endpoint without the lock, so it may still be queued
        queue.remove(waiter);
        const ThreadState::WaitEnd end = thread->unregister_wait(waiter.generation);
        if (!end.result)
            return end.result;
        result = end.result;
        if (*result != SCE_KERNEL_ERROR_NOTIFY_CALLBACK)
            break;
    }
    finish(*result);
    if (callbacks_check == CallbacksCheck::never || (callbacks_check == CallbacksCheck::on_delete && *result != SCE_KERNEL_ERROR_WAIT_DELETE))
        return result;
    // Run the callbacks notified during the wait. Only the thread exiting changes the result.
    if (const WaitResult ran = run_pending_callbacks(object, lock, thread, target.callbacks); !ran)
        return ran;
    return result;
}

// Like wait_loop() with finish, for a wait whose end does nothing more
template <typename Object, typename Entry, typename Take>
[[nodiscard]] WaitResult wait_loop(Object &object, WaitQueue<Entry> &queue, std::unique_lock<std::mutex> &lock, const ThreadStatePtr &thread, WaitTarget target, Entry entry, SceUInt32 *timeout, CallbacksCheck callbacks_check, Take take) {
    return wait_loop(object, queue, lock, thread, target, std::move(entry), timeout, callbacks_check, take, [](SceInt32) {});
}

// Ends thread's wait of generation for reason if it is queued in queue, and takes it off the queue.
// outputs gives the queued waiter what that end leaves it before it goes, unless the thread exits.
// Returns whether this ended the wait: false if the wait isn't queued here, or if a notify already
// ended one with no endpoint. With the lock held.
template <typename Entry, typename Outputs>
bool depart_queued(WaitQueue<Entry> &queue, ThreadState &thread, uint64_t generation, Departure reason, Outputs outputs) {
    auto *const waiter = queue.find_if([&](auto &w) { return w.thread.get() == &thread && w.generation == generation; });
    // A claim already took it off the queue, or the thread waits again with a new generation
    if (!waiter)
        return false;
    const bool ended = thread.end_wait(generation, departure_result(reason));
    if (ended && reason != Departure::exit)
        outputs(*waiter);
    queue.remove(*waiter);
    return ended;
}

// Like depart_queued() with outputs, for a wait whose end leaves it nothing
template <typename Entry>
bool depart_queued(WaitQueue<Entry> &queue, ThreadState &thread, uint64_t generation, Departure reason) {
    return depart_queued(queue, thread, generation, reason, [](auto &) {});
}
