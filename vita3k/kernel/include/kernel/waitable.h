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

#include <kernel/thread/wait.h>
#include <kernel/thread/wait_queue.h>

#include <cstdint>
#include <mutex>

// When a wait runs the callbacks notified during it, before it returns
enum class CallbacksCheck {
    // Unless take succeeded
    after_wait,
    // Also when take succeeded
    after_take_or_wait,
    // Only when a delete ended the sleep
    on_delete,
    // Never
    never,
};

// The object side of a wait: the queues a thread waits in, and what leaving them does. It has no
// identity of its own. A sync object adds one, and a thread's end and the display's vblank are
// waitables inside their owners.
class Waitable {
public:
    Waitable(const Waitable &) = delete;
    Waitable &operator=(const Waitable &) = delete;

    // Ends thread's registered wait of generation for a callback notify, takes it off the queue and
    // serves the waiters behind it, if that wait is still queued here. A later wait of the thread,
    // and a wait that a delete or another claim already ended, are left alone. The one way into a
    // waitable from outside its own calls, through the endpoint of a registration. Takes the lock.
    void depart_waiter(ThreadState &thread, uint64_t generation) {
        const std::lock_guard<std::mutex> guard(lock_domain);
        if (depart_locked(thread, generation, Departure::callback))
            serve_locked();
    }

protected:
    // lock is the waitable's lock domain, its own or one it shares, for as long as it lives
    explicit Waitable(std::mutex &lock)
        : lock_domain(lock) {}
    ~Waitable() = default;

    // Ends thread's queued wait of generation for reason, gives it the outputs of that end unless the
    // thread exits, and takes it off the queue. Returns whether that ended the wait. With the lock
    // held.
    virtual bool depart_locked(ThreadState &thread, uint64_t generation, Departure reason) = 0;
    // Hands out what the waiters can get now, in queue order. With the lock held.
    virtual void serve_locked() {}
    // Brings the waitable up to date for thread's queued wait, firing what was due by until, and
    // returns when that wait has to look again by itself, the maximum if only the waitable's own
    // operations end it. Only a timer does anything. With the lock held.
    virtual Deadline recheck_locked(const ThreadState &thread, Deadline until) { return Deadline::max(); }

    template <typename Object, typename Entry, typename Take, typename Finish>
    friend WaitResult wait_loop(Object &object, WaitQueue<Entry> &queue, std::unique_lock<std::mutex> &lock, const ThreadStatePtr &thread, WaitTarget target, Entry entry, SceUInt32 *timeout, CallbacksCheck callbacks_check, Take take, Finish finish);

    std::mutex &lock_domain;
};
