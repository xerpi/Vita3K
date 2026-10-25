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
#include <kernel/types.h>
#include <util/intrusive_list.h>

#include <cassert>
#include <concepts>
#include <cstddef>
#include <memory>

// Only declared here so ThreadState can hold a WaitQueue. The template bodies need the full type.
struct ThreadState;
using ThreadStatePtr = std::shared_ptr<ThreadState>;

// Threads blocked on a sync object, in FIFO or thread priority order.
// Entry is the per-waiter data the object needs, such as a requested count.
// Every method must be called with the object's lock held. The queue links the waiters it holds
// through them, so queueing allocates nothing, and a waiter must stay where it is while queued.
template <typename Entry>
class WaitQueue {
public:
    struct Waiter {
        ThreadStatePtr thread;
        // Taken when the wait starts, the queue is not reordered if it changes
        int priority = 0;
        Entry entry;
        // The thread's registration of the wait (ThreadState::register_wait)
        uint64_t generation = 0;
        // Its place in the queue, which keeps a queued waiter where it is
        IntrusiveLink<Waiter> link;
    };

    // How the queue orders its waiters
    enum class Order {
        fifo,
        priority,
    };

    WaitQueue() = default;
    explicit WaitQueue(SceUInt32 attr)
        : by_priority(attr & SCE_KERNEL_ATTR_TH_PRIO) {}
    explicit WaitQueue(Order order)
        : by_priority(order == Order::priority) {}

    // The waiters in queue order
    auto begin() const {
        return waiters.begin();
    }
    auto end() const {
        return waiters.end();
    }

    bool empty() const {
        return waiters.empty();
    }

    std::size_t size() const {
        return waiters.size();
    }

    // Returns the waiter that would be woken first, or null.
    Waiter *front() {
        return waiters.empty() ? nullptr : &waiters.front();
    }

    // Returns whether a thread of this priority would be first in the queue, ahead of every waiter.
    bool first_for(int priority) const {
        if (waiters.empty())
            return true;
        // Lower value is higher priority, and equal priorities keep FIFO order
        return by_priority && waiters.front().priority > priority;
    }

    // Returns the first waiter accepted by pick, or null.
    Waiter *find_if(std::predicate<Waiter &> auto pick) {
        for (Waiter &w : waiters)
            if (pick(w))
                return &w;
        return nullptr;
    }

    // Returns how many waiters pick accepts.
    std::size_t count_if(std::predicate<const Waiter &> auto pick) const {
        std::size_t accepted = 0;
        for (const Waiter &w : waiters)
            if (pick(w))
                ++accepted;
        return accepted;
    }

    // Takes every waiter that pick accepts off the queue, in queue order. pick may end the waiter's
    // wait first.
    void remove_if(std::predicate<Waiter &> auto pick) {
        for (auto it = waiters.begin(); it != waiters.end();) {
            if (pick(*it))
                it = waiters.erase(it);
            else
                ++it;
        }
    }

    // Ends the registered wait of waiter with result, for a pick of remove_if() that takes it off the
    // queue.
    void claim(Waiter &waiter, SceInt32 result) {
        // Every claim takes its waiter off the queue before it lets go of the object's lock, so a
        // queued registration is still waiting
        [[maybe_unused]] const bool ended = waiter.thread->end_wait(waiter.generation, result);
        assert(ended);
    }

    // Ends the registered wait of waiter with result and takes it off the queue.
    void end(Waiter &waiter, SceInt32 result = SCE_KERNEL_OK) {
        remove(waiter);
        claim(waiter, result);
    }

    // Ends the registered wait of every waiter with result, in queue order. Returns how many there were.
    std::size_t end_all(SceInt32 result) {
        std::size_t ended = 0;
        while (Waiter *waiter = front()) {
            end(*waiter, result);
            ++ended;
        }
        return ended;
    }

    // Queues a waiter that the caller will block itself. Used by waits that need their own loop.
    void push(Waiter &waiter) {
        auto pos = waiters.end();
        if (by_priority) {
            // Lower value is higher priority, equal priorities keep FIFO order
            pos = waiters.begin();
            while (pos != waiters.end() && pos->priority <= waiter.priority)
                ++pos;
            // A waiter whose place is the front goes right behind a head that keeps its place, so
            // it lands ahead of the waiters already behind that head, whatever their priority
            if (pos == waiters.begin() && pos != waiters.end() && keeps_place(pos->entry))
                ++pos;
        }
        waiters.insert(pos, waiter);
    }

    // Takes a waiter off the queue if it is still on it. Another queue's waiter asserts.
    void remove(Waiter &waiter) {
        waiters.remove(waiter);
    }

private:
    // Returns whether a waiter at the head stays there when another one would go in front of it, for
    // entries that can say so
    static bool keeps_place(const Entry &entry) {
        if constexpr (requires { entry.keeps_place(); })
            return entry.keeps_place();
        else
            return false;
    }

    bool by_priority = false;
    IntrusiveList<Waiter, &Waiter::link> waiters;
};
