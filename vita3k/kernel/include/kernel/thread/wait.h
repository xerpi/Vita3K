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
#include <util/intrusive_list.h>

#include <chrono>
#include <cstddef>
#include <expected>
#include <mutex>
#include <optional>
#include <utility>

struct ThreadState;

// Returned instead of a result when the waiting thread exits or is being deleted. The guest never sees it.
struct ThreadExiting {};

// Result of a wait: a guest return code, or ThreadExiting.
using WaitResult = std::expected<SceInt32, ThreadExiting>;

using Deadline = std::chrono::steady_clock::time_point;

// What a waiting thread is blocked on, as sceKernelGetThreadInfo reports it.
struct WaitTarget {
    // One of SCE_KERNEL_WAITTYPE_*, or 0 if not waiting
    SceUInt32 type = 0;
    // UID of the object waited on, or -1 if there is none, like for a delay
    SceUID id = -1;
};

// Options for one wait call: the timeout and whether it runs callbacks
struct WaitOptions {
    // Points to the timeout in microseconds. The wait writes the time left back to it. Null waits
    // forever.
    SceUInt32 *timeout = nullptr;
    // Whether this is the callback variant of the call. It runs notified callbacks, and thread info
    // shows its wait type with the callback bit.
    bool callbacks = false;
};

// One wait of a thread, on the waiting call's stack. The thread points to it from begin() until
// sleep() returns, so the call must not return in between. A wait in no queue, like a delay, uses it
// as it is. A wait in one object's queue is a QueuedThreadWait, which holds the object lock. The
// wait is running while result is empty.
class ThreadWait {
public:
    // object_lock is the call's hold of the object's lock for a queued wait, empty for a wait in no
    // queue
    ThreadWait(ThreadState &thread, WaitTarget target, WaitOptions options, Deadline deadline, std::unique_lock<std::mutex> object_lock = {})
        : thread(thread)
        , target(target)
        , callbacks(options.callbacks)
        , deadline(deadline)
        , time_left(options.timeout)
        , object_lock(std::move(object_lock)) {}
    ThreadWait(const ThreadWait &) = delete;
    ThreadWait &operator=(const ThreadWait &) = delete;

    // Runs the notified callbacks if the wait takes them, with the object lock released meanwhile.
    // Returns ThreadExiting if the thread exits.
    [[nodiscard]] WaitResult run_callbacks();

    ThreadState &thread;
    // What thread info shows the thread waiting on
    const WaitTarget target;
    // Callback variant: a callback notify ends the wait, and thread info shows the callback bit
    const bool callbacks;
    const Deadline deadline;
    // Where whoever ends the wait writes the time left, or null
    SceUInt32 *const time_left;

protected:
    ~ThreadWait() = default;

    // Makes this the thread's current wait, or returns what it ends with instead
    [[nodiscard]] std::optional<WaitResult> begin();
    // Sleeps until the wait ends, with the object lock released only while asleep, then drops the
    // wait and returns how it ended
    [[nodiscard]] WaitResult sleep();
    // Ends the running wait with result, writes its time left and wakes the thread. end() takes the
    // thread lock, end_locked() needs it held.
    void end(WaitResult result);
    void end_locked(WaitResult result);

    // The thread's priority when the wait began, which a queued wait is queued at
    int priority = 0;

private:
    friend struct ThreadState;
    template <typename R>
    friend class WaitQueue;

    // Ends the running wait with result: a timeout, the thread exiting, or for a queued wait a
    // notified callback. Called with the thread lock unlocked. A queued wait is called with its
    // object lock held, and leaves its queue and serves the waiters behind it. The default ends the
    // wait under the thread lock, unless a signal or a callback notify ended it first.
    virtual void claim(WaitResult result);
    // Fires the timer events due by until, and returns when to look at the clock again
    virtual Deadline recheck(Deadline) { return Deadline::max(); }

    // Set once by whoever ends the wait, under the thread lock
    std::optional<WaitResult> result;
    // Only the waiting call uses it, a notify never does
    std::unique_lock<std::mutex> object_lock;
};

// Converts a guest timeout in microseconds into a deadline. A null timeout waits forever.
inline Deadline deadline_from(const SceUInt32 *timeout) {
    if (!timeout)
        return Deadline::max();
    return std::chrono::steady_clock::now() + std::chrono::microseconds(*timeout);
}

// Returns the microseconds left before the deadline, 0 once it passed.
inline SceUInt32 time_left_until(Deadline deadline) {
    const auto left = std::chrono::duration_cast<std::chrono::microseconds>(deadline - std::chrono::steady_clock::now()).count();
    return left > 0 ? static_cast<SceUInt32>(left) : 0;
}

// Turns a wait result into the value returned to the guest.
// ThreadExiting becomes SCE_KERNEL_OK, which the guest never sees: after the HLE call returns,
// run_loop sees the exit or delete request and stops the thread before the next guest instruction.
inline SceInt32 guest_result(WaitResult r) {
    return r.value_or(SCE_KERNEL_OK);
}

// An entry in a WaitQueue: the wait and its request. An ordinary wait keeps its node on its own
// stack, a multiple event wait has one per event. The node is queued only while the wait runs, and
// whoever ends the wait removes it first. Guarded by the queue's lock.
template <typename R>
struct WaitNode {
    ThreadWait &wait;
    R request;
    // The thread's priority when the node was queued. The queue is not reordered if it changes.
    int priority = 0;
    IntrusiveLink<WaitNode> link;
};

enum class QueueOrder {
    fifo,
    // Lower value is higher priority, and equal priorities keep FIFO order
    priority,
};

// The waits blocked on one sync object, in FIFO or priority order. Every method needs the object's
// lock, and none releases it. The nodes are linked in place, so queueing allocates nothing.
template <typename R>
class WaitQueue final : IntrusiveList<WaitNode<R>, &WaitNode<R>::link> {
    using List = IntrusiveList<WaitNode<R>, &WaitNode<R>::link>;

public:
    explicit WaitQueue(QueueOrder order)
        : by_priority(order == QueueOrder::priority) {}

    // The nodes in queue order
    using List::begin, List::end, List::empty, List::size, List::contains, List::remove;

    // The node that would be woken first, or null
    WaitNode<R> *front() { return empty() ? nullptr : &List::front(); }

    // Whether a thread of this priority would be first in the queue, ahead of every waiter
    bool first_for(int priority) const {
        return empty() || (by_priority && List::front().priority > priority);
    }

    // Inserts a node at the given priority, or at the back of a FIFO queue. behind_head keeps a
    // node that would go first behind the current head, ahead of every other waiter.
    void push(WaitNode<R> &node, int priority, bool behind_head = false) {
        node.priority = priority;
        auto pos = end();
        if (by_priority) {
            pos = begin();
            while (pos != end() && pos->priority <= node.priority)
                ++pos;
            if (behind_head && pos == begin() && pos != end())
                ++pos;
        }
        this->insert(pos, node);
    }

    // Removes a queued node and ends its wait with the given result. Only the lock holder ends a
    // queued wait, so the wait is still running. Takes the thread's lock.
    void complete(WaitNode<R> &node, WaitResult result = SCE_KERNEL_OK) {
        remove(node);
        node.wait.end(result);
    }

    // Completes every node in queue order with the given result. Returns how many it completed.
    std::size_t complete_all(SceInt32 result) {
        std::size_t ended = 0;
        while (WaitNode<R> *const node = front()) {
            complete(*node, result);
            ++ended;
        }
        return ended;
    }

private:
    const bool by_priority;
};

// When run() runs the callbacks notified during a call, before it returns. A try that takes what
// the call waits for returns at once.
enum class FinalCallbackCheck {
    // Never: callbacks still run at the top of each pass
    none,
    // After a block, also one refused when it began, unless the thread is exiting
    after_wait,
    // Only after a block that a delete ended
    on_delete,
};

// One blocking call on a sync object, on the caller's stack: the thread's wait, its node in the
// object's queue and its hold of the object's lock. The object's wait method runs the passes with
// run(), or its own loop of the callbacks, its own try and block(). The caller keeps the object
// alive for the whole call.
template <typename Owner, typename R>
class QueuedThreadWait : public ThreadWait {
public:
    // Takes over guard, the caller's hold of the object's lock
    QueuedThreadWait(Owner &owner, WaitQueue<R> &queue, std::unique_lock<std::mutex> guard, ThreadState &thread, WaitTarget target, R request, WaitOptions options)
        : ThreadWait(thread, target, options, deadline_from(options.timeout), std::move(guard))
        , node{ *this, std::move(request) }
        , owner(owner)
        , queue(queue) {}

    // Queues the node and sleeps until the wait ends, then returns how it ended. Returns what the
    // wait ends with instead if it can't begin. behind_head keeps the node behind the queue's head.
    [[nodiscard]] WaitResult block(bool behind_head = false) {
        if (const std::optional<WaitResult> refused = begin())
            return *refused;
        queue.push(node, priority, behind_head);
        // Whoever ends the wait takes the node off, under the queue's lock
        return sleep();
    }

    // Runs the passes of the call: the notified callbacks, then try_now(), which returns the call's
    // result or nothing to block, then the block. A callback notify during the block starts a new
    // pass. final picks when the callbacks notified during the call run before it returns.
    template <typename Try>
    [[nodiscard]] WaitResult run(FinalCallbackCheck final, Try try_now) {
        while (true) {
            if (const WaitResult ran = run_callbacks(); !ran)
                return ran;
            if (const std::optional<SceInt32> taken = try_now())
                return *taken;
            const WaitResult r = block();
            if (!r)
                return r;
            if (*r == SCE_KERNEL_ERROR_NOTIFY_CALLBACK)
                continue;
            if (final == FinalCallbackCheck::none || (final == FinalCallbackCheck::on_delete && *r != SCE_KERNEL_ERROR_WAIT_DELETE))
                return r;
            // Only the thread exiting changes the result
            if (const WaitResult ran = run_callbacks(); !ran)
                return ran;
            return r;
        }
    }

    WaitNode<R> node;

protected:
    // Takes the node off and ends the wait with result, then serves the remaining waiters
    void claim(WaitResult result) override {
        queue.complete(node, result);
        owner.serve_locked();
    }

    Owner &owner;
    WaitQueue<R> &queue;
};
