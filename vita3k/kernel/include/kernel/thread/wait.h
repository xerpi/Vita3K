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

// Options for one call that may wait: its timeout and whether it runs callbacks. The export makes
// them where the call reads its timeout, and they write the time left back when the export returns,
// after the call's other outputs.
struct WaitOptions {
    WaitOptions(SceUInt32 *timeout = nullptr, bool callbacks = false, bool timeout_in_place = false)
        : timeout(timeout)
        , callbacks(callbacks)
        , timeout_in_place(timeout_in_place)
        , timeout_value(timeout ? *timeout : 0)
        , left(timeout_value) {}
    WaitOptions(const WaitOptions &) = delete;
    WaitOptions &operator=(const WaitOptions &) = delete;
    ~WaitOptions() {
        if (timeout && !timeout_in_place)
            *timeout = left;
    }

    // The caller's timeout in microseconds. Null waits forever.
    SceUInt32 *const timeout;
    // Whether this is the callback variant of the call. It runs notified callbacks, and thread info
    // shows its wait type with the callback bit.
    const bool callbacks;
    // A message pipe writes the time left to timeout each time its wait ends, and nothing when it
    // returns
    const bool timeout_in_place;
    // The timeout when the call began
    const SceUInt32 timeout_value;
    // The time left, set by whoever ends a wait of the call and when a timeout refuses one, under
    // the waiting thread's lock
    SceUInt32 left;
};

// One wait of a thread, on the waiting call's stack. The thread points to it from begin() until
// retire(), so the call must not return in between. A wait in no queue, like a delay, uses it as
// it is. A wait in one object's queue is a QueuedThreadWait, which holds the object lock. The wait
// is running while result is empty. begin(), end() and callback_ends() expect the thread lock held.
class ThreadWait {
public:
    // A wait in no queue
    ThreadWait(ThreadState &thread, WaitTarget target, WaitOptions &options, Deadline deadline)
        : thread(thread)
        , target(target)
        , deadline(deadline)
        , queued(false)
        , options(options) {}
    // A wait in one object's queue. It takes over the caller's hold of the object lock.
    ThreadWait(ThreadState &thread, WaitTarget target, WaitOptions &options, Deadline deadline, std::unique_lock<std::mutex> object_lock)
        : thread(thread)
        , target(target)
        , deadline(deadline)
        , queued(true)
        , options(options)
        , object_lock(std::move(object_lock)) {}
    ThreadWait(const ThreadWait &) = delete;
    ThreadWait &operator=(const ThreadWait &) = delete;

    // Runs the notified callbacks if the wait takes them, with the object lock released meanwhile.
    // Returns ThreadExiting if the thread exits.
    [[nodiscard]] WaitResult run_callbacks();
    // Makes the sleeping wait look at the clock again. Used when a timer's next event changes.
    void wake();

    ThreadState &thread;
    // What thread info shows the thread waiting on
    const WaitTarget target;
    const Deadline deadline;
    // Whether the wait is in one object's queue, which only its object lock holder can end. A delay,
    // a signal wait and a multiple event wait are in no queue.
    const bool queued;

protected:
    ~ThreadWait() = default;

    // Locks the waiting thread
    [[nodiscard]] std::unique_lock<std::mutex> lock_thread() const;
    // Makes this the thread's current wait, or returns what it ends with instead: ThreadExiting, a
    // pending signal taken, WAIT_TIMEOUT (the time left becomes 0) or a pending NOTIFY_CALLBACK.
    [[nodiscard]] std::optional<WaitResult> begin();
    // Sleeps until the wait ends, with the object lock released only while asleep, and returns how
    // it ended. The waiter ends the wait itself for its timeout, its exit or, if it is queued, a
    // notified callback, and then its node is still queued, since a waker removes it first.
    WaitResult sleep();
    // Drops the ended wait and returns how it ended, ThreadExiting if the thread is exiting
    [[nodiscard]] WaitResult retire();
    // Ends the running wait with result, writes its time left and wakes the thread
    void end(WaitResult result);
    // Fires the timer events due by until and returns when sleep() looks at the clock again. A wait
    // without timer events never needs to look.
    virtual Deadline watch(Deadline) { return Deadline::max(); }

    // The thread's priority when the wait began, which a queued wait is queued at
    int priority = 0;

private:
    friend struct ThreadState;
    template <typename R>
    friend class WaitQueue;

    // Whether a notified callback ends the wait
    bool callback_ends() const;

    // The call's options, which keep its time left
    WaitOptions &options;
    // Set once by whoever ends the wait, under the thread lock
    std::optional<WaitResult> result;
    // A timer changed since the waiter last looked. Set by wake(), cleared on each look of sleep(),
    // under the thread lock.
    bool recheck_pending = false;
    // Only the waiting call uses it, a notify never does
    std::unique_lock<std::mutex> object_lock;
};

// Converts the timeout of a call in microseconds into a deadline. No timeout waits forever.
inline Deadline deadline_from(const WaitOptions &options) {
    if (!options.timeout)
        return Deadline::max();
    return std::chrono::steady_clock::now() + std::chrono::microseconds(options.timeout_value);
}

// Turns a wait result into the value returned to the guest. ThreadExiting becomes SCE_KERNEL_OK,
// which the guest never sees, since run_loop stops the thread before its next guest instruction.
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
    using List::begin, List::end, List::empty, List::size, List::remove;

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
        const auto lock = node.wait.lock_thread();
        node.wait.end(result);
    }

    // Completes every node in queue order with the given result. Returns how many it completed.
    std::size_t complete_all(SceInt32 result) {
        const std::size_t ended = size();
        while (WaitNode<R> *const node = front())
            complete(*node, result);
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
// object's queue and its hold of the object's lock. The caller keeps the object alive for the whole
// call.
template <typename R>
class QueuedThreadWait : public ThreadWait {
public:
    // Takes over guard, the caller's hold of the object's lock
    QueuedThreadWait(WaitQueue<R> &queue, std::unique_lock<std::mutex> guard, ThreadState &thread, WaitTarget target, R request, WaitOptions &options)
        : ThreadWait(thread, target, options, deadline_from(options), std::move(guard))
        , node{ *this, std::move(request) }
        , queue(queue) {}

    // Queues the node and sleeps until the wait ends, then returns how it ended. Returns what the
    // wait ends with instead if it can't begin. A waiter that ended the wait itself takes its node
    // off and calls leave with how the sleep ended, under the object lock. behind_head keeps the
    // node behind the queue's head.
    template <typename Leave>
    [[nodiscard]] WaitResult block(Leave leave, bool behind_head = false) {
        if (const auto lock = lock_thread(); const std::optional<WaitResult> refused = begin())
            return *refused;
        queue.push(node, priority, behind_head);
        const WaitResult ended = sleep();
        if (queue.remove(node))
            leave(ended);
        return retire();
    }

    // Runs the passes of the call: the notified callbacks, then try_now(), which returns the call's
    // result or nothing to block, then block(leave). A callback notify during the block starts a
    // new pass. final picks when the callbacks notified during the call run before it returns.
    template <typename Leave, typename Try>
    [[nodiscard]] WaitResult run(FinalCallbackCheck final, Leave leave, Try try_now) {
        while (true) {
            if (const WaitResult ran = run_callbacks(); !ran)
                return ran;
            if (const std::optional<SceInt32> taken = try_now())
                return *taken;
            const WaitResult r = block(leave);
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
    WaitQueue<R> &queue;
};
