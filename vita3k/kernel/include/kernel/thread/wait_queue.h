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

// Forward-declares ThreadState so thread_state.h can embed a WaitQueue<...>
// member. Template bodies resolve ThreadState at instantiation time.

#include <kernel/types.h>

#include <algorithm>
#include <chrono>
#include <concepts>
#include <expected>
#include <list>
#include <memory>
#include <mutex>

struct KernelState;
struct ThreadState;
using ThreadStatePtr = std::shared_ptr<ThreadState>;

// Set by the thread itself from sceKernelExitThread / sceKernelExitDeleteThread.
// External host-thread teardown uses host_thread_exit_requested instead.
enum class SelfExitRequest {
    exit,
    exit_delete,
};

struct WaitInfo {
    SceUInt32 type = 0;
    SceUID uid = 0;
    const char *reason = "";
};

// Wait was interrupted by self-exit-from-callback or external host-thread teardown.
struct ExitSignal {};

using WaitResult = std::expected<SceInt32, ExitSignal>;
using Deadline = std::chrono::steady_clock::time_point;

// pTimeout: NULL = forever, *p==0 = poll, *p>0 = microseconds.
inline Deadline deadline_from(const SceUInt32 *timeout) {
    if (!timeout)
        return Deadline::max();
    return std::chrono::steady_clock::now() + std::chrono::microseconds{ *timeout };
}

inline void writeback_timeout(SceUInt32 *timeout, Deadline deadline, SceInt32 result) {
    if (!timeout || deadline == Deadline::max())
        return;
    if (result == SCE_KERNEL_ERROR_WAIT_TIMEOUT) {
        *timeout = 0;
        return;
    }

    const auto remaining = std::chrono::duration_cast<std::chrono::microseconds>(
        deadline - std::chrono::steady_clock::now())
                               .count();
    *timeout = (remaining <= 0) ? 0 : static_cast<SceUInt32>(remaining);
}

// On ExitSignal, returns SCE_KERNEL_OK and lets the guest-run checkpoint catch
// exit_request / host_thread_exit_requested before the guest resumes.
inline SceInt32 unwrap_or_bail(WaitResult r) {
    return r.value_or(SCE_KERNEL_OK);
}

inline SceInt32 unwrap_or_bail(WaitResult r, SceUInt32 *timeout, Deadline deadline) {
    if (r)
        writeback_timeout(timeout, deadline, *r);
    return unwrap_or_bail(std::move(r));
}

enum class WaitOrder {
    FIFO,
    Priority,
};

inline WaitOrder wait_order_from_attr(SceUInt32 attr) {
    return (attr & SCE_KERNEL_ATTR_TH_PRIO) ? WaitOrder::Priority : WaitOrder::FIFO;
}

// WaitEntry is the primitive-specific payload. The queue owns the common waiter
// bookkeeping so primitive callers don't initialize thread/priority/claim state.
template <typename Primitive, typename Entry = typename Primitive::WaitEntry>
class WaitQueue {
public:
    using WaitEntry = Entry;
    struct Waiter {
        ThreadStatePtr thread;
        // Snapshot at enqueue time. The queue is not re-sorted on priority changes.
        int priority = 0;
        // claimed=false: waiter still owns its queue slot, must self-remove.
        // claimed=true:  signaler/drainer finalized the wait, read `error`.
        bool claimed = false;
        SceInt32 error = SCE_KERNEL_OK;
        WaitEntry entry;
    };

    explicit WaitQueue(WaitOrder order = WaitOrder::FIFO)
        : order(order) {}

    WaitResult enqueue_and_wait(std::unique_lock<std::mutex> &primitive_lock,
        ThreadStatePtr thread,
        WaitEntry &entry,
        Deadline deadline,
        bool cb,
        WaitInfo info) {
        Waiter waiter{
            .thread = std::move(thread),
            .entry = std::move(entry),
        };
        waiter.priority = waiter.thread->enter_wait(info);
        push(&waiter);
        const WaitResult result = wait(primitive_lock, waiter, deadline, cb, info);
        entry = std::move(waiter.entry);
        return result;
    }

    // Parks the calling thread until either:
    //   - its entry is claimed by a signaler/drainer, in which case the claim code is returned
    //   - the deadline elapses, in which case SCE_KERNEL_ERROR_WAIT_TIMEOUT is returned
    //   - the thread is asked to exit, in which case ExitSignal is returned
    // cb=true dispatches pending callbacks at park boundaries.
    WaitResult wait(std::unique_lock<std::mutex> &primitive_lock,
        Waiter &waiter,
        Deadline deadline,
        bool cb,
        WaitInfo info) {
        auto &t = *waiter.thread;
        while (true) {
            primitive_lock.unlock();
            const auto park_result = t.park_until(deadline);
            primitive_lock.lock();

            if (waiter.claimed) {
                t.leave_wait();
                return waiter.error;
            }
            if (park_result == decltype(park_result)::timed_out) {
                remove(&waiter);
                t.leave_wait();
                return SCE_KERNEL_ERROR_WAIT_TIMEOUT;
            }

            bool run_callbacks = false;
            bool should_bail = false;
            {
                std::lock_guard<std::mutex> lock(t.mutex);
                should_bail = t.stop_requested_locked();
                if (cb) {
                    if (!should_bail && t.callbacks_pending) {
                        t.callbacks_pending = false;
                        run_callbacks = true;
                    }
                }
            }

            if (should_bail) {
                remove(&waiter);
                t.leave_wait();
                return std::unexpected{ ExitSignal{} };
            }

            if (run_callbacks) {
                t.leave_wait();
                while (true) {
                    primitive_lock.unlock();
                    t.process_callbacks();
                    primitive_lock.lock();

                    if (waiter.claimed)
                        return waiter.error;

                    bool callbacks_pending = false;
                    {
                        std::lock_guard<std::mutex> lock(t.mutex);
                        if (t.stop_requested_locked()) {
                            remove(&waiter);
                            return std::unexpected{ ExitSignal{} };
                        }
                        if (cb && t.callbacks_pending) {
                            t.callbacks_pending = false;
                            callbacks_pending = true;
                        }
                    }
                    if (!callbacks_pending)
                        break;
                }
                t.enter_wait(info);
                continue;
            }
            // Spurious permit, re-park.
        }
    }

    // Appends a waiter to the queue, honoring the configured FIFO/priority order.
    void push(Waiter *waiter) {
        if (order == WaitOrder::Priority) {
            // Lower numeric value = higher priority. Equal priorities stay FIFO.
            const auto it = std::find_if(entries.begin(), entries.end(),
                [&](const Waiter *q) { return q->priority > waiter->priority; });
            entries.insert(it, waiter);
        } else {
            entries.push_back(waiter);
        }
    }

    // Whether the queue is empty.
    bool empty() const {
        return entries.empty();
    }

    // Number of waiters currently in the queue.
    std::size_t size() const {
        return entries.size();
    }

    // Wakes the first waiter on the list that satisfies the try_claim predicate.
    bool wake_one(std::predicate<WaitEntry &, const ThreadStatePtr &> auto try_claim) {
        for (auto it = entries.begin(); it != entries.end(); ++it) {
            Waiter &waiter = **it;
            if (try_claim(waiter.entry, waiter.thread)) {
                wake(**it, SCE_KERNEL_OK);
                entries.erase(it);
                return true;
            }
        }
        return false;
    }

    // Wakes every waiter that satisfies the try_claim predicate. Returns the number of waiters woken.
    std::size_t wake_many(std::predicate<WaitEntry &, const ThreadStatePtr &> auto try_claim) {
        std::size_t n = 0;
        for (auto it = entries.begin(); it != entries.end();) {
            Waiter &waiter = **it;
            if (try_claim(waiter.entry, waiter.thread)) {
                wake(waiter, SCE_KERNEL_OK);
                it = entries.erase(it);
                ++n;
            } else {
                ++it;
            }
        }
        return n;
    }

    // Wakes every waiter with the given error code and empties the queue.
    // pre_wake, if provided, is called on each entry before it is woken.
    template <std::invocable<WaitEntry &> PreWake = decltype([](WaitEntry &) {})>
    void drain_with_error(SceInt32 code, PreWake pre_wake = {}) {
        for (Waiter *waiter : entries) {
            pre_wake(waiter->entry);
            wake(*waiter, code);
        }
        entries.clear();
    }

private:
    void remove(Waiter *waiter) {
        std::erase(entries, waiter);
    }

    // Marks an entry as claimed with the given code and unparks its thread.
    // Caller is responsible for removing the entry from the queue.
    static void wake(Waiter &waiter, SceInt32 code) {
        waiter.error = code;
        waiter.claimed = true;
        waiter.thread->unpark();
    }

    WaitOrder order;
    std::list<Waiter *> entries;
};
