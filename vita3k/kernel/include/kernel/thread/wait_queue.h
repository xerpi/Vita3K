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

// On ExitSignal, returns SCE_KERNEL_OK and lets the guest-run checkpoint catch
// exit_request / host_thread_exit_requested before the guest resumes.
inline SceInt32 unwrap_or_bail(WaitResult r) {
    return r.value_or(SCE_KERNEL_OK);
}

struct WaitEntryBase {
    ThreadStatePtr thread;
    // Snapshot at enqueue time. The queue is not re-sorted on priority changes.
    int priority = 0;
    // claimed=false: waiter still owns its queue slot, must self-remove.
    // claimed=true:  signaler/drainer finalized the wait, read `error`.
    bool claimed = false;
    SceInt32 error = SCE_KERNEL_OK;
};

enum class WaitOrder {
    FIFO,
    Priority,
};

inline WaitOrder wait_order_from_attr(SceUInt32 attr) {
    return (attr & SCE_KERNEL_ATTR_TH_PRIO) ? WaitOrder::Priority : WaitOrder::FIFO;
}

// Entries are stack-resident in the waiter, the queue stores pointers.
// Caller must hold the owning primitive's mutex around any queue operation.
template <std::derived_from<WaitEntryBase> WaitEntry>
class WaitQueue {
public:
    explicit WaitQueue(WaitOrder order = WaitOrder::FIFO)
        : order(order) {}

    // Parks the calling thread until either:
    //   - its entry is claimed by a signaler/drainer, in which case the claim code is returned
    //   - the deadline elapses, in which case SCE_KERNEL_ERROR_WAIT_TIMEOUT is returned
    //   - the thread is asked to exit, in which case ExitSignal is returned
    // cb=true dispatches pending callbacks at park boundaries.
    WaitResult wait(std::unique_lock<std::mutex> &primitive_lock,
        WaitEntry &entry,
        Deadline deadline,
        bool cb,
        WaitInfo info) {
        auto &t = *entry.thread;
        while (true) {
            primitive_lock.unlock();
            const auto park_result = t.park_until(deadline);
            primitive_lock.lock();

            if (entry.claimed) {
                t.leave_wait();
                return entry.error;
            }
            if (park_result == decltype(park_result)::timed_out) {
                remove(&entry);
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
                remove(&entry);
                t.leave_wait();
                return std::unexpected{ ExitSignal{} };
            }

            if (run_callbacks) {
                t.leave_wait();
                primitive_lock.unlock();
                t.process_callbacks();
                primitive_lock.lock();
                {
                    std::lock_guard<std::mutex> lock(t.mutex);
                    if (t.stop_requested_locked()) {
                        remove(&entry);
                        return std::unexpected{ ExitSignal{} };
                    }
                }
                t.enter_wait(info);
                continue;
            }
            // Spurious permit, re-park.
        }
    }

    // Appends a waiter to the queue, honoring the configured FIFO/priority order.
    void push(WaitEntry *entry) {
        if (order == WaitOrder::Priority) {
            // Lower numeric value = higher priority. Equal priorities stay FIFO.
            const auto it = std::find_if(entries.begin(), entries.end(),
                [&](const WaitEntry *q) { return q->priority > entry->priority; });
            entries.insert(it, entry);
        } else {
            entries.push_back(entry);
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
    bool wake_one(std::predicate<WaitEntry &> auto try_claim) {
        for (auto it = entries.begin(); it != entries.end(); ++it) {
            if (try_claim(**it)) {
                wake(**it, SCE_KERNEL_OK);
                entries.erase(it);
                return true;
            }
        }
        return false;
    }

    // Wakes every waiter that satisfies the try_claim predicate. Returns the number of waiters woken.
    std::size_t wake_many(std::predicate<WaitEntry &> auto try_claim) {
        std::size_t n = 0;
        for (auto it = entries.begin(); it != entries.end();) {
            if (try_claim(**it)) {
                wake(**it, SCE_KERNEL_OK);
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
        for (WaitEntry *e : entries) {
            pre_wake(*e);
            wake(*e, code);
        }
        entries.clear();
    }

private:
    void remove(WaitEntry *entry) {
        std::erase(entries, entry);
    }

    // Marks an entry as claimed with the given code and unparks its thread.
    // Caller is responsible for removing the entry from the queue.
    static void wake(WaitEntry &e, SceInt32 code) {
        e.error = code;
        e.claimed = true;
        e.thread->unpark();
    }

    WaitOrder order;
    std::list<WaitEntry *> entries;
};
