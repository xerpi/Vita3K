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

#include <cpu/state.h>
#include <kernel/callback.h>
#include <kernel/thread/wait_queue.h>
#include <kernel/types.h>
#include <mem/block.h>
#include <mem/ptr.h>

#include <chrono>
#include <condition_variable>
#include <deque>
#include <expected>
#include <list>
#include <mutex>
#include <optional>
#include <string>
#include <variant>
#include <vector>

struct CPUContext;

struct ThreadParams;

typedef std::unique_ptr<CPUState, std::function<void(CPUState *)>> CPUStatePtr;
typedef std::function<void(CPUState &, uint32_t, SceUID)> CallImport;
typedef std::function<std::string(Address)> ResolveNIDName;

// is_suspended is orthogonal and ORs the SUSPEND bit on top in vita_status().
enum class ThreadStatus {
    running, // Running guest code (or about to)
    waiting, // Parked in a WaitQueue
    dormant, // No active guest call, ready to be (re)started
    dead, // Terminal: JIT error
};

enum class DebugRequest {
    none,
    suspend,
    step,
};

struct RegisterArgs {
    std::vector<uint32_t> values;
};

struct ArglenArgs {
    SceSize arglen = 0;
    Ptr<void> argp;
};

using GuestArgs = std::variant<ArglenArgs, RegisterArgs>;

enum class GuestCallError {
    not_dormant, // thread wasn't dormant when the call was requested
    terminated, // host exit/delete request or JIT error during the call
};

using GuestCallResult = std::expected<uint32_t, GuestCallError>;

// Queued on the target's wait_thread_end_joiners while a waiter is parked in
// sceKernelWaitThreadEnd[CB].
struct WaitThreadEndJoinerEntry : WaitEntryBase {
    SceInt32 returned_value = 0;
};

struct ThreadState {
    enum class ParkResult {
        woken,
        timed_out,
    };

    // Immutable after init().
    SceUID id;
    std::string name;
    Address entry_point;
    Block stack;
    int stack_size;
    Block tls;

    int priority;
    SceInt32 affinity_mask;
    uint64_t last_vblank_waited;
    uint64_t start_tick;

    CPUStatePtr cpu;

    // Never taken while holding a primitive's mutex.
    mutable std::mutex mutex;

    ThreadStatus status = ThreadStatus::dormant;
    bool is_suspended = false;
    WaitInfo wait_info;
    uint32_t returned_value = 0;

    std::optional<SelfExitRequest> exit_request;
    // External request to wind down the host thread. Guest self-exit uses exit_request.
    bool host_thread_exit_requested = false;

    bool signal_pending = false;
    bool callbacks_pending = false;

    std::list<CallbackPtr> callbacks;
    WaitQueue<WaitThreadEndJoinerEntry> wait_thread_end_joiners;

    ThreadState() = delete;
    explicit ThreadState(SceUID id, KernelState &kernel, MemState &mem);
    int init(const char *name, Ptr<const void> entry_point, int init_priority,
        SceInt32 affinity_mask, int stack_size, const SceKernelThreadOptParam *option);

    int start(SceSize arglen, Ptr<void> argp, bool fire_start = false);
    void exit(SceInt32 status);
    void exit_delete(SceInt32 status);
    void request_host_thread_exit();

    void run_loop();

    // Same host thread only: used by callbacks during CheckCallback / WaitCB / thread events.
    uint32_t call_guest_inline(Address pc, RegisterArgs args);
    // Any host thread: enqueue onto this Vita thread's host loop.
    GuestCallResult call_guest_on_thread(Address pc, RegisterArgs args);
    GuestCallResult call_guest_on_thread(Address pc, SceSize arglen, Ptr<void> argp = {});

    // Returns the priority snapshot for WaitQueue entries.
    int enter_wait(WaitInfo info);
    void leave_wait();

    uint32_t process_callbacks();

    ParkResult park_until(std::chrono::steady_clock::time_point deadline);
    void park() { park_until(std::chrono::steady_clock::time_point::max()); }
    void unpark();

    void suspend();
    void resume(bool step = false);

    Address stack_top() const;
    // Caller must already hold mutex.
    bool should_stop_guest_locked() const;
    // Caller must already hold mutex.
    bool should_unwind_wait_locked(bool cb) const;
    // Caller must already hold mutex.
    SceUInt32 vita_status_locked() const;
    SceUInt32 vita_status() const;
    std::string log_stack_traceback() const;

private:
    struct QueuedGuestCall {
        Address pc = 0;
        GuestArgs args;
        bool completed = false;
        GuestCallResult result = std::unexpected{ GuestCallError::terminated };
        std::condition_variable cv;
    };

    void raise_wait_thread_end_joiners();
    void push_arguments(const std::vector<uint32_t> &args);
    void dispatch_abort(CPUState &cpu);
    void apply_guest_args(Address pc, const GuestArgs &args);
    void complete_pending_guest_calls_locked(GuestCallError error);
    GuestCallResult enqueue_guest_call(QueuedGuestCall &call);
    // Fires start event, runs guest, fires end event, completes queued call, resets to dormant.
    // Returns true if the run_loop should break (dead, exit_delete, or host exit).
    bool execute_run(QueuedGuestCall *queued_call, bool fire_start);

    // Dynarmic execution seam. Future native exec should move this behind the backend.
    // Returns on halt PC, JIT error (status=dead), exit_request, or host_thread_exit_requested.
    void run_guest_until_halt();

    void set_status_locked(ThreadStatus next);

    KernelState &kernel;
    MemState &mem;

    CPUContext init_cpu_ctx;

    DebugRequest debug_request = DebugRequest::none;
    bool fire_start_event = false;

    std::condition_variable lifecycle_cv;
    std::deque<QueuedGuestCall *> pending_guest_calls;

    // Separate from `mutex`: signalers can unpark without taking thread.mutex.
    std::mutex park_mutex;
    std::condition_variable park_cv;
    bool park_permit = false;
};

typedef std::shared_ptr<ThreadState> ThreadStatePtr;
