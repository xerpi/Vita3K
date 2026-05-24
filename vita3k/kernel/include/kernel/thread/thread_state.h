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
#include <expected>
#include <list>
#include <mutex>
#include <optional>
#include <string>
#include <string_view>
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
    char name[SCE_UID_NAMELEN + 1]{};
    Address entry_point;
    Block stack;
    int stack_size;
    Block tls;

    // Scheduling priority, lower is higher. Mutable via sceKernelChangeThreadPriority.
    int priority;
    // CPU affinity mask, mutable via sceKernelChangeThreadCpuAffinityMask.
    SceInt32 affinity_mask;
    // Last vblank count this thread waited for (SceDisplay bookkeeping).
    uint64_t last_vblank_waited;
    // Creation tick, for sceKernelGetThreadInfo runClocks.
    uint64_t start_tick;

    // Backend CPU/JIT state.
    CPUStatePtr cpu;

    // Guards the mutable state below. Never taken while holding a primitive's mutex.
    mutable std::mutex mutex;

    // Lifecycle state. is_suspended is orthogonal (see vita_status()).
    ThreadStatus status = ThreadStatus::dormant;
    bool is_suspended = false;
    // What the thread is blocked on, reported by sceKernelGetThreadInfo.
    WaitInfo wait_info;
    // Entry return value / exit status (guest r0).
    uint32_t returned_value = 0;

    // A sceKernelSignal is pending for this thread.
    bool signal_pending = false;
    // A registered callback has a coalesced notification waiting.
    bool callbacks_pending = false;

    // Callbacks created on this thread.
    std::list<CallbackPtr> callbacks;
    // Threads parked in sceKernelWaitThreadEnd[CB] on this one.
    WaitQueue<WaitThreadEndJoinerEntry> wait_thread_end_joiners;

    ThreadState() = delete;
    explicit ThreadState(SceUID id, KernelState &kernel, MemState &mem);
    // Allocates stack/TLS and sets up the initial CPU context. Leaves the thread dormant.
    int init(std::string_view name, Ptr<const void> entry_point, int init_priority,
        SceInt32 affinity_mask, int stack_size, const SceKernelThreadOptParam *option);

    // Starts a dormant thread at its entry point. Fire-and-forget.
    int start(SceSize arglen, Ptr<void> argp, bool fire_start = false);
    // Guest self-exit: requests the thread stop and return status.
    void exit(SceInt32 status);
    // Guest self-exit that also deletes the thread once stopped.
    void exit_delete(SceInt32 status);
    // External teardown: stops the thread and winds down its host thread.
    void request_host_thread_exit();

    // Host thread body. One per guest thread, runs queued guest calls until teardown.
    void run_loop();

    // Same host thread only: used by callbacks during CheckCallback / WaitCB / thread events.
    uint32_t call_guest_inline(Address pc, RegisterArgs args);
    // Any host thread: run a guest function on this dormant thread and wait for the result.
    GuestCallResult call_guest_on_thread(Address pc, GuestArgs args);

    // Enters the waiting state. Returns the priority snapshot for WaitQueue entries.
    int enter_wait(WaitInfo info);
    void leave_wait();

    // Runs this thread's pending callbacks. Returns the number dispatched.
    uint32_t process_callbacks();

    // Parks this thread until unparked or the deadline elapses.
    ParkResult park_until(std::chrono::steady_clock::time_point deadline);
    void park() { park_until(std::chrono::steady_clock::time_point::max()); }
    void unpark();

    // Debugger pause / resume.
    void suspend();
    void resume(bool step = false);

    Address stack_top() const;
    // True once the thread must stop, by guest self-exit or host teardown. Caller must hold mutex.
    bool stop_requested_locked() const;
    // Vita SCE_KERNEL_THREAD_STATUS_* bits. Caller must hold mutex.
    SceUInt32 vita_status_locked() const;
    SceUInt32 vita_status() const;
    std::string log_stack_traceback() const;

private:
    struct QueuedGuestCall {
        Address pc = 0;
        GuestArgs args;
        // Fire thread start/end events (thread entry only).
        bool fire_events = false;
        bool completed = false;
        GuestCallResult result = std::unexpected{ GuestCallError::terminated };
    };

    void raise_wait_thread_end_joiners();
    void push_arguments(const std::vector<uint32_t> &args);
    void dispatch_abort(CPUState &cpu);
    // Loads the initial context and writes pc + args for a guest entry.
    void apply_guest_args(Address pc, const GuestArgs &args);
    // Completes the pending call with an error (teardown). Caller must hold mutex.
    void complete_pending_guest_call_locked(GuestCallError error);
    // Applies args, fires the start event, runs the guest, fires the end event,
    // completes the call and resets to dormant. Returns true if run_loop should
    // break (dead, exit_delete, or host exit).
    bool execute_run(QueuedGuestCall &call);

    // Dynarmic execution seam. Future native exec should move this behind the backend.
    // Returns on halt PC, JIT error (status=dead), exit_request, or host_thread_exit_requested.
    void run_guest_until_halt();

    // Transitions status with a debug-build legality check. Caller must hold mutex.
    void set_status_locked(ThreadStatus next);

    KernelState &kernel;
    MemState &mem;

    // Initial CPU context applied at each guest entry.
    CPUContext init_cpu_ctx;

    DebugRequest debug_request = DebugRequest::none;

    // Guest self-exit (sceKernelExitThread).
    std::optional<SelfExitRequest> exit_request;
    // External teardown.
    bool host_thread_exit_requested = false;

    // Signals lifecycle transitions to the host loop and call_guest_on_thread waiters.
    std::condition_variable lifecycle_cv;
    // Descriptor for the thread's own entry point (start).
    QueuedGuestCall entry_call;
    // Guest call awaiting pickup by run_loop, or null.
    QueuedGuestCall *pending_guest_call = nullptr;

    // Separate from `mutex`: signalers can unpark without taking thread.mutex.
    std::mutex park_mutex;
    std::condition_variable park_cv;
    bool park_permit = false;
};

typedef std::shared_ptr<ThreadState> ThreadStatePtr;
