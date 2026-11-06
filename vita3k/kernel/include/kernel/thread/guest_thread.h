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

#include <cpu/common.h>
#include <kernel/thread/thread_state.h>

#include <algorithm>
#include <array>
#include <condition_variable>
#include <initializer_list>
#include <memory>
#include <mutex>
#include <optional>

struct KernelState;
class Callback;
using CallbackPtr = std::shared_ptr<Callback>;

// An import's result, which the kernel writes to the registers when the call returns to guest code
struct CallResult {
    // r0 to r3, of which count are written
    std::array<uint32_t, 4> regs{};
    uint8_t count = 0;
    // A float result, in s0 with its bits in regs[0]
    bool in_s0 = false;
};

// A guest thread: its Vita OS state, its CPU and the run loop its host thread runs
struct GuestThread final {
    // The thread's Vita OS facts. Outside the kernel only the fixed ones are read directly, the rest
    // through the methods below, with mutex held.
    ThreadState state;
    // The pointer is fixed. The registers are only touched by the thread itself, or while
    // something holds it.
    const CPUStatePtr cpu;

    // A dormant thread with its stack and TLS, which runs on cpu
    GuestThread(SceUID id, KernelState &kernel, MemState &mem, CPUStatePtr cpu, const char *name, Address entry_point, int init_priority, SceInt32 affinity_mask, int stack_size, SceUInt32 attr);

    // Lifecycle
    // Starts a dormant thread at entry_point.
    int start(SceSize arglen, Ptr<void> argp, bool run_entry_callback = false);
    // Ends the thread like sceKernelExitThread: the end handler runs, then it parks dormant
    void exit(SceInt32 status);
    // Ends the thread like sceKernelExitDeleteThread: the end handler runs, then it's deleted
    void exit_delete(SceInt32 status);
    // Deletes the thread like sceKernelDeleteThread, or returns NOT_DORMANT if it isn't dormant
    SceInt32 remove_dormant();
    // Deletes a dormant thread, or one being torn down, without running its end handler
    void remove();
    // Runs the thread on its host thread: parks it dormant between starts, runs its guest code and
    // ends it, until it's deleted
    void run_loop();
    // Calls fn on the thread's CPU from inside a kernel call and returns its r0, or the exit status
    // if the thread ends during it. fresh_vfp_state starts it with the VFP state of a new thread.
    uint32_t call_guest(Address fn, std::initializer_list<uint32_t> args, bool fresh_vfp_state = false);
    // Starts the dormant thread from another thread and returns its exit status once it ends
    uint32_t start_and_wait(SceSize args, const Ptr<void> argp = Ptr<void>{});

    // Status and info
    ThreadStatus get_status() const;
    // The thread's info, as sceKernelGetThreadInfo reports it
    SceKernelThreadInfo info();
    // The room left on the stack, all of it for a dormant thread
    int stack_free_size() const;
    // Moves the stack pointer down by size and returns the new one
    uint32_t stack_alloc(size_t size);
    // Moves the stack pointer back up by size
    void stack_free(size_t size);

    // Priority and affinity
    // The priority the thread runs at
    int get_priority() const;
    // Changes the priority the thread runs at, and the one its TLS shows. Returns the old priority.
    int change_priority(int new_priority);
    // Changes the cores the thread may run on, and the ones its TLS shows. Returns the old mask.
    SceInt32 change_affinity_mask(SceInt32 new_affinity_mask);

    // Waits
    // Blocks this thread until the deadline passes.
    [[nodiscard]] WaitResult delay_until(Deadline deadline, bool callbacks);
    // Blocks this thread until a signal is sent to it or the timeout passes, writing the time left back.
    [[nodiscard]] WaitResult wait_for_signal(SceUInt32 *timeout, bool callbacks);
    // Sends a signal to this thread. Fails if the previous one was not consumed yet.
    SceInt32 send_signal();
    // Blocks waiter until this thread becomes dormant, then writes its exit status to exit_status.
    [[nodiscard]] WaitResult wait_for_thread_end(const GuestThreadPtr &waiter, SceInt32 *exit_status, Deadline deadline, bool callbacks);
    // Waits on target until a wake, a notified callback, the thread's end or the deadline. A
    // notified callback ends it with NOTIFY_CALLBACK.
    [[nodiscard]] WaitResult wait(WaitTarget target, Deadline deadline);
    // Runs the notified callbacks for a wait that ended with NOTIFY_CALLBACK, with no lock held.
    // Returns ThreadExiting if the thread exits or is deleted meanwhile.
    [[nodiscard]] WaitResult run_callbacks();
    // Wakes this thread from wait().
    void wake();
    // A host call that blocks the thread inside an export, like an audio output, during which the
    // thread counts as stopped. It shows WAITING on shown while it lives, or stays RUNNING without
    // a target.
    class HostWait {
    public:
        explicit HostWait(GuestThread &thread, std::optional<WaitTarget> shown = WaitTarget{});
        ~HostWait();
        HostWait(const HostWait &) = delete;
        HostWait &operator=(const HostWait &) = delete;

    private:
        GuestThread &thread;
    };
    // The vblank a display wait for vcount vblanks ends on: vcount after the one the thread's last
    // display wait ended on, and at least next_vblank. The next display wait counts from it.
    uint64_t next_display_wait(uint64_t next_vblank, int vcount);

    // Callbacks
    // Runs the notified callbacks of this thread and returns how many ran
    SceUInt32 process_callbacks();
    // Returns whether a notified callback would run now. None runs inside a callback. Takes mutex.
    bool has_pending_callbacks();
    // Adds a callback this thread created
    void add_callback(const CallbackPtr &cb) { state.callbacks.push_back(cb); }
    // Both are called by a callback of this thread with the callback's lock held
    // Counts a notified callback, waking a wait with callbacks
    void add_notified_callback();
    // Stops counting a notified callback
    void remove_notified_callback();

    // Holds: the debugger, the emulator pause and the ForVM suspend
    // Stops the thread for the debugger the next time it would run guest code, or lets it run again.
    // A pause or a ForVM suspend may still hold it after the debugger lets go.
    void set_suspended(bool suspended);
    // Lets a thread the debugger stopped run one instruction, then waits until it stops again or is
    // dormant or deleted. Returns false and doesn't step while something else holds the thread.
    bool step();
    // Waits until the thread doesn't run guest code
    void wait_until_not_running();
    // Stops for a ForVM suspend before a new object wait, with object_lock released until nothing
    // holds the thread. Called with object_lock held. Returns whether it stopped.
    bool hold_before_wait(std::unique_lock<std::mutex> &object_lock);
    // Whether the debugger holds the thread and it has stopped. Once the debugger has let every
    // thread go, only a breakpoint or a finished step holds a thread for it again.
    bool stopped_at_breakpoint() const;
    // Stops the thread's guest code when the emulator pause starts, or lets it run when it ends
    void pause_changed();

    // ForVM suspend
    // Suspends the thread for sceKernelSuspendThreadForVM: it doesn't run guest code until as many
    // vm_resume calls. vm_suspend returns once the thread runs no guest code. Each returns the
    // suspend count after it, or an error for a dormant thread or a resume with no suspend.
    int vm_suspend();
    int vm_resume();
    // Waits until a ForVM suspend has stopped the thread, then gets the context it stopped with, its
    // TPIDRURW and whether it stopped inside a kernel call. Returns NOT_SUSPENDED if no ForVM
    // suspend holds it.
    SceInt32 get_vm_context(CPUContext &context, uint32_t &tpidrurw, bool &in_kernel_call);

private:
    using Ending = ThreadState::Ending;

    // The private methods are called with mutex held, unless they say otherwise
    // Makes the thread dormant or deleted and wakes its end waiters
    void end_as(ThreadStatus status);
    // The status the guest sees: SUSPENDED for a thread a ForVM suspend holds at its stop
    ThreadStatus shown_status() const;
    // Sets how the thread ends, which stops its guest code for good and drops a step it didn't take
    // unless it's none
    void set_ending(Ending how);
    // Whether the debugger, the emulator pause or a ForVM suspend holds the thread
    bool held() const;
    // Waits with mutex held through lock until nothing holds the thread. in_call marks a stop
    // inside a kernel call, where the registers are the call's.
    void stop_for_holds(std::unique_lock<std::mutex> &lock, bool in_call);
    // Whether the thread may run guest code now, or kernel code that changes its registers
    bool runs() const { return state.status == ThreadStatus::running && !parked && !held_in_call && !host_waiting; }
    // Asks the thread to end, keeping a stronger request already made
    void request_ending(Ending how) { set_ending(std::max(state.ending, how)); }

    // A call of guest code from inside a kernel call
    struct GuestCall {
        Address fn;
        std::initializer_list<uint32_t> args;
        // Starts with the FPSCR and VFP exception setting of a new thread
        bool fresh_vfp_state;
    };

    // Runs guest code at one level, the thread's run or call, with mutex held through lock, which
    // it releases around guest code and the handling of each exit. Returns r0 once the level's
    // function returns, or nothing once the thread is ending.
    std::optional<uint32_t> run_guest(std::unique_lock<std::mutex> &lock, std::optional<GuestCall> call);
    // Sets the registers up for call, with mutex held
    void set_up_call(const GuestCall &call);
    // Ends the thread's run and returns whether it parks dormant for another start
    bool finish_run(std::unique_lock<std::mutex> &lock);
    // Calls the guest's data abort handler with the abort context on the stack, without mutex held
    void dispatch_abort(Address fault_addr);
    // Sets up a call of fn with args: the first four in r0 to r3, the rest on the stack, and the
    // kernel's halt instruction as the return address in LR
    void place_call(Address fn, std::initializer_list<uint32_t> args);
    // Writes an import's result to the registers the calling convention returns it in, without mutex held
    void write_call_result(const CallResult &result);

    // Fixed when the thread is created
    KernelState &kernel;

    mutable std::mutex mutex;
    // The one condition variable. The thread's waits, its dormant park and its holds wait on it,
    // and so do other threads that wait for its status. Every change any of them reads notifies all.
    std::condition_variable cv;

    // Host flags, guarded by mutex. The guest never sees them.
    // The debugger holds the thread, also after a breakpoint or a finished step
    bool debugger_hold = false;
    // The debugger asked for one instruction, which the next run of guest code takes
    bool step_requested = false;
    // The thread waits for its holds before guest code, with its registers still
    bool parked = false;
    // The thread waits for its holds inside a kernel call, with the call's registers
    bool held_in_call = false;
    // Blocked in a host wait inside an export
    bool host_waiting = false;
    // Running guest code inside cpu->run()
    bool running_guest = false;

    // Guards state.end_waiters, taken after mutex
    std::mutex end_waiters_mutex;
};

// The guest thread this host thread runs, or null on a host thread that runs no guest code. Signal
// handlers may read it.
GuestThread *current_thread();
