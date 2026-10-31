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
#include <cpu/guest_boundary.h>
#include <kernel/thread/wait.h>
#include <kernel/types.h>
#include <mem/block.h>
#include <mem/ptr.h>

#include <algorithm>
#include <condition_variable>
#include <initializer_list>
#include <list>
#include <memory>
#include <mutex>
#include <optional>
#include <string>

struct ThreadState;
using ThreadStatePtr = std::shared_ptr<ThreadState>;
struct KernelState;
class Callback;
using CallbackPtr = std::shared_ptr<Callback>;

// The attribute of every user thread, which thread info always reports
constexpr SceUInt32 THREAD_ATTR_USER = 0x80000000U;
// Attributes a thread may be created with, which the Vita Development Wiki calls
// SCE_KERNEL_THREAD_ATTR_NOUSE_VFP and SCE_KERNEL_THREAD_ATTR_VM
constexpr SceUInt32 THREAD_ATTR_NOUSE_VFP = 0x01000000U;
constexpr SceUInt32 THREAD_ATTR_VM = 0x04000000U;
// Another attribute a Vita accepts for a thread, with no known name or effect
constexpr SceUInt32 THREAD_ATTR_UNKNOWN_2000 = 0x00002000U;
// The FPSCR a thread starts with: round to nearest, flush-to-zero and default NaN
constexpr uint32_t INITIAL_FPSCR = 0x03000000U;

// Values are what sceKernelGetThreadInfo reports
enum class ThreadStatus : SceUInt32 {
    running = SCE_KERNEL_THREAD_STATUS_RUNNING,
    waiting = SCE_KERNEL_THREAD_STATUS_WAITING, // Waiting to be awaken by sync object or operation
    dormant = SCE_KERNEL_THREAD_STATUS_DORMANT, // Waiting for a job
    deleted = SCE_KERNEL_THREAD_STATUS_DELETED, // Gone for guest calls, until its host thread ends
    suspended = SCE_KERNEL_THREAD_STATUS_SUSPENDED, // Suspended by debugger
};

struct ThreadState : private GuestBoundary {
    // Fixed when the thread is created
    const SceUID id;
    const std::string name;
    const Address entry_point;
    // What thread info reports, the creation attributes and THREAD_ATTR_USER
    const SceUInt32 attr;
    // The stack size, a whole number of pages
    const int stack_size;
    const Block stack;
    const Block tls;
    // The pointer is fixed. The registers are only touched by the thread itself, or while
    // something holds it.
    const CPUStatePtr cpu;
    // The priority and the cores it was created with, which each start goes back to
    const int init_priority;
    const SceInt32 init_affinity_mask;

    // A dormant thread with its stack and TLS, which runs on cpu
    ThreadState(SceUID id, KernelState &kernel, MemState &mem, CPUStatePtr cpu, const char *name, Address entry_point, int init_priority, SceInt32 affinity_mask, int stack_size, SceUInt32 attr);

    // Lifecycle
    // Starts a dormant thread at entry_point. Takes end's lock, then mutex.
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
    // Calls a guest function on the thread's CPU from inside a kernel call, usually one the thread
    // itself made, and returns its r0, or the exit status if the thread ends during it. With
    // fresh_vfp_state, the call starts with the FPSCR and VFP exception setting of a new thread.
    uint32_t run_callback(Address callback_address, std::initializer_list<uint32_t> args, bool fresh_vfp_state = false);
    // Starts the dormant thread from another thread and returns its exit status once it ends. Used
    // for module start and stop and the GXM display queue.
    uint32_t run_guest_function(SceSize args, const Ptr<void> argp = Ptr<void>{});

    // Status and info
    ThreadStatus get_status() const;
    // The thread's info, as sceKernelGetThreadInfo reports it
    SceKernelThreadInfo info();
    // The room left on the stack, all of it for a dormant thread
    int stack_free_size() const;
    // A word of the thread's TLS, by its TLS_* index
    int &tls_slot(int index) const { return tls.get_ptr<int>().get(mem)[index]; }

    // Priority and affinity
    // The priority the thread runs at
    int get_priority() const;
    // Changes the priority the thread runs at, and the one its TLS shows. Returns the old priority.
    int change_priority(int new_priority);
    // Changes the cores the thread may run on, and the ones its TLS shows. Returns the old mask.
    SceInt32 change_affinity_mask(SceInt32 new_affinity_mask);

    // Waits
    // Blocks this thread in a delay or a signal wait, which are in no queue, until the deadline, a
    // signal for a signal wait or the thread's exit ends it. Returns how the wait ended.
    [[nodiscard]] WaitResult wait_in_no_queue(WaitTarget target, WaitOptions &options, Deadline deadline);
    // Sends a signal to this thread. Fails if the previous one was not consumed yet.
    SceInt32 send_signal();
    // Blocks the waiting thread until this thread ends or the timeout passes. Writes back the exit
    // status and the time left.
    [[nodiscard]] WaitResult wait_for_thread_end(ThreadState &waiter, SceInt32 &exit_status, WaitOptions &options);
    // Shows the thread WAITING while it lives, for a host call that blocks the thread, like an audio
    // output, and RUNNING after
    class HostWait {
    public:
        explicit HostWait(ThreadState &thread);
        ~HostWait();
        HostWait(const HostWait &) = delete;
        HostWait &operator=(const HostWait &) = delete;

    private:
        ThreadState &thread;
    };
    // The vblank the thread's last display wait ended on. Only the thread's display waits use it,
    // under the display's lock.
    uint64_t last_vblank_waited = 0;

    // Callbacks
    // Runs the notified callbacks of this thread and returns how many ran. Called by the thread itself.
    SceUInt32 process_callbacks();
    // Adds a callback this thread created. Called by the thread itself.
    void add_callback(const CallbackPtr &cb) { callbacks.push_back(cb); }

    // Holds: the debugger, the emulator pause and the ForVM suspend
    // Stops the thread for the debugger the next time it would run guest code, or lets it run again.
    // A pause or a ForVM suspend may still hold it after the debugger lets go.
    void set_suspended(bool suspended);
    // Lets a thread the debugger stopped run one instruction, then waits until it stops again or is
    // dormant or deleted. Returns false and doesn't step while something else holds the thread.
    bool step();
    // Waits until the thread doesn't run guest code
    void wait_until_not_running();
    // Whether the debugger's hold stopped the thread at a breakpoint
    bool stopped_at_breakpoint() const;
    // Stops the thread while the emulator is paused, or lets it run again
    void set_paused(bool paused);
    // Suspends the thread for sceKernelSuspendThreadForVM: it doesn't run guest code until as many
    // vm_resume calls. vm_suspend returns once it has stopped. Each returns the suspend count after
    // it, or an error for a dormant thread or a resume with no suspend.
    int vm_suspend();
    int vm_resume();
    // Waits until a ForVM suspend has stopped the thread, then gets the context it stopped with, its
    // TPIDRURW and whether it stopped inside a kernel call. Returns NOT_SUSPENDED if no ForVM
    // suspend holds it.
    SceInt32 get_vm_context(CPUContext &context, uint32_t &tpidrurw, bool &in_kernel_call);

private:
    // How a thread ends, each value stronger than the ones before it
    enum class Ending : uint8_t {
        none,
        // An exit, or a return from the entry: the end handler runs, then the thread parks dormant
        exit,
        // The end handler runs, then the host thread ends
        exit_delete,
        // The host thread ends without running the end handler
        remove,
    };

    // What keeps the thread from running guest code. It runs only while nothing holds it.
    struct Holds {
        // The emulator is paused
        bool pause = false;
        // The debugger stopped it, or it hit a breakpoint or finished a step
        bool debugger = false;
        // The sceKernelSuspendThreadForVM calls no sceKernelResumeThreadForVM undid yet
        int vm = 0;
    };

    friend class ThreadWait;
    // Reads whether the thread still waits or is exiting
    friend class MultiEventWait;
    // A callback notify ends the wait or removes it from its queue
    friend class Callback;

    // Marks a callback notified, ends a wait in no queue that takes callbacks and wakes a queued one
    // to leave its queue. Takes the thread lock, which goes after the callback lock.
    void notify_callback();
    // Returns once a queued wait that a notified callback ends has left its queue. Takes the thread
    // lock and blocks, so the caller holds no lock.
    void wait_for_departure();

    // The private methods are called with mutex held, unless they say otherwise
    // Whether the thread is in a wait that hasn't ended
    bool still_waits() const { return active_wait && !active_wait->result; }
    void update_status(ThreadStatus status);
    // Whether the thread is ending
    bool exiting() const { return ending != Ending::none; }
    // Asks the thread to end, keeping a stronger request already made
    void request_ending(Ending how) { ending = std::max(ending, how); }

    // Runs guest code from first at one level, the thread's run or a call, with mutex held through
    // lock. Returns r0 once the level's function returns, or nothing once the thread is ending.
    std::optional<uint32_t> run_guest(std::unique_lock<std::mutex> &lock, StopPoint first);
    // The kernel side of the thread's guest code, see GuestBoundary. Takes the thread lock.
    GuestResume resume_guest(StopPoint where, bool debugger_stop, bool returned) override;
    void kernel_call(uint32_t nid) override;
    // Calls the guest's data abort handler with the abort's context on the stack
    void guest_abort() override;

    // Fixed when the thread is created
    KernelState &kernel;
    MemState &mem;

    mutable std::mutex mutex;

    // The rest is guarded by mutex, unless it says otherwise

    // Lifecycle
    ThreadStatus status = ThreadStatus::dormant;
    // The exit status the guest sees: DORMANT until the first start, NOT_DORMANT while the thread
    // runs, then what it returned or exited with
    uint32_t returned_value = SCE_KERNEL_ERROR_DORMANT;
    Ending ending = Ending::none;
    // Run the thread start event once before guest code. Set by start().
    bool run_start_callback = false;
    // The one condition variable. The thread's waits, its dormant park and its hold gate wait on it,
    // and so do other threads that wait for its status. Every change any of them reads notifies all.
    std::condition_variable cv;

    // Priority and affinity
    int priority;
    SceInt32 affinity_mask;

    // Waits
    // The wait the thread is in, owned by the waiting call, or null
    ThreadWait *active_wait = nullptr;
    // A sceKernelSendSignal is pending for this thread.
    bool signal_pending = false;

    // Callbacks
    // A callback of this thread was notified since the last callback pass began. Every notify sets
    // it, and the start of a pass clears it. Canceling or deleting a callback leaves it set.
    bool callbacks_pending = false;
    // Callbacks this thread created, in creation order. The kernel owns them. Only this thread
    // touches the list. A pass that runs inside a callback may erase entries of deleted callbacks,
    // but never the one an outer pass is running, since that pass keeps it alive.
    std::list<std::weak_ptr<Callback>> callbacks;

    // Holds
    Holds holds;
    // The debugger asked for one instruction, after which the thread holds again
    bool step_requested = false;
    // Where the thread stopped the last time something held it
    StopPoint stop_point = StopPoint::guest_code;

    // The threads blocked in sceKernelWaitThreadEnd on this one
    struct ThreadEnd {
        // One end wait, which the thread's end gives the exit status
        struct Request {
            // Where the exit status goes. The waiting call owns this variable.
            SceInt32 &exit_status;
        };

        // Guards the queue and whether the thread has ended. Take it before the thread's lock or a
        // waiter's lock, and hold only one thread lock at a time under it. The thread's start and end
        // also take it, so the thread can't start or end while a waiter holds it.
        std::mutex mutex;
        WaitQueue<Request> waiters{ QueueOrder::fifo };
    };
    ThreadEnd end;
};
