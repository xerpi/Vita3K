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

#include <kernel/thread/wait_queue.h>
#include <kernel/types.h>
#include <mem/block.h>
#include <mem/ptr.h>

#include <list>
#include <memory>
#include <string>

class Callback;

// Values are what sceKernelGetThreadInfo reports
enum class ThreadStatus : SceUInt32 {
    running = SCE_KERNEL_THREAD_STATUS_RUNNING,
    waiting = SCE_KERNEL_THREAD_STATUS_WAITING, // Waiting to be awaken by sync object or operation
    dormant = SCE_KERNEL_THREAD_STATUS_DORMANT, // Waiting for a job
    deleted = SCE_KERNEL_THREAD_STATUS_DELETED, // Gone for guest calls, until its host thread ends
    suspended = SCE_KERNEL_THREAD_STATUS_SUSPENDED, // Held by a ForVM suspend, only reported
};

// A guest thread as the Vita OS knows it. The GuestThread that owns it guards what changes with its
// mutex.
struct ThreadState final {
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

    struct EndWaitEntry {
        // Where to write the exit status, or null
        SceInt32 *exit_status;
    };

    // A dormant thread with its stack, and a TLS with room for user_tls_size bytes of the process's
    // own TLS
    ThreadState(SceUID id, MemState &mem, const char *name, Address entry_point, int init_priority, SceInt32 affinity_mask, int stack_size, SceUInt32 attr, SceSize user_tls_size);

    // A word of the thread's TLS, by its TLS_* index
    int &tls_slot(int index) const { return tls.get_ptr<int>().get(mem)[index]; }

    // Whether the thread is ending
    bool exiting() const { return ending != Ending::none; }
    bool ended() const { return status == ThreadStatus::dormant || status == ThreadStatus::deleted; }
    // Whether a callback of this thread was notified and hasn't run yet
    bool callbacks_notified() const { return notified_callbacks > 0; }
    // Whether a notified callback ends the current wait. Callbacks don't nest, so a wait inside a
    // callback runs none.
    bool callback_ends_wait() const { return wait_target.callbacks && !is_processing_callbacks && callbacks_notified(); }
    // Whether the thread's current wait is over, by a wake, its end or a callback it runs, though
    // the thread hasn't gone on yet
    bool wait_over() const { return exiting() || wake_pending || callback_ends_wait(); }

    // Fixed when the thread is created
    const SceUID id;
    const std::string name;
    const Address entry_point;
    // What thread info reports, the creation attributes and SCE_KERNEL_THREAD_ATTR_USER
    const SceUInt32 attr;
    // The stack size, a whole number of pages
    const int stack_size;
    const Block stack;
    const Block tls;
    // The priority and the cores it was created with, which each start goes back to
    const int init_priority;
    const SceInt32 init_affinity_mask;
    MemState &mem;

    // Lifecycle. RUNNING, WAITING, DORMANT or DELETED, SUSPENDED is only reported.
    ThreadStatus status = ThreadStatus::dormant;
    // The exit status the guest sees: DORMANT until the first start, NOT_DORMANT while the thread
    // runs, then what it returned or exited with
    uint32_t returned_value = SCE_KERNEL_ERROR_DORMANT;
    Ending ending = Ending::none;
    // Run the thread start event once before guest code. Set by start().
    bool run_start_callback = false;

    // Priority and affinity
    int priority;
    SceInt32 affinity_mask;

    // Waits
    // What the thread waits on while waiting, empty otherwise
    WaitTarget wait_target;
    // Set by wake() and consumed by the next wait().
    bool wake_pending = false;
    // A sceKernelSendSignal is pending for this thread.
    bool signal_pending = false;
    // The vblank the thread's last display wait ended on
    uint64_t last_vblank_waited = 0;
    // Threads blocked in sceKernelWaitThreadEnd on this one, guarded by the end waiters lock
    WaitQueue<EndWaitEntry> end_waiters;

    // Callbacks
    // Callbacks of this thread with notifications they haven't run for.
    SceUInt32 notified_callbacks = 0;
    // Set while the thread runs its callbacks. They don't nest.
    bool is_processing_callbacks = false;
    // Callbacks this thread created, in creation order. The kernel owns them. Only this thread
    // touches the list.
    std::list<std::weak_ptr<Callback>> callbacks;

    // ForVM suspend
    // The sceKernelSuspendThreadForVM calls no sceKernelResumeThreadForVM undid yet
    int vm_suspend_count = 0;
};
