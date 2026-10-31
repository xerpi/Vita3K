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

#include <cpu/functions.h>
#include <cpu/state.h>
#include <kernel/thread/thread_state.h>

#include <kernel/state.h>
#include <kernel/sync_primitives.h>
#include <mem/ptr.h>
#include <util/align.h>

#include <util/log.h>

#include <algorithm>
#include <cassert>
#include <cstring>
#include <memory>
#include <tuple>
#include <utility>

// Set in the wait type of a wait that runs callbacks
constexpr SceUInt32 WAITTYPE_CB_BIT = 0x80000000U;

// The priority a thread created with priority runs at. The default range stands for the game's own.
static int created_priority(int priority) {
    if (priority > SCE_KERNEL_LOWEST_PRIORITY_USER) {
        assert(SCE_KERNEL_HIGHEST_DEFAULT_PRIORITY <= priority && priority <= SCE_KERNEL_LOWEST_DEFAULT_PRIORITY);
        return priority - SCE_KERNEL_DEFAULT_PRIORITY + SCE_KERNEL_GAME_DEFAULT_PRIORITY_ACTUAL;
    }
    return priority;
}

// The size of the kernel's part of a thread's TLS, below the part the guest sees
constexpr size_t KERNEL_TLS_SIZE = 0x800;

// The registers each run of a thread starts with
static CPUContext initial_context(Address stack_top) {
    CPUContext ctx;
    // A Vita starts every thread with these in r2 to r12, and some games rely on r9 not being 0
    std::fill(ctx.cpu_registers.begin() + 2, ctx.cpu_registers.begin() + 13, 0xDEADBEEF);
    ctx.set_sp(stack_top);
    ctx.fpscr = INITIAL_FPSCR;
    return ctx;
}

ThreadState::ThreadState(SceUID id, KernelState &kernel, MemState &mem, CPUStatePtr cpu, const char *name, Address entry_point, int init_priority, SceInt32 affinity_mask, int stack_size, SceUInt32 attr)
    : id(id)
    , name(name)
    , entry_point(entry_point)
    , attr(attr | THREAD_ATTR_USER)
    , stack_size(align(stack_size, KiB(4)))
    , stack(alloc_block(mem, this->stack_size, fmt::format("Stack for thread {} (#{})", name, id).c_str()))
    , tls(alloc_block(mem, KERNEL_TLS_SIZE + kernel.tls_msize, fmt::format("TLS for thread {} (#{})", name, id).c_str()))
    , cpu(std::move(cpu))
    , init_priority(created_priority(init_priority))
    , init_affinity_mask(affinity_mask)
    , kernel(kernel)
    , mem(mem)
    , priority(this->init_priority)
    , affinity_mask(affinity_mask) {
    memset(stack.get_ptr<void>().get(mem), 0xcc, this->stack_size);
    memset(tls.get_ptr<void>().get(mem), 0, KERNEL_TLS_SIZE + kernel.tls_msize);

    tls_slot(TLS_THREAD_ID) = id;
    tls_slot(TLS_SP_TOP) = stack.get() + this->stack_size;
    tls_slot(TLS_SP_BOTTOM) = stack.get();
    // A Vita starts every thread with a different random value in each of these. These are the
    // values one thread had on a PS Vita TV, the same for every thread so runs repeat.
    tls_slot(TLS_RESERVED_6) = static_cast<int>(0xFEFA7314);
    tls_slot(TLS_STACK_CHECK_GUARD) = static_cast<int>(0xC325C2A7);
    tls_slot(TLS_CURRENT_PRIORITY) = priority;
    tls_slot(TLS_CPU_AFFINITY_MASK) = affinity_mask;

    const Ptr<uint8_t> user_tls_ptr = tls.get_ptr<uint8_t>() + KERNEL_TLS_SIZE;
    write_tpidruro(*this->cpu, user_tls_ptr.address());
    if (kernel.tls_address) {
        assert(kernel.tls_psize <= kernel.tls_msize);
        memcpy(user_tls_ptr.get(mem), kernel.tls_address.get(mem), kernel.tls_psize);
    }
}

int ThreadState::start(SceSize arglen, const Ptr<void> argp, bool run_entry_callback) {
    // Threads waiting for this one to end read its status under the end queue's lock, so hold it
    const std::lock_guard<std::mutex> end_lock(end.mutex);
    const std::lock_guard<std::mutex> thread_lock(mutex);
    if (status != ThreadStatus::dormant)
        return SCE_KERNEL_ERROR_NOT_DORMANT;
    run_start_callback = run_entry_callback;
    returned_value = SCE_KERNEL_ERROR_NOT_DORMANT;
    // Each run starts at the priority and on the cores the thread was created with, and its TLS
    // shows them
    priority = init_priority;
    affinity_mask = init_affinity_mask;
    tls_slot(TLS_CURRENT_PRIORITY) = priority;
    tls_slot(TLS_CPU_AFFINITY_MASK) = affinity_mask;
    load_context(*cpu, initial_context(stack.get() + stack_size));
    // A thread starts with TPIDRURW at 0, also when it starts again
    write_tpidrurw(*cpu, 0);
    // The entry gets the size of the argument data and its copy on the stack, or null for none
    Address data_addr = 0;
    if (argp && arglen > 0) {
        data_addr = stack_alloc(*cpu, align(arglen, 8));
        memcpy(Ptr<uint8_t>(data_addr).get(mem), argp.get(mem), arglen);
    }
    place_call(*cpu, entry_point, { arglen, data_addr });

    update_status(ThreadStatus::running);
    return SCE_KERNEL_OK;
}

void ThreadState::exit(SceInt32 status) {
    const std::lock_guard<std::mutex> lock(mutex);
    request_ending(Ending::exit);
    returned_value = static_cast<uint32_t>(status);
}

void ThreadState::exit_delete(SceInt32 status) {
    const std::lock_guard<std::mutex> lock(mutex);
    request_ending(Ending::exit_delete);
    returned_value = static_cast<uint32_t>(status);
}

void ThreadState::remove() {
    const std::lock_guard<std::mutex> lock(mutex);
    request_ending(Ending::remove);
    // A dormant thread is gone at once
    if (status == ThreadStatus::dormant)
        update_status(ThreadStatus::deleted);
    else if (status == ThreadStatus::running)
        stop(*cpu);

    // Wake the thread if it is parked dormant, held at its gate or blocked in a wait
    cv.notify_all();
}

SceInt32 ThreadState::remove_dormant() {
    const std::lock_guard<std::mutex> lock(mutex);
    if (status != ThreadStatus::dormant)
        return SCE_KERNEL_ERROR_NOT_DORMANT;
    // A dormant thread is gone at once, and the status change wakes its parked host thread to end
    request_ending(Ending::remove);
    update_status(ThreadStatus::deleted);
    return SCE_KERNEL_OK;
}

void ThreadState::run_loop() {
    // Set thread-local CPU state so signal handlers can access it, also while the thread ends. The
    // guard puts back what was there before, so a recycled host thread never sees a stale CPUState
    // pointer. execute() sets it for each run of guest code too.
    struct CpuStateGuard {
        CPUState *const previous = get_current_cpu_state();
        ~CpuStateGuard() { set_current_cpu_state(previous); }
    } cpu_state_guard;
    set_current_cpu_state(cpu.get());

    std::unique_lock<std::mutex> lock(mutex);
    while (true) {
        // A thread that returns from its entry ends like one that exits, end handler included
        if (const std::optional<uint32_t> returned = run_guest(lock, StopPoint::guest_code)) {
            returned_value = *returned;
            request_ending(Ending::exit);
        }

        // The thread is ending. One that ran ends here: an exit leaves it dormant, while a delete
        // makes it gone for guest calls before its end waiters wake.
        if (status != ThreadStatus::dormant && status != ThreadStatus::deleted) {
            if (ending != Ending::remove && kernel.thread_event_end) {
                // The thread is RUNNING here, since its last check or retire showed it so. The
                // handler runs guest code on it, which a pending ending would end at once, and an
                // exit in the handler must not change the exit status.
                const uint32_t old_returned_value = returned_value;
                const Ending old_ending = std::exchange(ending, Ending::none);
                lock.unlock();
                // A thread event handler starts with the VFP state of a new thread, like a callback
                const int ret = run_callback(kernel.thread_event_end.address(), { SCE_KERNEL_THREAD_EVENT_TYPE_END, static_cast<uint32_t>(id), 0, kernel.thread_event_end_arg }, true);
                if (ret != 0)
                    LOG_WARN("Thread end event handler returned {}", log_hex(ret));
                lock.lock();
                returned_value = old_returned_value;
                // Keep a stronger request made while the handler ran, like a teardown's remove
                request_ending(old_ending);
            }

            lock.unlock();
            // A mutex or a write lock the thread still holds goes to its next waiter, and the
            // event flags it holds are free again
            kernel.objects.for_each<HeavyMutex>([&](Mutex &mutex) { mutex.release_ended_owner(*this); });
            kernel.objects.for_each<RWLock>([&](RWLock &rwlock) { rwlock.release_ended_writer(*this); });
            kernel.objects.for_each<EventFlag>([&](EventFlag &flag) { flag.release_ended_holder(*this); });
            // The callbacks the thread created end with it, with their pending notifications
            for (const auto &weak_cb : callbacks) {
                if (const CallbackPtr cb = weak_cb.lock())
                    kernel.objects.remove<Callback>(cb->uid, id);
            }
            callbacks.clear();
            // Lock order: the end queue's lock before the thread lock
            const std::lock_guard<std::mutex> end_lock(end.mutex);
            lock.lock();
            update_status(ending == Ending::exit ? ThreadStatus::dormant : ThreadStatus::deleted);
            const SceInt32 exit_status = static_cast<SceInt32>(returned_value);
            // Completing a waiter takes its thread lock, so release ours first
            lock.unlock();
            // Wake the waiters in queue order and give each the exit status
            while (auto *const node = end.waiters.front()) {
                node->request.exit_status = exit_status;
                end.waiters.complete(*node);
            }
            // Take the thread lock back before the end queue's lock is released
            lock.lock();
        }
        if (ending != Ending::exit)
            break;
        // The thread is dormant now, and the next run_guest parks it until a start or a delete
        ending = Ending::none;
    }
}

std::optional<uint32_t> ThreadState::run_guest(std::unique_lock<std::mutex> &lock, StopPoint first) {
    while (true) {
        // Park until the thread runs: dormant between starts, or busy elsewhere for a call that
        // another host thread makes
        cv.wait(lock, [&] { return status == ThreadStatus::running || exiting(); });
        if (exiting())
            return std::nullopt;

        // The thread start event runs once per start, with the VFP state of a new thread
        const bool start_event = std::exchange(run_start_callback, false) && kernel.thread_event_start;
        lock.unlock();
        if (start_event) {
            const int ret = run_callback(kernel.thread_event_start.address(), { SCE_KERNEL_THREAD_EVENT_TYPE_START, static_cast<uint32_t>(id), 0, kernel.thread_event_start_arg }, true);
            if (ret != 0)
                LOG_WARN("Thread start event handler returned {}", log_hex(ret));
        }
        // execute() leaves when the thread is ending or stops running, and the next pass parks
        const std::optional<uint32_t> returned = execute(*cpu, *this, first);
        lock.lock();
        if (returned)
            return returned;
    }
}

GuestResume ThreadState::resume_guest(StopPoint where, bool debugger_stop, bool returned) {
    std::unique_lock<std::mutex> lock(mutex);
    // A finished step or a breakpoint holds the thread for the debugger
    if (debugger_stop)
        holds.debugger = true;
    if (returned || exiting() || status != ThreadStatus::running)
        return GuestResume::leave;
    stop_point = where;
    // The holds can change while the thread waits, and with them what its status shows
    while (!exiting()) {
        // The guest sees the debugger's hold and a ForVM suspend as SUSPENDED, but not a pause
        const bool shown = holds.debugger || holds.vm > 0;
        if (!shown && !holds.pause)
            break;
        const ThreadStatus held_status = shown ? ThreadStatus::suspended : ThreadStatus::running;
        if (status != held_status)
            update_status(held_status);
        cv.wait(lock);
    }
    if (status != ThreadStatus::running)
        update_status(ThreadStatus::running);
    if (exiting())
        return GuestResume::leave;
    return std::exchange(step_requested, false) ? GuestResume::step : GuestResume::run;
}

void ThreadState::kernel_call(uint32_t nid) {
    kernel.call_import(*cpu, nid, id);
}

uint32_t ThreadState::run_callback(Address callback_address, std::initializer_list<uint32_t> args, bool fresh_vfp_state) {
    std::unique_lock<std::mutex> thread_lock(mutex);

    // Save the thread's context before the call overwrites PC and LR
    const CPUContext previous_ctx = save_context(*cpu);
    const uint32_t previous_tpidruro = read_tpidruro(*cpu);
    // A callback sees the thread's TPIDRURW, and the thread gets its own value back after it
    const uint32_t previous_tpidrurw = read_tpidrurw(*cpu);
    int &vfp_exception = tls_slot(TLS_VFP_EXCEPTION);
    const int previous_vfp_exception = vfp_exception;
    if (fresh_vfp_state) {
        write_fpscr(*cpu, INITIAL_FPSCR);
        vfp_exception = 0;
    }

    place_call(*cpu, callback_address, args);

    // The callback runs inside a kernel call, so a hold before its first instruction finds the
    // thread on its way back from one
    const std::optional<uint32_t> returned = run_guest(thread_lock, StopPoint::syscall);

    // The caller and the callback should keep to the calling convention, but put everything back
    load_context(*cpu, previous_ctx);
    write_tpidruro(*cpu, previous_tpidruro);
    write_tpidrurw(*cpu, previous_tpidrurw);
    if (fresh_vfp_state)
        vfp_exception = previous_vfp_exception;

    // A callback the thread's end cut short returns the exit status
    return returned.value_or(returned_value);
}

void ThreadState::guest_abort() {
    CPUState &cpu = *this->cpu;
    const uint32_t fault_addr = cpu.abort_fault_addr.load();
    // DABT = type 0
    const Address handler = kernel.exception_handlers[0].load();
    if (!handler)
        return;

    // The handler reads a KuKernelAbortContext on the stack: r0 to r12, sp, lr, pc and the fault
    // address. protect_tree already unprotected the page and the CPU may have run past the faulting
    // instruction, so the call is only a notification and nothing is restored from the context.
    const uint32_t ctx_size = 17 * 4;
    const uint32_t sp_orig = read_sp(cpu);
    const uint32_t sp_aligned = align_down(sp_orig - ctx_size, 8);

    auto *ctx = Ptr<uint32_t>(sp_aligned).get(*cpu.mem);
    for (int i = 0; i < 13; i++)
        ctx[i] = read_reg(cpu, i);
    ctx[13] = sp_aligned + ctx_size;
    ctx[14] = read_lr(cpu);
    ctx[15] = read_pc(cpu);
    ctx[16] = fault_addr;

    LOG_DEBUG("DABT handler=0x{:08X} FAR=0x{:08X} PC=0x{:08X} SP=0x{:08X} sp_aligned=0x{:08X}",
        handler, fault_addr, ctx[15], sp_orig, sp_aligned);

    run_callback(handler, { sp_aligned });
}

uint32_t ThreadState::run_guest_function(SceSize args, const Ptr<void> argp) {
    start(args, argp);
    std::unique_lock<std::mutex> lock(mutex);
    cv.wait(lock, [&] { return status == ThreadStatus::dormant || status == ThreadStatus::deleted; });
    return returned_value;
}

void ThreadState::update_status(ThreadStatus status) {
    this->status = status;
    cv.notify_all();
}

WaitResult ThreadState::wait_in_no_queue(WaitTarget target, WaitOptions &options, Deadline deadline) {
    ThreadWait wait(*this, target, options, deadline);
    // Set when a callback notify ended the last sleep
    bool notified = false;
    while (true) {
        // Run pending callbacks before each sleep. The deadline is already set, so their time counts
        // against it.
        if (const WaitResult ran = wait.run_callbacks(); !ran)
            return ran;
        if (const std::lock_guard<std::mutex> lock(mutex); const std::optional<WaitResult> refused = wait.begin()) {
            // A delay doesn't begin if a callback became pending after the check above. Loop back
            // and run it if a notify ended the last sleep. A signal wait sleeps through it.
            if (notified && *refused && **refused == SCE_KERNEL_ERROR_NOTIFY_CALLBACK)
                continue;
            return *refused;
        }
        // Only a send, the timeout, a callback notify or the thread exiting can end the wait, all
        // under the thread lock
        std::ignore = wait.sleep();
        const WaitResult ended = wait.retire();
        if (!ended || *ended != SCE_KERNEL_ERROR_NOTIFY_CALLBACK)
            return ended;
        notified = true;
    }
}

SceInt32 ThreadState::send_signal() {
    const std::lock_guard<std::mutex> lock(mutex);
    // End a signal wait in progress. Otherwise keep the signal pending for the next wait.
    if (still_waits() && active_wait->target.type == SCE_KERNEL_WAITTYPE_SIGNAL) {
        active_wait->end(SCE_KERNEL_OK);
        return SCE_KERNEL_OK;
    }
    return std::exchange(signal_pending, true) ? SCE_KERNEL_ERROR_ALREADY_SENT : SCE_KERNEL_OK;
}

ThreadStatus ThreadState::get_status() const {
    const std::lock_guard<std::mutex> lock(mutex);
    return status;
}

int ThreadState::stack_free_size() const {
    const std::lock_guard<std::mutex> lock(mutex);
    // A dormant thread's whole stack is free, a started one's is free below its stack pointer
    if (status == ThreadStatus::dormant)
        return stack_size;
    return static_cast<int>(read_sp(*cpu) - stack.get());
}

ThreadState::HostWait::HostWait(ThreadState &thread)
    : thread(thread) {
    const std::lock_guard<std::mutex> lock(thread.mutex);
    thread.update_status(ThreadStatus::waiting);
}

ThreadState::HostWait::~HostWait() {
    const std::lock_guard<std::mutex> lock(thread.mutex);
    thread.update_status(ThreadStatus::running);
}

WaitResult ThreadState::wait_for_thread_end(ThreadState &waiter, SceInt32 &exit_status, WaitOptions &options) {
    QueuedThreadWait wait(end.waiters, std::unique_lock{ end.mutex }, waiter, { SCE_KERNEL_WAITTYPE_WAITTHEND, id }, ThreadEnd::Request{ .exit_status = exit_status }, options);
    // A waiter that leaves frees nothing for the others
    const auto leave = [](WaitResult) {};
    // The end queue is never deleted, and callbacks run only at the top of each pass
    return wait.run(FinalCallbackCheck::none, leave, [&]() -> std::optional<SceInt32> {
        // Briefly takes the ended thread's lock, which is allowed under the end queue's lock
        const std::lock_guard<std::mutex> lock(mutex);
        if (status != ThreadStatus::dormant && status != ThreadStatus::deleted)
            return std::nullopt;
        exit_status = static_cast<SceInt32>(returned_value);
        return SCE_KERNEL_OK;
    });
}

void ThreadWait::wake() {
    const std::lock_guard<std::mutex> lock(thread.mutex);
    recheck_pending = true;
    thread.cv.notify_all();
}

bool ThreadWait::callback_ends() const {
    return options.callbacks && thread.callbacks_pending;
}

std::unique_lock<std::mutex> ThreadWait::lock_thread() const {
    return std::unique_lock(thread.mutex);
}

std::optional<WaitResult> ThreadWait::begin() {
    if (thread.exiting())
        return std::unexpected{ ThreadExiting{} };
    // Take a signal sent earlier without waiting
    if (target.type == SCE_KERNEL_WAITTYPE_SIGNAL && std::exchange(thread.signal_pending, false))
        return SCE_KERNEL_OK;
    // Check the deadline first, so a timeout wins over a pending callback
    if (std::chrono::steady_clock::now() >= deadline) {
        options.left = 0;
        if (options.timeout && options.timeout_in_place)
            *options.timeout = 0;
        return SCE_KERNEL_ERROR_WAIT_TIMEOUT;
    }
    // A queued wait never begins with a callback pending, so a callback notify that finds one
    // pending later was made during the wait. Signal and multiple event waits sleep through it.
    if (target.type != SCE_KERNEL_WAITTYPE_SIGNAL && target.type != SCE_KERNEL_WAITTYPE_MP_EVENTS && callback_ends())
        return SCE_KERNEL_ERROR_NOTIFY_CALLBACK;
    // Each pass starts with no result
    result.reset();
    priority = thread.priority;
    thread.active_wait = this;
    thread.update_status(ThreadStatus::waiting);
    return std::nullopt;
}

void ThreadWait::end(WaitResult ended) {
    result = ended;
    // The time left is taken when the wait ends, also for an exit. A message pipe writes it at once,
    // while the waker still holds the queue's lock.
    if (options.timeout) {
        const auto left = std::chrono::duration_cast<std::chrono::microseconds>(deadline - std::chrono::steady_clock::now()).count();
        options.left = left > 0 ? static_cast<SceUInt32>(left) : 0;
        if (options.timeout_in_place)
            *options.timeout = options.left;
    }
    thread.cv.notify_all();
}

WaitResult ThreadWait::sleep() {
    while (true) {
        // Check the wait first, so timer events fire only while the wait is running and the thread
        // is not exiting. The look also takes any recheck wake() asked for. The waiter ends the wait
        // itself in the same hold that looked, so a party that ended it first wins.
        {
            const auto lock = lock_thread();
            recheck_pending = false;
            if (!result && thread.exiting())
                end(std::unexpected{ ThreadExiting{} });
            // A queued wait leaves its queue itself for a notified callback, under its object lock
            if (!result && queued && callback_ends())
                end(SCE_KERNEL_ERROR_NOTIFY_CALLBACK);
            if (result)
                return *result;
        }
        const Deadline now = std::chrono::steady_clock::now();
        // Fire the timer events due by now or the deadline, under the same object lock hold as the
        // timeout check below
        const Deadline look = watch(std::min(now, deadline));
        if (now >= deadline) {
            const auto lock = lock_thread();
            // A timer event due by the deadline may have ended the wait in watch()
            if (!result)
                end(SCE_KERNEL_ERROR_WAIT_TIMEOUT);
            return *result;
        }
        // Sleep until the next clock check, the deadline or a wake()
        if (queued)
            object_lock.unlock();
        {
            std::unique_lock<std::mutex> lock(thread.mutex);
            const auto over = [&] { return thread.exiting() || result || recheck_pending || (queued && callback_ends()); };
            const Deadline until = std::min(look, deadline);
            if (until == Deadline::max())
                thread.cv.wait(lock, over);
            else
                thread.cv.wait_until(lock, until, over);
        }
        if (queued)
            object_lock.lock();
    }
}

WaitResult ThreadWait::retire() {
    // Drop the ended wait and show the thread running again, even if it is held. The hold gate
    // still stops it before callbacks or guest code run.
    const std::lock_guard<std::mutex> lock(thread.mutex);
    thread.active_wait = nullptr;
    thread.update_status(ThreadStatus::running);
    if (thread.exiting())
        return std::unexpected{ ThreadExiting{} };
    return *result;
}

WaitResult ThreadWait::run_callbacks() {
    {
        const std::lock_guard<std::mutex> lock(thread.mutex);
        if (!callback_ends())
            return SCE_KERNEL_OK;
    }
    if (queued)
        object_lock.unlock();
    thread.process_callbacks();
    if (queued)
        object_lock.lock();
    // The thread exited or was deleted during the callbacks, so the wait must not complete. Else the
    // callbacks may have changed what the caller waits for, so it rechecks before waiting again.
    const std::lock_guard<std::mutex> lock(thread.mutex);
    if (thread.exiting())
        return std::unexpected{ ThreadExiting{} };
    return SCE_KERNEL_OK;
}

SceUInt32 ThreadState::process_callbacks() {
    {
        // A notify from here on is for a callback this pass may already have passed, so it stays
        // recorded for the next check. Set under the thread lock, since notify_callback() reads it.
        const std::lock_guard<std::mutex> lock(mutex);
        callbacks_pending = false;
    }
    SceUInt32 processed = 0;
    // Callbacks nest: a callback that checks or waits with callbacks runs the other pending ones
    // inside it. So the pass moves to the next entry only after the callback ran.
    for (auto it = callbacks.begin(); it != callbacks.end();) {
        const CallbackPtr cb = it->lock();
        // Deleted since it was added
        if (!cb) {
            it = callbacks.erase(it);
            continue;
        }
        const std::optional<Callback::Notification> notification = cb->take_notification();
        if (!notification) {
            ++it;
            continue;
        }
        // A callback starts with the VFP state of a new thread, and the thread gets its own back after it
        const uint32_t ret = run_callback(cb->cb_func.address(),
            { static_cast<uint32_t>(notification->notifier_id), notification->count, static_cast<uint32_t>(notification->arg), cb->userdata.address() }, true);
        ++processed;
        // A callback that exits the thread also ends the wait it runs in
        {
            const std::lock_guard<std::mutex> lock(mutex);
            // The thread's other callbacks are deleted when it ends, with their notifications
            if (exiting())
                break;
        }
        // A callback that returns nonzero deletes itself
        if (ret != 0)
            kernel.objects.remove<Callback>(cb->uid, id);
        ++it;
    }
    return processed;
}

void ThreadState::notify_callback() {
    const std::lock_guard<std::mutex> lock(mutex);
    callbacks_pending = true;
    if (!still_waits() || !active_wait->callback_ends())
        return;
    // A wait in no queue ends here, in the same step that counts the notify. A queued wait can only
    // end under its queue's lock, which goes before the thread lock, so the waiter wakes and leaves
    // its queue itself.
    if (!active_wait->queued)
        active_wait->end(SCE_KERNEL_ERROR_NOTIFY_CALLBACK);
    else
        cv.notify_all();
}

void ThreadState::wait_for_departure() {
    std::unique_lock<std::mutex> lock(mutex);
    // A queued wait that ended is dropped only after it served the waiters behind it, so wait for
    // that. A wait in no queue that began after the notify sleeps through the pending callback.
    cv.wait(lock, [&] { return !active_wait || !active_wait->queued || (!active_wait->result && !active_wait->callback_ends()); });
}

SceKernelThreadInfo ThreadState::info() {
    const std::lock_guard<std::mutex> lock(mutex);
    SceKernelThreadInfo info{};
    strncpy(info.name, name.c_str(), KERNELOBJECT_MAX_NAME_LENGTH);
    info.attr = attr;
    info.stack = Ptr<void>(stack.get());
    info.stackSize = stack_size;
    info.initPriority = init_priority;
    info.currentPriority = priority;
    info.initCpuAffinityMask = init_affinity_mask;
    info.currentCpuAffinityMask = affinity_mask;
    info.entry = SceKernelThreadEntry(entry_point);
    // A thread whose kernel wait is over is READY until it goes on, and waits on nothing, like on a
    // Vita, where the waker ends the wait. A host wait, like an audio output, sets no wait and stays
    // WAITING.
    const bool ready = active_wait && status == ThreadStatus::waiting && (active_wait->result || exiting());
    const WaitTarget target = active_wait && !ready ? active_wait->target : WaitTarget{};
    info.status = ready ? SCE_KERNEL_THREAD_STATUS_READY : std::to_underlying(status);
    info.waitType = active_wait && !ready && active_wait->options.callbacks ? target.type | WAITTYPE_CB_BIT : target.type;
    info.waitId = target.id;
    info.exitStatus = returned_value;
    info.fNotifyCallback = callbacks_pending ? 1 : 0;
    return info;
}

int ThreadState::get_priority() const {
    const std::lock_guard<std::mutex> lock(mutex);
    return priority;
}

int ThreadState::change_priority(int new_priority) {
    const std::lock_guard<std::mutex> lock(mutex);
    tls_slot(TLS_CURRENT_PRIORITY) = new_priority;
    return std::exchange(priority, new_priority);
}

SceInt32 ThreadState::change_affinity_mask(SceInt32 new_affinity_mask) {
    const std::lock_guard<std::mutex> lock(mutex);
    tls_slot(TLS_CPU_AFFINITY_MASK) = new_affinity_mask;
    return std::exchange(affinity_mask, new_affinity_mask);
}

void ThreadState::set_suspended(bool suspended) {
    const std::lock_guard<std::mutex> lock(mutex);
    holds.debugger = suspended;
    step_requested = false;
    if (suspended && status == ThreadStatus::running)
        stop(*cpu);
    // The gate checks the holds again, and with them what the status shows
    cv.notify_all();
}

bool ThreadState::step() {
    std::unique_lock<std::mutex> lock(mutex);
    // A step happens now or not at all, and a pause or a ForVM suspend still holds the thread
    if (holds.pause || holds.vm > 0)
        return false;
    holds.debugger = false;
    step_requested = true;
    cv.notify_all();
    // The thread holds for the debugger again after the instruction
    cv.wait(lock, [&] { return (holds.debugger && status == ThreadStatus::suspended) || status == ThreadStatus::dormant || status == ThreadStatus::deleted; });
    return true;
}

void ThreadState::set_paused(bool paused) {
    const std::lock_guard<std::mutex> lock(mutex);
    holds.pause = paused;
    if (paused && status == ThreadStatus::running)
        stop(*cpu);
    cv.notify_all();
}

int ThreadState::vm_suspend() {
    std::unique_lock<std::mutex> lock(mutex);
    if (status == ThreadStatus::dormant)
        return SCE_KERNEL_ERROR_DORMANT;
    const int count = ++holds.vm;
    // A thread a pause already holds at its gate shows as suspended now
    cv.notify_all();
    if (status == ThreadStatus::running)
        stop(*cpu);
    // Like on a Vita, return once the thread has stopped, so its context holds still
    cv.wait(lock, [&] { return status != ThreadStatus::running || holds.vm == 0 || exiting(); });
    return count;
}

// What a ForVM resume returns for a thread no ForVM suspend holds, an error with no public name
constexpr int VM_NOT_SUSPENDED = static_cast<int>(0x80028033);

int ThreadState::vm_resume() {
    const std::lock_guard<std::mutex> lock(mutex);
    // Like on a Vita, a dormant thread fails before the count does
    if (status == ThreadStatus::dormant)
        return SCE_KERNEL_ERROR_DORMANT;
    if (holds.vm == 0)
        return VM_NOT_SUSPENDED;
    // The gate waits for the last resume, and so may a suspend that waits for the thread to stop
    if (--holds.vm == 0)
        cv.notify_all();
    return holds.vm;
}

SceInt32 ThreadState::get_vm_context(CPUContext &context, uint32_t &tpidrurw, bool &in_kernel_call) {
    std::unique_lock<std::mutex> lock(mutex);
    // A thread whose wait timed out while suspended stops on its way back to its code
    cv.wait(lock, [&] { return status != ThreadStatus::running || holds.vm == 0 || exiting(); });
    if (holds.vm == 0)
        return SCE_KERNEL_ERROR_NOT_SUSPENDED;
    context = save_context(*cpu);
    tpidrurw = read_tpidrurw(*cpu);
    // Inside a kernel call means waiting, or held on the way back from one
    in_kernel_call = status == ThreadStatus::waiting || (status == ThreadStatus::suspended && stop_point == StopPoint::syscall);
    return SCE_KERNEL_OK;
}

void ThreadState::wait_until_not_running() {
    std::unique_lock<std::mutex> lock(mutex);
    cv.wait(lock, [&] { return status != ThreadStatus::running; });
}

bool ThreadState::stopped_at_breakpoint() const {
    const std::lock_guard<std::mutex> lock(mutex);
    return holds.debugger && status == ThreadStatus::suspended && hit_breakpoint(*cpu);
}
