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
#include <sstream>
#include <tuple>
#include <utility>

// Set in the wait type of a wait that runs callbacks
constexpr SceUInt32 WAITTYPE_CB_BIT = 0x80000000U;
// What a thread waiting for a thread signal reports as its wait UID on a Vita, which isn't a UID
constexpr SceUID SIGNAL_WAIT_ID = 0x00010221;

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
    , init_cpu_ctx(initial_context(stack.get() + this->stack_size))
    , kernel(kernel)
    , mem(mem)
    , priority(this->init_priority)
    , affinity_mask(affinity_mask) {
    memset(stack.get_ptr<void>().get(mem), 0xcc, this->stack_size);

    const size_t tls_size = KERNEL_TLS_SIZE + kernel.tls_msize;
    const Ptr<uint8_t> base_tls_ptr = tls.get_ptr<uint8_t>();
    memset(base_tls_ptr.get(mem), 0, tls_size);

    int *tls_array = tls.get_ptr<int>().get(mem);

    tls_array[TLS_THREAD_ID] = id;
    tls_array[TLS_SP_TOP] = stack.get() + this->stack_size;
    tls_array[TLS_SP_BOTTOM] = stack.get();
    // A Vita starts every thread with a different random value in each of these. These are the
    // values one thread had on a PS Vita TV, the same for every thread so runs repeat.
    tls_array[TLS_RESERVED_6] = static_cast<int>(0xFEFA7314);
    tls_array[TLS_STACK_CHECK_GUARD] = static_cast<int>(0xC325C2A7);
    tls_array[TLS_CURRENT_PRIORITY] = priority;
    tls_array[TLS_CPU_AFFINITY_MASK] = affinity_mask;

    const Ptr<uint8_t> user_tls_ptr = base_tls_ptr + KERNEL_TLS_SIZE;
    write_tpidruro(*this->cpu, user_tls_ptr.address());
    if (kernel.tls_address) {
        assert(kernel.tls_psize <= kernel.tls_msize);
        memcpy(user_tls_ptr.get(mem), kernel.tls_address.get(mem), kernel.tls_psize);
    }
}

int ThreadState::start(Address entry, SceSize arglen, const Ptr<void> argp, bool run_entry_callback) {
    // Threads waiting for this one to end read its status under the end queue's lock, so hold it
    const std::lock_guard<std::mutex> end_lock(end.mutex);
    std::unique_lock<std::mutex> thread_lock(mutex);
    if (status != ThreadStatus::dormant)
        return SCE_KERNEL_ERROR_NOT_DORMANT;

    run_start_callback = run_entry_callback;
    returned_value = SCE_KERNEL_ERROR_NOT_DORMANT;
    // Each run starts at the priority and on the cores the thread was created with
    set_priority(init_priority);
    set_affinity_mask(init_affinity_mask);
    load_context(*cpu, init_cpu_ctx);
    // A thread starts with TPIDRURW at 0, also when it starts again
    write_tpidrurw(*cpu, 0);
    write_pc(*cpu, entry);
    write_lr(*cpu, kernel.halt_instruction_pc);
    write_reg(*cpu, 0, arglen);

    // Copy data to stack
    if (argp && arglen > 0) {
        const Address data_addr = stack_alloc(*cpu, align(arglen, 8));
        memcpy(Ptr<uint8_t>(data_addr).get(mem), argp.get(mem), arglen);
        write_reg(*cpu, 1, data_addr);
    } else {
        write_reg(*cpu, 1, 0);
    }

    update_status(ThreadStatus::running);

    return SCE_KERNEL_OK;
}

void ThreadState::exit(SceInt32 status) {
    std::lock_guard<std::mutex> guard(mutex);
    request_ending(Ending::exit);
    returned_value = static_cast<uint32_t>(status);
}

void ThreadState::exit_delete(SceInt32 status) {
    std::lock_guard<std::mutex> lock(mutex);
    request_ending(Ending::exit_delete);
    returned_value = static_cast<uint32_t>(status);
}

void ThreadState::remove() {
    const std::lock_guard<std::mutex> lock(mutex);
    request_removal();
}

SceInt32 ThreadState::remove_dormant() {
    const std::lock_guard<std::mutex> lock(mutex);
    if (status != ThreadStatus::dormant)
        return SCE_KERNEL_ERROR_NOT_DORMANT;
    request_removal();
    return SCE_KERNEL_OK;
}

void ThreadState::request_removal() {
    request_ending(Ending::remove);
    // A dormant thread is gone at once
    if (status == ThreadStatus::dormant)
        update_status(ThreadStatus::deleted);

    if (status == ThreadStatus::running)
        stop(*cpu);

    // Wake the thread if it is parked dormant, held at its gate or blocked in a wait
    cv.notify_all();
}

void ThreadState::run_loop(bool top_level) {
    bool guest_returned = false;

    // Set thread-local CPU state so signal handlers can access it. The guard puts back what was
    // there before on any exit, so a nested run_loop for a callback leaves the outer one's state
    // set, and a recycled host thread never sees a stale CPUState pointer.
    struct CpuStateGuard {
        CPUState *const previous = get_current_cpu_state();
        ~CpuStateGuard() { set_current_cpu_state(previous); }
    } cpu_state_guard;
    set_current_cpu_state(cpu.get());

    std::unique_lock<std::mutex> lock(mutex);

    auto run_thread_end_callback = [&]() {
        if (!kernel.thread_event_end)
            return;

        const ThreadStatus old_status = status;
        const uint32_t old_returned_value = returned_value;
        // The handler runs guest code on this thread, which a pending ending would end at once
        const Ending old_ending = std::exchange(ending, Ending::none);
        status = ThreadStatus::running;

        lock.unlock();
        // A thread event handler starts with the VFP state of a new thread, like a callback
        const int ret = run_callback(kernel.thread_event_end.address(), { SCE_KERNEL_THREAD_EVENT_TYPE_END, static_cast<uint32_t>(id), 0, kernel.thread_event_end_arg }, true);
        if (ret != 0)
            LOG_WARN("Thread end event handler returned {}", log_hex(ret));
        lock.lock();

        status = old_status;
        returned_value = old_returned_value;
        // Keeping a stronger request that came while the handler ran, like a teardown's remove
        request_ending(old_ending);
    };

    // A mutex or a write lock the thread still holds when it ends goes to its next waiter, and the
    // event flags it holds are free again
    auto release_locks = [&]() {
        lock.unlock();
        kernel.objects.for_each<HeavyMutex>([&](Mutex &mutex) { mutex.release_ended_owner(*this); });
        kernel.objects.for_each<RWLock>([&](RWLock &rwlock) { rwlock.release_ended_writer(*this); });
        kernel.objects.for_each<EventFlag>([&](EventFlag &flag) { flag.release_ended_holder(*this); });
        lock.lock();
    };

    // The callbacks the thread created end with it, with their pending notifications
    auto delete_callbacks = [&]() {
        lock.unlock();
        for (const auto &weak_cb : callbacks) {
            if (const CallbackPtr cb = weak_cb.lock())
                kernel.delete_callback(cb->uid, id);
        }
        callbacks.clear();
        lock.lock();
    };

    while (true) {
        // This level is done once its guest function returned or the thread is ending
        if (exiting() || guest_returned) {
            // A nested level returns to the one below it
            if (!top_level)
                break;
            // A thread that returns from its entry ends like one that exits, end handler included
            if (guest_returned)
                request_ending(Ending::exit);
            // The top level ends a thread that ran. An exit leaves it dormant, while a delete makes
            // it gone for guest calls before its end waiters wake.
            if (status != ThreadStatus::dormant && status != ThreadStatus::deleted) {
                if (ending != Ending::remove)
                    run_thread_end_callback();
                release_locks();
                delete_callbacks();
                end_for_waiters(lock);
            }
            if (ending != Ending::exit)
                break;
            ending = Ending::none;
            guest_returned = false;
            // Status is now dormant: we'll park until the next start() or a delete
        }

        // Park until we have something to do.
        if (status != ThreadStatus::running) {
            cv.wait(lock, [&] { return status == ThreadStatus::running || exiting(); });
            continue;
        }

        // Fire the thread-start event once per start.
        if (run_start_callback) {
            run_start_callback = false;
            lock.unlock();
            if (kernel.thread_event_start) {
                // A thread event handler starts with the VFP state of a new thread, like a callback
                const int ret = run_callback(kernel.thread_event_start.address(), { SCE_KERNEL_THREAD_EVENT_TYPE_START, static_cast<uint32_t>(id), 0, kernel.thread_event_start_arg }, true);
                if (ret != 0)
                    LOG_WARN("Thread start event handler returned {}", log_hex(ret));
            }
            lock.lock();
        }

        // Active JIT loop. Lock held on entry and exit; unlocked only around run/step. A nested
        // level runs a callback or handler from inside a kernel call.
        StopPoint last_stop = top_level ? StopPoint::guest_code : StopPoint::syscall;
        while (!exiting() && !guest_returned && status == ThreadStatus::running) {
            wait_while_held(lock, last_stop);
            if (exiting())
                break;
            const bool do_step = std::exchange(step_requested, false);

            lock.unlock();

            // Single step or run
            const int res = do_step ? step(*cpu) : run(*cpu);

            // handle svc call if this was what stopped the cpu
            const bool syscall = cpu->svc_called;
            if (syscall) {
                const uint32_t nid = *Ptr<uint32_t>(read_pc(*cpu) + 4).get(mem);
                kernel.call_import(*cpu, nid, id);
                clear_exclusive(*cpu);
            }

            // handle pending abort (exception handler from page fault)
            if (cpu->abort_pending.exchange(false))
                dispatch_abort(*cpu);

            lock.lock();

            last_stop = syscall ? StopPoint::syscall : StopPoint::guest_code;
            // A finished step or a breakpoint holds the thread for the debugger
            if (do_step || hit_breakpoint(*cpu))
                holds.debugger = true;

            // The guest function of this level returned to the halt sentinel
            if (res != 0) {
                returned_value = read_reg(*cpu, 0);
                guest_returned = true;
            }
        }
    }
}

// What a ForVM resume returns for a thread no ForVM suspend holds, an error with no public name
constexpr int VM_NOT_SUSPENDED = static_cast<int>(0x80028033);

void ThreadState::push_arguments(std::span<const uint32_t> args) {
    Address sp = read_sp(*cpu);
    for (size_t i = 0; i < std::min(args.size(), static_cast<size_t>(4)); i++) {
        write_reg(*cpu, i, args[i]);
    }
    if (args.size() > 4) {
        // TODO align to 16 bytes
        const size_t remain_size = args.size() - 4;
        sp -= 4 * remain_size;
        memcpy(Ptr<uint32_t>(sp).get(mem), &args[4], remain_size * 4);
    }
    write_sp(*cpu, sp);
}

uint32_t ThreadState::run_callback(Address callback_address, std::initializer_list<uint32_t> args, bool fresh_vfp_state) {
    std::unique_lock<std::mutex> thread_lock(mutex);

    // save the current context before overwriting PC/LR for the callback
    const CPUContext previous_ctx = save_context(*cpu);
    const uint32_t previous_tpidruro = read_tpidruro(*cpu);
    // A callback sees the thread's TPIDRURW, and the thread gets its own value back after it
    const uint32_t previous_tpidrurw = read_tpidrurw(*cpu);
    const uint32_t previous_returned_value = returned_value;
    int &vfp_exception = tls_slot(TLS_VFP_EXCEPTION);
    const int previous_vfp_exception = vfp_exception;
    if (fresh_vfp_state) {
        write_fpscr(*cpu, INITIAL_FPSCR);
        vfp_exception = 0;
    }

    // we shouldn't have to clean the context I believe
    write_pc(*cpu, callback_address);
    write_lr(*cpu, kernel.halt_instruction_pc);
    push_arguments({ args.begin(), args.size() });
    thread_lock.unlock();

    // unlock but then immediately lock back in the run_loop function
    // shouldn't cause an issue, but maybe we could use a recursive mutex instead
    run_loop(false);

    thread_lock.lock();

    // restore the previous context
    // actually, in most case I don't think this is necessary as the caller
    // and the callee should respect the same calling convention
    // but do it just in case
    load_context(*cpu, previous_ctx);
    write_tpidruro(*cpu, previous_tpidruro);
    write_tpidrurw(*cpu, previous_tpidrurw);
    if (fresh_vfp_state)
        vfp_exception = previous_vfp_exception;

    // The callback's result isn't the thread's exit status, unless the callback ended the thread
    const uint32_t result = returned_value;
    if (!exiting())
        returned_value = previous_returned_value;
    return result;
}

void ThreadState::dispatch_abort(CPUState &cpu) {
    const uint32_t fault_addr = cpu.abort_fault_addr.load();
    // DABT = type 0
    const Address handler = kernel.exception_handlers[0].load();
    if (!handler)
        return;

    // Build KuKernelAbortContext on guest stack for the handler to read.
    // Note: by the time we get here, the page has already been unprotected
    // by the protect_tree mechanism, and the CPU may have executed past
    // the faulting instruction. The handler is called as a notification;
    // we don't restore from AbortContext afterward.
    // { r0-r12, sp, lr, pc, FAR } = 17 uint32_t = 68 bytes
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

    // run_callback saves/restores full CPU context internally
    run_callback(handler, { sp_aligned });
}

uint32_t ThreadState::run_guest_function(Address callback_address, SceSize args, const Ptr<void> argp) {
    start(callback_address, args, argp, false);
    std::unique_lock<std::mutex> lock(mutex);
    cv.wait(lock, [&]() { return status == ThreadStatus::dormant || status == ThreadStatus::deleted; });
    return returned_value;
}

void ThreadState::update_status(ThreadStatus status) {
    this->status = status;
    cv.notify_all();
}

void ThreadState::end_for_waiters(std::unique_lock<std::mutex> &lock) {
    // Lock order: the end queue's lock before the thread lock
    lock.unlock();
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

WaitResult ThreadState::delay_until(Deadline deadline, bool callbacks) {
    // Set when a callback notify ended the last sleep
    bool notified = false;
    WaitOptions options(nullptr, callbacks);
    ThreadWait wait(*this, { .type = SCE_KERNEL_WAITTYPE_DELAY }, options, deadline);
    while (true) {
        // Run pending callbacks before each sleep
        if (const WaitResult ran = wait.run_callbacks(); !ran)
            return ran;
        if (const std::optional<WaitResult> refused = wait.begin()) {
            if (!*refused)
                return *refused;
            // Reaching the deadline is the expected outcome of a delay
            if (**refused == SCE_KERNEL_ERROR_WAIT_TIMEOUT)
                return SCE_KERNEL_OK;
            // A callback became pending after the check above. Return 0, unless a notify ended the
            // last sleep. In that case loop back and run it.
            if (!notified)
                return SCE_KERNEL_OK;
            continue;
        }
        // A delay is in no queue. Only its timeout, a callback notify or the thread exiting can end
        // it, all under the thread lock.
        std::ignore = wait.sleep();
        const WaitResult ended = wait.retire();
        if (!ended)
            return ended;
        if (*ended != SCE_KERNEL_ERROR_NOTIFY_CALLBACK)
            return SCE_KERNEL_OK;
        notified = true;
    }
}

WaitResult ThreadState::wait_for_signal(WaitOptions &options) {
    ThreadWait wait(*this, { SCE_KERNEL_WAITTYPE_SIGNAL, SIGNAL_WAIT_ID }, options, deadline_from(options));
    while (true) {
        // Run pending callbacks before looking for a signal. The deadline is already set, so this
        // time counts against the timeout.
        if (const WaitResult ran = wait.run_callbacks(); !ran)
            return ran;
        if (const std::optional<WaitResult> refused = wait.begin())
            return *refused;
        // A signal wait is in no queue. Only a send, its timeout, a callback notify or the thread
        // exiting can end it, all under the thread lock.
        std::ignore = wait.sleep();
        const WaitResult ended = wait.retire();
        if (!ended || *ended != SCE_KERNEL_ERROR_NOTIFY_CALLBACK)
            return ended;
    }
}

SceInt32 ThreadState::send_signal() {
    const std::lock_guard<std::mutex> lock(mutex);
    // End a signal wait in progress. Otherwise keep the signal pending for the next wait.
    if (still_waits() && active_wait->target.type == SCE_KERNEL_WAITTYPE_SIGNAL) {
        active_wait->end_locked(SCE_KERNEL_OK);
        return SCE_KERNEL_OK;
    }
    if (signal_pending)
        return SCE_KERNEL_ERROR_ALREADY_SENT;
    signal_pending = true;
    return SCE_KERNEL_OK;
}

ThreadStatus ThreadState::get_status() const {
    const std::lock_guard<std::mutex> lock(mutex);
    return status;
}

std::expected<uint32_t, SceInt32> ThreadState::exit_status() const {
    const std::lock_guard<std::mutex> lock(mutex);
    if (status != ThreadStatus::dormant)
        return std::unexpected(SCE_KERNEL_ERROR_NOT_DORMANT);
    // A thread that never started has no exit status yet
    if (returned_value == static_cast<uint32_t>(SCE_KERNEL_ERROR_DORMANT))
        return std::unexpected(SCE_KERNEL_ERROR_DORMANT);
    return returned_value;
}

int ThreadState::stack_free_size() const {
    const std::lock_guard<std::mutex> lock(mutex);
    // A dormant thread's whole stack is free, a started one's is free below its stack pointer
    if (status == ThreadStatus::dormant)
        return stack_size;
    return static_cast<int>(read_sp(*cpu) - stack.get());
}

void ThreadState::begin_host_wait() {
    const std::lock_guard<std::mutex> lock(mutex);
    update_status(ThreadStatus::waiting);
}

void ThreadState::end_host_wait() {
    const std::lock_guard<std::mutex> lock(mutex);
    update_status(ThreadStatus::running);
}

uint64_t ThreadState::next_display_wait(uint64_t next_vblank, int vcount) {
    const std::lock_guard<std::mutex> lock(mutex);
    last_vblank_waited = std::max(next_vblank, last_vblank_waited + vcount);
    return last_vblank_waited;
}

WaitResult ThreadState::wait_for_thread_end(ThreadState &waiter, SceInt32 &exit_status, WaitOptions &options) {
    QueuedThreadWait wait(end, end.waiters, std::unique_lock{ end.mutex }, waiter, { SCE_KERNEL_WAITTYPE_WAITTHEND, id }, ThreadEnd::Request{ .exit_status = exit_status }, options);
    // The end queue is never deleted, and callbacks run only at the top of each pass
    return wait.run(FinalCallbackCheck::none, [&]() -> std::optional<SceInt32> {
        // Briefly takes the ended thread's lock, which is allowed under the end queue's lock
        const std::lock_guard<std::mutex> lock(mutex);
        if (status != ThreadStatus::dormant && status != ThreadStatus::deleted)
            return std::nullopt;
        exit_status = static_cast<SceInt32>(returned_value);
        return SCE_KERNEL_OK;
    });
}

void ThreadState::wake() {
    const std::lock_guard<std::mutex> lock(mutex);
    recheck_pending = true;
    cv.notify_all();
}

std::optional<WaitResult> ThreadWait::begin() {
    const std::lock_guard<std::mutex> lock(thread.mutex);
    return thread.begin_locked(*this);
}

std::optional<WaitResult> ThreadState::begin_locked(ThreadWait &wait) {
    if (exiting())
        return std::unexpected{ ThreadExiting{} };
    // Take a signal sent earlier without waiting
    const SceUInt32 type = wait.target.type;
    if (type == SCE_KERNEL_WAITTYPE_SIGNAL && std::exchange(signal_pending, false))
        return SCE_KERNEL_OK;
    // Check the deadline first, so a timeout wins over a pending callback
    if (std::chrono::steady_clock::now() >= wait.deadline) {
        WaitOptions &options = wait.options;
        options.left = 0;
        if (options.timeout && options.timeout_in_place)
            *options.timeout = 0;
        return SCE_KERNEL_ERROR_WAIT_TIMEOUT;
    }
    // A queued wait never begins with a callback pending, so a callback notify that finds one
    // pending later was made during the wait. Signal and multiple event waits sleep through it.
    if (type != SCE_KERNEL_WAITTYPE_SIGNAL && type != SCE_KERNEL_WAITTYPE_MP_EVENTS && callback_ends(wait))
        return SCE_KERNEL_ERROR_NOTIFY_CALLBACK;
    // Each pass starts with no result
    wait.result.reset();
    wait.priority = priority;
    active_wait = &wait;
    update_status(ThreadStatus::waiting);
    return std::nullopt;
}

void ThreadWait::end(WaitResult ended) {
    const std::lock_guard<std::mutex> lock(thread.mutex);
    end_locked(ended);
}

void ThreadWait::end_locked(WaitResult ended) {
    result = ended;
    // The time left is taken when the wait ends, also for an exit. A message pipe writes it at once,
    // while the waker still holds the queue's lock.
    if (options.timeout) {
        options.left = time_left_until(deadline);
        if (options.timeout_in_place)
            *options.timeout = options.left;
    }
    thread.cv.notify_all();
}

std::optional<WaitResult> ThreadWait::sleep() {
    while (true) {
        // Check the wait first, so timer events fire only while the wait is running and the thread
        // is not exiting. The look also takes any recheck wake() asked for. The waiter ends the wait
        // itself in the same hold that looked, so a party that ended it first wins.
        {
            const std::lock_guard<std::mutex> lock(thread.mutex);
            thread.recheck_pending = false;
            if (result)
                return std::nullopt;
            if (thread.exiting()) {
                end_locked(std::unexpected{ ThreadExiting{} });
                return result;
            }
            // A queued wait leaves its queue itself for a notified callback, under its object lock
            if (queued && thread.callback_ends(*this)) {
                end_locked(SCE_KERNEL_ERROR_NOTIFY_CALLBACK);
                return result;
            }
        }
        const Deadline now = std::chrono::steady_clock::now();
        // Fire the timer events due by now or the deadline, under the same object lock hold as the
        // timeout check below
        const Deadline look = watch(std::min(now, deadline));
        if (now >= deadline) {
            const std::lock_guard<std::mutex> lock(thread.mutex);
            // A timer event due by the deadline may have ended the wait in watch()
            if (result)
                return std::nullopt;
            end_locked(SCE_KERNEL_ERROR_WAIT_TIMEOUT);
            return result;
        }
        // Sleep until the next clock check, the deadline or a wake(). The next pass checks the wait
        // again.
        if (queued)
            object_lock.unlock();
        {
            std::unique_lock<std::mutex> lock(thread.mutex);
            const auto over = [&] { return thread.exiting() || result || thread.recheck_pending || (queued && thread.callback_ends(*this)); };
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
    // Report ThreadExiting if the thread is exiting, whatever ended the wait
    if (thread.exiting())
        return std::unexpected{ ThreadExiting{} };
    return *result;
}

WaitResult ThreadWait::run_callbacks() {
    {
        const std::lock_guard<std::mutex> lock(thread.mutex);
        if (!thread.callback_ends(*this))
            return SCE_KERNEL_OK;
    }
    if (queued)
        object_lock.unlock();
    thread.process_callbacks();
    // The thread exited or was deleted during the callbacks, so the wait must not complete. Else the
    // callbacks may have changed what the caller waits for, so it rechecks before waiting again.
    // Look before taking the object lock back.
    bool exiting;
    {
        const std::lock_guard<std::mutex> lock(thread.mutex);
        exiting = thread.exiting();
    }
    if (queued)
        object_lock.lock();
    if (exiting)
        return std::unexpected{ ThreadExiting{} };
    return SCE_KERNEL_OK;
}

SceUInt32 ThreadState::process_callbacks() {
    {
        // Set under the thread lock because notify_callback() reads the flag too
        const std::lock_guard<std::mutex> lock(mutex);
        if (is_processing_callbacks)
            return 0;
        is_processing_callbacks = true;
        // A notify from here on is for a callback this pass may already have passed, so it stays
        // recorded for the next check
        callbacks_pending = false;
    }
    SceUInt32 processed = 0;
    for (auto it = callbacks.begin(); it != callbacks.end();) {
        const CallbackPtr cb = it->lock();
        // Deleted since it was added
        if (!cb) {
            it = callbacks.erase(it);
            continue;
        }
        ++it;
        const std::optional<Callback::Notification> notification = cb->take_notification();
        if (!notification)
            continue;
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
            kernel.delete_callback(cb->uid, id);
    }
    const std::lock_guard<std::mutex> lock(mutex);
    is_processing_callbacks = false;
    return processed;
}

void ThreadState::notify_callback() {
    const std::lock_guard<std::mutex> lock(mutex);
    callbacks_pending = true;
    if (!still_waits() || !callback_ends(*active_wait))
        return;
    // A wait in no queue ends here, in the same step that counts the notify. A queued wait can only
    // end under its queue's lock, which goes before the thread lock, so the waiter wakes and leaves
    // its queue itself.
    if (!active_wait->queued)
        active_wait->end_locked(SCE_KERNEL_ERROR_NOTIFY_CALLBACK);
    else
        cv.notify_all();
}

void ThreadState::wait_for_departure() {
    std::unique_lock<std::mutex> lock(mutex);
    // A queued wait that ended is dropped only after it served the waiters behind it, so wait for
    // that. A wait in no queue that began after the notify sleeps through the pending callback.
    cv.wait(lock, [&] { return !active_wait || !active_wait->queued || (!active_wait->result && !callback_ends(*active_wait)); });
}

void ThreadState::add_callback(const CallbackPtr &cb) {
    callbacks.push_back(cb);
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
    // Vita, where the waker ends the wait. A host wait, like an audio output, has no target and
    // stays WAITING.
    const WaitTarget target = active_wait ? active_wait->target : WaitTarget{};
    const bool callbacks = active_wait && active_wait->callbacks;
    const bool over = (active_wait && active_wait->result) || exiting();
    const bool ready = status == ThreadStatus::waiting && target.type != 0 && over;
    const WaitTarget reported = ready ? WaitTarget{} : target;
    info.status = ready ? SCE_KERNEL_THREAD_STATUS_READY : std::to_underlying(status);
    info.waitType = callbacks && !ready ? reported.type | WAITTYPE_CB_BIT : reported.type;
    info.waitId = reported.id;
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
    const int old_priority = priority;
    set_priority(new_priority);
    return old_priority;
}

void ThreadState::set_priority(int new_priority) {
    priority = new_priority;
    tls_slot(TLS_CURRENT_PRIORITY) = new_priority;
}

SceInt32 ThreadState::get_affinity_mask() const {
    const std::lock_guard<std::mutex> lock(mutex);
    return affinity_mask;
}

SceInt32 ThreadState::change_affinity_mask(SceInt32 new_affinity_mask) {
    const std::lock_guard<std::mutex> lock(mutex);
    const SceInt32 old_affinity_mask = affinity_mask;
    set_affinity_mask(new_affinity_mask);
    return old_affinity_mask;
}

void ThreadState::set_affinity_mask(SceInt32 new_affinity_mask) {
    affinity_mask = new_affinity_mask;
    tls_slot(TLS_CPU_AFFINITY_MASK) = new_affinity_mask;
}

void ThreadState::wait_while_held(std::unique_lock<std::mutex> &lock, StopPoint where) {
    if (!holds.any() || exiting())
        return;
    stop_point = where;
    // The holds can change while the thread waits, and with them what its status shows
    while (holds.any() && !exiting()) {
        const ThreadStatus shown = holds.shown() ? ThreadStatus::suspended : ThreadStatus::running;
        if (status != shown)
            update_status(shown);
        cv.wait(lock);
    }
    if (status != ThreadStatus::running)
        update_status(ThreadStatus::running);
}

void ThreadState::suspend() {
    const std::lock_guard<std::mutex> lock(mutex);
    holds.debugger = true;
    if (status == ThreadStatus::running)
        stop(*cpu);
    // A thread a pause already holds at its gate shows as suspended now
    cv.notify_all();
}

bool ThreadState::resume(bool step) {
    const std::lock_guard<std::mutex> lock(mutex);
    // A step happens now or not at all
    if (step && (holds.pause || holds.vm > 0))
        return false;
    holds.debugger = false;
    step_requested = step;
    cv.notify_all();
    return !holds.any();
}

void ThreadState::pause() {
    const std::lock_guard<std::mutex> lock(mutex);
    holds.pause = true;
    if (status == ThreadStatus::running)
        stop(*cpu);
}

void ThreadState::resume_from_pause() {
    const std::lock_guard<std::mutex> lock(mutex);
    holds.pause = false;
    cv.notify_all();
}

int ThreadState::vm_suspend() {
    std::unique_lock<std::mutex> lock(mutex);
    if (status == ThreadStatus::dormant)
        return SCE_KERNEL_ERROR_DORMANT;
    const int count = ++holds.vm;
    // A thread a pause already holds at its gate shows as suspended now
    cv.notify_all();
    if (status == ThreadStatus::running) {
        stop(*cpu);
        // Like on a Vita, return once the thread has stopped, so its context holds still
        wait_for_vm_stop(lock);
    }
    return count;
}

void ThreadState::wait_for_vm_stop(std::unique_lock<std::mutex> &lock) {
    cv.wait(lock, [&] { return status != ThreadStatus::running || !vm_suspended() || exiting(); });
}

SceInt32 ThreadState::get_vm_context(CPUContext &context, uint32_t &tpidrurw, bool &in_kernel_call) {
    std::unique_lock<std::mutex> lock(mutex);
    // A thread whose wait timed out while suspended stops on its way back to its code
    wait_for_vm_stop(lock);
    if (!vm_suspended())
        return SCE_KERNEL_ERROR_NOT_SUSPENDED;
    context = save_context(*cpu);
    tpidrurw = read_tpidrurw(*cpu);
    in_kernel_call = in_wait();
    return SCE_KERNEL_OK;
}

void ThreadState::wait_until_stopped_for_debugger() {
    std::unique_lock<std::mutex> lock(mutex);
    cv.wait(lock, [&] { return stopped_for_debugger() || status == ThreadStatus::dormant || status == ThreadStatus::deleted; });
}

void ThreadState::wait_until_not_running() {
    std::unique_lock<std::mutex> lock(mutex);
    cv.wait(lock, [&] { return status != ThreadStatus::running; });
}

bool ThreadState::stopped_at_breakpoint() const {
    const std::lock_guard<std::mutex> lock(mutex);
    return stopped_for_debugger() && hit_breakpoint(*cpu);
}

int ThreadState::vm_resume() {
    const std::lock_guard<std::mutex> lock(mutex);
    // Like on a Vita, a dormant thread fails before the count does
    if (status == ThreadStatus::dormant)
        return SCE_KERNEL_ERROR_DORMANT;
    if (holds.vm == 0)
        return VM_NOT_SUSPENDED;
    if (--holds.vm > 0)
        return holds.vm;
    // The gate waits for this, and so may a suspend that waits for the thread to stop
    cv.notify_all();
    return 0;
}
