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

#include <cpu/state.h>
#include <kernel/thread/guest_thread.h>

#include <kernel/state.h>
#include <kernel/sync_primitives.h>
#include <mem/ptr.h>
#include <util/align.h>

#include <util/log.h>

#include <algorithm>
#include <bit>
#include <cassert>
#include <cstring>
#include <memory>
#include <utility>

// The priority a thread created with priority runs at. The default range stands for the game's own.
static int created_priority(int priority) {
    if (priority > SCE_KERNEL_LOWEST_PRIORITY_USER) {
        assert(SCE_KERNEL_HIGHEST_DEFAULT_PRIORITY <= priority && priority <= SCE_KERNEL_LOWEST_DEFAULT_PRIORITY);
        return priority - SCE_KERNEL_DEFAULT_PRIORITY + SCE_KERNEL_GAME_DEFAULT_PRIORITY_ACTUAL;
    }
    return priority;
}

// The registers each run of a thread starts with
static CPUContext initial_context(Address stack_top) {
    CPUContext ctx;
    // A Vita starts every thread with these in r2 to r12, and some games rely on r9 not being 0
    std::fill(ctx.cpu_registers.begin() + 2, ctx.cpu_registers.begin() + 13, 0xDEADBEEF);
    ctx.set_sp(stack_top);
    ctx.fpscr = SCE_KERNEL_THREAD_INITIAL_FPSCR;
    return ctx;
}

ThreadState::ThreadState(SceUID id, MemState &mem, const char *name, Address entry_point, int init_priority, SceInt32 affinity_mask, int stack_size, SceUInt32 attr, SceSize user_tls_size)
    : id(id)
    , name(name)
    , entry_point(entry_point)
    , attr(attr | SCE_KERNEL_THREAD_ATTR_USER)
    , stack_size(align(stack_size, KiB(4)))
    , stack(alloc_block(mem, this->stack_size, fmt::format("Stack for thread {} (#{})", name, id).c_str()))
    , tls(alloc_block(mem, SCE_KERNEL_TLS_SIZE + user_tls_size, fmt::format("TLS for thread {} (#{})", name, id).c_str()))
    , init_priority(created_priority(init_priority))
    , init_affinity_mask(affinity_mask)
    , mem(mem)
    , priority(this->init_priority)
    , affinity_mask(affinity_mask) {
    memset(stack.get_ptr<void>().get(mem), 0xcc, this->stack_size);
    memset(tls.get_ptr<void>().get(mem), 0, SCE_KERNEL_TLS_SIZE + user_tls_size);

    tls_slot(TLS_THREAD_ID) = id;
    tls_slot(TLS_SP_TOP) = stack.get() + this->stack_size;
    tls_slot(TLS_SP_BOTTOM) = stack.get();
    // A Vita starts every thread with a different random value in each of these. These are the
    // values one thread had on a PS Vita TV, the same for every thread so runs repeat.
    tls_slot(TLS_RESERVED_6) = static_cast<int>(0xFEFA7314);
    tls_slot(TLS_STACK_CHECK_GUARD) = static_cast<int>(0xC325C2A7);
    tls_slot(TLS_CURRENT_PRIORITY) = priority;
    tls_slot(TLS_CPU_AFFINITY_MASK) = affinity_mask;
}

GuestThread::GuestThread(SceUID id, KernelState &kernel, MemState &mem, CPUStatePtr cpu, const char *name, Address entry_point, int init_priority, SceInt32 affinity_mask, int stack_size, SceUInt32 attr)
    : state(id, mem, name, entry_point, init_priority, affinity_mask, stack_size, attr, kernel.tls_msize)
    , cpu(std::move(cpu))
    , kernel(kernel) {
    const Ptr<uint8_t> user_tls_ptr = state.tls.get_ptr<uint8_t>() + SCE_KERNEL_TLS_SIZE;
    this->cpu->set_tpidruro(user_tls_ptr.address());
    if (kernel.tls_address) {
        assert(kernel.tls_psize <= kernel.tls_msize);
        memcpy(user_tls_ptr.get(mem), kernel.tls_address.get(mem), kernel.tls_psize);
    }
}

int GuestThread::start(SceSize arglen, const Ptr<void> argp, bool run_entry_callback) {
    const std::lock_guard<std::mutex> thread_lock(mutex);
    if (state.status != ThreadStatus::dormant)
        return SCE_KERNEL_ERROR_NOT_DORMANT;
    state.run_start_callback = run_entry_callback;
    state.returned_value = SCE_KERNEL_ERROR_NOT_DORMANT;
    // Each run starts at the priority and on the cores the thread was created with, and its TLS
    // shows them
    state.priority = state.init_priority;
    state.affinity_mask = state.init_affinity_mask;
    state.tls_slot(TLS_CURRENT_PRIORITY) = state.priority;
    state.tls_slot(TLS_CPU_AFFINITY_MASK) = state.affinity_mask;
    cpu->load_context(initial_context(state.stack.get() + state.stack_size));
    // A thread starts with TPIDRURW at 0, also when it starts again
    cpu->set_tpidrurw(0);
    // The entry gets the size of the argument data and its copy on the stack, or null for none
    Address data_addr = 0;
    if (argp && arglen > 0) {
        data_addr = stack_alloc(align(arglen, 8));
        memcpy(Ptr<uint8_t>(data_addr).get(state.mem), argp.get(state.mem), arglen);
    }
    place_call(state.entry_point, { arglen, data_addr });

    state.status = ThreadStatus::running;
    cv.notify_all();
    return SCE_KERNEL_OK;
}

void GuestThread::exit(SceInt32 status) {
    const std::lock_guard<std::mutex> lock(mutex);
    request_ending(Ending::exit);
    state.returned_value = static_cast<uint32_t>(status);
}

void GuestThread::exit_delete(SceInt32 status) {
    const std::lock_guard<std::mutex> lock(mutex);
    request_ending(Ending::exit_delete);
    state.returned_value = static_cast<uint32_t>(status);
}

void GuestThread::remove() {
    const std::lock_guard<std::mutex> lock(mutex);
    request_ending(Ending::remove);
    // A dormant thread is gone at once
    if (state.status == ThreadStatus::dormant)
        end_as(ThreadStatus::deleted);
    else if (state.status == ThreadStatus::running)
        cpu->request_stop();

    // Wake the thread if it is parked dormant, held before its guest code or blocked in a wait
    cv.notify_all();
}

SceInt32 GuestThread::remove_dormant() {
    const std::lock_guard<std::mutex> lock(mutex);
    if (state.status != ThreadStatus::dormant)
        return SCE_KERNEL_ERROR_NOT_DORMANT;
    // A dormant thread is gone at once, and the status change wakes its parked host thread to end
    request_ending(Ending::remove);
    end_as(ThreadStatus::deleted);
    return SCE_KERNEL_OK;
}

namespace {
thread_local GuestThread *running_thread = nullptr;

// Makes thread the current guest thread of the host thread until the end of the scope
struct CurrentThread {
    explicit CurrentThread(GuestThread &thread) { running_thread = &thread; }
    ~CurrentThread() { running_thread = nullptr; }
};
} // namespace

GuestThread *current_thread() {
    return running_thread;
}

void GuestThread::run_loop() {
    // For all of run_loop, also while the thread ends in finish_run
    const CurrentThread current(*this);

    std::unique_lock<std::mutex> lock(mutex);
    do {
        // A thread that returns from its entry ends like one that exits, end handler included
        if (const std::optional<uint32_t> returned = run_guest(lock, std::nullopt)) {
            state.returned_value = *returned;
            request_ending(Ending::exit);
        }
    } while (finish_run(lock));
}

bool GuestThread::finish_run(std::unique_lock<std::mutex> &lock) {
    // The thread is ending. One that ran ends here: an exit leaves it dormant, while a delete
    // makes it gone for guest calls before its end waiters wake.
    if (state.status != ThreadStatus::dormant && state.status != ThreadStatus::deleted) {
        if (state.ending != Ending::remove && kernel.thread_event_end) {
            // The thread is RUNNING here, since wait() puts it back to RUNNING. The handler runs
            // guest code on it, which a pending ending would end at once, and an exit in the
            // handler must not change the exit status.
            const uint32_t old_returned_value = state.returned_value;
            const Ending old_ending = state.ending;
            state.ending = Ending::none;
            lock.unlock();
            // A thread event handler starts with the VFP state of a new thread, like a callback
            const int ret = call_guest(kernel.thread_event_end.address(), { SCE_KERNEL_THREAD_EVENT_TYPE_END, static_cast<uint32_t>(state.id), 0, kernel.thread_event_end_arg }, true);
            if (ret != 0)
                LOG_WARN("Thread end event handler returned {}", log_hex(ret));
            lock.lock();
            state.returned_value = old_returned_value;
            // Keep a stronger request made while the handler ran, like a teardown's remove
            request_ending(old_ending);
        }

        lock.unlock();
        // A mutex or a write lock the thread still holds goes to its next waiter, and the
        // event flags it holds are free again
        kernel.objects.for_each<HeavyMutex>([&](Mutex &mutex) { mutex.release_ended_owner(state); });
        kernel.objects.for_each<RWLock>([&](RWLock &rwlock) { rwlock.release_ended_writer(state); });
        kernel.objects.for_each<EventFlag>([&](EventFlag &flag) { flag.release_ended_holder(state); });
        // The callbacks the thread created end with it, with their pending notifications
        for (const auto &weak_cb : state.callbacks) {
            if (const CallbackPtr cb = weak_cb.lock())
                kernel.delete_callback(cb->uid, state.id);
        }
        state.callbacks.clear();
        lock.lock();
        end_as(state.ending == Ending::exit ? ThreadStatus::dormant : ThreadStatus::deleted);
    }
    if (state.ending != Ending::exit)
        return false;
    // The thread is dormant now, and the next run_guest parks it until a start or a delete
    state.ending = Ending::none;
    return true;
}

std::optional<uint32_t> GuestThread::run_guest(std::unique_lock<std::mutex> &lock, std::optional<GuestCall> call) {
    while (true) {
        // Park while the thread is dormant between starts
        cv.wait(lock, [&] { return state.status == ThreadStatus::running || state.exiting(); });
        if (state.exiting())
            return std::nullopt;

        // The thread start event runs once per start, with the VFP state of a new thread
        if (std::exchange(state.run_start_callback, false) && kernel.thread_event_start) {
            lock.unlock();
            const int ret = call_guest(kernel.thread_event_start.address(), { SCE_KERNEL_THREAD_EVENT_TYPE_START, static_cast<uint32_t>(state.id), 0, kernel.thread_event_start_arg }, true);
            if (ret != 0)
                LOG_WARN("Thread start event handler returned {}", log_hex(ret));
            lock.lock();
        }

        // A call's registers are set up after the holds, so a hold inside a kernel call shows that call's registers
        stop_for_holds(lock, call.has_value());
        if (state.exiting())
            return std::nullopt;
        if (call) {
            set_up_call(*std::exchange(call, std::nullopt));
            // The next pass stops for the holds again, at the called function's first instruction
            continue;
        }
        const bool step = step_requested;

        running_guest = true;
        lock.unlock();
        const GuestExit exit = cpu->run(step ? CPUState::RunMode::single_step : CPUState::RunMode::run);
        lock.lock();
        // A ForVM suspend waits for this
        running_guest = false;
        cv.notify_all();
        if (step) {
            // A stop request made before the step ends it before its instruction, so the step goes
            // again after the holds
            if (exit.kind == GuestExit::Kind::stopped && !exit.debugger_stop)
                continue;
            // The step is taken, so a call that a stepped import makes doesn't step too
            step_requested = false;
        } else if (exit.debugger_stop) {
            // A breakpoint holds the thread, also before a pending abort handler
            debugger_hold = true;
        }
        lock.unlock();

        // In this order, also when the import ended the thread: the import, the holds at its end,
        // its result, the exclusive monitor, then the abort handler
        if (exit.kind == GuestExit::Kind::import) {
            const CallResult result = kernel.call_import(*cpu, exit.value, state.id);
            // A hold here shows the registers before the call result is written
            lock.lock();
            stop_for_holds(lock, true);
            lock.unlock();
            write_call_result(result);
            cpu->clear_exclusive();
        }
        // A host fault on protected guest memory, in guest code or in the import's host code
        if (const std::optional<Address> fault = cpu->take_abort())
            dispatch_abort(*fault);
        lock.lock();
        if (exit.kind == GuestExit::Kind::returned)
            return exit.value;
    }
}

uint32_t GuestThread::call_guest(Address fn, std::initializer_list<uint32_t> args, bool fresh_vfp_state) {
    // Only the thread's own host thread runs its guest code
    assert(current_thread() == this);
    std::unique_lock<std::mutex> thread_lock(mutex);

    // Save the thread's context before the call overwrites PC and LR
    const CPUContext previous_ctx = cpu->save_context();
    const uint32_t previous_tpidruro = cpu->get_tpidruro();
    // A callback sees the thread's TPIDRURW, and the thread gets its own value back after it
    const uint32_t previous_tpidrurw = cpu->get_tpidrurw();
    int &vfp_exception = state.tls_slot(TLS_VFP_EXCEPTION);
    const int previous_vfp_exception = vfp_exception;

    const std::optional<uint32_t> returned = run_guest(thread_lock, GuestCall{ fn, args, fresh_vfp_state });

    // The caller and the callback should keep to the calling convention, but put everything back
    cpu->load_context(previous_ctx);
    cpu->set_tpidruro(previous_tpidruro);
    cpu->set_tpidrurw(previous_tpidrurw);
    if (fresh_vfp_state)
        vfp_exception = previous_vfp_exception;

    // A call cut short by the thread's end returns the exit status
    return returned.value_or(state.returned_value);
}

void GuestThread::set_up_call(const GuestCall &call) {
    if (call.fresh_vfp_state) {
        cpu->set_fpscr(SCE_KERNEL_THREAD_INITIAL_FPSCR);
        state.tls_slot(TLS_VFP_EXCEPTION) = 0;
    }
    place_call(call.fn, call.args);
}

void GuestThread::place_call(Address fn, std::initializer_list<uint32_t> args) {
    cpu->set_pc(fn);
    // The call returns to the halt instruction, in Thumb mode
    cpu->set_lr(kernel.halt_instruction.get() | 1);
    Address sp = cpu->get_sp();
    for (size_t i = 0; i < std::min<size_t>(args.size(), 4); i++)
        cpu->set_reg(i, args.begin()[i]);
    if (args.size() > 4) {
        // TODO align to 16 bytes
        sp -= 4 * (args.size() - 4);
        std::ranges::copy(args.begin() + 4, args.end(), Ptr<uint32_t>(sp).get(state.mem));
    }
    cpu->set_sp(sp);
}

void GuestThread::write_call_result(const CallResult &result) {
    if (result.in_s0) {
        cpu->set_float_reg(0, std::bit_cast<float>(result.regs[0]));
        return;
    }
    for (uint8_t i = 0; i < result.count; i++)
        cpu->set_reg(i, result.regs[i]);
}

uint32_t GuestThread::stack_alloc(size_t size) {
    const uint32_t new_sp = cpu->get_sp() - size;
    cpu->set_sp(new_sp);
    return new_sp;
}

void GuestThread::stack_free(size_t size) {
    cpu->set_sp(cpu->get_sp() + size);
}

void GuestThread::dispatch_abort(Address fault_addr) {
    CPUState &cpu = *this->cpu;
    // DABT = type 0
    const Address handler = kernel.exception_handlers[0].load();
    if (!handler)
        return;

    // The handler reads a KuKernelAbortContext on the stack: r0 to r12, sp, lr, pc and the fault
    // address. protect_tree already unprotected the page and the CPU may have run past the faulting
    // instruction, so the call is only a notification and nothing is restored from the context.
    const uint32_t ctx_size = 17 * 4;
    const uint32_t sp_orig = cpu.get_sp();
    const uint32_t sp_aligned = align_down(sp_orig - ctx_size, 8);

    auto *ctx = Ptr<uint32_t>(sp_aligned).get(cpu.mem);
    for (int i = 0; i < 13; i++)
        ctx[i] = cpu.get_reg(i);
    ctx[13] = sp_aligned + ctx_size;
    ctx[14] = cpu.get_lr();
    ctx[15] = cpu.get_pc();
    ctx[16] = fault_addr;

    LOG_DEBUG("DABT handler=0x{:08X} FAR=0x{:08X} PC=0x{:08X} SP=0x{:08X} sp_aligned=0x{:08X}",
        handler, fault_addr, ctx[15], sp_orig, sp_aligned);

    call_guest(handler, { sp_aligned });
}

uint32_t GuestThread::start_and_wait(SceSize args, const Ptr<void> argp) {
    start(args, argp);
    // A guest thread that waits here, in a module start or stop, counts as stopped until the module
    // thread ends. The boot and host threads run no guest thread.
    GuestThread *const caller = current_thread();
    std::optional<HostWait> waiting;
    if (caller)
        waiting.emplace(*caller, std::nullopt);
    // Declared after waiting, so it unlocks before ~HostWait takes the caller's mutex
    std::unique_lock<std::mutex> lock(mutex);
    cv.wait(lock, [&] { return state.status == ThreadStatus::dormant || state.status == ThreadStatus::deleted; });
    return state.returned_value;
}

void GuestThread::end_as(ThreadStatus status) {
    state.status = status;
    cv.notify_all();
    const std::lock_guard<std::mutex> end_lock(end_waiters_mutex);
    state.end_waiters.wake_if([&](auto &waiter) {
        if (waiter.entry.exit_status)
            *waiter.entry.exit_status = static_cast<SceInt32>(state.returned_value);
        return true;
    });
}

WaitResult GuestThread::delay_until(Deadline deadline, bool callbacks) {
    while (true) {
        WaitResult r = wait({ .type = SCE_KERNEL_WAITTYPE_DELAY, .callbacks = callbacks }, deadline);
        if (r && *r == SCE_KERNEL_ERROR_NOTIFY_CALLBACK)
            r = run_callbacks();
        if (!r)
            return r;
        // Reaching the deadline is the expected outcome of a delay
        if (*r == SCE_KERNEL_ERROR_WAIT_TIMEOUT)
            return SCE_KERNEL_OK;
    }
}

WaitResult GuestThread::wait_for_signal(SceUInt32 *timeout, bool callbacks) {
    const auto take_signal = [this] {
        const std::lock_guard<std::mutex> lock(mutex);
        return std::exchange(state.signal_pending, false);
    };
    // A signal sent before ends the wait at once, with the timeout untouched
    if (take_signal())
        return SCE_KERNEL_OK;
    const Deadline deadline = deadline_from(timeout);
    WaitResult r = SCE_KERNEL_OK;
    while (!take_signal()) {
        r = wait({ SCE_KERNEL_WAITTYPE_SIGNAL, SCE_KERNEL_SIGNAL_WAIT_ID, callbacks }, deadline);
        if (r && *r == SCE_KERNEL_ERROR_NOTIFY_CALLBACK)
            r = run_callbacks();
        if (!r || *r != SCE_KERNEL_OK)
            break;
    }
    writeback_timeout(timeout, deadline);
    return r;
}

SceInt32 GuestThread::send_signal() {
    {
        const std::lock_guard<std::mutex> lock(mutex);
        if (state.signal_pending)
            return SCE_KERNEL_ERROR_ALREADY_SENT;
        state.signal_pending = true;
    }
    wake();
    return SCE_KERNEL_OK;
}

ThreadStatus GuestThread::get_status() const {
    const std::lock_guard<std::mutex> lock(mutex);
    return shown_status();
}

ThreadStatus GuestThread::shown_status() const {
    // A ForVM suspend shows once the thread has stopped for it, and the host's holds never show
    if (state.status == ThreadStatus::running && state.vm_suspend_count > 0 && (parked || held_in_call))
        return ThreadStatus::suspended;
    return state.status;
}

bool GuestThread::held() const {
    // A step lets the debugger's hold through until the thread's next stop
    return !state.exiting() && ((debugger_hold && !step_requested) || state.vm_suspend_count > 0 || kernel.is_threads_paused());
}

void GuestThread::stop_for_holds(std::unique_lock<std::mutex> &lock, bool in_call) {
    if (!held())
        return;
    bool &stopped = in_call ? held_in_call : parked;
    stopped = true;
    cv.notify_all();
    // The holds can change while the thread waits
    cv.wait(lock, [&] { return !held(); });
    stopped = false;
    // A step from inside a call runs no instruction, it ends at the next stop
    if (in_call)
        step_requested = false;
}

int GuestThread::stack_free_size() const {
    const std::lock_guard<std::mutex> lock(mutex);
    // A dormant thread's whole stack is free, a started one's is free below its stack pointer
    if (state.status == ThreadStatus::dormant)
        return state.stack_size;
    return static_cast<int>(cpu->get_sp() - state.stack.get());
}

GuestThread::HostWait::HostWait(GuestThread &thread, std::optional<WaitTarget> shown)
    : thread(thread) {
    // Only the thread's own host thread blocks it in a host wait
    assert(current_thread() == &thread);
    const std::lock_guard<std::mutex> lock(thread.mutex);
    thread.host_waiting = true;
    if (shown) {
        thread.state.wait_target = *shown;
        thread.state.status = ThreadStatus::waiting;
    }
    thread.cv.notify_all();
}

GuestThread::HostWait::~HostWait() {
    std::unique_lock<std::mutex> lock(thread.mutex);
    // The rest of the call runs once the debugger lets go
    thread.cv.wait(lock, [&] { return !thread.debugger_freezes(); });
    thread.host_waiting = false;
    if (thread.state.status == ThreadStatus::waiting) {
        thread.state.wait_target = {};
        thread.state.status = ThreadStatus::running;
        thread.cv.notify_all();
    }
}

uint64_t GuestThread::next_display_wait(uint64_t next_vblank, int vcount) {
    const std::lock_guard<std::mutex> lock(mutex);
    state.last_vblank_waited = std::max(next_vblank, state.last_vblank_waited + vcount);
    return state.last_vblank_waited;
}

WaitResult GuestThread::wait_for_thread_end(const GuestThreadPtr &waiter, SceInt32 *exit_status, Deadline deadline, bool callbacks) {
    std::unique_lock<std::mutex> lock(mutex);
    if (state.status == ThreadStatus::dormant || state.status == ThreadStatus::deleted) {
        if (exit_status)
            *exit_status = static_cast<SceInt32>(state.returned_value);
        return SCE_KERNEL_OK;
    }
    std::unique_lock<std::mutex> end_lock(end_waiters_mutex);
    lock.unlock();
    return state.end_waiters.wait(end_lock, waiter, { SCE_KERNEL_WAITTYPE_WAITTHEND, state.id, callbacks }, { exit_status }, deadline);
}

WaitResult GuestThread::wait(WaitTarget target, Deadline deadline) {
    std::unique_lock<std::mutex> lock(mutex);
    state.wait_target = target;
    state.status = ThreadStatus::waiting;
    cv.notify_all();
    // Ends at a wake, a callback, the thread's end or the deadline, but not while the debugger holds it
    while (debugger_freezes() || (!state.wait_over() && std::chrono::steady_clock::now() < deadline)) {
        if (debugger_freezes() || deadline == Deadline::max())
            cv.wait(lock);
        else
            cv.wait_until(lock, deadline);
    }
    const bool satisfied = state.wait_over();
    // Before the call writes its result to the registers. A thread something holds goes on to its
    // holds, which keep it from its callbacks and its code.
    state.status = ThreadStatus::running;
    cv.notify_all();
    const bool callbacks_due = state.callback_ends_wait();
    state.wait_target = {};
    if (state.exiting())
        return std::unexpected{ ThreadExiting{} };
    state.wake_pending = false;
    if (callbacks_due)
        return SCE_KERNEL_ERROR_NOTIFY_CALLBACK;
    return satisfied ? SCE_KERNEL_OK : SCE_KERNEL_ERROR_WAIT_TIMEOUT;
}

WaitResult GuestThread::run_callbacks() {
    process_callbacks();
    const std::lock_guard<std::mutex> lock(mutex);
    // The thread exited or was deleted during the callbacks, so the wait must not complete
    if (state.exiting())
        return std::unexpected{ ThreadExiting{} };
    // The callbacks may have changed what the caller waits for, so it rechecks before waiting again
    return SCE_KERNEL_OK;
}

void GuestThread::wake() {
    const std::lock_guard<std::mutex> lock(mutex);
    state.wake_pending = true;
    cv.notify_all();
}

bool GuestThread::has_pending_callbacks() {
    const std::lock_guard<std::mutex> lock(mutex);
    return !state.is_processing_callbacks && state.callbacks_notified();
}

SceUInt32 GuestThread::process_callbacks() {
    {
        // Thread info reads the flag too, through wait_over()
        const std::lock_guard<std::mutex> lock(mutex);
        if (state.is_processing_callbacks)
            return 0;
        state.is_processing_callbacks = true;
    }
    SceUInt32 processed = 0;
    for (auto it = state.callbacks.begin(); it != state.callbacks.end();) {
        const CallbackPtr cb = it->lock();
        // Deleted since it was added
        if (!cb) {
            it = state.callbacks.erase(it);
            continue;
        }
        ++it;
        const std::optional<Callback::Notification> notification = cb->take_notification();
        if (!notification)
            continue;
        // A callback starts with the VFP state of a new thread, and the thread gets its own back after it
        const uint32_t ret = call_guest(cb->cb_func.address(),
            { static_cast<uint32_t>(notification->notifier_id), notification->count, static_cast<uint32_t>(notification->arg), cb->userdata.address() }, true);
        ++processed;
        // A callback that exits the thread also ends the wait it runs in
        {
            const std::lock_guard<std::mutex> lock(mutex);
            // The thread's other callbacks are deleted when it ends, with their notifications
            if (state.exiting())
                break;
        }
        // A callback that returns nonzero deletes itself
        if (ret != 0)
            kernel.delete_callback(cb->uid, state.id);
    }
    const std::lock_guard<std::mutex> lock(mutex);
    state.is_processing_callbacks = false;
    return processed;
}

void GuestThread::add_notified_callback() {
    const std::lock_guard<std::mutex> lock(mutex);
    ++state.notified_callbacks;
    cv.notify_all();
}

void GuestThread::remove_notified_callback() {
    const std::lock_guard<std::mutex> lock(mutex);
    if (state.notified_callbacks > 0)
        --state.notified_callbacks;
}

SceKernelThreadInfo GuestThread::info() {
    const std::lock_guard<std::mutex> lock(mutex);
    SceKernelThreadInfo info{};
    strncpy(info.name, state.name.c_str(), KERNELOBJECT_MAX_NAME_LENGTH);
    info.attr = state.attr;
    info.stack = Ptr<void>(state.stack.get());
    info.stackSize = state.stack_size;
    info.initPriority = state.init_priority;
    info.currentPriority = state.priority;
    info.initCpuAffinityMask = state.init_affinity_mask;
    info.currentCpuAffinityMask = state.affinity_mask;
    info.entry = SceKernelThreadEntry(state.entry_point);
    // A thread whose kernel wait is over is READY until it goes on, and waits on nothing, like on a
    // Vita, where the waker ends the wait. A host wait ends with its host call and stays WAITING.
    const bool ready = state.status == ThreadStatus::waiting && !host_waiting && state.wait_over();
    const WaitTarget reported = ready ? WaitTarget{} : state.wait_target;
    info.status = ready ? SCE_KERNEL_THREAD_STATUS_READY : std::to_underlying(shown_status());
    info.waitType = reported.callbacks ? reported.type | SCE_KERNEL_WAITTYPE_CB_BIT : reported.type;
    info.waitId = reported.id;
    info.exitStatus = state.returned_value;
    info.fNotifyCallback = state.callbacks_notified() ? 1 : 0;
    return info;
}

int GuestThread::get_priority() const {
    const std::lock_guard<std::mutex> lock(mutex);
    return state.priority;
}

int GuestThread::change_priority(int new_priority) {
    const std::lock_guard<std::mutex> lock(mutex);
    state.tls_slot(TLS_CURRENT_PRIORITY) = new_priority;
    return std::exchange(state.priority, new_priority);
}

SceInt32 GuestThread::change_affinity_mask(SceInt32 new_affinity_mask) {
    const std::lock_guard<std::mutex> lock(mutex);
    state.tls_slot(TLS_CPU_AFFINITY_MASK) = new_affinity_mask;
    return std::exchange(state.affinity_mask, new_affinity_mask);
}

void GuestThread::set_suspended(bool suspended) {
    const std::lock_guard<std::mutex> lock(mutex);
    debugger_hold = suspended;
    if (suspended && running_guest)
        cpu->request_stop();
    cv.notify_all();
}

bool GuestThread::step() {
    std::unique_lock<std::mutex> lock(mutex);
    // Only a live stop at the guest code or inside a call has a next instruction. A wait, a host
    // wait, a pause or a ForVM suspend would keep holding the thread, so it doesn't step.
    if (!debugger_hold || !(parked || held_in_call) || state.exiting() || state.vm_suspend_count > 0 || kernel.is_threads_paused())
        return false;
    step_requested = true;
    cv.notify_all();
    cv.wait(lock, [&] { return !step_requested && stopped_for_debugger(); });
    return true;
}

bool GuestThread::hold_before_wait(std::unique_lock<std::mutex> &object_lock) {
    std::unique_lock<std::mutex> lock(mutex);
    // Only a ForVM suspend stops a thread before a new wait, so a host hold never changes the wait's result
    if (state.vm_suspend_count == 0 || state.exiting())
        return false;
    object_lock.unlock();
    stop_for_holds(lock, true);
    lock.unlock();
    object_lock.lock();
    return true;
}

void GuestThread::wait_for_debugger_stop() {
    std::unique_lock<std::mutex> lock(mutex);
    cv.wait(lock, [&] { return stopped_for_debugger(); });
}

bool GuestThread::debugger_context(CPUContext &context) const {
    const std::lock_guard<std::mutex> lock(mutex);
    if (!stopped_for_debugger())
        return false;
    context = cpu->save_context();
    return true;
}

bool GuestThread::set_debugger_regs(uint32_t first, std::span<const uint32_t> words) {
    const std::lock_guard<std::mutex> lock(mutex);
    if (!debugger_hold || !parked || state.exiting())
        return false;
    CPUContext context = cpu->save_context();
    for (size_t i = 0; i < words.size(); i++)
        context.set_word(first + static_cast<uint32_t>(i), words[i]);
    cpu->load_context(context);
    return true;
}

std::optional<ThreadStatus> GuestThread::snapshot_context(CPUContext &context) const {
    const std::lock_guard<std::mutex> lock(mutex);
    if (!at_stop())
        return std::nullopt;
    context = cpu->save_context();
    return shown_status();
}

void GuestThread::pause_changed() {
    const std::lock_guard<std::mutex> lock(mutex);
    if (kernel.is_threads_paused() && running_guest)
        cpu->request_stop();
    cv.notify_all();
}

int GuestThread::vm_suspend() {
    std::unique_lock<std::mutex> lock(mutex);
    if (state.status == ThreadStatus::dormant)
        return SCE_KERNEL_ERROR_DORMANT;
    const int count = ++state.vm_suspend_count;
    if (running_guest)
        cpu->request_stop();
    cv.notify_all();
    // Guest code stops before this returns. A thread inside a kernel call stops on its way out.
    cv.wait(lock, [&] { return !running_guest || state.vm_suspend_count == 0; });
    return count;
}

int GuestThread::vm_resume() {
    const std::lock_guard<std::mutex> lock(mutex);
    // Like on a Vita, a dormant thread fails before the count does
    if (state.status == ThreadStatus::dormant)
        return SCE_KERNEL_ERROR_DORMANT;
    if (state.vm_suspend_count == 0)
        return SCE_KERNEL_ERROR_VM_NOT_SUSPENDED;
    // The thread's holds wait for the last resume, and so may a suspend that waits for it to stop
    if (--state.vm_suspend_count == 0)
        cv.notify_all();
    return state.vm_suspend_count;
}

SceInt32 GuestThread::get_vm_context(CPUContext &context, uint32_t &tpidrurw, bool &in_kernel_call) {
    std::unique_lock<std::mutex> lock(mutex);
    // Waits until the thread runs no guest code or kernel code that changes its registers
    cv.wait(lock, [&] { return !runs() || state.vm_suspend_count == 0; });
    if (state.vm_suspend_count == 0)
        return SCE_KERNEL_ERROR_NOT_SUSPENDED;
    context = cpu->save_context();
    tpidrurw = cpu->get_tpidrurw();
    // Inside a kernel call means waiting, in a host wait or held inside one
    in_kernel_call = state.status == ThreadStatus::waiting || held_in_call || host_waiting;
    return SCE_KERNEL_OK;
}
