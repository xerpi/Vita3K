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
#include <mem/ptr.h>
#include <util/align.h>

#include <util/log.h>

#include <cassert>
#include <chrono>
#include <cstring>
#include <memory>
#include <sstream>
#include <utility>

int ThreadState::init(std::string_view name, Ptr<const void> entry_point, int init_priority, SceInt32 affinity_mask, int stack_size, const SceKernelThreadOptParam *option = nullptr) {
    constexpr size_t KERNEL_TLS_SIZE = 0x800;

    // the stack size should be page-aligned
    stack_size = align(stack_size, KiB(4));

    const std::size_t n = std::min(name.size(), static_cast<std::size_t>(SCE_UID_NAMELEN));
    if (n > 0)
        std::memcpy(this->name, name.data(), n);
    this->name[n] = '\0';
    this->entry_point = entry_point.address();

    int core_num = kernel.corenum_allocator.new_corenum();
    if (core_num < 0) {
        LOG_ERROR("Out of core number to allocate, use 0");
        core_num = 0;
    }

    if (init_priority > SCE_KERNEL_LOWEST_PRIORITY_USER) {
        assert(SCE_KERNEL_HIGHEST_DEFAULT_PRIORITY <= init_priority && init_priority <= SCE_KERNEL_LOWEST_DEFAULT_PRIORITY);
        priority = init_priority - SCE_KERNEL_DEFAULT_PRIORITY + SCE_KERNEL_GAME_DEFAULT_PRIORITY_ACTUAL;
    } else {
        priority = init_priority;
    }
    this->affinity_mask = affinity_mask;
    this->stack_size = stack_size;
    start_tick = rtc_get_ticks(kernel.base_tick.tick);
    last_vblank_waited = 0;

    cpu = init_cpu(kernel.cpu_opt, id, static_cast<std::size_t>(core_num), mem);
    if (!cpu) {
        return SCE_KERNEL_ERROR_ERROR;
    }
    if (kernel.debugger.watch_code) {
        set_log_code(*cpu, true);
    }
    if (kernel.debugger.watch_memory) {
        set_log_mem(*cpu, true);
    }

    std::string alloc_name = fmt::format("Stack for thread {} (#{})", name, id);
    stack = alloc_block(mem, stack_size, alloc_name.c_str());
    memset(stack.get_ptr<void>().get(mem), 0xcc, stack_size);

    alloc_name = fmt::format("TLS for thread {} (#{})", name, id);
    const size_t tls_size = KERNEL_TLS_SIZE + kernel.tls_msize;
    tls = alloc_block(mem, tls_size, alloc_name.c_str());
    const Ptr<uint8_t> base_tls_ptr = tls.get_ptr<uint8_t>();
    memset(base_tls_ptr.get(mem), 0, tls_size);

    int *tls_array = tls.get_ptr<int>().get(mem);

    tls_array[TLS_PROCESS_ID] = 1; // stubbed. unused
    tls_array[TLS_THREAD_ID] = id;
    tls_array[TLS_SP_TOP] = stack.get();
    tls_array[TLS_SP_BOTTOM] = stack.get() + stack_size;
    tls_array[TLS_CURRENT_PRIORITY] = priority;
    tls_array[TLS_CPU_AFFINITY_MASK] = affinity_mask;

    const Ptr<uint8_t> user_tls_ptr = base_tls_ptr + KERNEL_TLS_SIZE;
    write_tpidruro(*cpu, user_tls_ptr.address());
    if (kernel.tls_address) {
        assert(kernel.tls_psize <= kernel.tls_msize);
        memcpy(user_tls_ptr.get(mem), kernel.tls_address.get(mem), kernel.tls_psize);
    }

    CPUContext ctx;
    ctx.set_sp(stack_top());
    if (option) {
        ctx.cpu_registers[0] = option->attr;
        ctx.cpu_registers[1] = option->size;
    }
    this->init_cpu_ctx = ctx;

    return 0;
}

void ThreadState::raise_wait_thread_end_joiners() {
    std::lock_guard<std::mutex> lock(mutex);
    const SceInt32 final_value = static_cast<SceInt32>(returned_value);
    wait_thread_end_joiners.wake_many([&](WaitThreadEndJoinerEntry &e) {
        e.returned_value = final_value;
        return true;
    });
}

void ThreadState::set_status_locked(ThreadStatus next) {
#ifndef NDEBUG
    const ThreadStatus from = status;
    bool valid = (from == next);
    if (!valid) {
        switch (from) {
        case ThreadStatus::dormant: valid = (next == ThreadStatus::running); break;
        case ThreadStatus::running: valid = (next == ThreadStatus::waiting || next == ThreadStatus::dormant || next == ThreadStatus::dead); break;
        case ThreadStatus::waiting: valid = (next == ThreadStatus::running); break;
        case ThreadStatus::dead: valid = false; break;
        }
    }
    assert(valid && "illegal ThreadStatus transition");
#endif
    status = next;
}

void ThreadState::apply_guest_args(Address pc, const GuestArgs &args) {
    load_context(*cpu, init_cpu_ctx);
    write_pc(*cpu, pc);
    write_lr(*cpu, kernel.halt_instruction_pc);
    if (const auto *a = std::get_if<ArglenArgs>(&args)) {
        write_reg(*cpu, 0, a->arglen);
        if (a->argp && a->arglen > 0) {
            const Address data_addr = stack_alloc(*cpu, align(a->arglen, 8));
            memcpy(Ptr<uint8_t>(data_addr).get(mem), a->argp.get(mem), a->arglen);
            write_reg(*cpu, 1, data_addr);
        } else {
            write_reg(*cpu, 1, 0);
        }
    } else {
        push_arguments(std::get<RegisterArgs>(args).values);
    }
}

int ThreadState::start(SceSize arglen, Ptr<void> argp, bool fire_start) {
    {
        std::lock_guard<std::mutex> lock(mutex);
        if (status != ThreadStatus::dormant)
            return SCE_KERNEL_ERROR_RUNNING;

        entry_call = { .pc = entry_point, .args = ArglenArgs{ arglen, argp }, .fire_events = fire_start };
        pending_guest_call = &entry_call;
        is_suspended = std::exchange(kernel.debugger.wait_for_debugger, false);
        exit_request.reset();
        set_status_locked(ThreadStatus::running);
    }
    lifecycle_cv.notify_all();
    return SCE_KERNEL_OK;
}

void ThreadState::exit(SceInt32 status) {
    {
        std::lock_guard<std::mutex> lock(mutex);
        exit_request = SelfExitRequest::exit;
        returned_value = static_cast<uint32_t>(status);
    }
    stop(*cpu);
}

void ThreadState::exit_delete(SceInt32 status) {
    {
        std::lock_guard<std::mutex> lock(mutex);
        exit_request = SelfExitRequest::exit_delete;
        returned_value = static_cast<uint32_t>(status);
    }
    stop(*cpu);
}

void ThreadState::request_host_thread_exit() {
    {
        std::lock_guard<std::mutex> lock(mutex);
        host_thread_exit_requested = true;
        complete_pending_guest_call_locked(GuestCallError::terminated);
    }
    stop(*cpu);
    lifecycle_cv.notify_all();
    unpark();
}

ThreadState::ParkResult ThreadState::park_until(std::chrono::steady_clock::time_point deadline) {
    std::unique_lock<std::mutex> lock(park_mutex);
    if (!park_cv.wait_until(lock, deadline, [&] { return park_permit; }))
        return ParkResult::timed_out;
    park_permit = false;
    return ParkResult::woken;
}

void ThreadState::unpark() {
    {
        std::lock_guard<std::mutex> lock(park_mutex);
        park_permit = true;
    }
    park_cv.notify_one();
}

int ThreadState::enter_wait(WaitInfo info) {
    std::lock_guard<std::mutex> lock(mutex);
    set_status_locked(ThreadStatus::waiting);
    wait_info = info;
    return priority;
}

void ThreadState::leave_wait() {
    std::lock_guard<std::mutex> lock(mutex);
    set_status_locked(ThreadStatus::running);
    wait_info = {};
}

bool ThreadState::stop_requested_locked() const {
    return exit_request || host_thread_exit_requested;
}

SceUInt32 ThreadState::vita_status_locked() const {
    SceUInt32 bits = 0;
    switch (status) {
    case ThreadStatus::running:
        bits = SCE_KERNEL_THREAD_STATUS_RUNNING;
        break;
    case ThreadStatus::waiting:
        bits = SCE_KERNEL_THREAD_STATUS_WAITING;
        break;
    case ThreadStatus::dormant:
        bits = SCE_KERNEL_THREAD_STATUS_DORMANT;
        break;
    case ThreadStatus::dead:
        bits = SCE_KERNEL_THREAD_STATUS_DEAD;
        break;
    }
    if (is_suspended)
        bits |= SCE_KERNEL_THREAD_STATUS_SUSPENDED;
    return bits;
}

SceUInt32 ThreadState::vita_status() const {
    std::lock_guard<std::mutex> lock(mutex);
    return vita_status_locked();
}

void ThreadState::run_guest_until_halt() {
    while (true) {
        bool do_step = false;
        {
            std::unique_lock<std::mutex> lock(mutex);
            lifecycle_cv.wait(lock, [&] {
                return !is_suspended || host_thread_exit_requested
                    || exit_request;
            });
            if (stop_requested_locked())
                return;
            do_step = (debug_request == DebugRequest::step);
            if (do_step)
                debug_request = DebugRequest::none;
        }

        const int res = do_step ? step(*cpu) : run(*cpu);

        if (cpu->svc_called) {
            const uint32_t nid = *Ptr<uint32_t>(read_pc(*cpu) + 4).get(mem);
            kernel.call_import(*cpu, nid, id);
            clear_exclusive(*cpu);
        }
        if (cpu->abort_pending.exchange(false))
            dispatch_abort(*cpu);

        if (res < 0) {
            LOG_ERROR("Thread {} ({}) experienced a cpu error.", name, cpu->thread_id);
            std::lock_guard<std::mutex> lock(mutex);
            set_status_locked(ThreadStatus::dead);
            returned_value = 0xDEADDEAD;
            return;
        }

        if (do_step || hit_breakpoint(*cpu)) {
            std::lock_guard<std::mutex> lock(mutex);
            is_suspended = true;
            debug_request = DebugRequest::suspend;
            continue;
        }

        if (res > 0)
            return;
    }
}

bool ThreadState::execute_run(QueuedGuestCall &call) {
    apply_guest_args(call.pc, call.args);

    if (call.fire_events && kernel.thread_event_start) {
        const uint32_t r = call_guest_inline(kernel.thread_event_start.address(),
            RegisterArgs{ { SCE_KERNEL_THREAD_EVENT_TYPE_START,
                static_cast<uint32_t>(id), 0, kernel.thread_event_start_arg } });
        if (r != 0)
            LOG_WARN("Thread start event handler returned {}", log_hex(r));
    }

    run_guest_until_halt();

    bool dead, exit_del, host_thread_exit, fire_end;
    {
        std::lock_guard<std::mutex> lock(mutex);
        dead = status == ThreadStatus::dead;
        exit_del = exit_request == SelfExitRequest::exit_delete;
        host_thread_exit = host_thread_exit_requested;
        fire_end = !dead && !host_thread_exit
            && exit_request == SelfExitRequest::exit
            && kernel.thread_event_end;
        if (!dead && !host_thread_exit && !exit_request)
            returned_value = read_reg(*cpu, 0);
    }

    if (fire_end) {
        const uint32_t r = call_guest_inline(kernel.thread_event_end.address(),
            RegisterArgs{ { SCE_KERNEL_THREAD_EVENT_TYPE_END,
                static_cast<uint32_t>(id), 0, kernel.thread_event_end_arg } });
        if (r != 0)
            LOG_WARN("Thread end event handler returned {}", log_hex(r));
    }

    const bool terminal = dead || exit_del || host_thread_exit;
    {
        std::lock_guard<std::mutex> lock(mutex);
        is_suspended = false;
        wait_info = {};
        call.result = terminal
            ? GuestCallResult(std::unexpected{ GuestCallError::terminated })
            : GuestCallResult(returned_value);
        call.completed = true;
        if (!terminal) {
            set_status_locked(ThreadStatus::dormant);
            exit_request.reset();
        }
    }
    lifecycle_cv.notify_all();

    if (!host_thread_exit)
        raise_wait_thread_end_joiners();

    return terminal;
}

void ThreadState::run_loop() {
    set_current_cpu_state(cpu.get());
    struct CpuStateGuard {
        ~CpuStateGuard() { set_current_cpu_state(nullptr); }
    } cpu_state_guard;

    while (true) {
        QueuedGuestCall *call = nullptr;
        {
            std::unique_lock<std::mutex> lock(mutex);
            lifecycle_cv.wait(lock, [&] {
                return host_thread_exit_requested
                    || (pending_guest_call && !is_suspended);
            });
            if (host_thread_exit_requested) {
                complete_pending_guest_call_locked(GuestCallError::terminated);
                lifecycle_cv.notify_all();
                break;
            }
            call = std::exchange(pending_guest_call, nullptr);
        }

        if (execute_run(*call))
            break;
    }
}

void ThreadState::push_arguments(const std::vector<uint32_t> &args) {
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

uint32_t ThreadState::call_guest_inline(Address pc, RegisterArgs args) {
    std::unique_lock<std::mutex> lock(mutex);
    if (stop_requested_locked())
        return 0;

    const CPUContext prev_ctx = save_context(*cpu);
    const uint32_t prev_tpidruro = read_tpidruro(*cpu);
    const std::optional<SelfExitRequest> outer_exit = std::exchange(exit_request, std::nullopt);

    write_pc(*cpu, pc);
    write_lr(*cpu, kernel.halt_instruction_pc);
    push_arguments(args.values);
    lock.unlock();

    run_guest_until_halt();

    lock.lock();
    exit_request = std::max(outer_exit, exit_request);
    const uint32_t r0 = (status != ThreadStatus::dead) ? read_reg(*cpu, 0) : 0;
    load_context(*cpu, prev_ctx);
    write_tpidruro(*cpu, prev_tpidruro);
    return r0;
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
    // the faulting instruction. The handler is called as a notification
    // and we don't restore from AbortContext afterward.
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

    call_guest_inline(handler, RegisterArgs{ { sp_aligned } });
}

void ThreadState::complete_pending_guest_call_locked(GuestCallError error) {
    if (!pending_guest_call)
        return;
    pending_guest_call->result = std::unexpected{ error };
    pending_guest_call->completed = true;
    pending_guest_call = nullptr;
}

GuestCallResult ThreadState::call_guest_on_thread(Address pc, GuestArgs args) {
    QueuedGuestCall call{ .pc = pc, .args = std::move(args) };

    std::unique_lock<std::mutex> lock(mutex);
    if (host_thread_exit_requested)
        return std::unexpected{ GuestCallError::terminated };
    if (status != ThreadStatus::dormant)
        return std::unexpected{ GuestCallError::not_dormant };

    pending_guest_call = &call;
    exit_request.reset();
    set_status_locked(ThreadStatus::running);
    lifecycle_cv.notify_all();
    lifecycle_cv.wait(lock, [&] { return call.completed; });
    return call.result;
}

ThreadState::ThreadState(SceUID id, KernelState &kernel, MemState &mem)
    : id(id)
    , kernel(kernel)
    , mem(mem) {
}

Address ThreadState::stack_top() const {
    return stack.get() + stack_size;
}

void ThreadState::suspend() {
    {
        const std::lock_guard<std::mutex> lock(mutex);
        assert(status == ThreadStatus::running);
        debug_request = DebugRequest::suspend;
    }
    stop(*cpu);
}

void ThreadState::resume(bool step) {
    {
        const std::lock_guard<std::mutex> lock(mutex);
        assert(is_suspended || status == ThreadStatus::dormant);
        is_suspended = false;
        debug_request = step ? DebugRequest::step : DebugRequest::none;
    }
    lifecycle_cv.notify_all();
    unpark();
}

std::string ThreadState::log_stack_traceback() const {
    constexpr Address START_OFFSET = 0;
    constexpr Address END_OFFSET = 1024;
    std::string str;
    const Address sp = read_sp(*cpu);
    for (Address addr = sp - START_OFFSET; addr <= sp + END_OFFSET; addr += 4) {
        if (Ptr<uint32_t>(addr).valid(mem)) {
            const Address value = *Ptr<uint32_t>(addr).get(mem);
            const auto mod = kernel.find_module_by_addr(value);
            if (mod)
                fmt::format_to(std::back_inserter(str), "0x{:X} (module: {})\n", value, mod->module_name);
        }
    }
    return str;
}
