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

#include <kernel/state.h>
#include <kernel/sync_primitives.h>

#include <chrono>

WaitResult Semaphore::wait_for(ThreadStatePtr t, int n, Deadline d, bool cb) {
    std::unique_lock<std::mutex> lock(mutex);
    if (being_deleted)
        return SCE_KERNEL_ERROR_WAIT_DELETE;
    if (val >= n) {
        val -= n;
        return SCE_KERNEL_OK;
    }
    const WaitInfo info = wait_info(cb);
    SemaWaitEntry e;
    e.need_count = n;
    e.priority = t->enter_wait(info);
    e.thread = std::move(t);
    waiters.push(&e);
    return waiters.wait(lock, e, d, cb, info);
}

SceInt32 Semaphore::poll(int n) {
    std::lock_guard<std::mutex> lock(mutex);
    if (being_deleted)
        return SCE_KERNEL_ERROR_WAIT_DELETE;
    if (val < n)
        return SCE_KERNEL_ERROR_SEMA_ZERO;
    val -= n;
    return SCE_KERNEL_OK;
}

SceInt32 Semaphore::signal(int n) {
    std::lock_guard<std::mutex> lock(mutex);
    if (val + n > max_val)
        return SCE_KERNEL_ERROR_SEMA_OVF;
    val += n;
    waiters.wake_many([&](SemaWaitEntry &e) {
        if (val < e.need_count)
            return false;
        val -= e.need_count;
        return true;
    });
    return SCE_KERNEL_OK;
}

SceUInt32 Semaphore::cancel(SceInt32 setCount) {
    std::lock_guard<std::mutex> lock(mutex);
    const SceUInt32 n = static_cast<SceUInt32>(waiters.size());
    waiters.drain_with_error(SCE_KERNEL_ERROR_WAIT_CANCEL);
    val = (setCount < 0) ? init_val : setCount;
    return n;
}

void Semaphore::mark_deleted() {
    std::lock_guard<std::mutex> lock(mutex);
    being_deleted = true;
    waiters.drain_with_error(SCE_KERNEL_ERROR_WAIT_DELETE);
}

template <SyncWeight W>
std::optional<SceInt32> MutexT<W>::try_acquire_locked(ThreadStatePtr t, int n, MemState &mem) {
    const bool recursive = (attr & SCE_KERNEL_MUTEX_ATTR_RECURSIVE) != 0;
    if (lock_count == 0) {
        owner = t;
        lock_count = n;
    } else if (owner == t) {
        if (!recursive)
            return recursive_error_code;
        if (n > std::numeric_limits<int>::max() - lock_count)
            return lock_ovf_error_code;
        lock_count += n;
    } else {
        return std::nullopt;
    }
    publish_to_workarea_if_lw(mem);
    return SCE_KERNEL_OK;
}

template <SyncWeight W>
WaitResult MutexT<W>::lock(ThreadStatePtr t, int n, Deadline d, MemState &mem, bool cb) {
    std::unique_lock<std::mutex> lock(mutex);
    if (being_deleted)
        return delete_error_code;
    if (const auto r = try_acquire_locked(t, n, mem))
        return *r;
    const WaitInfo info = wait_info(cb);
    MutexWaitEntry e;
    e.lock_count = n;
    e.priority = t->enter_wait(info);
    e.thread = std::move(t);
    waiters.push(&e);
    return waiters.wait(lock, e, d, cb, info);
}

template <SyncWeight W>
SceInt32 MutexT<W>::try_lock(ThreadStatePtr t, int n, MemState &mem) {
    std::lock_guard<std::mutex> lock(mutex);
    if (being_deleted)
        return delete_error_code;
    if (const auto r = try_acquire_locked(t, n, mem))
        return *r;
    return failed_to_own_error_code;
}

template <SyncWeight W>
SceInt32 MutexT<W>::unlock(ThreadStatePtr t, int n, MemState &mem) {
    std::lock_guard<std::mutex> lock(mutex);
    if (owner != t)
        return not_owned_error_code;
    if (n > lock_count)
        return unlock_udf_error_code;
    lock_count -= n;
    if (lock_count == 0) {
        owner.reset();
        // Hand the mutex to the first waiter (lock_count == 0, anyone wins).
        waiters.wake_one([&](MutexWaitEntry &e) {
            owner = e.thread;
            lock_count = e.lock_count;
            return true;
        });
    }
    publish_to_workarea_if_lw(mem);
    return SCE_KERNEL_OK;
}

template <SyncWeight W>
SceUInt32 MutexT<W>::cancel(ThreadStatePtr caller, SceInt32 newCount)
    requires(!is_lw)
{
    std::lock_guard<std::mutex> lock(mutex);
    const SceUInt32 n = static_cast<SceUInt32>(waiters.size());
    waiters.drain_with_error(SCE_KERNEL_ERROR_WAIT_CANCEL_MUTEX);
    if (newCount == 0) {
        owner.reset();
        lock_count = 0;
    } else {
        owner = caller;
        lock_count = (newCount == -1) ? init_count : newCount;
    }
    return n;
}

template <SyncWeight W>
void MutexT<W>::mark_deleted() {
    std::lock_guard<std::mutex> lock(mutex);
    being_deleted = true;
    waiters.drain_with_error(delete_error_code);
}

template class MutexT<SyncWeight::Heavy>;
template class MutexT<SyncWeight::Light>;

template <SyncWeight W>
WaitResult CondvarT<W>::wait(ThreadStatePtr t, Deadline d, MemState &mem, bool cb) {
    std::unique_lock<std::mutex> lock(mutex);
    if (being_deleted)
        return delete_error_code;
    // Per spec: release the associated mutex with one unlock, atomically with
    // entering WAITING; re-acquire with one lock on wake.
    if (const SceInt32 err = associated_mutex->unlock(t, 1, mem); err != SCE_KERNEL_OK)
        return err;

    const WaitInfo info = wait_info(cb);
    CondvarWaitEntry e;
    e.priority = t->enter_wait(info);
    e.thread = t;
    waiters.push(&e);

    const WaitResult r = waiters.wait(lock, e, d, cb, info);
    lock.unlock();

    if (!r || *r != SCE_KERNEL_OK)
        return r;
    return associated_mutex->lock(t, 1, Deadline::max(), mem, false);
}

template <SyncWeight W>
SceInt32 CondvarT<W>::signal(SignalTarget target) {
    std::lock_guard<std::mutex> lock(mutex);
    switch (target.type) {
    case CondvarSignalTarget::Type::Specific: {
        // sceKernelSignalCondTo: target must be waiting on this condvar.
        const bool woken = waiters.wake_one([&](CondvarWaitEntry &e) {
            return e.thread->id == target.thread_id;
        });
        if (!woken)
            return SCE_KERNEL_ERROR_COND_ERROR;
        return SCE_KERNEL_OK;
    }
    case CondvarSignalTarget::Type::Any:
        waiters.wake_one([](CondvarWaitEntry &) { return true; });
        return SCE_KERNEL_OK;
    case CondvarSignalTarget::Type::All:
        waiters.wake_many([](CondvarWaitEntry &) { return true; });
        return SCE_KERNEL_OK;
    }
    return SCE_KERNEL_OK;
}

template <SyncWeight W>
void CondvarT<W>::mark_deleted() {
    std::lock_guard<std::mutex> lock(mutex);
    being_deleted = true;
    waiters.drain_with_error(delete_error_code);
}

template class CondvarT<SyncWeight::Heavy>;
template class CondvarT<SyncWeight::Light>;

bool EventFlag::satisfies(SceUInt32 cur, SceUInt32 bits, SceUInt32 mode) {
    const SceUInt32 op = mode & (SCE_EVENT_WAITAND | SCE_EVENT_WAITOR);
    return (op == SCE_EVENT_WAITAND) ? ((cur & bits) == bits)
                                     : ((cur & bits) != 0);
}

void EventFlag::apply_clear_locked(SceUInt32 wait_mode, SceUInt32 bits) {
    if (wait_mode & SCE_EVENT_WAITCLEAR)
        pattern = 0;
    else if (wait_mode & SCE_EVENT_WAITCLEAR_PAT)
        pattern &= ~bits;
}

WaitResult EventFlag::wait(ThreadStatePtr t, SceUInt32 bits, SceUInt32 wait_mode,
    SceUInt32 *p_result, Deadline d, bool cb) {
    std::unique_lock<std::mutex> lock(mutex);
    if (being_deleted)
        return SCE_KERNEL_ERROR_WAIT_DELETE;
    if (single_attr() && !waiters.empty())
        return SCE_KERNEL_ERROR_EVF_MULTI;

    if (satisfies(pattern, bits, wait_mode)) {
        if (p_result)
            *p_result = pattern;
        apply_clear_locked(wait_mode, bits);
        return SCE_KERNEL_OK;
    }
    const WaitInfo info = wait_info(cb);
    EventFlagWaitEntry e;
    e.bit_pattern = bits;
    e.wait_mode = wait_mode;
    e.out_bits = p_result;
    e.priority = t->enter_wait(info);
    e.thread = std::move(t);
    waiters.push(&e);
    const WaitResult r = waiters.wait(lock, e, d, cb, info);
    if (r && *r != SCE_KERNEL_OK && p_result)
        *p_result = pattern;
    return r;
}

SceInt32 EventFlag::poll(SceUInt32 bits, SceUInt32 wait_mode, SceUInt32 *p_result) {
    std::lock_guard<std::mutex> lock(mutex);
    if (p_result)
        *p_result = pattern;
    if (satisfies(pattern, bits, wait_mode)) {
        apply_clear_locked(wait_mode, bits);
        return SCE_KERNEL_OK;
    }
    return SCE_KERNEL_ERROR_EVF_COND;
}

SceInt32 EventFlag::set(SceUInt32 bits) {
    std::lock_guard<std::mutex> lock(mutex);
    pattern |= bits;
    waiters.wake_many([&](EventFlagWaitEntry &e) {
        if (!satisfies(pattern, e.bit_pattern, e.wait_mode))
            return false;
        if (e.out_bits)
            *e.out_bits = pattern;
        apply_clear_locked(e.wait_mode, e.bit_pattern);
        return true;
    });
    return SCE_KERNEL_OK;
}

SceInt32 EventFlag::clear(SceUInt32 bits) {
    std::lock_guard<std::mutex> lock(mutex);
    pattern &= bits;
    return SCE_KERNEL_OK;
}

SceUInt32 EventFlag::cancel(SceUInt32 new_pattern) {
    std::lock_guard<std::mutex> lock(mutex);
    const SceUInt32 n = static_cast<SceUInt32>(waiters.size());
    waiters.drain_with_error(SCE_KERNEL_ERROR_WAIT_CANCEL, [&](EventFlagWaitEntry &e) {
        if (e.out_bits)
            *e.out_bits = new_pattern;
    });
    pattern = new_pattern;
    return n;
}

void EventFlag::mark_deleted() {
    std::lock_guard<std::mutex> lock(mutex);
    being_deleted = true;
    waiters.drain_with_error(SCE_KERNEL_ERROR_WAIT_DELETE);
}

std::optional<SceInt32> RWLock::try_acquire_locked(ThreadStatePtr t, bool is_write) {
    const bool recursive_attr = (attr & SCE_KERNEL_MUTEX_ATTR_RECURSIVE) != 0;
    const bool already_owns = owners.contains(t);

    // Read lock is implicitly recursive.
    const bool can_take = (state == RWLock::State::Unlocked)
        || (!is_write && state == RWLock::State::ReadLocked)
        || ((recursive_attr || !is_write) && already_owns);

    if (can_take) {
        auto it = owners.find(t);
        if (it != owners.end())
            it->second++;
        else
            owners.emplace(t, 1);
        state = is_write ? RWLock::State::WriteLocked : RWLock::State::ReadLocked;
        return SCE_KERNEL_OK;
    }
    if (!recursive_attr && already_owns)
        return SCE_KERNEL_ERROR_RW_LOCK_RECURSIVE;
    return std::nullopt;
}

WaitResult RWLock::lock(ThreadStatePtr t, bool is_write, Deadline d) {
    std::unique_lock<std::mutex> lock(mutex);
    if (being_deleted)
        return SCE_KERNEL_ERROR_WAIT_DELETE;
    if (const auto r = try_acquire_locked(t, is_write))
        return *r;
    const WaitInfo info = wait_info();
    RWLockWaitEntry e;
    e.is_write = is_write;
    e.priority = t->enter_wait(info);
    e.thread = std::move(t);
    waiters.push(&e);
    return waiters.wait(lock, e, d, false, info);
}

SceInt32 RWLock::try_lock(ThreadStatePtr t, bool is_write) {
    std::lock_guard<std::mutex> lock(mutex);
    if (being_deleted)
        return SCE_KERNEL_ERROR_WAIT_DELETE;
    if (const auto r = try_acquire_locked(t, is_write))
        return *r;
    return SCE_KERNEL_ERROR_RW_LOCK_FAILED_TO_LOCK;
}

SceInt32 RWLock::unlock(ThreadStatePtr t, bool is_write) {
    std::lock_guard<std::mutex> lock(mutex);
    auto it = owners.find(t);
    if (it == owners.end())
        return SCE_KERNEL_ERROR_RW_LOCK_FAILED_TO_UNLOCK;
    it->second--;
    if (it->second == 0)
        owners.erase(it);
    if (!owners.empty())
        return SCE_KERNEL_OK;
    state = RWLock::State::Unlocked;
    // Write-priority: once a writer is at the head we stop granting reads.
    waiters.wake_many([&](RWLockWaitEntry &e) {
        if (state == RWLock::State::WriteLocked)
            return false;
        if (state == RWLock::State::ReadLocked && e.is_write)
            return false;
        owners.emplace(e.thread, 1);
        state = e.is_write ? RWLock::State::WriteLocked : RWLock::State::ReadLocked;
        return true;
    });
    return SCE_KERNEL_OK;
}

RWLock::CancelCounts RWLock::cancel(ThreadStatePtr caller, SceInt32 flag) {
    std::lock_guard<std::mutex> lock(mutex);
    CancelCounts counts{ 0, 0 };
    waiters.drain_with_error(SCE_KERNEL_ERROR_WAIT_CANCEL, [&](RWLockWaitEntry &e) {
        (e.is_write ? counts.write : counts.read)++;
    });
    owners.clear();
    if (flag & SCE_KERNEL_RW_LOCK_CANCEL_WITH_WRITE_LOCK) {
        owners.emplace(caller, 1);
        state = RWLock::State::WriteLocked;
    } else {
        state = RWLock::State::Unlocked;
    }
    return counts;
}

void RWLock::mark_deleted() {
    std::lock_guard<std::mutex> lock(mutex);
    being_deleted = true;
    waiters.drain_with_error(SCE_KERNEL_ERROR_WAIT_DELETE);
}

WaitResult SimpleEvent::wait_or_poll(ThreadStatePtr t, SceUInt32 wait_pattern, SceUInt32 *result_pattern,
    SceUInt64 *user_data_out, Deadline d, bool is_wait, bool alertable) {
    std::unique_lock<std::mutex> lock(mutex);
    if (being_deleted)
        return SCE_KERNEL_ERROR_WAIT_DELETE;
    if (result_pattern)
        *result_pattern = pattern;

    if (pattern & wait_pattern) {
        if (auto_reset)
            pattern &= ~wait_pattern;
        if (user_data_out)
            *user_data_out = user_data;
        return SCE_KERNEL_OK;
    }

    if (!is_wait)
        return SCE_KERNEL_ERROR_EVENT_COND;

    const WaitInfo info = wait_info(alertable);
    SimpleEventWaitEntry e;
    e.wait_pattern = wait_pattern;
    e.result_pattern = result_pattern;
    e.user_data = user_data_out;
    e.priority = t->enter_wait(info);
    e.thread = std::move(t);
    waiters.push(&e);

    const WaitResult r = waiters.wait(lock, e, d, alertable, info);
    if (alertable && !r)
        return SCE_KERNEL_OK;
    if (r && *r != SCE_KERNEL_OK) {
        if (result_pattern)
            *result_pattern = pattern;
        if (user_data_out)
            *user_data_out = user_data;
    }
    return r;
}

SceInt32 SimpleEvent::set_or_pulse(SceUInt32 pattern_to_set, SceUInt64 ud, bool is_set) {
    std::lock_guard<std::mutex> lock(mutex);
    pattern |= pattern_to_set;
    user_data = ud;

    waiters.wake_many([&](SimpleEventWaitEntry &e) {
        if ((pattern & e.wait_pattern) == 0)
            return false;
        if (e.result_pattern)
            *e.result_pattern = pattern;
        if (e.user_data)
            *e.user_data = user_data;
        if (auto_reset)
            pattern &= ~e.wait_pattern;
        return true;
    });

    if (!is_set)
        pattern = 0; // pulse
    return SCE_KERNEL_OK;
}

SceInt32 SimpleEvent::clear(SceUInt32 clear_pattern) {
    std::lock_guard<std::mutex> lock(mutex);
    pattern &= clear_pattern;
    return SCE_KERNEL_OK;
}

SceUInt32 SimpleEvent::cancel() {
    std::lock_guard<std::mutex> lock(mutex);
    const SceUInt32 n = static_cast<SceUInt32>(waiters.size());
    waiters.drain_with_error(SCE_KERNEL_ERROR_WAIT_CANCEL);
    return n;
}

void SimpleEvent::mark_deleted() {
    std::lock_guard<std::mutex> lock(mutex);
    being_deleted = true;
    waiters.drain_with_error(SCE_KERNEL_ERROR_WAIT_DELETE);
}

void MsgPipe::wake_one_receiver_locked() {
    receivers.wake_one([&](MsgPipeRecvWaitEntry &e) {
        return data_buffer.Used() >= e.request_size;
    });
}

void MsgPipe::wake_one_sender_locked() {
    senders.wake_one([&](MsgPipeSendWaitEntry &e) {
        return data_buffer.Free() >= e.request_size;
    });
}

std::expected<SceSize, ExitSignal> MsgPipe::recv(ThreadStatePtr t, void *p_recv, SceSize recv_size,
    SceUInt32 wait_mode, Deadline d, bool cb) {
    if (recv_size > data_buffer.Capacity())
        return static_cast<SceSize>(SCE_KERNEL_ERROR_ILLEGAL_SIZE);

    const bool ASAP = !(wait_mode & SCE_KERNEL_MSG_PIPE_MODE_FULL);
    const bool peek = (wait_mode & SCE_KERNEL_MSG_PIPE_MODE_DONT_REMOVE) != 0;
    const bool no_wait = (wait_mode & SCE_KERNEL_MSG_PIPE_MODE_DONT_WAIT) != 0;

    std::unique_lock<std::mutex> lock(mutex);
    if (being_deleted)
        return static_cast<SceSize>(SCE_KERNEL_ERROR_WAIT_DELETE);

    const std::size_t used = data_buffer.Used();
    if (used >= recv_size || (ASAP && used >= 1)) {
        const SceSize n = peek
            ? static_cast<SceSize>(data_buffer.Peek(p_recv, recv_size))
            : static_cast<SceSize>(data_buffer.Remove(p_recv, recv_size));
        if (!peek)
            wake_one_sender_locked();
        return n;
    }
    if (no_wait)
        return SceSize{ 0 };
    const WaitInfo info = recv_wait_info(cb);
    MsgPipeRecvWaitEntry e;
    e.request_size = ASAP ? 1u : recv_size;
    e.priority = t->enter_wait(info);
    e.thread = std::move(t);
    receivers.push(&e);

    const WaitResult r = receivers.wait(lock, e, d, cb, info);
    if (!r)
        return r;
    if (*r != SCE_KERNEL_OK)
        return static_cast<SceSize>(*r);

    const SceSize n = peek
        ? static_cast<SceSize>(data_buffer.Peek(p_recv, recv_size))
        : static_cast<SceSize>(data_buffer.Remove(p_recv, recv_size));
    if (!peek)
        wake_one_sender_locked();
    return n;
}

std::expected<SceSize, ExitSignal> MsgPipe::send(ThreadStatePtr t, const void *p_send, SceSize send_size,
    SceUInt32 wait_mode, Deadline d, bool cb) {
    if (send_size > data_buffer.Capacity())
        return static_cast<SceSize>(SCE_KERNEL_ERROR_ILLEGAL_SIZE);

    const bool ASAP = !(wait_mode & SCE_KERNEL_MSG_PIPE_MODE_FULL);
    const bool no_wait = (wait_mode & SCE_KERNEL_MSG_PIPE_MODE_DONT_WAIT) != 0;

    std::unique_lock<std::mutex> lock(mutex);
    if (being_deleted)
        return static_cast<SceSize>(SCE_KERNEL_ERROR_WAIT_DELETE);

    const std::size_t free = data_buffer.Free();
    if (free >= send_size || (ASAP && free >= 1)) {
        const SceSize n = static_cast<SceSize>(data_buffer.Insert(p_send, send_size));
        wake_one_receiver_locked();
        return n;
    }
    if (no_wait)
        return SceSize{ 0 };
    const WaitInfo info = send_wait_info(cb);
    MsgPipeSendWaitEntry e;
    e.request_size = ASAP ? 1u : send_size;
    e.priority = t->enter_wait(info);
    e.thread = std::move(t);
    senders.push(&e);

    const WaitResult r = senders.wait(lock, e, d, cb, info);
    if (!r)
        return r;
    if (*r != SCE_KERNEL_OK)
        return static_cast<SceSize>(*r);

    const SceSize n = static_cast<SceSize>(data_buffer.Insert(p_send, send_size));
    wake_one_receiver_locked();
    return n;
}

MsgPipe::CancelCounts MsgPipe::cancel() {
    std::lock_guard<std::mutex> lock(mutex);
    CancelCounts counts{
        static_cast<SceUInt32>(senders.size()),
        static_cast<SceUInt32>(receivers.size()),
    };
    senders.drain_with_error(SCE_KERNEL_ERROR_WAIT_CANCEL);
    receivers.drain_with_error(SCE_KERNEL_ERROR_WAIT_CANCEL);
    return counts;
}

SceInt32 MsgPipe::mark_deleted_and_drain() {
    std::lock_guard<std::mutex> lock(mutex);
    being_deleted = true;
    senders.drain_with_error(SCE_KERNEL_ERROR_WAIT_DELETE);
    receivers.drain_with_error(SCE_KERNEL_ERROR_WAIT_DELETE);
    return SCE_KERNEL_OK;
}

// Timer parks waiters on its own condvar instead of WaitQueue / park.

#include <algorithm>
#include <cstring>
#include <limits>
#include <list>
#include <util/lock_and_find.h>

namespace {

uint64_t get_current_time() {
    return std::chrono::duration_cast<std::chrono::microseconds>(
        std::chrono::high_resolution_clock::now().time_since_epoch())
        .count();
}

} // namespace

void Timer::schedule_next_event_locked() {
    next_event = get_current_time() + event_interval;
    condvar.notify_all();
}

SceInt32 Timer::set(SceInt32 type, SceKernelSysClock *interval, SceInt32 repeats) {
    if (!interval)
        return SCE_KERNEL_ERROR_INVALID_ARGUMENT;

    const std::lock_guard<std::mutex> lock(mutex);
    is_pulse = type != 0;
    is_repeat = repeats != 0;
    event_interval = *interval;
    // Per spec: the timer enters non-notification state when the event is set.
    event_set = false;

    if (is_started)
        schedule_next_event_locked();

    return SCE_KERNEL_OK;
}

SceInt32 Timer::wait_or_poll(ThreadStatePtr t, SceUInt32 *result_pattern,
    SceUInt64 *user_data, bool is_wait, bool alertable) {
    std::unique_lock<std::mutex> lock(mutex);
    if (being_deleted)
        return SCE_KERNEL_ERROR_WAIT_DELETE;

    if (result_pattern)
        *result_pattern = SCE_KERNEL_EVENT_TIMER;
    if (user_data)
        *user_data = 0;

    uint64_t current_time = get_current_time();
    auto advance_next_event = [&]() {
        if (is_repeat) {
            // Walk forward to the first scheduled tick past current_time.
            next_event += ((current_time - next_event - 1) / event_interval + 1) * event_interval;
        } else {
            next_event = std::numeric_limits<uint64_t>::max();
        }
    };

    if (next_event < current_time) {
        if (!is_pulse)
            event_set = true;
        advance_next_event();
    }

    if (event_set) {
        if (attr & SCE_KERNEL_EVENT_ATTR_AUTO_RESET)
            event_set = false;
        return SCE_KERNEL_OK;
    }
    if (!is_wait)
        return SCE_KERNEL_ERROR_EVENT_COND;

    const WaitInfo info{
        .type = alertable ? SCE_KERNEL_WAITTYPE_EVENT_CB : SCE_KERNEL_WAITTYPE_EVENT,
        .uid = uid,
        .reason = "timer",
    };
    t->enter_wait(info);

    while (true) {
        const uint64_t wait_time = (next_event > current_time) ? next_event - current_time : 0;
        condvar.wait_for(lock, std::chrono::microseconds(wait_time), [&] {
            return being_deleted || event_set || get_current_time() > next_event;
        });
        if (being_deleted) {
            t->leave_wait();
            return SCE_KERNEL_ERROR_WAIT_DELETE;
        }
        current_time = get_current_time();
        if (event_set || current_time > next_event)
            break;
    }

    event_set = !is_pulse && !(attr & SCE_KERNEL_EVENT_ATTR_AUTO_RESET);
    advance_next_event();
    condvar.notify_all();
    t->leave_wait();
    return SCE_KERNEL_OK;
}

void Timer::mark_deleted() {
    std::lock_guard<std::mutex> lock(mutex);
    being_deleted = true;
    condvar.notify_all();
}

SceInt32 Timer::clear() {
    std::lock_guard<std::mutex> lock(mutex);
    event_set = false;
    return SCE_KERNEL_OK;
}

SceInt32 Timer::start() {
    std::lock_guard<std::mutex> lock(mutex);
    if (is_started)
        return 1;
    is_started = true;
    time = get_current_time();
    if (event_interval != 0)
        schedule_next_event_locked();
    return SCE_KERNEL_OK;
}

SceInt32 Timer::stop() {
    std::lock_guard<std::mutex> lock(mutex);
    const bool was_stopped = !is_started;
    is_started = false;
    time = get_current_time();
    next_event = std::numeric_limits<uint64_t>::max();
    return static_cast<int>(was_stopped);
}
