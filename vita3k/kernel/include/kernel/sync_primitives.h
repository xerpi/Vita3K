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

#include <kernel/thread/thread_state.h>
#include <kernel/thread/wait_queue.h>
#include <kernel/types.h>
#include <util/byte_ring_buffer.h>

#include <map>
#include <memory>
#include <optional>
#include <string>
#include <type_traits>
#include <variant>

struct KernelState;
struct MemState;

// Shared base for every kernel sync object. Per-primitive teardown lives in
// each class's mark_deleted() because the drain code is queue-typed.
struct SyncObjectBase {
    std::mutex mutex;
    SceUID uid = 0;
    std::string name;
    SceUInt32 attr = 0;
    bool being_deleted = false;

    SyncObjectBase(SceUID uid, std::string name, SceUInt32 attr)
        : uid(uid)
        , name(std::move(name))
        , attr(attr) {}
};


struct SemaWaitEntry : WaitEntryBase {
    int need_count = 0;
};

class Semaphore : public SyncObjectBase {
public:
    Semaphore(SceUID uid, std::string name, SceUInt32 attr,
        int init_val, int max_val)
        : SyncObjectBase(uid, std::move(name), attr)
        , init_val(init_val)
        , val(init_val)
        , max_val(max_val)
        , waiters(wait_order_from_attr(attr)) {}

    WaitResult wait_for(ThreadStatePtr t, int n, Deadline d, bool cb);
    SceInt32 poll(int n);
    SceInt32 signal(int n);
    SceUInt32 cancel(SceInt32 setCount);
    void mark_deleted();

    int initial_value() const { return init_val; }
    int current_value() const { return val; }
    int max_value() const { return max_val; }
    std::size_t num_waiters() const { return waiters.size(); }

private:
    WaitInfo wait_info(bool cb) const {
        return {
            .type = cb ? SCE_KERNEL_WAITTYPE_SEMAPHORE_CB : SCE_KERNEL_WAITTYPE_SEMAPHORE,
            .uid = uid,
            .reason = "semaphore"
        };
    }

    int init_val;
    int val;
    int max_val;
    WaitQueue<SemaWaitEntry> waiters;
};
using SemaphorePtr = std::shared_ptr<Semaphore>;
using SemaphorePtrs = std::map<SceUID, SemaphorePtr>;

// Heavy and Light variants share one template. The Light specialization mirrors
// lockCount/owner into a guest SceKernelLwMutexWork for the SDK fast-path and
// has no cancel() API.

enum class SyncWeight {
    Heavy,
    Light
};

struct MutexWaitEntry : WaitEntryBase {
    int lock_count = 0;
};

template <SyncWeight W>
class MutexT : public SyncObjectBase {
public:
    static constexpr bool is_lw = (W == SyncWeight::Light);

    MutexT(SceUID uid, std::string name, SceUInt32 attr, int init_count)
        requires(!is_lw)
        : SyncObjectBase(uid, std::move(name), attr)
        , init_count(init_count)
        , waiters(wait_order_from_attr(attr)) {}

    MutexT(SceUID uid, std::string name, SceUInt32 attr,
        int init_count, Ptr<SceKernelLwMutexWork> workarea)
        requires(is_lw)
        : SyncObjectBase(uid, std::move(name), attr)
        , init_count(init_count)
        , workarea(workarea)
        , waiters(wait_order_from_attr(attr)) {}

    WaitResult lock(ThreadStatePtr t, int n, Deadline d, MemState &mem, bool cb);
    SceInt32 try_lock(ThreadStatePtr t, int n, MemState &mem);
    SceInt32 unlock(ThreadStatePtr t, int n, MemState &mem);
    SceUInt32 cancel(ThreadStatePtr caller, SceInt32 newCount)
        requires(!is_lw);
    void mark_deleted();

    SceUID current_owner_id() const { return owner ? owner->id : 0; }
    int current_lock_count() const { return lock_count; }
    int initial_count() const { return init_count; }
    std::size_t num_waiters() const { return waiters.size(); }
    Ptr<SceKernelLwMutexWork> get_workarea() const
        requires(is_lw)
    { return workarea; }

private:
    // Attempts to acquire the mutex. nullopt = would block; otherwise the
    // syscall return (SCE_KERNEL_OK or a negative error).
    std::optional<SceInt32> try_acquire_locked(ThreadStatePtr t, int n, MemState &mem);
    void publish_to_workarea_if_lw(MemState &mem) {
        if constexpr (is_lw) {
            if (!workarea)
                return;
            auto *wa = workarea.get(mem);
            wa->lockCount = lock_count;
            wa->owner = owner ? owner->id : 0;
        }
    }

    static constexpr SceInt32 delete_error_code = is_lw
        ? SCE_KERNEL_ERROR_WAIT_DELETE_LW_MUTEX
        : SCE_KERNEL_ERROR_WAIT_DELETE_MUTEX;
    static constexpr SceInt32 recursive_error_code = is_lw
        ? SCE_KERNEL_ERROR_LW_MUTEX_RECURSIVE
        : SCE_KERNEL_ERROR_MUTEX_RECURSIVE;
    static constexpr SceInt32 lock_ovf_error_code = is_lw
        ? SCE_KERNEL_ERROR_LW_MUTEX_LOCK_OVF
        : SCE_KERNEL_ERROR_MUTEX_LOCK_OVF;
    static constexpr SceInt32 unlock_udf_error_code = is_lw
        ? SCE_KERNEL_ERROR_LW_MUTEX_UNLOCK_UDF
        : SCE_KERNEL_ERROR_MUTEX_UNLOCK_UDF;
    static constexpr SceInt32 not_owned_error_code = is_lw
        ? SCE_KERNEL_ERROR_LW_MUTEX_NOT_OWNED
        : SCE_KERNEL_ERROR_MUTEX_NOT_OWNED;
    static constexpr SceInt32 failed_to_own_error_code = is_lw
        ? SCE_KERNEL_ERROR_LW_MUTEX_FAILED_TO_OWN
        : SCE_KERNEL_ERROR_MUTEX_FAILED_TO_OWN;

    WaitInfo wait_info(bool cb) const {
        constexpr SceUInt32 type = is_lw ? SCE_KERNEL_WAITTYPE_LW_MUTEX : SCE_KERNEL_WAITTYPE_MUTEX;
        constexpr SceUInt32 type_cb = is_lw ? SCE_KERNEL_WAITTYPE_LW_MUTEX_CB : SCE_KERNEL_WAITTYPE_MUTEX_CB;
        return {
            .type = cb ? type_cb : type,
            .uid = uid,
            .reason = is_lw ? "lwmutex" : "mutex"
        };
    }

    int init_count;
    int lock_count = 0;
    ThreadStatePtr owner;
    [[no_unique_address]] std::conditional_t<is_lw,
        Ptr<SceKernelLwMutexWork>, std::monostate>
        workarea{};
    WaitQueue<MutexWaitEntry> waiters;
};

using Mutex = MutexT<SyncWeight::Heavy>;
using LwMutex = MutexT<SyncWeight::Light>;
using MutexPtr = std::shared_ptr<Mutex>;
using LwMutexPtr = std::shared_ptr<LwMutex>;
using MutexPtrs = std::map<SceUID, MutexPtr>;
using LwMutexPtrs = std::map<SceUID, LwMutexPtr>;

// Same Heavy/Light split as Mutex; differs only in the paired mutex type.

struct CondvarWaitEntry : WaitEntryBase {};

struct CondvarSignalTarget {
    enum class Type {
        Any,
        Specific,
        All
    } type;
    SceUID thread_id; // for Specific
    explicit CondvarSignalTarget(Type type)
        : type(type)
        , thread_id(0) {}
    CondvarSignalTarget(Type type, SceUID thread_id)
        : type(type)
        , thread_id(thread_id) {}
};

template <SyncWeight W>
class CondvarT : public SyncObjectBase {
public:
    static constexpr bool is_lw = (W == SyncWeight::Light);

    using PairedMutex = std::conditional_t<is_lw, LwMutex, Mutex>;
    using PairedMutexPtr = std::shared_ptr<PairedMutex>;
    using SignalTarget = CondvarSignalTarget;

    CondvarT(SceUID uid, std::string name, SceUInt32 attr, PairedMutexPtr associated_mutex)
        : SyncObjectBase(uid, std::move(name), attr)
        , associated_mutex(std::move(associated_mutex))
        , waiters(wait_order_from_attr(this->associated_mutex->attr)) {}

    WaitResult wait(ThreadStatePtr t, Deadline d, MemState &mem, bool cb);
    SceInt32 signal(SignalTarget target);
    void mark_deleted();

    const PairedMutexPtr &mutex_obj() const { return associated_mutex; }
    std::size_t num_waiters() const { return waiters.size(); }

private:
    static constexpr SceInt32 delete_error_code = is_lw
        ? SCE_KERNEL_ERROR_WAIT_DELETE_LW_COND
        : SCE_KERNEL_ERROR_WAIT_DELETE_COND;

    WaitInfo wait_info(bool cb) const {
        constexpr SceUInt32 type = is_lw ? SCE_KERNEL_WAITTYPE_LW_COND_SIGNAL : SCE_KERNEL_WAITTYPE_COND_SIGNAL;
        constexpr SceUInt32 type_cb = is_lw ? SCE_KERNEL_WAITTYPE_LW_COND_SIGNAL_CB : SCE_KERNEL_WAITTYPE_COND_SIGNAL_CB;
        return {
            .type = cb ? type_cb : type,
            .uid = uid,
            .reason = is_lw ? "lwcond" : "cond"
        };
    }

    PairedMutexPtr associated_mutex;
    WaitQueue<CondvarWaitEntry> waiters;
};

using Condvar = CondvarT<SyncWeight::Heavy>;
using LwCondVar = CondvarT<SyncWeight::Light>;
using CondvarPtr = std::shared_ptr<Condvar>;
using LwCondVarPtr = std::shared_ptr<LwCondVar>;
using CondvarPtrs = std::map<SceUID, CondvarPtr>;
using LwCondVarPtrs = std::map<SceUID, LwCondVarPtr>;


struct EventFlagWaitEntry : WaitEntryBase {
    SceUInt32 bit_pattern = 0;
    SceUInt32 wait_mode = 0;
    SceUInt32 *out_bits = nullptr; // signaler writes the pre-clear pattern here
};

class EventFlag : public SyncObjectBase {
public:
    EventFlag(SceUID uid, std::string name, SceUInt32 attr, SceUInt32 initial_pattern)
        : SyncObjectBase(uid, std::move(name), attr)
        , pattern(initial_pattern)
        , waiters(wait_order_from_attr(attr)) {}

    WaitResult wait(ThreadStatePtr t, SceUInt32 bits, SceUInt32 wait_mode,
        SceUInt32 *p_result, Deadline d, bool cb);
    SceInt32 poll(SceUInt32 bits, SceUInt32 wait_mode, SceUInt32 *p_result);
    SceInt32 set(SceUInt32 bits);
    SceInt32 clear(SceUInt32 bits);
    SceUInt32 cancel(SceUInt32 new_pattern);
    void mark_deleted();

    SceUInt32 current_pattern() const { return pattern; }
    std::size_t num_waiters() const { return waiters.size(); }

private:
    static bool satisfies(SceUInt32 cur, SceUInt32 bits, SceUInt32 mode);
    void apply_clear_locked(SceUInt32 wait_mode, SceUInt32 bits);
    bool single_attr() const { return (attr & 0x1000) != 0; }

    WaitInfo wait_info(bool cb) const {
        return {
            .type = cb ? SCE_KERNEL_WAITTYPE_EVENTFLAG_CB : SCE_KERNEL_WAITTYPE_EVENTFLAG,
            .uid = uid,
            .reason = "eventflag"
        };
    }

    SceUInt32 pattern;
    WaitQueue<EventFlagWaitEntry> waiters;
};
using EventFlagPtr = std::shared_ptr<EventFlag>;
using EventFlagPtrs = std::map<SceUID, EventFlagPtr>;


struct RWLockWaitEntry : WaitEntryBase {
    bool is_write = false;
};

class RWLock : public SyncObjectBase {
public:
    RWLock(SceUID uid, std::string name, SceUInt32 attr)
        : SyncObjectBase(uid, std::move(name), attr)
        , waiters(wait_order_from_attr(attr)) {}

    struct CancelCounts {
        SceUInt32 read;
        SceUInt32 write;
    };

    WaitResult lock(ThreadStatePtr t, bool is_write, Deadline d);
    SceInt32 try_lock(ThreadStatePtr t, bool is_write);
    SceInt32 unlock(ThreadStatePtr t, bool is_write);
    CancelCounts cancel(ThreadStatePtr caller, SceInt32 flag);
    void mark_deleted();

private:
    enum class State {
        Unlocked,
        ReadLocked,
        WriteLocked,
    };

    std::optional<SceInt32> try_acquire_locked(ThreadStatePtr t, bool is_write);

    WaitInfo wait_info() const {
        return {
            .type = SCE_KERNEL_WAITTYPE_RW_LOCK,
            .uid = uid,
            .reason = "rwlock"
        };
    }

    State state = State::Unlocked;
    std::map<ThreadStatePtr, int> owners;
    WaitQueue<RWLockWaitEntry> waiters;
};
using RWLockPtr = std::shared_ptr<RWLock>;
using RWLockPtrs = std::map<SceUID, RWLockPtr>;


struct SimpleEventWaitEntry : WaitEntryBase {
    SceUInt32 wait_pattern = 0;
    SceUInt32 *result_pattern = nullptr;
    SceUInt64 *user_data = nullptr;
};

class SimpleEvent : public SyncObjectBase {
public:
    SimpleEvent(SceUID uid, std::string name, SceUInt32 attr, SceUInt32 init_pattern)
        : SyncObjectBase(uid, std::move(name), attr)
        , pattern(init_pattern)
        , auto_reset((attr & SCE_KERNEL_EVENT_ATTR_AUTO_RESET) != 0)
        , waiters(wait_order_from_attr(attr)) {}

    WaitResult wait_or_poll(ThreadStatePtr t, SceUInt32 wait_pattern, SceUInt32 *result_pattern,
        SceUInt64 *user_data, Deadline d, bool is_wait, bool alertable);
    SceInt32 set_or_pulse(SceUInt32 pattern_to_set, SceUInt64 user_data, bool is_set);
    SceInt32 clear(SceUInt32 clear_pattern);
    SceUInt32 cancel();
    void mark_deleted();

    SceUInt32 current_pattern() const { return pattern; }

private:
    WaitInfo wait_info(bool cb) const {
        return {
            .type = cb ? SCE_KERNEL_WAITTYPE_EVENT_CB : SCE_KERNEL_WAITTYPE_EVENT,
            .uid = uid,
            .reason = "event"
        };
    }

    SceUInt32 pattern;
    SceUInt64 user_data = 0;
    bool auto_reset;
    WaitQueue<SimpleEventWaitEntry> waiters;
};
using SimpleEventPtr = std::shared_ptr<SimpleEvent>;
using SimpleEventPtrs = std::map<SceUID, SimpleEventPtr>;


// Uses an internal condvar instead of WaitQueue / park because timer waits
// are bounded by the timer's next_event deadline, which advances on each tick.
class Timer : public SyncObjectBase {
public:
    Timer(SceUID uid, std::string name, SceUInt32 attr)
        : SyncObjectBase(uid, std::move(name), attr) {}

    SceInt32 set(SceInt32 type, SceKernelSysClock *interval, SceInt32 repeats);
    SceInt32 wait_or_poll(ThreadStatePtr t, SceUInt32 *result_pattern,
        SceUInt64 *user_data, bool is_wait, bool alertable);
    SceInt32 clear();
    SceInt32 start();
    SceInt32 stop();
    void mark_deleted();

    uint64_t start_time() const { return time; }
    void set_start_time(uint64_t t) { time = t; }

    // Wakes every parked waiter. Used by process_exit on emulator shutdown.
    void notify_all_waiters() { condvar.notify_all(); }

private:
    void schedule_next_event_locked();

    std::condition_variable condvar;
    bool is_started = false;
    bool is_repeat = false;
    bool is_pulse = false;
    bool event_set = false;
    uint64_t time = 0;
    uint64_t next_event = 0;
    uint64_t event_interval = 0;
};
using TimerPtr = std::shared_ptr<Timer>;
using TimerPtrs = std::map<SceUID, TimerPtr>;


struct MsgPipeRecvWaitEntry : WaitEntryBase {
    SceSize request_size = 0; // ASAP = 1, FULL = recv_size
};
struct MsgPipeSendWaitEntry : WaitEntryBase {
    SceSize request_size = 0;
};

class MsgPipe : public SyncObjectBase {
public:
    MsgPipe(SceUID uid, std::string name, SceUInt32 attr, SceSize buf_size)
        : SyncObjectBase(uid, std::move(name), attr)
        , data_buffer(buf_size)
        , receivers(wait_order_from_attr(attr))
        , senders(WaitOrder::FIFO) {} // senders always FIFO. Receivers honor attr

    struct CancelCounts {
        SceUInt32 send;
        SceUInt32 recv;
    };

    std::expected<SceSize, ExitSignal> recv(ThreadStatePtr t, void *p_recv,
        SceSize recv_size, SceUInt32 wait_mode, Deadline d, bool cb);
    std::expected<SceSize, ExitSignal> send(ThreadStatePtr t, const void *p_send,
        SceSize send_size, SceUInt32 wait_mode, Deadline d, bool cb);
    CancelCounts cancel();
    SceInt32 mark_deleted_and_drain();

private:
    void wake_one_receiver_locked();
    void wake_one_sender_locked();

    WaitInfo recv_wait_info(bool cb) const {
        return {
            .type = cb ? SCE_KERNEL_WAITTYPE_MSG_PIPE_CB : SCE_KERNEL_WAITTYPE_MSG_PIPE,
            .uid = uid,
            .reason = "msgpipe_recv"
        };
    }
    WaitInfo send_wait_info(bool cb) const {
        return {
            .type = cb ? SCE_KERNEL_WAITTYPE_MSG_PIPE_CB : SCE_KERNEL_WAITTYPE_MSG_PIPE,
            .uid = uid,
            .reason = "msgpipe_send"
        };
    }

    ByteRingBuffer data_buffer;
    WaitQueue<MsgPipeRecvWaitEntry> receivers;
    WaitQueue<MsgPipeSendWaitEntry> senders;
};
using MsgPipePtr = std::shared_ptr<MsgPipe>;
using MsgPipePtrs = std::map<SceUID, MsgPipePtr>;
