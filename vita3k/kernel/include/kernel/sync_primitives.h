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

#include <kernel/kernel_object.h>
#include <kernel/thread/wait.h>
#include <kernel/types.h>
#include <util/byte_ring_buffer.h>
#include <util/intrusive_list.h>

#include <array>
#include <bitset>
#include <expected>
#include <limits>
#include <memory>
#include <span>
#include <string>

// Returns the error for an invalid sync object name or for attributes outside allowed_attr, or 0
SceInt32 check_object_name(const char *name, SceUInt32 attr, SceUInt32 allowed_attr = ~0U);

// The ID of the one process Vita3K runs
constexpr SceUID PROCESS_ID = 1;

// Returns the user data of a system event a thread caused, like a delete: its UID in the high word
// and the process ID in the low word.
constexpr SceUInt64 system_event_user_data(SceUID thread_id) {
    return (static_cast<SceUInt64>(thread_id) << 32) | static_cast<SceUInt32>(PROCESS_ID);
}

// The base of the sync objects, with their attributes and their name cut to the maximum name
// length
class SyncObject : public KernelObject {
public:
    SyncObject(SceUInt32 attr, const char *name);

    const SceUInt32 attr;
    const std::string name;

protected:
    // The order of a wait queue, by priority with SCE_KERNEL_ATTR_TH_PRIO, else first come first served
    QueueOrder queue_order() const { return attr & SCE_KERNEL_ATTR_TH_PRIO ? QueueOrder::priority : QueueOrder::fifo; }
};

struct ThreadState;
class MultiEventWait;

// The base of simple events, timers and message pipes. It holds the bit pattern, the user data of
// the last set and the queue of waiting threads.
class Event : public SyncObject {
public:
    // The create event comes with the creating thread's user data
    Event(SceUInt32 attr, const char *name, SceUInt32 init_pattern, SceUID creator);

    // Waits until any bit of wait_pattern is set. Writes the matched bits to result_pattern, the
    // user data to user_data and the remaining time back to the timeout.
    SceInt32 wait(ThreadState &thread, SceUInt32 wait_pattern, SceUInt32 *result_pattern, SceUInt64 *user_data, WaitOptions &options);
    SceInt32 poll(SceUInt32 wait_pattern, SceUInt32 *result_pattern, SceUInt64 *user_data);
    SceInt32 clear(SceUInt32 clear_pattern);
    // Wakes every waiter with WAIT_CANCEL. A timer also disarms its pending event.
    SceInt32 cancel(SceUInt32 *num_wait_threads);
    // Fills info, or returns UNKNOWN_EVENT_ID if it was deleted.
    SceInt32 info(SceKernelEventInfo &info);
    // Sets one of the system events, like the open event, with user_data for its waiters, or returns
    // UNKNOWN_EVENT_ID if it was deleted.
    SceInt32 set_system_event(SceUInt32 event, SceUInt64 user_data);

protected:
    // The request of a single event wait, or of one entry of a multiple event wait
    struct Request {
        SceUInt32 pattern = 0;
        // Output pointers of a single wait, owned by the caller
        SceUInt32 *result_pattern = nullptr;
        SceUInt64 *user_data = nullptr;
        // The multiple event wait and entry index this request reports to, null for a single wait
        MultiEventWait *multi = nullptr;
        SceUInt32 index = 0;
    };

    // A single event wait, which fires the timer events due while it sleeps
    class Wait;
    // Hands the bits to the waiters they match
    void serve();

    // Queues, watches and removes the nodes of its entries
    friend class MultiEventWait;

    // Takes the lock. Completes the entry if any matching bits are set, else queues its node when
    // queue is true. Returns SCE_KERNEL_OK if the entry completed, EVENT_COND if not, or
    // UNKNOWN_EVENT_ID if the event was deleted.
    SceInt32 scan_token(WaitNode<Request> &token, bool queue);
    // Takes the lock. Removes the node of an entry if it is still queued, then serves the waiters
    // behind it.
    void remove_token(WaitNode<Request> &token);
    // Takes the lock and runs fire_due(), or returns the maximum if the event was deleted
    Deadline watch_timer(Deadline until);

    // Timer events. Only a timer overrides these.
    // Fires the timer event if it is due, before an operation reads the bits
    virtual void catch_up() {}
    // Fires the timer event if it is due by until, and returns when the next one is due, or the
    // maximum if none will be
    virtual Deadline fire_due(Deadline until) { return Deadline::max(); }
    // Drops the pending timer event, for a cancel
    virtual void disarm() {}

    // Sets or pulses the bits and the user data, and wakes the waiters they match
    void fire(SceUInt32 set_pattern, SceUInt64 user_data, bool is_set);
    // Fires the delete event, then wakes the remaining waiters with WAIT_DELETE
    void on_delete(SceUID deleter) override;
    // Hands the matching bits to the request if any are set, and clears them on an auto reset
    // event. Returns whether any were set. If the multiple event wait takes no more results, the
    // bits stay set.
    bool take(const Request &request);
    // Ends every wait with the given result, for a delete or a cancel. Returns how many it ended.
    std::size_t end_waits(SceInt32 result);

    // Guarded by the lock
    WaitQueue<Request> waiters{ queue_order() };
    SceUInt32 pattern;
    SceUInt64 last_user_data;
};

// A shared pointer to a simple event, timer or message pipe, as an Event
using EventPtr = std::shared_ptr<Event>;

class SimpleEvent final : public WithUidClass<Event, UidClass::simple_event> {
public:
    using WithUidClass::WithUidClass;

    // Returns the error a create with these arguments gets, or SCE_KERNEL_OK
    static SceInt32 check_create(const char *name, SceUInt32 attr, SceUInt32 init_pattern);

    // Sets the bits and the user data, and wakes the waiters they match. The bits stay set if
    // is_set, else they are cleared again for a pulse.
    SceInt32 set(SceUInt32 set_pattern, SceUInt64 user_data, bool is_set);
    // Wakes every waiter with WAIT_CANCEL, then replaces the whole pattern and the user data.
    SceInt32 cancel(SceUInt32 set_pattern, SceUInt64 user_data, SceUInt32 *num_wait_threads);
    // Sets result to the event's bit pattern, or returns UNKNOWN_EVENT_ID if it was deleted.
    SceInt32 get_pattern(SceUInt32 &result);
};

using SimpleEventPtr = std::shared_ptr<SimpleEvent>;

class Timer final : public WithUidClass<Event, UidClass::timer> {
public:
    Timer(SceUInt32 attr, const char *name, SceUID creator);

    SceInt32 set_event(SceUID type, SceKernelSysClock interval, SceInt32 repeats);
    SceInt32 start();
    SceInt32 stop();
    // Replaces the count and returns its old value, or the error.
    std::expected<uint64_t, SceInt32> set_time(uint64_t new_time);
    // Sets time to how much time the timer counted, or returns UNKNOWN_TIMER_ID if it was deleted.
    SceInt32 get_time(uint64_t &time) const;
    // Sets time to the clock time the count runs from, 0 while the timer is stopped, or returns
    // UNKNOWN_TIMER_ID if it was deleted.
    SceInt32 get_base(uint64_t &time) const;
    // Sets time to how much time the armed event has left, or returns EVENT_NOT_SET with no event
    // armed or UNKNOWN_TIMER_ID if it was deleted.
    SceInt32 remaining_time(uint64_t &time);
    // Fills info, or returns UNKNOWN_TIMER_ID if it was deleted.
    SceInt32 info(SceKernelTimerInfo &info);

private:
    // Fires the event if it is due: sets or pulses SCE_KERNEL_EVENT_TIMER, and queues a repeating
    // event again an interval after it was due
    void catch_up() override;
    Deadline fire_due(Deadline until) override;
    void disarm() override;
    // Queues the event to be due at the clock time deadline, and wakes the waiters so they see the
    // new time
    void queue_event(uint64_t deadline);
    // Returns when the next event is due on the clock of wait deadlines, even if that time already
    // passed, or the maximum if none is queued
    Deadline event_deadline() const;
    // Returns how much time the timer counted
    uint64_t count() const;

    // Guarded by the lock
    bool is_started = false;
    // An event is set, and a fired one shot event stays armed
    bool armed = false;
    bool is_repeat = false;
    bool is_pulse = false;
    // The clock time the count runs from while the timer is started, 0 if it never started
    uint64_t base = 0;
    // The count while the timer is stopped
    uint64_t stopped_count = 0;
    // The count the armed event is due at
    uint64_t schedule = 0;
    // The clock time the next event is due, the maximum while none is queued
    uint64_t next_event = std::numeric_limits<uint64_t>::max();
    uint64_t event_interval = 0;
};

using TimerPtr = std::shared_ptr<Timer>;

// One sceKernelWaitMultipleEvents call and the thread's wait for it. It has one entry per listed
// event, each with a node to queue on that event, and records how the entries ended. It lives on
// the waiting thread's stack. The nodes are queued only between scan() and dequeue(). The waiting
// thread's mutex guards the results and the decisions based on them. An event that completes an
// entry takes that mutex inside its own lock.
class MultiEventWait final : public ThreadWait {
public:
    // Whether one event ends the wait, or it needs all of them
    enum class Mode {
        any,
        all,
    };

    // Keeps the time left in options, which outlive it
    MultiEventWait(KernelState &kernel, ThreadState &thread, std::span<const SceKernelWaitEvent> list, Mode mode, WaitOptions &options);

    // Waits for any or all of the events
    WaitResult run();
    // Returns the results of the entries that ended, in the order they ended, with the UIDs the
    // caller listed. Call after run(). Valid while this object lives.
    std::span<const SceKernelResultEvent> results() const;

private:
    // Event reports entry results through accept()
    friend class Event;

    // One listed event
    struct Entry {
        Entry(ThreadWait &wait, Event::Request request)
            : token(wait, std::move(request)) {}

        // The UID reported in the result. Entries that name the same object all report the UID of
        // the first one.
        SceUID result_uid;
        // The event, or null if the UID doesn't name one
        EventPtr event;
        // The node queued on the event, with this entry's pattern
        WaitNode<Event::Request> token;
    };

    // Records the result of entry index, and ends the wait if this entry completes it. Returns
    // false if the entry already ended or the wait takes no more results, so the event keeps its
    // bits. Takes the thread lock, which goes after the event lock.
    bool accept(SceUInt32 index, SceInt32 result, SceUInt32 bits, SceUInt64 user_data = 0);

    // Fires the due timer events of the listed events, and returns when to look at the clock again
    Deadline watch(Deadline until) override;
    // Completes the entries whose event is set or deleted and queues the others, then starts the
    // wait. In Mode::any it stops queueing once an entry completes. Returns the result instead if
    // the entries already end the wait, the deadline passed or the thread is exiting.
    std::optional<WaitResult> scan();
    // Removes the queued nodes from their events
    void dequeue();

    const Mode mode;
    const SceUInt32 count;
    // The constructor builds the first count entries in place, since a node can't be copied and
    // needs its thread
    std::array<std::optional<Entry>, SCE_KERNEL_MULTIPLE_EVENTS_MAX> entries;

    // Guarded by the waiting thread's mutex
    // Set while scan() runs. Entries that end during the scan only record their results.
    bool scanning = false;
    // The entries that ended. Their results fill the start of ended, in the order they ended.
    std::bitset<SCE_KERNEL_MULTIPLE_EVENTS_MAX> finished;
    std::array<SceKernelResultEvent, SCE_KERNEL_MULTIPLE_EVENTS_MAX> ended{};
};

class Semaphore final : public WithUidClass<SyncObject, UidClass::semaphore> {
public:
    const SceInt32 init_val;
    const SceInt32 max;

    Semaphore(SceUInt32 attr, const char *name, SceInt32 init_val, SceInt32 max_val);

    // Returns the error a create with these arguments gets, or SCE_KERNEL_OK
    static SceInt32 check_create(const char *name, SceUInt32 attr, SceInt32 init_val, SceInt32 max_val);

    SceInt32 wait(ThreadState &thread, SceInt32 need_count, WaitOptions &options);
    SceInt32 poll(ThreadState &thread, SceInt32 need_count);
    SceInt32 signal(SceInt32 count);
    SceInt32 cancel(SceInt32 set_count, SceUInt32 *num_wait_threads);
    // Fills info with what the semaphore's info reports, or returns UNKNOWN_SEMA_ID if it was deleted.
    SceInt32 info(SceKernelSemaInfo &info) const;

private:
    struct Request {
        SceInt32 need_count;
    };

    // Wakes waiters in queue order while the count covers them, and stops at the first it can't
    void serve();

    void on_delete(SceUID deleter) override;
    // Takes need_count if the count covers it and no waiter is queued ahead of thread
    bool try_take(const ThreadState &thread, SceInt32 need_count);

    // Guarded by the lock
    WaitQueue<Request> waiters{ queue_order() };
    SceInt32 val;
};

using SemaphorePtr = std::shared_ptr<Semaphore>;

class Mutex;
using MutexPtr = std::shared_ptr<Mutex>;

class Condvar : public SyncObject {
public:
    const MutexPtr associated_mutex;
    const Ptr<SceKernelLwCondWork> workarea;

    Condvar(SceUInt32 attr, const char *name, MutexPtr associated_mutex, Ptr<SceKernelLwCondWork> workarea = {});
    ~Condvar() override;
    // Adds the condition variable to its mutex's list, so deleting the mutex reaches it. Returns
    // the unknown mutex error if the mutex was deleted. Call once after construction.
    SceInt32 attach();

    SceInt32 wait(MemState &mem, ThreadState &thread, WaitOptions &options);
    // Wakes the first waiter in queue order, if any
    SceInt32 signal();
    // Wakes every waiter
    SceInt32 signal_all();
    // Wakes the waiting thread thread_id, or returns ILLEGAL_THREAD_ID if it doesn't wait here
    SceInt32 signal_to(SceUID thread_id);
    // Fills the info of a heavy or a lightweight condition variable, or returns its unknown ID error if it was deleted.
    SceInt32 info(SceKernelCondInfo &info) const;
    SceInt32 info(SceKernelLwCondInfo &info) const;

private:
    // A condition wait. It takes nothing and only waits for a signal.
    struct Request {};

    // Guarded by the lock
    WaitQueue<Request> waiters{ queue_order() };

    friend class Mutex;

    // A condition uses the lock of its mutex, so its waits change together with the mutex state
    std::mutex &object_mutex() const override;
    void on_delete(SceUID deleter) override;
    bool lightweight() const { return get_uid_class() == UidClass::lw_cond; }

    // Link in the mutex's list of condition variables
    IntrusiveLink<Condvar> mutex_link;
};

using CondvarPtr = std::shared_ptr<Condvar>;

// The two kinds of condition variable, told apart by their UID class
using LwCond = WithUidClass<Condvar, UidClass::lw_cond>;
using HeavyCond = WithUidClass<Condvar, UidClass::cond>;

class Mutex : public SyncObject {
public:
    const SceInt32 init_count;
    const Ptr<SceKernelLwMutexWork> workarea;

    Mutex(SceUInt32 attr, const char *name, SceInt32 init_count, SceUID creator, Ptr<SceKernelLwMutexWork> workarea = {});
    // Returns the error for a name, attributes or count the caller can't use, or 0. Kernel callers may
    // use attributes games can't.
    static SceInt32 check_create(const char *name, SceUInt32 attr, SceInt32 init_count, bool kernel);

    // Locks the mutex. A relock is the mutex wait at the end of a condition wait, which runs the
    // callbacks notified during it only if a delete ends it.
    SceInt32 acquire(MemState &mem, ThreadState &thread, SceInt32 count, WaitOptions &options, bool relock = false);
    SceInt32 try_acquire(MemState &mem, ThreadState &thread, SceInt32 count);
    SceInt32 release(MemState &mem, ThreadState &thread, SceInt32 unlock_count);
    SceInt32 cancel(ThreadState &thread, SceInt32 new_count, SceUInt32 *num_wait_threads);
    // Gives the mutex to its next waiter if thread, which is ending, owns it.
    void release_ended_owner(const ThreadState &thread);
    // Fills the info of a heavy or a lightweight mutex, or returns its unknown ID error if it was deleted.
    SceInt32 info(SceKernelMutexInfo &info) const;
    SceInt32 info(SceKernelLwMutexInfo &info) const;

private:
    // A mutex lock, or the relock at the end of a condition wait
    struct Request {
        SceInt32 lock_count;
    };

    // Gives a free mutex to the first waiter, if there is one
    void serve();

    // Guarded by the lock
    WaitQueue<Request> waiters{ queue_order() };
    SceInt32 lock_count;
    // The UID of the thread that holds it, or 0. A UID and not the thread, since a lightweight mutex
    // stays locked by an owner that ended and was deleted.
    SceUID owner;
    // The condition variables that use this mutex
    IntrusiveList<Condvar, &Condvar::mutex_link> conditions;

    friend class Condvar;

    void on_delete(SceUID deleter) override;
    bool lightweight() const { return get_uid_class() == UidClass::lw_mutex; }
    // Unlocks the mutex unlock_count times for thread, or returns why it can't
    SceInt32 give_back(MemState &mem, ThreadState &thread, SceInt32 unlock_count);
    // Returns ILLEGAL_COUNT for a lock or unlock count the mutex can't take, or SCE_KERNEL_OK
    SceInt32 check_count(SceInt32 count) const;
    // Takes the mutex if it is free or owned by thread, or returns why it can't
    SceInt32 try_take(MemState &mem, const ThreadState &thread, SceInt32 count);
    // Writes the owner and lock count to a lightweight mutex's work area, which the guest reads
    void update_workarea(MemState &mem);
};

// The two kinds of mutex, told apart by their UID class
using LwMutex = WithUidClass<Mutex, UidClass::lw_mutex>;
using HeavyMutex = WithUidClass<Mutex, UidClass::mutex>;

class RWLock final : public WithUidClass<SyncObject, UidClass::rw_lock> {
public:
    using WithUidClass::WithUidClass;

    // Returns the error a create with these arguments gets, or SCE_KERNEL_OK
    static SceInt32 check_create(const char *name, SceUInt32 attr);

    // is_write picks the write lock over a read lock
    SceInt32 acquire(ThreadState &thread, bool is_write, WaitOptions &options);
    SceInt32 try_acquire(ThreadState &thread, bool is_write);
    SceInt32 release(ThreadState &thread, bool is_write);
    SceInt32 cancel(ThreadState &thread, SceUInt32 *num_read_wait_threads, SceUInt32 *num_write_wait_threads, SceInt32 flag);
    // Releases the write lock if thread, which is ending, holds it. A read lock stays held, like on
    // a Vita.
    void release_ended_writer(const ThreadState &thread);
    // Fills info with what the rwlock's info reports, or returns UNKNOWN_RW_LOCK_ID if it was deleted.
    SceInt32 info(SceKernelRWLockInfo &info) const;

private:
    // A waiter asks for nothing more than its queue says
    struct Request {};

    // Gives the lock to the waiters that get it next: every waiting reader while no writer holds or
    // waits for it, or else the first waiting writer once nobody holds it
    void serve();

    void on_delete(SceUID deleter) override;
    // Takes the lock if it is free for this kind of lock, or returns why it can't
    SceInt32 try_take(ThreadState &thread, bool is_write);

    // Guarded by the lock
    // The read locks of every reader, or the writer's count. Readers aren't tracked one by one, like
    // on a Vita, so any thread may read unlock while readers hold it.
    SceInt32 lock_count = 0;
    // The UID of the thread that holds the write lock, or 0
    SceUID write_owner = 0;
    // The threads waiting for a read lock and for the write lock, each in the lock's queue order
    WaitQueue<Request> readers{ queue_order() };
    WaitQueue<Request> writers{ queue_order() };
};

using RWLockPtr = std::shared_ptr<RWLock>;

class EventFlag final : public WithUidClass<SyncObject, UidClass::event_flag> {
public:
    // The pattern the event flag was created with, for its info
    const SceUInt32 init_pattern;

    EventFlag(SceUInt32 attr, const char *name, SceUInt32 init_pattern);

    // Returns the error a create with these arguments gets, or SCE_KERNEL_OK
    static SceInt32 check_create(const char *name, SceUInt32 attr);

    SceInt32 wait(ThreadState &thread, SceUInt32 pattern, SceUInt32 wait_mode, SceUInt32 *out_bits, WaitOptions &options);
    SceInt32 poll(SceUID thread_id, SceUInt32 pattern, SceUInt32 wait_mode, SceUInt32 *out_bits);
    SceInt32 set(SceUInt32 pattern);
    SceInt32 clear(SceUInt32 pattern);
    SceInt32 cancel(SceUInt32 pattern, SceUInt32 *num_wait_threads);
    // Fills info, or returns UNKNOWN_EVF_ID if it was deleted.
    SceInt32 info(SceKernelEventFlagInfo &info) const;
    // Drops the holds of thread, which is ending.
    void release_ended_holder(const ThreadState &thread);

private:
    struct Request {
        SceUInt32 wait_mode;
        SceUInt32 pattern;
        SceUInt32 *out_bits;
    };

    // Wakes the waiters the flags match, in queue order
    void serve();

    void on_delete(SceUID deleter) override;
    // Returns whether another thread's wait holds the flag
    bool held_by_other(SceUID thread_id) const;
    // If the flags match the request's pattern, writes them to its out bits, clears the ones its
    // mode asks for and returns true. Otherwise changes nothing.
    bool take(const Request &request);
    // Gives back one hold of the holder, the wait that took it ended
    void drop_hold();

    // Guarded by the lock
    WaitQueue<Request> waiters{ queue_order() };
    SceUInt32 flags;
    // Without SCE_EVENT_WAITMULTIPLE, the thread whose waits hold the flag and how many waits hold
    // it. Other threads can't wait on or poll a held flag. A wait releases its hold when it gets the
    // bits, when a cancel or delete wakes it, or when it times out in the queue. A wait that ends
    // before sleeping without getting the bits, like one with a timeout of 0, keeps its hold until
    // the thread ends or the flag is deleted.
    SceUID holder = 0;
    int holds = 0;
};

using EventFlagPtr = std::shared_ptr<EventFlag>;

// One piece of a message pipe transfer, a buffer in guest memory
struct MsgPipeVector {
    Ptr<void> base;
    SceSize size;
};

// A message pipe. It also works as an event and fires create and delete events.
class MsgPipe final : public WithUidClass<Event, UidClass::msg_pipe> {
public:
    MsgPipe(SceUInt32 attr, const char *name, std::size_t buf_size, SceUID creator);

    // Returns the error a create with these arguments gets, or SCE_KERNEL_OK
    static SceInt32 check_create(const char *name, SceUInt32 type, SceUInt32 attr, SceSize buf_size);

    // Both move the bytes of the pieces in order, waiting until a FULL call moved all of them or an
    // ASAP call any. The size moved so far is kept in size, which may be null.
    SceInt32 receive(ThreadState &thread, MemState &mem, SceUInt32 wait_mode, std::span<const MsgPipeVector> pieces, SceSize *size, WaitOptions &options);
    SceInt32 send(ThreadState &thread, MemState &mem, SceUInt32 wait_mode, std::span<const MsgPipeVector> pieces, SceSize *size, WaitOptions &options);
    // Both move what they can at once, without waiting
    SceInt32 try_receive(ThreadState &thread, MemState &mem, SceUInt32 wait_mode, std::span<const MsgPipeVector> pieces, SceSize *size);
    SceInt32 try_send(ThreadState &thread, MemState &mem, SceUInt32 wait_mode, std::span<const MsgPipeVector> pieces, SceSize *size);
    SceInt32 cancel(SceUInt32 *num_send_wait_threads, SceUInt32 *num_receive_wait_threads);
    // Fills info, or returns UNKNOWN_MSG_PIPE_ID if it was deleted.
    SceInt32 info(SceKernelMppInfo &info) const;

private:
    // A send or receive. It holds the guest buffers and the progress, which carries over between
    // passes of the wait.
    struct Transfer {
        MemState &mem;
        // The pieces not moved yet, the first one from offset on
        std::span<const MsgPipeVector> pieces;
        // Where the caller gets the size moved, or null
        SceSize *size;
        SceSize total;
        bool full;
        // A DONT_REMOVE receive, which copies bytes without taking them
        bool peek = false;
        SceSize left = total;
        SceSize offset = 0;

        // Returns whether it moved all the bytes its mode requires
        bool done() const { return full ? left == 0 : left < total; }
        SceSize moved() const { return total - left; }
        // Returns the guest bytes at its position, at most n and within one piece
        std::span<char> run(SceSize n);
        void advance(SceSize n) {
            offset += n;
            left -= n;
        }
        void report() const {
            if (size)
                *size = moved();
        }
    };

    // The transfer a call asks for, or ILLEGAL_MODE or ILLEGAL_SIZE for a bad call
    std::expected<Transfer, SceInt32> transfer_for(MemState &mem, SceUInt32 wait_mode, std::span<const MsgPipeVector> pieces, SceSize *size) const;

    // Continues the transfers of the first receiver and the first sender until neither can finish,
    // then wakes the event waiters the bits match
    void serve();

    // Whether the queue's head got part of its transfer. It then stays at the head.
    static bool partly_served(WaitQueue<Transfer> &queue) {
        const auto *const head = queue.front();
        return head && head->request.moved() > 0;
    }
    // Whether a thread of its priority would be first in the queue. Nothing goes ahead of a partly
    // served head.
    static bool first_in_line(WaitQueue<Transfer> &queue, const ThreadState &thread);

    void on_delete(SceUID deleter) override;
    // Moves the sender's bytes to the waiting receivers, then puts the rest into the pipe. Returns
    // whether the sender is done.
    bool give(Transfer &sender);
    // Moves bytes to the receiver from the pipe, then from the waiting senders, which put their rest
    // into the pipe. Returns whether the receiver is done.
    bool take(Transfer &receiver);
    // Puts as much of the sender's rest into the pipe as its mode allows
    void fill(Transfer &sender);
    // Moves the pipe's bytes to the receiver, up to what it still wants
    void drain(Transfer &receiver);
    // Returns the bytes a receive can get at once, the pipe's and those the waiting senders still have
    SceSize available() const;
    // Copies n bytes from the sender to the receiver, and takes them from the sender unless the
    // receiver peeks
    static void copy(Transfer &sender, Transfer &receiver, SceSize n);
    // Posts the end of a send or receive as a pipe event. Sets IN for a send or OUT for a receive
    // if bytes moved, and the error bit if it failed for any reason other than a suspend or a stop.
    // The user data holds the result and the size moved. A send that leaves bytes in the pipe sets
    // DATA_EXIST, and a receive that empties the pipe clears it.
    void post(bool sending, SceInt32 result, SceSize moved);
    // Takes the lock. The blocking part of a send or a receive: moves the bytes, waiting as needed,
    // then posts the pipe event.
    SceInt32 transfer(ThreadState &thread, bool sending, const Transfer &request, WaitOptions &options);
    // Takes the lock. Moves what a send or receive can at once, without waiting, and posts the pipe
    // event.
    SceInt32 try_transfer(ThreadState &thread, bool sending, Transfer &request);

    // Guarded by the lock. At most one side waits, except right after a waiter came back from its
    // callbacks.
    WaitQueue<Transfer> senders;
    WaitQueue<Transfer> receivers;
    ByteRingBuffer data_buffer;
};

using MsgPipePtr = std::shared_ptr<MsgPipe>;

// Returns the Event of the simple event, timer or message pipe with this UID, or null
EventPtr find_event(const KernelObjects &objects, SceUID uid);
