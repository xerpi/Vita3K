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

#include <kernel/thread/thread_state.h>
#include <kernel/types.h>
#include <util/align.h>
#include <util/log.h>

#include <algorithm>
#include <tuple>

SceInt32 check_object_name(const char *name, SceUInt32 attr, SceUInt32 allowed_attr) {
    if (!name)
        return SCE_KERNEL_ERROR_ILLEGAL_ADDR;
    if ((strlen(name) > KERNELOBJECT_MAX_NAME_LENGTH) && (attr & SCE_KERNEL_ATTR_OPENABLE))
        return SCE_KERNEL_ERROR_UID_NAME_TOO_LONG;
    if (attr & ~allowed_attr)
        return SCE_KERNEL_ERROR_ILLEGAL_ATTR;
    return SCE_KERNEL_OK;
}

SyncObject::SyncObject(SceUInt32 attr, const char *name)
    : attr(attr)
    , name(name, strnlen(name, KERNELOBJECT_MAX_NAME_LENGTH)) {}

// ************************
// * Multiple event waits *
// ************************

EventPtr find_event(const KernelObjects &objects, SceUID uid) {
    return std::dynamic_pointer_cast<Event>(objects.find(uid));
}

// The wait UID reported by a thread in a multiple event wait. It matches none of its events.
constexpr SceUID MULTI_EVENTS_WAIT_ID = static_cast<SceUID>(0x80000000);
// Object UID for an entry whose UID is neither an object nor a thread. No real object uses it.
constexpr SceUID NO_OBJECT = SCE_KERNEL_ERROR_UNKNOWN_EVENT_ID;

MultiEventWait::MultiEventWait(KernelState &kernel, ThreadState &thread, std::span<const SceKernelWaitEvent> wait_list, Mode mode, WaitOptions &options)
    : ThreadWait(thread, { SCE_KERNEL_WAITTYPE_MP_EVENTS, MULTI_EVENTS_WAIT_ID }, options, deadline_from(options))
    , mode(mode)
    , count(static_cast<SceUInt32>(wait_list.size())) {
    // The object UID of each entry, or NO_OBJECT
    std::array<SceUID, SCE_KERNEL_MULTIPLE_EVENTS_MAX> objects;
    for (SceUInt32 i = 0; i < count; i++) {
        const SceUID uid = wait_list[i].eventId;
        objects[i] = kernel.objects.find(uid) || kernel.get_thread(uid) ? uid : NO_OBJECT;
        Entry &entry = entries[i].emplace(*this, Event::Request{ .pattern = wait_list[i].eventPattern, .multi = this, .index = i });
        entry.event = find_event(kernel.objects, uid);
        // Entries for the same object all report the UID passed in the first such entry
        const auto named = std::span{ objects }.first(i + 1);
        entry.result_uid = wait_list[std::ranges::find(named, objects[i]) - named.begin()].eventId;
    }
}

Deadline MultiEventWait::watch(Deadline until) {
    Deadline look = Deadline::max();
    for (SceUInt32 i = 0; i < count; i++) {
        if (entries[i]->event)
            look = std::min(look, entries[i]->event->watch_timer(until));
    }
    return look;
}

WaitResult MultiEventWait::run() {
    while (true) {
        if (const WaitResult r = run_callbacks(); !r)
            return r;
        if (const std::optional<WaitResult> done = scan()) {
            dequeue();
            // Ended without sleeping. Callbacks don't run again here.
            return *done;
        }
        std::ignore = sleep();
        const WaitResult ended = retire();
        // The wait has ended, so no token can take a result anymore
        dequeue();
        if (!ended)
            return ended;
        if (*ended == SCE_KERNEL_ERROR_NOTIFY_CALLBACK)
            continue;
        // Run the callbacks notified during the sleep before returning. Only the thread exiting
        // changes the result.
        if (const WaitResult ran = run_callbacks(); !ran)
            return ran;
        return ended;
    }
}

bool MultiEventWait::accept(SceUInt32 index, SceInt32 result, SceUInt32 bits, SceUInt64 user_data) {
    const auto lock = lock_thread();
    // Tokens are only queued between the scan and the cleanup, and the thread has no other wait
    // meanwhile. If the thread is neither scanning nor waiting, this wait has already ended.
    if (finished.test(index) || !(scanning || thread.still_waits()))
        return false;
    ended[finished.count()] = { entries[index]->result_uid, result, bits, 0, user_data };
    finished.set(index);
    // Finish the wait on the first result in OR mode or the last one in AND mode. During the scan,
    // the end of scan() decides instead.
    if (!scanning && (mode == Mode::any || finished.count() == count))
        end(SCE_KERNEL_OK);
    return true;
}

std::span<const SceKernelResultEvent> MultiEventWait::results() const {
    return std::span{ ended }.first(finished.count());
}

std::optional<WaitResult> MultiEventWait::scan() {
    {
        const auto lock = lock_thread();
        scanning = true;
    }
    // In OR mode, stop queueing after the first result, but keep collecting the results of the rest
    bool queueing = true;
    for (SceUInt32 i = 0; i < count; i++) {
        {
            const auto lock = lock_thread();
            if (finished.test(i))
                continue;
        }
        const SceInt32 scanned = entries[i]->event ? entries[i]->event->scan_token(entries[i]->token, queueing) : SCE_KERNEL_ERROR_UNKNOWN_EVENT_ID;
        if (scanned == SCE_KERNEL_ERROR_UNKNOWN_EVENT_ID)
            accept(i, SCE_KERNEL_ERROR_UNKNOWN_EVENT_ID, SCE_KERNEL_EVENT_ERROR);
        if (scanned != SCE_KERNEL_ERROR_EVENT_COND && mode == Mode::any)
            queueing = false;
    }

    const auto lock = lock_thread();
    scanning = false;
    // A thread exit takes priority over any result, same as after a sleep
    if (thread.exiting())
        return std::unexpected{ ThreadExiting{} };
    // Decide by the results, not by what was queued. An event can end a token right after the scan
    // queued it.
    const std::size_t done = finished.count();
    if (done == count || (mode == Mode::any && done != 0))
        return SCE_KERNEL_OK;
    // Sleep even if a callback is already pending. The wait has no queue of its own, so a
    // callback notify ends it directly.
    return begin();
}

void MultiEventWait::dequeue() {
    for (SceUInt32 i = 0; i < count; i++) {
        if (entries[i]->event)
            entries[i]->event->remove_token(entries[i]->token);
    }
}

// *****************
// * Simple events *
// *****************

Event::Event(SceUInt32 attr, const char *name, SceUInt32 init_pattern, SceUID creator)
    : SyncObject(attr, name)
    , pattern(init_pattern | SCE_KERNEL_EVENT_CREATE)
    , last_user_data(system_event_user_data(creator)) {}

SceInt32 SimpleEvent::check_create(const char *name, SceUInt32 attr, SceUInt32 init_pattern) {
    // The notify attribute is accepted and reported, nothing reads it
    if (const SceInt32 error = check_object_name(name, attr, SCE_KERNEL_EVENT_ATTR_AUTO_RESET | SCE_KERNEL_ATTR_TH_PRIO | SCE_KERNEL_ATTR_OPENABLE | SCE_KERNEL_ATTR_NOTIFY_CB_WAKEUP_ONLY))
        return error;
    if (init_pattern & ~SCE_KERNEL_EVENT_USER_DEFINED_MASK)
        return SCE_KERNEL_ERROR_ILLEGAL_PATTERN;
    return SCE_KERNEL_OK;
}

void Event::on_delete(SceUID deleter) {
    fire(SCE_KERNEL_EVENT_DELETE, system_event_user_data(deleter), true);
    end_waits(SCE_KERNEL_ERROR_WAIT_DELETE);
}

bool Event::take(const Request &request) {
    const SceUInt32 bits = pattern & request.pattern;
    if (!bits)
        return false;
    // If the multiple event wait already ended, leave the bits on the event
    if (request.multi && !request.multi->accept(request.index, SCE_KERNEL_OK, bits, last_user_data))
        return true;
    if (request.result_pattern)
        *request.result_pattern = bits;
    if (request.user_data)
        *request.user_data = last_user_data;
    // Never auto reset DATA_EXIST. It tracks whether a pipe's ring holds bytes, so only the ring
    // changes it.
    if (attr & SCE_KERNEL_EVENT_ATTR_AUTO_RESET)
        pattern &= ~(request.pattern & ~SCE_KERNEL_EVENT_DATA_EXIST);
    return true;
}

std::size_t Event::end_waits(SceInt32 result) {
    std::size_t ended = 0;
    while (auto *node = waiters.front()) {
        const Request &request = node->request;
        if (request.multi) {
            ended += request.multi->accept(request.index, result, SCE_KERNEL_EVENT_ERROR);
            waiters.remove(*node);
        } else {
            waiters.complete(*node, result);
            ended++;
        }
    }
    return ended;
}

SceInt32 Event::scan_token(WaitNode<Request> &token, bool queue) {
    const auto guard = lock();
    if (!guard)
        return SCE_KERNEL_ERROR_UNKNOWN_EVENT_ID;
    catch_up();
    if (take(token.request))
        return SCE_KERNEL_OK;
    if (queue)
        waiters.push(token, token.wait.thread.get_priority());
    return SCE_KERNEL_ERROR_EVENT_COND;
}

void Event::remove_token(WaitNode<Request> &token) {
    const auto guard = lock();
    // Serve the waiters behind the removed token, since it may have been blocking them
    if (guard && waiters.remove(token))
        serve();
}

Deadline Event::watch_timer(Deadline until) {
    const auto guard = lock();
    return guard ? fire_due(until) : Deadline::max();
}

class Event::Wait final : public QueuedThreadWait<Event::Request> {
public:
    Wait(Event &event, std::unique_lock<std::mutex> guard, ThreadState &thread, Request request, WaitOptions &options)
        : QueuedThreadWait(event.waiters, std::move(guard), thread, { SCE_KERNEL_WAITTYPE_EVENT, event.uid }, std::move(request), options)
        , event(event) {}

private:
    Deadline watch(Deadline until) override { return event.fire_due(until); }

    Event &event;
};

SceInt32 Event::wait(ThreadState &thread, SceUInt32 wait_pattern, SceUInt32 *result_pattern, SceUInt64 *user_data, WaitOptions &options) {
    auto guard = lock();
    if (!guard)
        return SCE_KERNEL_ERROR_UNKNOWN_EVENT_ID;

    // A wait for pattern 0 matches nothing, so it lasts until its timeout, a cancel or a delete
    Wait wait(*this, std::move(guard), thread, Request{ .pattern = wait_pattern, .result_pattern = result_pattern, .user_data = user_data }, options);
    // Bits taken in a pass after a callback notify return NOTIFY_CALLBACK
    SceInt32 take_result = SCE_KERNEL_OK;
    const auto leave = [this](WaitResult) { serve(); };
    const SceInt32 result = guest_result(wait.run(FinalCallbackCheck::after_wait, leave, [&]() -> std::optional<SceInt32> {
        if (was_deleted())
            return SCE_KERNEL_ERROR_WAIT_DELETE;
        // Fire timer events that are already due before taking. A pulse that was due before this
        // wait started must not reach the waiter.
        catch_up();
        if (take(wait.node.request))
            return take_result;
        // run() only tries again after a callback notify ended the block
        take_result = SCE_KERNEL_ERROR_NOTIFY_CALLBACK;
        return std::nullopt;
    }));
    // A wait ended by a delete or a cancel gets the error event and no user data. If the wait got
    // the bits, they are already written, also when it returns NOTIFY_CALLBACK.
    if (result < 0 && result != SCE_KERNEL_ERROR_WAIT_TIMEOUT && result != SCE_KERNEL_ERROR_NOTIFY_CALLBACK && result_pattern)
        *result_pattern = SCE_KERNEL_EVENT_ERROR;
    return result;
}

SceInt32 Event::poll(SceUInt32 wait_pattern, SceUInt32 *result_pattern, SceUInt64 *user_data) {
    const auto guard = lock();
    if (!guard)
        return SCE_KERNEL_ERROR_UNKNOWN_EVENT_ID;
    catch_up();
    if (wait_pattern == 0)
        return SCE_KERNEL_ERROR_INVALID_ARGUMENT;
    return take({ .pattern = wait_pattern, .result_pattern = result_pattern, .user_data = user_data }) ? SCE_KERNEL_OK : SCE_KERNEL_ERROR_EVENT_COND;
}

SceInt32 SimpleEvent::set(SceUInt32 set_pattern, SceUInt64 user_data, bool is_set) {
    const auto guard = lock();
    if (!guard)
        return SCE_KERNEL_ERROR_UNKNOWN_EVENT_ID;
    if (set_pattern & ~SCE_KERNEL_EVENT_USER_DEFINED_MASK)
        return SCE_KERNEL_ERROR_ILLEGAL_PATTERN;

    fire(set_pattern, user_data, is_set);
    return SCE_KERNEL_OK;
}

SceInt32 SimpleEvent::cancel(SceUInt32 set_pattern, SceUInt64 user_data, SceUInt32 *num_wait_threads) {
    const auto guard = lock();
    if (!guard)
        return SCE_KERNEL_ERROR_UNKNOWN_EVENT_ID;
    if (set_pattern & ~SCE_KERNEL_EVENT_USER_DEFINED_MASK)
        return SCE_KERNEL_ERROR_ILLEGAL_PATTERN;

    // Multiple event waits count as waiters too
    const auto nb_threads = static_cast<SceUInt32>(end_waits(SCE_KERNEL_ERROR_WAIT_CANCEL));
    // Replace every bit, the create event included. No waiter is left to receive them.
    pattern = set_pattern;
    last_user_data = user_data;
    if (num_wait_threads)
        *num_wait_threads = nb_threads;

    return SCE_KERNEL_OK;
}

SceInt32 SimpleEvent::get_pattern(SceUInt32 &result) {
    const auto guard = lock();
    if (!guard)
        return SCE_KERNEL_ERROR_UNKNOWN_EVENT_ID;
    result = pattern;
    return SCE_KERNEL_OK;
}

SceInt32 Event::set_system_event(SceUInt32 event, SceUInt64 user_data) {
    const auto guard = lock();
    if (!guard)
        return SCE_KERNEL_ERROR_UNKNOWN_EVENT_ID;
    fire(event, user_data, true);
    return SCE_KERNEL_OK;
}

void Event::fire(SceUInt32 set_pattern, SceUInt64 user_data, bool is_set) {
    // A set or pulse of no bits changes nothing, not even the user data
    if (set_pattern == 0)
        return;

    pattern |= set_pattern;
    last_user_data = user_data;
    serve();

    // A pulse clears its bits once the waiters had them, also the ones set before it, and keeps its
    // user data
    if (!is_set)
        pattern &= ~set_pattern;
}

void Event::serve() {
    for (auto it = waiters.begin(); it != waiters.end();) {
        auto &node = *it++;
        if (!take(node.request))
            continue;
        if (node.request.multi)
            waiters.remove(node);
        else
            waiters.complete(node);
    }
}

SceInt32 Event::clear(SceUInt32 clear_pattern) {
    const auto guard = lock();
    if (!guard)
        return SCE_KERNEL_ERROR_UNKNOWN_EVENT_ID;
    catch_up();
    if (clear_pattern & ~SCE_KERNEL_EVENT_USER_DEFINED_MASK)
        return SCE_KERNEL_ERROR_ILLEGAL_PATTERN;

    pattern &= clear_pattern;

    return SCE_KERNEL_OK;
}

SceInt32 Event::cancel(SceUInt32 *num_wait_threads) {
    const auto guard = lock();
    if (!guard)
        return SCE_KERNEL_ERROR_UNKNOWN_EVENT_ID;
    catch_up();

    // Multiple event waits count as waiters too
    const auto nb_threads = static_cast<SceUInt32>(end_waits(SCE_KERNEL_ERROR_WAIT_CANCEL));
    // A timer also drops its pending event. It keeps running and keeps the event interval and type.
    disarm();
    if (num_wait_threads)
        *num_wait_threads = nb_threads;

    return SCE_KERNEL_OK;
}

SceInt32 Event::info(SceKernelEventInfo &info) {
    const auto guard = lock();
    if (!guard)
        return SCE_KERNEL_ERROR_UNKNOWN_EVENT_ID;
    catch_up();

    info = {
        .eventId = uid,
        .attr = attr,
        .eventPattern = pattern,
        .userData = last_user_data,
        .numWaitThreads = static_cast<SceUInt32>(waiters.size()),
    };
    strncpy(info.name, name.c_str(), KERNELOBJECT_MAX_NAME_LENGTH + 1);
    return SCE_KERNEL_OK;
}

// *********
// * Timer *
// *********

static uint64_t get_current_time() {
    return std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::high_resolution_clock::now().time_since_epoch()).count();
}

Timer::Timer(SceUInt32 attr, const char *name, SceUID creator)
    : WithUidClass(attr, name, 0, creator) {}

// A timer event is due at least this long after it was queued or after the event before it
constexpr uint64_t TIMER_EVENT_FLOOR = 200;

void Timer::catch_up() {
    const uint64_t now = get_current_time();
    if (next_event > now)
        return;
    fire(SCE_KERNEL_EVENT_TIMER, 0, !is_pulse);
    // A one shot event leaves the queue but stays armed, so a start or a set time can queue it again
    if (!is_repeat) {
        next_event = std::numeric_limits<uint64_t>::max();
        return;
    }
    // A repeating event is due again at the first interval step after now, counted from when it was
    // due. The events it missed meanwhile are skipped.
    const uint64_t interval = std::max(event_interval, TIMER_EVENT_FLOOR);
    next_event = std::max(next_event + ((now - next_event) / interval + 1) * interval, now + TIMER_EVENT_FLOOR);
    schedule = next_event - base;
}

Deadline Timer::fire_due(Deadline until) {
    // A fired event is due again at least a moment after now, so one is all that can be due
    if (event_deadline() <= until)
        catch_up();
    return event_deadline();
}

void Timer::disarm() {
    armed = false;
    is_repeat = false;
    next_event = std::numeric_limits<uint64_t>::max();
}

void Timer::queue_event(uint64_t deadline) {
    next_event = std::max(deadline, get_current_time() + TIMER_EVENT_FLOOR);
    // The waiters sleep until the old event time, so wake them to see the new one
    for (const auto &waiter : waiters)
        waiter.wait.wake();
}

Deadline Timer::event_deadline() const {
    if (next_event == std::numeric_limits<uint64_t>::max())
        return Deadline::max();
    // An overdue event returns a time in the past, so a bounded watch() still fires it
    const auto ahead = static_cast<int64_t>(next_event - get_current_time());
    return std::chrono::steady_clock::now() + std::chrono::microseconds(ahead);
}

SceInt32 Timer::set_event(SceUID type, SceKernelSysClock interval, SceInt32 repeats) {
    const auto guard = lock();
    if (!guard)
        return SCE_KERNEL_ERROR_UNKNOWN_TIMER_ID;
    if (type != SCE_KERNEL_TIMER_TYPE_SET_EVENT && type != SCE_KERNEL_TIMER_TYPE_PULSE_EVENT)
        return SCE_KERNEL_ERROR_ILLEGAL_TYPE;
    catch_up();

    is_pulse = type == SCE_KERNEL_TIMER_TYPE_PULSE_EVENT;
    is_repeat = repeats != 0;
    event_interval = interval;
    armed = true;
    schedule = count() + interval;
    // A stopped timer queues the event when it starts
    if (is_started)
        queue_event(get_current_time() + interval);

    return SCE_KERNEL_OK;
}

SceInt32 Timer::start() {
    const auto guard = lock();
    if (!guard)
        return SCE_KERNEL_ERROR_UNKNOWN_TIMER_ID;

    if (is_started)
        return 1;

    // The count goes on from where it stopped, and an armed event is due after the time it has left,
    // at once if it already fired
    is_started = true;
    const uint64_t now = get_current_time();
    base = now - stopped_count;
    if (armed)
        queue_event(now + (schedule > stopped_count ? schedule - stopped_count : 0));

    return SCE_KERNEL_OK;
}

SceInt32 Timer::stop() {
    const auto guard = lock();
    if (!guard)
        return SCE_KERNEL_ERROR_UNKNOWN_TIMER_ID;

    if (!is_started)
        return 0;
    catch_up();

    stopped_count = count();
    is_started = false;
    next_event = std::numeric_limits<uint64_t>::max();

    return 1;
}

std::expected<uint64_t, SceInt32> Timer::set_time(uint64_t new_time) {
    const auto guard = lock();
    if (!guard)
        return std::unexpected(SCE_KERNEL_ERROR_UNKNOWN_TIMER_ID);
    catch_up();

    const uint64_t old_count = count();
    if (is_started) {
        const uint64_t now = get_current_time();
        base = now - new_time;
        // If the new time is before the armed event's time, the event is due when the count reaches
        // it again. Otherwise it stays due when it was.
        if (armed && new_time < schedule)
            queue_event(now + (schedule - new_time));
    } else {
        stopped_count = new_time;
    }
    return old_count;
}

uint64_t Timer::count() const {
    return is_started ? get_current_time() - base : stopped_count;
}

SceInt32 Timer::get_time(uint64_t &time) const {
    const auto guard = lock();
    if (!guard)
        return SCE_KERNEL_ERROR_UNKNOWN_TIMER_ID;
    time = count();
    return SCE_KERNEL_OK;
}

SceInt32 Timer::get_base(uint64_t &time) const {
    const auto guard = lock();
    if (!guard)
        return SCE_KERNEL_ERROR_UNKNOWN_TIMER_ID;
    time = is_started ? base : 0;
    return SCE_KERNEL_OK;
}

SceInt32 Timer::remaining_time(uint64_t &time) {
    const auto guard = lock();
    if (!guard)
        return SCE_KERNEL_ERROR_UNKNOWN_TIMER_ID;
    catch_up();
    if (!armed)
        return SCE_KERNEL_ERROR_EVENT_NOT_SET;
    time = schedule - count();
    return SCE_KERNEL_OK;
}

SceInt32 Timer::info(SceKernelTimerInfo &info) {
    const auto guard = lock();
    if (!guard)
        return SCE_KERNEL_ERROR_UNKNOWN_TIMER_ID;
    catch_up();

    // The base time is the base of a running timer, the count of a stopped one
    info = {
        .timerId = uid,
        .attr = attr,
        .active = is_started,
        .baseTime = is_started ? base : stopped_count,
        .currentTime = count(),
        .schedule = armed ? schedule : 0,
        .interval = event_interval,
        .type = is_pulse ? SCE_KERNEL_TIMER_TYPE_PULSE_EVENT : SCE_KERNEL_TIMER_TYPE_SET_EVENT,
        .repeat = is_repeat,
        .numWaitThreads = static_cast<SceUInt32>(waiters.size()),
    };
    strncpy(info.name, name.c_str(), KERNELOBJECT_MAX_NAME_LENGTH + 1);
    return SCE_KERNEL_OK;
}

// *********
// * Mutex *
// *********

Mutex::Mutex(SceUInt32 attr, const char *name, SceInt32 init_count, SceUID creator, Ptr<SceKernelLwMutexWork> workarea)
    : SyncObject(attr, name)
    , init_count(init_count)
    , workarea(workarea)
    , lock_count(init_count)
    , owner(init_count > 0 ? creator : 0) {}

SceInt32 Mutex::check_create(const char *name, SceUInt32 attr, SceInt32 init_count, bool kernel) {
    if (const SceInt32 error = check_object_name(name, attr, kernel ? ~0U : SCE_KERNEL_MUTEX_ATTR_RECURSIVE | SCE_KERNEL_MUTEX_ATTR_CEILING | SCE_KERNEL_ATTR_TH_PRIO | SCE_KERNEL_ATTR_OPENABLE))
        return error;
    if (init_count < 0 || (init_count > 1 && !(attr & SCE_KERNEL_MUTEX_ATTR_RECURSIVE)))
        return SCE_KERNEL_ERROR_ILLEGAL_COUNT;
    return SCE_KERNEL_OK;
}

SceInt32 Mutex::check_count(SceInt32 count) const {
    if (count <= 0 || (count > 1 && !(attr & SCE_KERNEL_MUTEX_ATTR_RECURSIVE)))
        return SCE_KERNEL_ERROR_ILLEGAL_COUNT;
    return SCE_KERNEL_OK;
}

void Mutex::on_delete(SceUID) {
    // End the waits on the conditions first, then the waits on the mutex. Condvar::wait() turns
    // this into the mutex delete error.
    for (Condvar &cond : conditions)
        cond.waiters.complete_all(SCE_KERNEL_ERROR_WAIT_DELETE_MUTEX);
    waiters.complete_all(SCE_KERNEL_ERROR_WAIT_DELETE);
    owner = 0;
}

SceInt32 Mutex::try_take(MemState &mem, const ThreadState &thread, SceInt32 count) {
    if (owner && owner != thread.id)
        return lightweight() ? SCE_KERNEL_ERROR_LW_MUTEX_FAILED_TO_OWN : SCE_KERNEL_ERROR_MUTEX_FAILED_TO_OWN;
    if (owner && !(attr & SCE_KERNEL_MUTEX_ATTR_RECURSIVE))
        return lightweight() ? SCE_KERNEL_ERROR_LW_MUTEX_RECURSIVE : SCE_KERNEL_ERROR_MUTEX_RECURSIVE;
    // A free mutex has a count of 0. A lightweight count wraps past the maximum, and a lock that
    // would leave it below 0 fails. A heavy count never goes past the maximum.
    const int64_t sum = static_cast<int64_t>(lock_count) + count;
    if (!lightweight() && sum > std::numeric_limits<SceInt32>::max())
        return SCE_KERNEL_ERROR_MUTEX_LOCK_OVF;
    if (sum < 0)
        return SCE_KERNEL_ERROR_LW_MUTEX_LOCK_OVF;
    lock_count = static_cast<SceInt32>(static_cast<SceUInt32>(sum));
    owner = thread.id;
    update_workarea(mem);
    return SCE_KERNEL_OK;
}

void Mutex::update_workarea(MemState &mem) {
    if (!lightweight())
        return;
    workarea.get(mem)->lockCount = lock_count;
    workarea.get(mem)->owner = owner;
}

SceInt32 Mutex::acquire(MemState &mem, ThreadState &thread, SceInt32 count, WaitOptions &options, bool relock) {
    auto guard = lock();
    if (!guard)
        return lightweight() ? SCE_KERNEL_ERROR_UNKNOWN_LW_MUTEX_ID : SCE_KERNEL_ERROR_UNKNOWN_MUTEX_ID;
    if (const SceInt32 error = check_count(count))
        return error;
    // A lock by the owner succeeds or fails at once and runs no callbacks
    if (owner == thread.id)
        return try_take(mem, thread, count);

    // A relock reports the condition mutex wait type with the mutex UID
    SceUInt32 type = lightweight() ? SCE_KERNEL_WAITTYPE_LW_MUTEX : SCE_KERNEL_WAITTYPE_MUTEX;
    if (relock)
        type = lightweight() ? SCE_KERNEL_WAITTYPE_LW_COND_LW_MUTEX : SCE_KERNEL_WAITTYPE_COND_MUTEX;
    QueuedThreadWait wait(waiters, std::move(guard), thread, { type, uid }, Request{ .lock_count = count }, options);
    const auto leave = [this](WaitResult) { serve(); };
    const WaitResult r = wait.run(relock ? FinalCallbackCheck::on_delete : FinalCallbackCheck::after_wait, leave, [&]() -> std::optional<SceInt32> {
        if (was_deleted())
            return SCE_KERNEL_ERROR_WAIT_DELETE;
        // A heavy mutex also needs no waiter ahead of this thread. A mutex acquired by this thread's
        // callbacks still blocks this wait.
        if (!owner && (lightweight() || waiters.first_for(thread.get_priority())))
            return try_take(mem, thread, count);
        return std::nullopt;
    });
    // A deleted mutex has no owner, and its work area may be freed already
    if (owner == thread.id)
        update_workarea(mem);
    return guest_result(r);
}

SceInt32 Mutex::try_acquire(MemState &mem, ThreadState &thread, SceInt32 count) {
    const auto guard = lock();
    if (!guard)
        return lightweight() ? SCE_KERNEL_ERROR_UNKNOWN_LW_MUTEX_ID : SCE_KERNEL_ERROR_UNKNOWN_MUTEX_ID;
    if (const SceInt32 error = check_count(count))
        return error;

    return try_take(mem, thread, count);
}

SceInt32 Mutex::release(MemState &mem, ThreadState &thread, SceInt32 unlock_count) {
    // give_back() returns the error for a deleted mutex
    const std::lock_guard<std::mutex> guard(object_mutex());
    return give_back(mem, thread, unlock_count);
}

SceInt32 Mutex::give_back(MemState &mem, ThreadState &thread, SceInt32 unlock_count) {
    // A deleted mutex, which a lightweight mutex reports as not owned
    if (was_deleted())
        return lightweight() ? SCE_KERNEL_ERROR_LW_MUTEX_NOT_OWNED : SCE_KERNEL_ERROR_UNKNOWN_MUTEX_ID;
    if (const SceInt32 error = check_count(unlock_count))
        return error;
    if (thread.id != owner)
        return lightweight() ? SCE_KERNEL_ERROR_LW_MUTEX_NOT_OWNED : SCE_KERNEL_ERROR_MUTEX_NOT_OWNED;
    if (unlock_count > lock_count)
        return lightweight() ? SCE_KERNEL_ERROR_LW_MUTEX_UNLOCK_UDF : SCE_KERNEL_ERROR_MUTEX_UNLOCK_UDF;

    lock_count -= unlock_count;
    if (lock_count == 0)
        owner = 0;
    serve();
    update_workarea(mem);

    return SCE_KERNEL_OK;
}

void Mutex::release_ended_owner(const ThreadState &thread) {
    const auto guard = lock();
    if (!guard || owner != thread.id)
        return;
    lock_count = 0;
    owner = 0;
    serve();
}

void Mutex::serve() {
    auto *const node = waiters.front();
    if (owner || !node)
        return;
    lock_count = node->request.lock_count;
    owner = node->wait.thread.id;
    waiters.complete(*node);
}

SceInt32 Mutex::cancel(ThreadState &thread, SceInt32 new_count, SceUInt32 *num_wait_threads) {
    const auto guard = lock();
    if (!guard)
        return lightweight() ? SCE_KERNEL_ERROR_UNKNOWN_LW_MUTEX_ID : SCE_KERNEL_ERROR_UNKNOWN_MUTEX_ID;

    // Any negative count restores the lock count the mutex was created with
    if (new_count < 0)
        new_count = init_count;
    if (new_count > 1 && !(attr & SCE_KERNEL_MUTEX_ATTR_RECURSIVE))
        return SCE_KERNEL_ERROR_ILLEGAL_COUNT;

    const auto nb_threads = static_cast<SceUInt32>(waiters.complete_all(SCE_KERNEL_ERROR_WAIT_CANCEL));
    lock_count = new_count;
    owner = new_count > 0 ? thread.id : 0;
    if (num_wait_threads)
        *num_wait_threads = nb_threads;

    return SCE_KERNEL_OK;
}

SceInt32 Mutex::info(SceKernelLwMutexInfo &info) const {
    const auto guard = lock();
    if (!guard)
        return SCE_KERNEL_ERROR_UNKNOWN_LW_MUTEX_ID;

    info = {
        .uid = uid,
        .attr = attr,
        .pWork = workarea,
        .initCount = init_count,
        .currentCount = lock_count,
        .currentOwnerId = owner,
        .numWaitThreads = static_cast<SceUInt32>(waiters.size()),
    };
    strncpy(info.name, name.c_str(), KERNELOBJECT_MAX_NAME_LENGTH + 1);
    return SCE_KERNEL_OK;
}

SceInt32 Mutex::info(SceKernelMutexInfo &info) const {
    const auto guard = lock();
    if (!guard)
        return SCE_KERNEL_ERROR_UNKNOWN_MUTEX_ID;

    info = {
        .mutexId = uid,
        .attr = attr,
        .initCount = static_cast<SceUInt32>(init_count),
        .currentCount = static_cast<SceUInt32>(lock_count),
        .currentOwnerId = owner,
        .numWaitThreads = static_cast<SceUInt32>(waiters.size()),
    };
    strncpy(info.name, name.c_str(), KERNELOBJECT_MAX_NAME_LENGTH + 1);
    return SCE_KERNEL_OK;
}

// **************
// * RWLock *
// **************

SceInt32 RWLock::check_create(const char *name, SceUInt32 attr) {
    return check_object_name(name, attr, SCE_KERNEL_RW_LOCK_ATTR_RECURSIVE | SCE_KERNEL_RW_LOCK_ATTR_CEILING | SCE_KERNEL_ATTR_TH_PRIO | SCE_KERNEL_ATTR_OPENABLE);
}

void RWLock::on_delete(SceUID) {
    writers.complete_all(SCE_KERNEL_ERROR_WAIT_DELETE);
    readers.complete_all(SCE_KERNEL_ERROR_WAIT_DELETE);
}

SceInt32 RWLock::try_take(ThreadState &thread, bool is_write) {
    // The writer may take the write lock again only if the lock is recursive
    if (is_write && write_owner == thread.id) {
        if (!(attr & SCE_KERNEL_RW_LOCK_ATTR_RECURSIVE))
            return SCE_KERNEL_ERROR_RW_LOCK_RECURSIVE;
        lock_count++;
        return SCE_KERNEL_OK;
    }

    // A waiting writer goes before new readers
    if (!is_write && !writers.empty())
        return SCE_KERNEL_ERROR_RW_LOCK_FAILED_TO_LOCK;

    // Readers share the lock, but a reader and a writer don't, even in the same thread
    if (write_owner || (is_write && lock_count > 0))
        return SCE_KERNEL_ERROR_RW_LOCK_FAILED_TO_LOCK;

    lock_count++;
    if (is_write)
        write_owner = thread.id;
    return SCE_KERNEL_OK;
}

SceInt32 RWLock::acquire(ThreadState &thread, bool is_write, WaitOptions &options) {
    auto guard = lock();
    if (!guard)
        return SCE_KERNEL_ERROR_UNKNOWN_RW_LOCK_ID;
    // A lock by the writer succeeds or fails at once and runs no callbacks. A reader asking for a
    // write lock waits, since readers aren't tracked individually.
    if (write_owner == thread.id)
        return try_take(thread, is_write);

    QueuedThreadWait wait(is_write ? writers : readers, std::move(guard), thread, { SCE_KERNEL_WAITTYPE_RW_LOCK, uid }, Request{}, options);
    const auto leave = [this](WaitResult) { serve(); };
    return guest_result(wait.run(FinalCallbackCheck::after_wait, leave, [&]() -> std::optional<SceInt32> {
        if (was_deleted())
            return SCE_KERNEL_ERROR_WAIT_DELETE;
        // A write lock that this thread's callbacks took blocks it like any other lock. Readers only
        // wait while a writer holds or waits for the lock, so try_take() needn't look at them.
        if (write_owner == thread.id || try_take(thread, is_write) != SCE_KERNEL_OK)
            return std::nullopt;
        return SCE_KERNEL_OK;
    }));
}

SceInt32 RWLock::try_acquire(ThreadState &thread, bool is_write) {
    const auto guard = lock();
    if (!guard)
        return SCE_KERNEL_ERROR_UNKNOWN_RW_LOCK_ID;

    return try_take(thread, is_write);
}

SceInt32 RWLock::release(ThreadState &thread, bool is_write) {
    const auto guard = lock();
    if (!guard)
        return SCE_KERNEL_ERROR_UNKNOWN_RW_LOCK_ID;

    // Only the writer may write unlock, and nobody may read unlock while a writer holds the lock
    if (is_write && write_owner != thread.id)
        return SCE_KERNEL_ERROR_RW_LOCK_NOT_OWNED;
    if (!is_write && write_owner)
        return SCE_KERNEL_ERROR_RW_LOCK_FAILED_TO_UNLOCK;
    if (!is_write && lock_count == 0)
        return SCE_KERNEL_ERROR_RW_LOCK_UNLOCK_UDF;

    if (--lock_count == 0)
        write_owner = 0;
    serve();
    return SCE_KERNEL_OK;
}

void RWLock::release_ended_writer(const ThreadState &thread) {
    const auto guard = lock();
    if (!guard || write_owner != thread.id)
        return;
    lock_count = 0;
    write_owner = 0;
    serve();
}

SceInt32 RWLock::info(SceKernelRWLockInfo &info) const {
    const auto guard = lock();
    if (!guard)
        return SCE_KERNEL_ERROR_UNKNOWN_RW_LOCK_ID;

    info = {
        .rwLockId = uid,
        .attr = attr,
        .lockCount = lock_count,
        .writeOwnerId = write_owner,
        .numReadWaitThreads = static_cast<SceUInt32>(readers.size()),
        .numWriteWaitThreads = static_cast<SceUInt32>(writers.size()),
    };
    strncpy(info.name, name.c_str(), KERNELOBJECT_MAX_NAME_LENGTH + 1);
    return SCE_KERNEL_OK;
}

void RWLock::serve() {
    if (write_owner)
        return;
    // Without a waiting writer every waiting reader gets in, also next to readers that hold the lock.
    // A waiting writer goes first, even before readers that waited longer, once nobody holds it.
    if (writers.empty()) {
        lock_count += static_cast<SceInt32>(readers.complete_all(SCE_KERNEL_OK));
    } else if (lock_count == 0) {
        WaitNode<Request> &writer = *writers.front();
        write_owner = writer.wait.thread.id;
        lock_count = 1;
        writers.complete(writer);
    }
}

SceInt32 RWLock::cancel(ThreadState &thread, SceUInt32 *num_read_wait_threads, SceUInt32 *num_write_wait_threads, SceInt32 flag) {
    const auto guard = lock();
    if (!guard)
        return SCE_KERNEL_ERROR_UNKNOWN_RW_LOCK_ID;

    // Cancel the writers first, then the readers
    const auto nb_writers = static_cast<SceUInt32>(writers.complete_all(SCE_KERNEL_ERROR_WAIT_CANCEL));
    const auto nb_readers = static_cast<SceUInt32>(readers.complete_all(SCE_KERNEL_ERROR_WAIT_CANCEL));

    // The lock is reset, and only the calling thread may hold it afterwards
    const bool write_locked = flag & SCE_KERNEL_RW_LOCK_CANCEL_WITH_WRITE_LOCK;
    lock_count = write_locked ? 1 : 0;
    write_owner = write_locked ? thread.id : 0;

    if (num_read_wait_threads)
        *num_read_wait_threads = nb_readers;
    if (num_write_wait_threads)
        *num_write_wait_threads = nb_writers;

    return SCE_KERNEL_OK;
}

// **************
// * Semaphore *
// **************

Semaphore::Semaphore(SceUInt32 attr, const char *name, SceInt32 init_val, SceInt32 max_val)
    : WithUidClass(attr, name)
    , init_val(init_val)
    , max(max_val)
    , val(init_val) {}

SceInt32 Semaphore::check_create(const char *name, SceUInt32 attr, SceInt32 init_val, SceInt32 max_val) {
    if (const SceInt32 error = check_object_name(name, attr, SCE_KERNEL_ATTR_TH_PRIO | SCE_KERNEL_ATTR_OPENABLE))
        return error;
    if (init_val < 0 || max_val <= 0 || init_val > max_val)
        return SCE_KERNEL_ERROR_ILLEGAL_COUNT;
    return SCE_KERNEL_OK;
}

void Semaphore::on_delete(SceUID) {
    waiters.complete_all(SCE_KERNEL_ERROR_WAIT_DELETE);
}

SceInt32 Semaphore::wait(ThreadState &thread, SceInt32 need_count, WaitOptions &options) {
    auto guard = lock();
    if (!guard)
        return SCE_KERNEL_ERROR_UNKNOWN_SEMA_ID;
    if (need_count <= 0 || need_count > max)
        return SCE_KERNEL_ERROR_ILLEGAL_COUNT;

    QueuedThreadWait wait(waiters, std::move(guard), thread, { SCE_KERNEL_WAITTYPE_SEMAPHORE, uid }, Request{ .need_count = need_count }, options);
    const auto leave = [this](WaitResult) { serve(); };
    return guest_result(wait.run(FinalCallbackCheck::after_wait, leave, [&]() -> std::optional<SceInt32> {
        if (was_deleted())
            return SCE_KERNEL_ERROR_WAIT_DELETE;
        if (try_take(thread, need_count))
            return SCE_KERNEL_OK;
        return std::nullopt;
    }));
}

SceInt32 Semaphore::poll(ThreadState &thread, SceInt32 need_count) {
    const auto guard = lock();
    if (!guard)
        return SCE_KERNEL_ERROR_UNKNOWN_SEMA_ID;
    if (need_count <= 0 || need_count > max)
        return SCE_KERNEL_ERROR_ILLEGAL_COUNT;

    return try_take(thread, need_count) ? SCE_KERNEL_OK : SCE_KERNEL_ERROR_SEMA_ZERO;
}

bool Semaphore::try_take(const ThreadState &thread, SceInt32 need_count) {
    // A waiter queued ahead goes first, even one the count doesn't cover yet
    if (val < need_count || !waiters.first_for(thread.get_priority()))
        return false;
    val -= need_count;
    return true;
}

SceInt32 Semaphore::signal(SceInt32 count) {
    const auto guard = lock();
    if (!guard)
        return SCE_KERNEL_ERROR_UNKNOWN_SEMA_ID;

    if (count < 0)
        return SCE_KERNEL_ERROR_ILLEGAL_COUNT;
    if (count > max - val)
        return SCE_KERNEL_ERROR_SEMA_OVF;
    val += count;
    serve();

    return SCE_KERNEL_OK;
}

void Semaphore::serve() {
    while (auto *const node = waiters.front()) {
        if (val < node->request.need_count)
            break;
        val -= node->request.need_count;
        waiters.complete(*node);
    }
}

SceInt32 Semaphore::cancel(SceInt32 set_count, SceUInt32 *num_wait_threads) {
    const auto guard = lock();
    if (!guard)
        return SCE_KERNEL_ERROR_UNKNOWN_SEMA_ID;

    if (set_count > max)
        return SCE_KERNEL_ERROR_ILLEGAL_COUNT;

    const auto nb_threads = static_cast<SceUInt32>(waiters.complete_all(SCE_KERNEL_ERROR_WAIT_CANCEL));
    // Any negative count restores the count the semaphore was created with
    val = set_count < 0 ? init_val : set_count;
    if (num_wait_threads)
        *num_wait_threads = nb_threads;
    return SCE_KERNEL_OK;
}

SceInt32 Semaphore::info(SceKernelSemaInfo &info) const {
    const auto guard = lock();
    if (!guard)
        return SCE_KERNEL_ERROR_UNKNOWN_SEMA_ID;

    info = {
        .semaId = uid,
        .attr = attr,
        .initCount = init_val,
        .currentCount = val,
        .maxCount = max,
        .numWaitThreads = static_cast<SceUInt32>(waiters.size()),
    };
    strncpy(info.name, name.c_str(), KERNELOBJECT_MAX_NAME_LENGTH + 1);
    return SCE_KERNEL_OK;
}

// **********************
// * Condition Variable *
// **********************

Condvar::Condvar(SceUInt32 attr, const char *name, MutexPtr associated_mutex, Ptr<SceKernelLwCondWork> workarea)
    : SyncObject(attr, name)
    , associated_mutex(std::move(associated_mutex))
    , workarea(workarea) {}

std::mutex &Condvar::object_mutex() const {
    return associated_mutex->object_mutex();
}

Condvar::~Condvar() {
    // A condition that was never deleted is still in the condition list of its mutex
    if (const auto guard = lock())
        associated_mutex->conditions.remove(*this);
}

SceInt32 Condvar::attach() {
    const auto guard = lock();
    // The condition uses the mutex lock, so the mutex can't be deleted meanwhile
    if (associated_mutex->was_deleted())
        return lightweight() ? SCE_KERNEL_ERROR_UNKNOWN_LW_MUTEX_ID : SCE_KERNEL_ERROR_UNKNOWN_MUTEX_ID;
    associated_mutex->conditions.push_back(*this);
    return SCE_KERNEL_OK;
}

void Condvar::on_delete(SceUID) {
    // wait() turns this into the condition delete error
    waiters.complete_all(SCE_KERNEL_ERROR_WAIT_DELETE);
    associated_mutex->conditions.remove(*this);
}

SceInt32 Condvar::wait(MemState &mem, ThreadState &thread, WaitOptions &options) {
    WaitResult signaled;
    {
        auto guard = lock();
        if (!guard)
            return lightweight() ? SCE_KERNEL_ERROR_UNKNOWN_LW_COND_ID : SCE_KERNEL_ERROR_UNKNOWN_COND_ID;
        // The mutex unlock and the queueing happen in one hold of the mutex lock
        if (const SceInt32 error = associated_mutex->give_back(mem, thread, 1))
            return error;
        // A condition holds no signal for later, so the wait only queues and sleeps
        QueuedThreadWait wait(waiters, std::move(guard), thread, { lightweight() ? SCE_KERNEL_WAITTYPE_LW_COND_SIGNAL : SCE_KERNEL_WAITTYPE_COND_SIGNAL, uid }, Request{}, options);
        // A waiter that leaves frees nothing for the others
        const auto leave = [](WaitResult) {};
        // Only a delete runs the callbacks notified during the wait
        signaled = wait.run(FinalCallbackCheck::on_delete, leave, [&]() -> std::optional<SceInt32> {
            if (was_deleted())
                return SCE_KERNEL_ERROR_WAIT_DELETE;
            return std::nullopt;
        });
        if (!signaled)
            return guest_result(signaled);
        // The condition was deleted during the wait or while callbacks ran
        if (*signaled == SCE_KERNEL_ERROR_WAIT_DELETE)
            return lightweight() ? SCE_KERNEL_ERROR_WAIT_DELETE_LW_COND : SCE_KERNEL_ERROR_WAIT_DELETE_COND;
        // The mutex was deleted while the thread was queued, so there is nothing to relock
        if (*signaled == SCE_KERNEL_ERROR_WAIT_DELETE_MUTEX)
            return lightweight() ? SCE_KERNEL_ERROR_WAIT_DELETE_LW_MUTEX : SCE_KERNEL_ERROR_WAIT_DELETE_MUTEX;
        // A signaled or timed out wait relocks the mutex, unless the condition was deleted after the
        // wait ended
        if ((*signaled != SCE_KERNEL_OK && *signaled != SCE_KERNEL_ERROR_WAIT_TIMEOUT) || was_deleted())
            return *signaled;
    }
    // Taking the mutex back is still part of the condition variable wait, a wait on the mutex with no
    // timeout of its own that leaves the time left as it was at the signal
    WaitOptions relock_options(nullptr, options.callbacks);
    const SceInt32 result = associated_mutex->acquire(mem, thread, 1, relock_options, true);
    // A relock that succeeds returns how the condition wait ended, a signal or a timeout
    if (result == SCE_KERNEL_OK)
        return *signaled;
    // A delete during the relock gives the mutex delete error. If the mutex was deleted while the
    // thread was off the condition queue, after a signal or while running a callback, the relock
    // gives the unknown mutex error. A heavy mutex returns that error. A lightweight mutex gives
    // the delete error instead, so the waiter isn't left blocked.
    if (result == SCE_KERNEL_ERROR_WAIT_DELETE || result == SCE_KERNEL_ERROR_UNKNOWN_LW_MUTEX_ID)
        return lightweight() ? SCE_KERNEL_ERROR_WAIT_DELETE_LW_MUTEX : SCE_KERNEL_ERROR_WAIT_DELETE_MUTEX;
    // Only a heavy mutex can be canceled
    if (result == SCE_KERNEL_ERROR_WAIT_CANCEL)
        return SCE_KERNEL_ERROR_WAIT_CANCEL_MUTEX;
    return result;
}

SceInt32 Condvar::signal() {
    const auto guard = lock();
    if (!guard)
        return lightweight() ? SCE_KERNEL_ERROR_UNKNOWN_LW_COND_ID : SCE_KERNEL_ERROR_UNKNOWN_COND_ID;
    if (auto *const node = waiters.front())
        waiters.complete(*node);
    return SCE_KERNEL_OK;
}

SceInt32 Condvar::signal_all() {
    const auto guard = lock();
    if (!guard)
        return lightweight() ? SCE_KERNEL_ERROR_UNKNOWN_LW_COND_ID : SCE_KERNEL_ERROR_UNKNOWN_COND_ID;
    waiters.complete_all(SCE_KERNEL_OK);
    return SCE_KERNEL_OK;
}

SceInt32 Condvar::signal_to(SceUID thread_id) {
    const auto guard = lock();
    if (!guard)
        return lightweight() ? SCE_KERNEL_ERROR_UNKNOWN_LW_COND_ID : SCE_KERNEL_ERROR_UNKNOWN_COND_ID;
    const auto waiter = std::ranges::find_if(waiters, [&](const auto &node) { return node.wait.thread.id == thread_id; });
    if (waiter == waiters.end())
        return SCE_KERNEL_ERROR_ILLEGAL_THREAD_ID;
    waiters.complete(*waiter);
    return SCE_KERNEL_OK;
}

SceInt32 Condvar::info(SceKernelCondInfo &info) const {
    const auto guard = lock();
    if (!guard)
        return SCE_KERNEL_ERROR_UNKNOWN_COND_ID;

    info = {
        .condId = uid,
        .attr = attr,
        .mutexId = associated_mutex->was_deleted() ? -1 : associated_mutex->uid,
        .numWaitThreads = static_cast<SceUInt32>(waiters.size()),
    };
    strncpy(info.name, name.c_str(), KERNELOBJECT_MAX_NAME_LENGTH + 1);
    return SCE_KERNEL_OK;
}

SceInt32 Condvar::info(SceKernelLwCondInfo &info) const {
    const auto guard = lock();
    if (!guard)
        return SCE_KERNEL_ERROR_UNKNOWN_LW_COND_ID;

    info = {
        .uid = uid,
        .attr = attr,
        .pWork = workarea,
        .pLwMutex = associated_mutex->workarea,
        .numWaitThreads = static_cast<SceUInt32>(waiters.size()),
    };
    strncpy(info.name, name.c_str(), KERNELOBJECT_MAX_NAME_LENGTH + 1);
    return SCE_KERNEL_OK;
}

// **************
// * Event Flag *
// **************

EventFlag::EventFlag(SceUInt32 attr, const char *name, SceUInt32 init_pattern)
    : WithUidClass(attr, name)
    , init_pattern(init_pattern)
    , flags(init_pattern) {}

SceInt32 EventFlag::check_create(const char *name, SceUInt32 attr) {
    return check_object_name(name, attr, SCE_EVENT_WAITMULTIPLE | SCE_KERNEL_ATTR_TH_PRIO | SCE_KERNEL_ATTR_OPENABLE);
}

// Returns the error a poll or wait for pattern with wait_mode gets, or SCE_KERNEL_OK
static SceInt32 check_wait(SceUInt32 pattern, SceUInt32 wait_mode) {
    if (pattern == 0)
        return SCE_KERNEL_ERROR_INVALID_ARGUMENT;
    if (wait_mode & ~(SCE_EVENT_WAITOR | SCE_EVENT_WAITCLEAR | SCE_EVENT_WAITCLEAR_PAT))
        return SCE_KERNEL_ERROR_ILLEGAL_MODE;
    // Only one way of clearing the bits after the wait
    if ((wait_mode & SCE_EVENT_WAITCLEAR) && (wait_mode & SCE_EVENT_WAITCLEAR_PAT))
        return SCE_KERNEL_ERROR_ILLEGAL_MODE;
    return SCE_KERNEL_OK;
}

void EventFlag::on_delete(SceUID) {
    waiters.complete_all(SCE_KERNEL_ERROR_WAIT_DELETE);
}

void EventFlag::drop_hold() {
    if (holds > 0 && --holds == 0)
        holder = 0;
}

bool EventFlag::held_by_other(SceUID thread_id) const {
    return holds > 0 && holder != thread_id;
}

SceInt32 EventFlag::wait(ThreadState &thread, SceUInt32 pattern, SceUInt32 wait_mode, SceUInt32 *out_bits, WaitOptions &options) {
    auto guard = lock();
    if (!guard)
        return SCE_KERNEL_ERROR_UNKNOWN_EVF_ID;
    if (const SceInt32 error = check_wait(pattern, wait_mode))
        return error;
    if (held_by_other(thread.id))
        return SCE_KERNEL_ERROR_EVF_MULTI;

    // A flag without SCE_EVENT_WAITMULTIPLE is held while the wait lasts. The wait gives the hold
    // back when it takes the flags, times out or is canceled.
    if (!(attr & SCE_EVENT_WAITMULTIPLE)) {
        holder = thread.id;
        ++holds;
    }
    QueuedThreadWait wait(waiters, std::move(guard), thread, { SCE_KERNEL_WAITTYPE_EVENTFLAG, uid }, Request{ .wait_mode = wait_mode, .pattern = pattern, .out_bits = out_bits }, options);
    // A waiter that ended the wait itself for a timeout or a callback notify writes the flags it
    // left, and a timeout gives the hold back, before the waiters behind are served. A thread exit
    // does neither.
    const auto leave = [&](WaitResult ended) {
        if (ended && out_bits)
            *out_bits = flags;
        if (ended == SCE_KERNEL_ERROR_WAIT_TIMEOUT)
            drop_hold();
        serve();
    };
    const SceInt32 result = guest_result(wait.run(FinalCallbackCheck::after_wait, leave, [&]() -> std::optional<SceInt32> {
        if (was_deleted())
            return SCE_KERNEL_ERROR_WAIT_DELETE;
        if (take(wait.node.request)) {
            drop_hold();
            return SCE_KERNEL_OK;
        }
        // A wait that times out right away returns these bits
        if (out_bits)
            *out_bits = flags;
        return std::nullopt;
    }));
    // A deleted flag's bits can't change anymore. Write them out for a wait that the delete ended,
    // also when the delete came while the thread was running callbacks outside the queue.
    if (result == SCE_KERNEL_ERROR_WAIT_DELETE && out_bits)
        *out_bits = flags;
    return result;
}

SceInt32 EventFlag::poll(SceUID thread_id, SceUInt32 pattern, SceUInt32 wait_mode, SceUInt32 *out_bits) {
    const auto guard = lock();
    if (!guard)
        return SCE_KERNEL_ERROR_UNKNOWN_EVF_ID;
    if (const SceInt32 error = check_wait(pattern, wait_mode))
        return error;
    if (held_by_other(thread_id))
        return SCE_KERNEL_ERROR_EVF_MULTI;

    // A poll holds nothing, so it drops no hold
    if (take({ .wait_mode = wait_mode, .pattern = pattern, .out_bits = out_bits }))
        return SCE_KERNEL_OK;
    if (out_bits)
        *out_bits = flags;
    return SCE_KERNEL_ERROR_EVF_COND;
}

void EventFlag::release_ended_holder(const ThreadState &thread) {
    const auto guard = lock();
    if (!guard || holder != thread.id)
        return;
    holder = 0;
    holds = 0;
}

SceInt32 EventFlag::set(SceUInt32 pattern) {
    const auto guard = lock();
    if (!guard)
        return SCE_KERNEL_ERROR_UNKNOWN_EVF_ID;

    flags |= pattern;
    serve();

    return SCE_KERNEL_OK;
}

bool EventFlag::take(const Request &request) {
    const bool matched = (request.wait_mode & SCE_EVENT_WAITOR) ? (flags & request.pattern) != 0 : (flags & request.pattern) == request.pattern;
    if (!matched)
        return false;
    if (request.out_bits)
        *request.out_bits = flags;
    if (request.wait_mode & SCE_EVENT_WAITCLEAR)
        flags = 0;
    if (request.wait_mode & SCE_EVENT_WAITCLEAR_PAT)
        flags &= ~request.pattern;
    return true;
}

void EventFlag::serve() {
    for (auto it = waiters.begin(); it != waiters.end();) {
        auto &node = *it++;
        if (!take(node.request))
            continue;
        drop_hold();
        waiters.complete(node);
    }
}

SceInt32 EventFlag::clear(SceUInt32 pattern) {
    const auto guard = lock();
    if (!guard)
        return SCE_KERNEL_ERROR_UNKNOWN_EVF_ID;

    flags &= pattern;

    return SCE_KERNEL_OK;
}

SceInt32 EventFlag::cancel(SceUInt32 pattern, SceUInt32 *num_wait_threads) {
    const auto guard = lock();
    if (!guard)
        return SCE_KERNEL_ERROR_UNKNOWN_EVF_ID;

    for (auto &node : waiters) {
        if (node.request.out_bits)
            *node.request.out_bits = pattern;
        drop_hold();
    }
    const auto nb_threads = static_cast<SceUInt32>(waiters.complete_all(SCE_KERNEL_ERROR_WAIT_CANCEL));
    flags = pattern;
    if (num_wait_threads)
        *num_wait_threads = nb_threads;
    return SCE_KERNEL_OK;
}

SceInt32 EventFlag::info(SceKernelEventFlagInfo &info) const {
    const auto guard = lock();
    if (!guard)
        return SCE_KERNEL_ERROR_UNKNOWN_EVF_ID;

    info = {
        .evfId = uid,
        .attr = attr,
        .initPattern = init_pattern,
        .currentPattern = flags,
        .numWaitThreads = static_cast<SceUInt32>(waiters.size()),
    };
    strncpy(info.name, name.c_str(), KERNELOBJECT_MAX_NAME_LENGTH + 1);
    return SCE_KERNEL_OK;
}

// *************
// * Msg Pipe  *
// *************

MsgPipe::MsgPipe(SceUInt32 attr, const char *name, std::size_t buf_size, SceUID creator)
    : WithUidClass(attr, name, 0, creator)
    , senders(attr & SCE_KERNEL_MSG_PIPE_ATTR_TH_PRIO_S ? QueueOrder::priority : QueueOrder::fifo)
    , receivers(attr & SCE_KERNEL_MSG_PIPE_ATTR_TH_PRIO_R ? QueueOrder::priority : QueueOrder::fifo)
    // A pipe's buffer is a whole number of 4 KiB pages
    , data_buffer(align(buf_size, 0x1000)) {}

SceInt32 MsgPipe::check_create(const char *name, SceUInt32 type, SceUInt32 attr, SceSize buf_size) {
    if (const SceInt32 error = check_object_name(name, attr))
        return error;
    if (type != SCE_KERNEL_MSG_PIPE_TYPE_USER_MAIN)
        return SCE_KERNEL_ERROR_ILLEGAL_TYPE;
    if (attr & ~(SCE_KERNEL_MSG_PIPE_ATTR_TH_MASK | SCE_KERNEL_ATTR_OPENABLE | SCE_KERNEL_EVENT_ATTR_AUTO_RESET | SCE_KERNEL_ATTR_NOTIFY_CB_WAKEUP_ONLY))
        return SCE_KERNEL_ERROR_ILLEGAL_ATTR;
    if (buf_size == 0)
        return SCE_KERNEL_ERROR_ILLEGAL_SIZE;
    return SCE_KERNEL_OK;
}

// Returns whether a send or receive takes this wait mode
static bool is_msg_pipe_mode(SceUInt32 wait_mode) {
    return !(wait_mode & ~(SCE_KERNEL_MSG_PIPE_MODE_FULL | SCE_KERNEL_MSG_PIPE_MODE_DONT_WAIT | SCE_KERNEL_MSG_PIPE_MODE_DONT_REMOVE));
}

void MsgPipe::on_delete(SceUID deleter) {
    Event::on_delete(deleter);
    senders.complete_all(SCE_KERNEL_ERROR_WAIT_DELETE);
    receivers.complete_all(SCE_KERNEL_ERROR_WAIT_DELETE);
}

std::span<char> MsgPipe::Transfer::run(SceSize n) {
    // Steps over the pieces already moved, and empty ones
    while (offset == pieces.front().size) {
        pieces = pieces.subspan(1);
        offset = 0;
    }
    const MsgPipeVector &current = pieces.front();
    return std::span{ current.base.cast<char>().get(mem), current.size }.subspan(offset, std::min(n, current.size - offset));
}

void MsgPipe::copy(Transfer &sender, Transfer &receiver, SceSize n) {
    // A peek reads through a copy of the sender, so the sender itself doesn't advance
    Transfer snapshot = sender;
    Transfer &from = receiver.peek ? snapshot : sender;
    while (n > 0) {
        const std::span<char> in = from.run(n);
        const std::span<char> out = receiver.run(static_cast<SceSize>(in.size()));
        std::ranges::copy(in.first(out.size()), out.begin());
        const SceSize moved = static_cast<SceSize>(out.size());
        from.advance(moved);
        receiver.advance(moved);
        n -= moved;
    }
}

void MsgPipe::fill(Transfer &sender) {
    // ASAP puts what fits, FULL all of its rest or nothing
    const SceSize room = static_cast<SceSize>(data_buffer.Free());
    if (sender.full && sender.left > room)
        return;
    SceSize n = std::min(sender.left, room);
    while (n > 0) {
        const std::span<char> in = sender.run(n);
        const SceSize moved = static_cast<SceSize>(data_buffer.Insert(in.data(), in.size()));
        sender.advance(moved);
        n -= moved;
    }
}

void MsgPipe::drain(Transfer &receiver) {
    const SceSize held = static_cast<SceSize>(data_buffer.Used());
    // A peek reads on from where it got to, and leaves the bytes in the pipe
    SceSize at = receiver.peek ? std::min(receiver.moved(), held) : 0;
    SceSize n = std::min(receiver.left, held - at);
    while (n > 0) {
        const std::span<char> out = receiver.run(n);
        const SceSize moved = static_cast<SceSize>(receiver.peek ? data_buffer.Peek(out.data(), out.size(), at) : data_buffer.Remove(out.data(), out.size()));
        at += moved;
        receiver.advance(moved);
        n -= moved;
    }
}

SceSize MsgPipe::available() const {
    SceSize bytes = static_cast<SceSize>(data_buffer.Used());
    for (const auto &node : senders)
        bytes += node.request.left;
    return bytes;
}

bool MsgPipe::give(Transfer &sender) {
    // A send reaches the first waiting receiver even with no bytes, which ends an ASAP receive
    while (auto *const node = receivers.front()) {
        Transfer &to = node->request;
        copy(sender, to, std::min(to.left, sender.left));
        to.report();
        // A FULL receive that still needs bytes stays at the front and keeps waiting
        if (to.full && to.left > 0)
            break;
        receivers.complete(*node);
        if (sender.left == 0)
            break;
    }
    fill(sender);
    sender.report();
    return sender.done();
}

bool MsgPipe::take(Transfer &receiver) {
    drain(receiver);
    // Each waiting sender gives its bytes to the receiver while it wants some, then puts its rest
    // into the pipe, up to the first sender that stays
    while (auto *const node = senders.front()) {
        Transfer &from = node->request;
        copy(from, receiver, std::min(receiver.left, from.left));
        // A peek frees nothing
        if (receiver.peek)
            break;
        fill(from);
        from.report();
        if (!from.done())
            break;
        senders.complete(*node);
    }
    receiver.report();
    return receiver.done();
}

void MsgPipe::serve() {
    // The first receiver goes first, so the pipe's bytes go out before those of waiting senders
    while (true) {
        if (auto *const receiver = receivers.front(); receiver && take(receiver->request))
            receivers.complete(*receiver);
        else if (auto *const sender = senders.front(); sender && give(sender->request))
            senders.complete(*sender);
        else
            break;
    }
    Event::serve();
}

void MsgPipe::post(bool sending, SceInt32 result, SceSize moved) {
    SceUInt32 bits = moved > 0 ? (sending ? SCE_KERNEL_EVENT_IN : SCE_KERNEL_EVENT_OUT) : 0;
    if (sending && moved > 0 && data_buffer.Used() > 0)
        bits |= SCE_KERNEL_EVENT_DATA_EXIST;
    // A suspended or stopped caller is not a pipe error
    if (result < 0 && result != SCE_KERNEL_ERROR_THREAD_STOPPED && result != SCE_KERNEL_ERROR_THREAD_SUSPENDED)
        bits |= SCE_KERNEL_EVENT_ERROR;
    if (!sending && data_buffer.Used() == 0)
        pattern &= ~SCE_KERNEL_EVENT_DATA_EXIST;
    // The number of bytes moved, or all ones on error
    const SceUInt32 size = moved | (result < 0 ? 0xFFFFFFFFU : 0);
    fire(bits, (static_cast<SceUInt64>(size) << 32) | static_cast<SceUInt32>(result), true);
}

std::expected<MsgPipe::Transfer, SceInt32> MsgPipe::transfer_for(MemState &mem, SceUInt32 wait_mode, std::span<const MsgPipeVector> pieces, SceSize *size) const {
    if (!is_msg_pipe_mode(wait_mode))
        return std::unexpected{ SCE_KERNEL_ERROR_ILLEGAL_MODE };
    uint64_t total = 0;
    for (const MsgPipeVector &piece : pieces)
        total += piece.size;
    if (total > data_buffer.Capacity())
        return std::unexpected{ SCE_KERNEL_ERROR_ILLEGAL_SIZE };
    return Transfer{
        .mem = mem,
        .pieces = pieces,
        .size = size,
        .total = static_cast<SceSize>(total),
        .full = (wait_mode & SCE_KERNEL_MSG_PIPE_MODE_FULL) != 0,
        .peek = (wait_mode & SCE_KERNEL_MSG_PIPE_MODE_DONT_REMOVE) != 0,
    };
}

SceInt32 MsgPipe::receive(ThreadState &thread, MemState &mem, SceUInt32 wait_mode, std::span<const MsgPipeVector> pieces, SceSize *size, WaitOptions &options) {
    // DONT_WAIT behaves like a try, then runs callbacks if the call allows them
    if (is_msg_pipe_mode(wait_mode) && (wait_mode & SCE_KERNEL_MSG_PIPE_MODE_DONT_WAIT)) {
        const SceInt32 result = try_receive(thread, mem, wait_mode, pieces, size);
        if (options.callbacks)
            thread.process_callbacks();
        return result;
    }
    const std::expected<Transfer, SceInt32> receiver = transfer_for(mem, wait_mode, pieces, size);
    return receiver ? transfer(thread, false, *receiver, options) : receiver.error();
}

SceInt32 MsgPipe::send(ThreadState &thread, MemState &mem, SceUInt32 wait_mode, std::span<const MsgPipeVector> pieces, SceSize *size, WaitOptions &options) {
    // DONT_WAIT behaves like a try, then runs callbacks if the call allows them
    if (is_msg_pipe_mode(wait_mode) && (wait_mode & SCE_KERNEL_MSG_PIPE_MODE_DONT_WAIT)) {
        const SceInt32 result = try_send(thread, mem, wait_mode, pieces, size);
        if (options.callbacks)
            thread.process_callbacks();
        return result;
    }
    const std::expected<Transfer, SceInt32> sender = transfer_for(mem, wait_mode, pieces, size);
    return sender ? transfer(thread, true, *sender, options) : sender.error();
}

bool MsgPipe::first_in_line(WaitQueue<Transfer> &queue, const ThreadState &thread) {
    return !partly_served(queue) && queue.first_for(thread.get_priority());
}

SceInt32 MsgPipe::transfer(ThreadState &thread, bool sending, const Transfer &request, WaitOptions &options) {
    auto guard = lock();
    if (!guard)
        return SCE_KERNEL_ERROR_UNKNOWN_MSG_PIPE_ID;
    // A zero byte transfer returns at once, before any callback runs, and posts no event
    if (request.total == 0) {
        request.report();
        return SCE_KERNEL_OK;
    }

    WaitQueue<Transfer> &queue = sending ? senders : receivers;
    QueuedThreadWait wait(queue, std::move(guard), thread, { SCE_KERNEL_WAITTYPE_MSG_PIPE, uid }, request, options);
    const auto leave = [this](WaitResult) { serve(); };
    Transfer &mine = wait.node.request;
    WaitResult r;
    // A notified callback ends the block and starts a new pass
    do {
        if (r = wait.run_callbacks(); !r)
            return guest_result(r);
        if (was_deleted())
            return SCE_KERNEL_ERROR_WAIT_DELETE;
        // Don't move anything while others on the same side are ahead in the queue. Write the size
        // after every attempt.
        const bool done = first_in_line(queue, thread) && (sending ? give(mine) : take(mine));
        mine.report();
        if (done)
            break;
        if (r = wait.block(leave, partly_served(queue)); !r)
            return guest_result(r);
    } while (*r == SCE_KERNEL_ERROR_NOTIFY_CALLBACK);
    // The pipe event comes before the callbacks notified during the wait, even without a wait
    post(sending, *r, mine.moved());
    // Run the callbacks notified during the wait. Only the thread exiting changes the result.
    if (const WaitResult ran = wait.run_callbacks(); !ran)
        return guest_result(ran);
    return *r;
}

SceInt32 MsgPipe::try_receive(ThreadState &thread, MemState &mem, SceUInt32 wait_mode, std::span<const MsgPipeVector> pieces, SceSize *size) {
    std::expected<Transfer, SceInt32> receiver = transfer_for(mem, wait_mode, pieces, size);
    return receiver ? try_transfer(thread, false, *receiver) : receiver.error();
}

SceInt32 MsgPipe::try_send(ThreadState &thread, MemState &mem, SceUInt32 wait_mode, std::span<const MsgPipeVector> pieces, SceSize *size) {
    std::expected<Transfer, SceInt32> sender = transfer_for(mem, wait_mode, pieces, size);
    return sender ? try_transfer(thread, true, *sender) : sender.error();
}

SceInt32 MsgPipe::try_transfer(ThreadState &thread, bool sending, Transfer &request) {
    const auto guard = lock();
    if (!guard)
        return SCE_KERNEL_ERROR_UNKNOWN_MSG_PIPE_ID;
    // Behind the waiters already on its side, or short of room or bytes for all of a FULL call, it
    // fails without writing a size
    const std::size_t ready = sending ? data_buffer.Free() : available();
    const bool can_move = first_in_line(sending ? senders : receivers, thread) && !(request.full && request.total > ready);
    const bool done = can_move && (sending ? give(request) : take(request));
    const SceInt32 result = done ? SCE_KERNEL_OK : (sending ? SCE_KERNEL_ERROR_MSG_PIPE_FULL : SCE_KERNEL_ERROR_MSG_PIPE_EMPTY);
    // Every try posts a pipe event, even a failed one
    post(sending, result, request.moved());
    return result;
}

SceInt32 MsgPipe::cancel(SceUInt32 *num_send_wait_threads, SceUInt32 *num_receive_wait_threads) {
    const auto guard = lock();
    if (!guard)
        return SCE_KERNEL_ERROR_UNKNOWN_MSG_PIPE_ID;

    const auto nb_senders = static_cast<SceUInt32>(senders.complete_all(SCE_KERNEL_ERROR_WAIT_CANCEL));
    const auto nb_receivers = static_cast<SceUInt32>(receivers.complete_all(SCE_KERNEL_ERROR_WAIT_CANCEL));
    // Empty the ring without posting an event. Each canceled waiter posts its own when it returns.
    data_buffer.Clear();
    pattern &= ~SCE_KERNEL_EVENT_DATA_EXIST;

    if (num_send_wait_threads)
        *num_send_wait_threads = nb_senders;
    if (num_receive_wait_threads)
        *num_receive_wait_threads = nb_receivers;

    return SCE_KERNEL_OK;
}

SceInt32 MsgPipe::info(SceKernelMppInfo &info) const {
    const auto guard = lock();
    if (!guard)
        return SCE_KERNEL_ERROR_UNKNOWN_MSG_PIPE_ID;

    info = {
        .mppId = uid,
        .attr = attr,
        .bufSize = static_cast<SceInt32>(data_buffer.Capacity()),
        .freeSize = static_cast<SceInt32>(data_buffer.Free()),
        .numSendWaitThreads = static_cast<SceInt32>(senders.size()),
        .numReceiveWaitThreads = static_cast<SceInt32>(receivers.size()),
    };
    strncpy(info.name, name.c_str(), KERNELOBJECT_MAX_NAME_LENGTH + 1);
    return SCE_KERNEL_OK;
}
