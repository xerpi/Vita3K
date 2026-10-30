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

#include <kernel/types.h>
#include <util/align.h>
#include <util/log.h>

#include <algorithm>
#include <tuple>

SceInt32 check_object_name(const char *name, SceUInt32 attr) {
    if (!name)
        return SCE_KERNEL_ERROR_ILLEGAL_ADDR;
    if ((strlen(name) > KERNELOBJECT_MAX_NAME_LENGTH) && (attr & SCE_KERNEL_ATTR_OPENABLE))
        return SCE_KERNEL_ERROR_UID_NAME_TOO_LONG;
    return SCE_KERNEL_OK;
}

std::string object_name(const char *name) {
    return { name, strnlen(name, KERNELOBJECT_MAX_NAME_LENGTH) };
}

// Returns the waiter queue order for an object's attr
static QueueOrder queue_order(SceUInt32 attr) {
    return attr & SCE_KERNEL_ATTR_TH_PRIO ? QueueOrder::priority : QueueOrder::fifo;
}

// ************************
// * Multiple event waits *
// ************************

EventPtr find_event(const KernelObjects &objects, SceUID uid) {
    if (SimpleEventPtr event = objects.find<SimpleEvent>(uid))
        return event;
    if (TimerPtr timer = objects.find<Timer>(uid))
        return timer;
    if (MsgPipePtr pipe = objects.find<MsgPipe>(uid))
        return pipe;
    return nullptr;
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
        if (const auto object = kernel.objects.find(uid))
            objects[i] = object->uid;
        else
            objects[i] = kernel.get_thread(uid) ? uid : NO_OBJECT;
        Entry &entry = entries[i].emplace(*this, Event::Request{ .pattern = wait_list[i].eventPattern, .multi = this, .index = i });
        entry.event = find_event(kernel.objects, uid);
        // Entries for the same object all report the UID passed in the first such entry
        const auto named = std::span{ objects }.first(i + 1);
        entry.result_uid = wait_list[std::ranges::find(named, objects[i]) - named.begin()].eventId;
    }
}

Deadline MultiEventWait::watch(Deadline until) {
    fire_due(until);
    // Timer events only fire when someone checks the timer. Look again at the next event of each
    // timer this thread is still queued on.
    Deadline look = Deadline::max();
    for (SceUInt32 i = 0; i < count; i++) {
        if (queued.test(i))
            look = std::min(look, entries[i]->event->watch_until(entries[i]->token));
    }
    return look;
}

WaitResult MultiEventWait::run() {
    while (true) {
        // Run pending callbacks before checking any event
        if (const WaitResult r = run_callbacks(); !r)
            return r;
        scan();
        if (const std::optional<WaitResult> done = arm()) {
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
        // Run callbacks notified during the sleep before returning. The pass runs even if the flag
        // isn't set yet, so it also takes a notification whose notify hasn't marked the thread.
        if (callbacks) {
            thread.process_callbacks();
            const std::lock_guard<std::mutex> lock(thread.mutex);
            if (thread.exiting())
                return std::unexpected{ ThreadExiting{} };
        }
        return ended;
    }
}

bool MultiEventWait::running() {
    const std::lock_guard<std::mutex> lock(thread.mutex);
    return scanning || thread.still_waits();
}

bool MultiEventWait::accept(SceUInt32 index, SceInt32 result, SceUInt32 bits, SceUInt64 user_data) {
    const std::lock_guard<std::mutex> lock(thread.mutex);
    // Tokens are only queued between the scan and the cleanup, and the thread has no other wait
    // meanwhile. If the thread is neither scanning nor waiting, this wait has already ended.
    if (finished.test(index) || !(scanning || thread.still_waits()))
        return false;
    ended[finished.count()] = { entries[index]->result_uid, result, bits, 0, user_data };
    finished.set(index);
    // Finish the wait on the first result in OR mode or the last one in AND mode. During the scan,
    // arm() decides instead.
    if (!scanning && (mode == Mode::any || finished.count() == count))
        end_locked(SCE_KERNEL_OK);
    return true;
}

std::span<const SceKernelResultEvent> MultiEventWait::results() const {
    return std::span{ ended }.first(finished.count());
}

void MultiEventWait::scan() {
    {
        const std::lock_guard<std::mutex> lock(thread.mutex);
        scanning = true;
    }
    queued.reset();
    // In OR mode, stop queueing after the first result, but keep collecting the results of the rest
    bool queueing = true;
    for (SceUInt32 i = 0; i < count; i++) {
        {
            const std::lock_guard<std::mutex> lock(thread.mutex);
            if (finished.test(i))
                continue;
        }
        const SceInt32 scanned = entries[i]->event ? entries[i]->event->scan_token(entries[i]->token, queueing) : SCE_KERNEL_ERROR_UNKNOWN_EVENT_ID;
        if (scanned == SCE_KERNEL_ERROR_UNKNOWN_EVENT_ID)
            accept(i, SCE_KERNEL_ERROR_UNKNOWN_EVENT_ID, SCE_KERNEL_EVENT_ERROR);
        if (scanned != SCE_KERNEL_ERROR_EVENT_COND)
            queueing = queueing && mode == Mode::all;
        else if (queueing)
            queued.set(i);
    }
}

std::optional<WaitResult> MultiEventWait::arm() {
    const std::lock_guard<std::mutex> lock(thread.mutex);
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
    return thread.begin_locked(*this);
}

void MultiEventWait::fire_due(Deadline until) {
    // Fire one event at a time, earliest first, so an OR wait gets the one that was due first. A
    // timer that turns out not to be due on a fresh clock read is skipped until the next check.
    Entries passed;
    while (true) {
        SceUInt32 earliest = count;
        Deadline earliest_at = until;
        for (SceUInt32 i = 0; i < count; i++) {
            if (!queued.test(i) || passed.test(i))
                continue;
            const Deadline at = entries[i]->event->next_fire();
            if (at <= earliest_at) {
                earliest = i;
                earliest_at = at;
            }
        }
        if (earliest == count)
            return;
        if (!entries[earliest]->event->update(until))
            passed.set(earliest);
    }
}

void MultiEventWait::dequeue() {
    for (SceUInt32 i = 0; i < count; i++) {
        if (queued.test(i))
            entries[i]->event->remove_token(entries[i]->token);
    }
}

// *****************
// * Simple events *
// *****************

Event::Event(SceUInt32 attr, const char *name, SceUInt32 init_pattern, SceUInt64 init_user_data, Timer *timer)
    : attr(attr)
    , name(object_name(name))
    , waiters(queue_order(attr))
    , pattern(init_pattern | SCE_KERNEL_EVENT_CREATE)
    , last_user_data(init_user_data)
    , timer(timer) {}

// The create event comes with the creating thread's user data, like the other system events
SimpleEvent::SimpleEvent(SceUInt32 attr, const char *name, SceUInt32 init_pattern, SceUID creator)
    : WithUidClass(attr, name, init_pattern, system_event_user_data(creator)) {}

SceInt32 SimpleEvent::check_create(const char *name, SceUInt32 attr, SceUInt32 init_pattern) {
    if (const SceInt32 error = check_object_name(name, attr))
        return error;
    // The notify attribute is accepted and reported, nothing reads it
    if (attr & ~(SCE_KERNEL_EVENT_ATTR_AUTO_RESET | SCE_KERNEL_ATTR_TH_PRIO | SCE_KERNEL_ATTR_OPENABLE | SCE_KERNEL_ATTR_NOTIFY_CB_WAKEUP_ONLY))
        return SCE_KERNEL_ERROR_ILLEGAL_ATTR;
    if (init_pattern & ~SCE_KERNEL_EVENT_USER_DEFINED_MASK)
        return SCE_KERNEL_ERROR_ILLEGAL_PATTERN;
    return SCE_KERNEL_OK;
}

void Event::on_delete(SceUID deleter) {
    // The waits for the delete event get it with the deleting thread's user data, then the others end
    fire(SCE_KERNEL_EVENT_DELETE, system_event_user_data(deleter), true);
    end_waits(SCE_KERNEL_ERROR_WAIT_DELETE);
}

bool Event::take(const Request &request) {
    const SceUInt32 bits = pattern & request.pattern;
    if (!bits)
        return false;
    if (request.multi) {
        // If the multiple event wait already ended, leave the bits on the event
        if (!request.multi->accept(request.index, SCE_KERNEL_OK, bits, last_user_data))
            return true;
    } else {
        if (request.result_pattern)
            *request.result_pattern = bits;
        if (request.user_data)
            *request.user_data = last_user_data;
    }
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
    if (queue) {
        waiters.push(token, token.wait.thread.get_priority());
    }
    return SCE_KERNEL_ERROR_EVENT_COND;
}

void Event::remove_token(WaitNode<Request> &token) {
    const auto guard = lock();
    if (!guard)
        return;
    // Serve the waiters behind the removed token, since it may have been blocking them
    waiters.remove(token);
    serve_locked();
}

Deadline Event::next_fire() {
    const auto guard = lock();
    return guard ? event_deadline() : Deadline::max();
}

Deadline Event::watch_until(const WaitNode<Request> &token) {
    const auto guard = lock();
    if (!guard || !waiters.contains(token))
        return Deadline::max();
    return event_deadline();
}

bool Event::update(Deadline until) {
    const auto guard = lock();
    return guard && event_deadline() <= until && catch_up();
}

bool Event::catch_up() {
    return timer && timer->catch_up();
}

Deadline Event::event_deadline() const {
    return timer ? timer->event_deadline() : Deadline::max();
}

// A single event wait. It also fires the timer events that come due while it waits, since every
// waiter watches the clock.
class Event::Wait final : public QueuedThreadWait<Event, Event::Request> {
public:
    using QueuedThreadWait::QueuedThreadWait;

private:
    Deadline watch(Deadline until) override {
        while (owner.event_deadline() <= until && owner.catch_up()) {
        }
        return owner.event_deadline();
    }
};

SceInt32 Event::wait(ThreadState &thread, SceUInt32 wait_pattern, SceUInt32 *result_pattern, SceUInt64 *user_data, WaitOptions &options) {
    auto guard = lock();
    if (!guard)
        return SCE_KERNEL_ERROR_UNKNOWN_EVENT_ID;

    // A wait for pattern 0 matches nothing, so it lasts until its timeout, a cancel or a delete
    Wait wait(*this, waiters, std::move(guard), thread, { SCE_KERNEL_WAITTYPE_EVENT, uid }, Request{ .pattern = wait_pattern, .result_pattern = result_pattern, .user_data = user_data }, options);
    // The result of the previous pass. Bits taken right after a callback notify return
    // NOTIFY_CALLBACK.
    WaitResult last = SCE_KERNEL_OK;
    while (true) {
        if (const WaitResult ran = wait.run_callbacks(); !ran) {
            last = ran;
            break;
        }
        if (was_deleted()) {
            last = SCE_KERNEL_ERROR_WAIT_DELETE;
            break;
        }
        // Fire timer events that are already due before taking. A pulse that was due before this
        // wait started must not reach the waiter.
        catch_up();
        if (take(wait.node.request))
            break;
        last = wait.block();
        if (last && *last == SCE_KERNEL_ERROR_NOTIFY_CALLBACK)
            continue;
        // Run the callbacks notified during the wait. Only the thread exiting changes the result.
        if (last) {
            if (const WaitResult ran = wait.run_callbacks(); !ran)
                last = ran;
        }
        break;
    }
    const SceInt32 result = guest_result(last);
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

    const Request request{ .pattern = wait_pattern, .result_pattern = result_pattern, .user_data = user_data };
    return take(request) ? SCE_KERNEL_OK : SCE_KERNEL_ERROR_EVENT_COND;
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

void Event::fire(SceUInt32 set_pattern, SceUInt64 user_data, bool is_set, Deadline due) {
    // A set or pulse of no bits changes nothing, not even the user data
    if (set_pattern == 0)
        return;

    pattern |= set_pattern;
    last_user_data = user_data;
    serve_locked(due);

    // A pulse clears its bits once the waiters had them, also the ones set before it, and keeps its
    // user data
    if (!is_set)
        pattern &= ~set_pattern;
}

void Event::serve_locked(Deadline due) {
    for (auto it = waiters.begin(); it != waiters.end();) {
        auto &node = *it++;
        const Request &request = node.request;
        // A running wait whose deadline came before the timer event times out first. The token of a
        // multiple event wait that already ended still goes to take(), which unlinks it.
        if (node.wait.deadline < due && (!request.multi || request.multi->running()))
            continue;
        if (!take(request))
            continue;
        if (request.multi)
            waiters.remove(node);
        else
            waiters.complete(node, SCE_KERNEL_OK);
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
    // Also cancel the pending sceKernelSetTimerEvent event. The timer keeps running and keeps the
    // event interval and type.
    if (timer) {
        timer->armed = false;
        timer->is_repeat = false;
        timer->next_event = std::numeric_limits<uint64_t>::max();
    }
    if (num_wait_threads)
        *num_wait_threads = nb_threads;

    return SCE_KERNEL_OK;
}

SceInt32 Event::info(SceKernelEventInfo &info) {
    const auto guard = lock();
    if (!guard)
        return SCE_KERNEL_ERROR_UNKNOWN_EVENT_ID;
    catch_up();

    info = {};
    info.eventId = uid;
    strncpy(info.name, name.c_str(), KERNELOBJECT_MAX_NAME_LENGTH + 1);
    info.attr = attr;
    info.eventPattern = pattern;
    info.userData = last_user_data;
    info.numWaitThreads = static_cast<SceUInt32>(waiters.size());
    return SCE_KERNEL_OK;
}

// *********
// * Timer *
// *********

inline uint64_t get_current_time() {
    return std::chrono::duration_cast<std::chrono::microseconds>(
        std::chrono::high_resolution_clock::now().time_since_epoch())
        .count();
}

Timer::Timer(SceUInt32 attr, const char *name, SceUID creator)
    : WithUidClass(attr, name, 0, system_event_user_data(creator), this) {}

// A timer event is due at least this long after it was queued or after the event before it
constexpr uint64_t TIMER_EVENT_FLOOR = 200;

bool Timer::catch_up() {
    const uint64_t now = get_current_time();
    if (next_event > now)
        return false;
    fire(SCE_KERNEL_EVENT_TIMER, 0, !is_pulse, event_deadline());
    // A one shot event leaves the queue but stays armed, so a start or a set time can queue it again
    if (!is_repeat) {
        next_event = std::numeric_limits<uint64_t>::max();
        return true;
    }
    // A repeating event is due again an interval after it was due. With nobody waiting, the events
    // it missed meanwhile change nothing, so they are skipped.
    const uint64_t interval = std::max(event_interval, TIMER_EVENT_FLOOR);
    uint64_t next = next_event + interval;
    if (waiters.empty() && next <= now)
        next += ((now - next) / interval + 1) * interval;
    next_event = std::max(next, now + TIMER_EVENT_FLOOR);
    schedule = next_event - base;
    return true;
}

void Timer::queue_event(uint64_t deadline) {
    next_event = std::max(deadline, get_current_time() + TIMER_EVENT_FLOOR);
    // The waiters sleep until the old event time, so wake them to see the new one
    for (const auto &waiter : waiters)
        waiter.wait.thread.wake();
}

Deadline Timer::event_deadline() const {
    if (next_event == std::numeric_limits<uint64_t>::max())
        return Deadline::max();
    // An overdue event returns a time in the past, so a bounded update() still fires it
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
    else
        next_event = std::numeric_limits<uint64_t>::max();

    return SCE_KERNEL_OK;
}

SceInt32 Timer::start() {
    const auto guard = lock();
    if (!guard)
        return SCE_KERNEL_ERROR_UNKNOWN_TIMER_ID;

    if (is_started)
        return 1;
    catch_up();

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

    info = {};
    info.timerId = uid;
    strncpy(info.name, name.c_str(), KERNELOBJECT_MAX_NAME_LENGTH + 1);
    info.attr = attr;
    info.active = is_started;
    // The base of a running timer, the count of a stopped one
    info.baseTime = is_started ? base : stopped_count;
    info.currentTime = count();
    info.schedule = armed ? schedule : 0;
    info.interval = event_interval;
    info.type = is_pulse ? SCE_KERNEL_TIMER_TYPE_PULSE_EVENT : SCE_KERNEL_TIMER_TYPE_SET_EVENT;
    info.repeat = is_repeat;
    info.numWaitThreads = static_cast<SceUInt32>(waiters.size());
    return SCE_KERNEL_OK;
}

// *********
// * Mutex *
// *********

Mutex::Mutex(SceUInt32 attr, const char *name, SceInt32 init_count, ThreadStatePtr thread, Ptr<SceKernelLwMutexWork> workarea)
    : attr(attr)
    , name(object_name(name))
    , init_count(init_count)
    , workarea(workarea)
    , waiters(queue_order(attr))
    , lock_count(init_count)
    , owner(init_count > 0 && thread ? thread->id : 0) {}

SceInt32 Mutex::check_create(const char *name, SceUInt32 attr, SceInt32 init_count, bool kernel) {
    if (const SceInt32 error = check_object_name(name, attr))
        return error;
    if (!kernel && attr & ~(SCE_KERNEL_MUTEX_ATTR_RECURSIVE | SCE_KERNEL_MUTEX_ATTR_CEILING | SCE_KERNEL_ATTR_TH_PRIO | SCE_KERNEL_ATTR_OPENABLE))
        return SCE_KERNEL_ERROR_ILLEGAL_ATTR;
    if (init_count < 0)
        return SCE_KERNEL_ERROR_ILLEGAL_COUNT;
    if (init_count > 1 && !(attr & SCE_KERNEL_MUTEX_ATTR_RECURSIVE))
        return SCE_KERNEL_ERROR_ILLEGAL_COUNT;
    return SCE_KERNEL_OK;
}

SceInt32 Mutex::check_count(SceInt32 count) const {
    if (count <= 0 || (count > 1 && !(attr & SCE_KERNEL_MUTEX_ATTR_RECURSIVE)))
        return SCE_KERNEL_ERROR_ILLEGAL_COUNT;
    return SCE_KERNEL_OK;
}

void Mutex::on_delete(SceUID) {
    // End the waits on the conditions first, then the waits on the mutex. The conditions use the
    // mutex lock, which is held here. Condvar::wait() turns this into the mutex delete error.
    for (Condvar &cond : conditions)
        cond.waiters.complete_all(SCE_KERNEL_ERROR_WAIT_DELETE_MUTEX);
    waiters.complete_all(SCE_KERNEL_ERROR_WAIT_DELETE);
    owner = 0;
}

SceInt32 Mutex::try_take(MemState &mem, const ThreadState &thread, SceInt32 count) {
    bool is_recursive = (attr & SCE_KERNEL_MUTEX_ATTR_RECURSIVE);

    // Already owned
    if (owner) {
        // Owned by ourselves
        if (owner == thread.id) {
            if (is_recursive) {
                if (lightweight()) {
                    // A lightweight count wraps past the maximum, and a lock that would leave it
                    // below 0 fails
                    const int64_t sum = static_cast<int64_t>(lock_count) + count;
                    if (sum < 0)
                        return SCE_KERNEL_ERROR_LW_MUTEX_LOCK_OVF;
                    lock_count = static_cast<SceInt32>(static_cast<SceUInt32>(sum));
                    workarea.get(mem)->lockCount = lock_count;
                    return SCE_KERNEL_OK;
                }
                if (count > std::numeric_limits<SceInt32>::max() - lock_count)
                    return SCE_KERNEL_ERROR_MUTEX_LOCK_OVF;
                lock_count += count;
                return SCE_KERNEL_OK;
            }
            if (lightweight())
                return SCE_KERNEL_ERROR_LW_MUTEX_RECURSIVE;

            return SCE_KERNEL_ERROR_MUTEX_RECURSIVE;
        }
        // Owned by someone else
        if (lightweight())
            return SCE_KERNEL_ERROR_LW_MUTEX_FAILED_TO_OWN;

        return SCE_KERNEL_ERROR_MUTEX_FAILED_TO_OWN;
    }
    // Not owned
    // Take ownership!

    lock_count += count;
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
    QueuedThreadWait wait(*this, waiters, std::move(guard), thread, { type, uid }, Request{ .lock_count = count }, options);
    // A relock runs the callbacks notified during the wait only if a delete ended it
    const WaitResult r = wait.run(relock ? FinalCallbackCheck::on_delete : FinalCallbackCheck::after_wait, [&]() -> std::optional<SceInt32> {
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
    const auto guard = lock();
    // Deleted meanwhile, which a lightweight mutex reports as not owned
    if (!guard)
        return lightweight() ? SCE_KERNEL_ERROR_LW_MUTEX_NOT_OWNED : SCE_KERNEL_ERROR_UNKNOWN_MUTEX_ID;
    return release_held(mem, thread, unlock_count);
}

SceInt32 Mutex::release_held(MemState &mem, ThreadState &thread, SceInt32 unlock_count) {
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
    serve_locked();
    update_workarea(mem);

    return SCE_KERNEL_OK;
}

void Mutex::release_ended_owner(const ThreadState &thread) {
    const auto guard = lock();
    if (!guard || owner != thread.id)
        return;
    lock_count = 0;
    owner = 0;
    serve_locked();
}

void Mutex::serve_locked() {
    if (owner)
        return;
    // Give the mutex to the first waiter and end its wait
    if (auto *const node = waiters.front()) {
        lock_count = node->request.lock_count;
        owner = node->wait.thread.id;
        waiters.complete(*node);
    }
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

SceInt32 LwMutex::info(SceKernelLwMutexInfo &info) const {
    const auto guard = lock();
    if (!guard)
        return SCE_KERNEL_ERROR_UNKNOWN_LW_MUTEX_ID;

    info = {};
    info.uid = uid;
    strncpy(info.name, name.c_str(), KERNELOBJECT_MAX_NAME_LENGTH + 1);
    info.attr = attr;
    info.pWork = workarea;
    info.initCount = init_count;
    info.currentCount = lock_count;
    info.currentOwnerId = owner;
    info.numWaitThreads = static_cast<SceUInt32>(waiters.size());
    return SCE_KERNEL_OK;
}

SceInt32 HeavyMutex::info(SceKernelMutexInfo &info) const {
    const auto guard = lock();
    if (!guard)
        return SCE_KERNEL_ERROR_UNKNOWN_MUTEX_ID;

    info = {};
    info.mutexId = uid;
    strncpy(info.name, name.c_str(), KERNELOBJECT_MAX_NAME_LENGTH + 1);
    info.attr = attr;
    info.initCount = init_count;
    info.currentCount = lock_count;
    info.currentOwnerId = owner;
    info.numWaitThreads = static_cast<SceUInt32>(waiters.size());
    return SCE_KERNEL_OK;
}

// **************
// * RWLock *
// **************

RWLock::RWLock(SceUInt32 attr, const char *name)
    : attr(attr)
    , name(object_name(name))
    , waiters(queue_order(attr)) {}

SceInt32 RWLock::check_create(const char *name, SceUInt32 attr) {
    if (const SceInt32 error = check_object_name(name, attr))
        return error;
    if (attr & ~(SCE_KERNEL_RW_LOCK_ATTR_RECURSIVE | SCE_KERNEL_RW_LOCK_ATTR_CEILING | SCE_KERNEL_ATTR_TH_PRIO | SCE_KERNEL_ATTR_OPENABLE))
        return SCE_KERNEL_ERROR_ILLEGAL_ATTR;
    return SCE_KERNEL_OK;
}

void RWLock::on_delete(SceUID) {
    waiters.complete_all(SCE_KERNEL_ERROR_WAIT_DELETE);
}

WaitNode<RWLock::Request> *RWLock::first_writer() {
    for (auto &node : waiters) {
        if (node.request.is_write)
            return &node;
    }
    return nullptr;
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
    if (!is_write && first_writer())
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

    QueuedThreadWait wait(*this, waiters, std::move(guard), thread, { SCE_KERNEL_WAITTYPE_RW_LOCK, uid }, Request{ .is_write = is_write }, options);
    return guest_result(wait.run(FinalCallbackCheck::after_wait, [&]() -> std::optional<SceInt32> {
        if (was_deleted())
            return SCE_KERNEL_ERROR_WAIT_DELETE;
        // A writer needs the lock to be free. A reader needs no writer and an empty queue. A lock
        // that this thread's callbacks took blocks it like any other lock.
        if (write_owner || (is_write ? lock_count != 0 : !waiters.empty()))
            return std::nullopt;
        lock_count++;
        if (is_write)
            write_owner = thread.id;
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

    if (is_write) {
        // Only the writer may write unlock
        if (write_owner != thread.id)
            return SCE_KERNEL_ERROR_RW_LOCK_NOT_OWNED;
    } else {
        if (write_owner)
            return SCE_KERNEL_ERROR_RW_LOCK_FAILED_TO_UNLOCK;
        if (lock_count == 0)
            return SCE_KERNEL_ERROR_RW_LOCK_UNLOCK_UDF;
    }

    if (--lock_count == 0)
        write_owner = 0;
    serve_locked();
    return SCE_KERNEL_OK;
}

void RWLock::release_ended_writer(const ThreadState &thread) {
    const auto guard = lock();
    if (!guard)
        return;
    if (write_owner != thread.id)
        return;
    lock_count = 0;
    write_owner = 0;
    serve_locked();
}

SceInt32 RWLock::info(SceKernelRWLockInfo &info) const {
    const auto guard = lock();
    if (!guard)
        return SCE_KERNEL_ERROR_UNKNOWN_RW_LOCK_ID;

    info = {};
    info.rwLockId = uid;
    strncpy(info.name, name.c_str(), KERNELOBJECT_MAX_NAME_LENGTH + 1);
    info.attr = attr;
    info.lockCount = lock_count;
    info.writeOwnerId = write_owner;
    const auto writers = static_cast<SceUInt32>(std::ranges::count_if(waiters, [](const auto &node) { return node.request.is_write; }));
    info.numReadWaitThreads = static_cast<SceUInt32>(waiters.size()) - writers;
    info.numWriteWaitThreads = writers;
    return SCE_KERNEL_OK;
}

void RWLock::serve_locked() {
    if (write_owner)
        return;

    // Without a waiting writer every waiting reader gets in, also next to readers that hold the lock.
    // A waiting writer goes first, even before readers that waited longer, once nobody holds it.
    auto *const writer = first_writer();
    if (!writer) {
        lock_count += static_cast<SceInt32>(waiters.complete_all(SCE_KERNEL_OK));
    } else if (lock_count == 0) {
        // Give the lock to the writer and end its wait
        write_owner = writer->wait.thread.id;
        lock_count = 1;
        waiters.complete(*writer);
    }
}

SceInt32 RWLock::cancel(ThreadState &thread, SceUInt32 *num_read_wait_threads, SceUInt32 *num_write_wait_threads, SceInt32 flag) {
    const auto guard = lock();
    if (!guard)
        return SCE_KERNEL_ERROR_UNKNOWN_RW_LOCK_ID;

    // Cancel the writers first, then the readers
    SceUInt32 nb_writers = 0;
    for (auto it = waiters.begin(); it != waiters.end();) {
        auto &node = *it++;
        if (!node.request.is_write)
            continue;
        waiters.complete(node, SCE_KERNEL_ERROR_WAIT_CANCEL);
        ++nb_writers;
    }
    const auto nb_readers = static_cast<SceUInt32>(waiters.complete_all(SCE_KERNEL_ERROR_WAIT_CANCEL));

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
    : attr(attr)
    , name(object_name(name))
    , init_val(init_val)
    , max(max_val)
    , waiters(queue_order(attr))
    , val(init_val) {}

SceInt32 Semaphore::check_create(const char *name, SceUInt32 attr, SceInt32 init_val, SceInt32 max_val) {
    if (const SceInt32 error = check_object_name(name, attr))
        return error;
    if (attr & ~(SCE_KERNEL_ATTR_TH_PRIO | SCE_KERNEL_ATTR_OPENABLE))
        return SCE_KERNEL_ERROR_ILLEGAL_ATTR;
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

    QueuedThreadWait wait(*this, waiters, std::move(guard), thread, { SCE_KERNEL_WAITTYPE_SEMAPHORE, uid }, Request{ .need_count = need_count }, options);
    return guest_result(wait.run(FinalCallbackCheck::after_wait, [&]() -> std::optional<SceInt32> {
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
    if (count > max - val) {
        return SCE_KERNEL_ERROR_SEMA_OVF;
    }
    val += count;
    serve_locked();

    return SCE_KERNEL_OK;
}

void Semaphore::serve_locked() {
    while (auto *const node = waiters.front()) {
        const SceInt32 need_count = node->request.need_count;
        if (val < need_count)
            break;
        val -= need_count;
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
    if (set_count < 0) {
        val = init_val;
    } else {
        val = set_count;
    }
    if (num_wait_threads)
        *num_wait_threads = nb_threads;
    return SCE_KERNEL_OK;
}

SceInt32 Semaphore::info(SceKernelSemaInfo &info) const {
    const auto guard = lock();
    if (!guard)
        return SCE_KERNEL_ERROR_UNKNOWN_SEMA_ID;

    info = {};
    info.semaId = uid;
    strncpy(info.name, name.c_str(), KERNELOBJECT_MAX_NAME_LENGTH + 1);
    info.attr = attr;
    info.initCount = init_val;
    info.currentCount = val;
    info.maxCount = max;
    info.numWaitThreads = static_cast<SceUInt32>(waiters.size());
    return SCE_KERNEL_OK;
}

// **********************
// * Condition Variable *
// **********************

// The condition shares its mutex's lock, because its waits always change together with the mutex
// state and deleting the mutex ends them.
Condvar::Condvar(SceUInt32 attr, const char *name, MutexPtr associated_mutex)
    : KernelObject(associated_mutex.get())
    , attr(attr)
    , name(object_name(name))
    , associated_mutex(std::move(associated_mutex))
    , waiters(queue_order(attr)) {}

Condvar::~Condvar() {
    // A condition that was never deleted is still in the condition list of its mutex
    const auto guard = lock();
    if (guard)
        associated_mutex->conditions.remove(*this);
}

SceInt32 Condvar::attach() {
    const auto guard = lock();
    // This is also the mutex lock, so the mutex can't be deleted meanwhile
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
        // The condition shares the mutex lock, so the unlock and the queueing happen atomically
        if (const SceInt32 error = associated_mutex->release_held(mem, thread, 1))
            return error;
        // A condition holds no signal for later, so the wait only queues and sleeps
        QueuedThreadWait wait(*this, waiters, std::move(guard), thread, { lightweight() ? SCE_KERNEL_WAITTYPE_LW_COND_SIGNAL : SCE_KERNEL_WAITTYPE_COND_SIGNAL, uid }, Request{}, options);
        // Only a delete runs the callbacks notified during the wait
        signaled = wait.run(FinalCallbackCheck::on_delete, [&]() -> std::optional<SceInt32> {
            if (was_deleted())
                return SCE_KERNEL_ERROR_WAIT_DELETE;
            return std::nullopt;
        });
        // The condition was deleted during the wait or while callbacks ran
        if (signaled && *signaled == SCE_KERNEL_ERROR_WAIT_DELETE)
            return lightweight() ? SCE_KERNEL_ERROR_WAIT_DELETE_LW_COND : SCE_KERNEL_ERROR_WAIT_DELETE_COND;
        // The mutex was deleted while the thread was queued, so there is nothing to relock
        if (signaled && *signaled == SCE_KERNEL_ERROR_WAIT_DELETE_MUTEX)
            return lightweight() ? SCE_KERNEL_ERROR_WAIT_DELETE_LW_MUTEX : SCE_KERNEL_ERROR_WAIT_DELETE_MUTEX;
        // A timed out wait still relocks the mutex
        if (!signaled || (*signaled != SCE_KERNEL_OK && *signaled != SCE_KERNEL_ERROR_WAIT_TIMEOUT))
            return guest_result(signaled);
        // If the condition was deleted after the wait ended, skip the relock and return the wait
        // result
        if (was_deleted())
            return *signaled;
    }
    // Taking the mutex back is still part of the condition variable wait, a wait on the mutex with no
    // timeout of its own that leaves the time left as it was at the signal
    WaitOptions relock_options(nullptr, options.callbacks);
    const SceInt32 result = associated_mutex->acquire(mem, thread, 1, relock_options, true);
    if (*signaled == SCE_KERNEL_ERROR_WAIT_TIMEOUT && result == SCE_KERNEL_OK)
        return SCE_KERNEL_ERROR_WAIT_TIMEOUT;
    // A delete during the relock gives the mutex delete error. If the mutex was deleted while the
    // thread was off the condition queue, after a signal or while running a callback, the relock
    // gives the unknown mutex error. A heavy mutex returns that error. A lightweight mutex gives
    // the delete error instead, so the waiter isn't left blocked.
    if (result == SCE_KERNEL_ERROR_WAIT_DELETE || result == SCE_KERNEL_ERROR_UNKNOWN_LW_MUTEX_ID)
        return lightweight() ? SCE_KERNEL_ERROR_WAIT_DELETE_LW_MUTEX : SCE_KERNEL_ERROR_WAIT_DELETE_MUTEX;
    // only heavy mutexes can be canceled
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

SceUInt32 Condvar::num_waiters() const {
    const auto guard = lock();
    return guard ? static_cast<SceUInt32>(waiters.size()) : 0;
}

SceInt32 HeavyCond::info(SceKernelCondInfo &info) const {
    const auto guard = lock();
    if (!guard)
        return SCE_KERNEL_ERROR_UNKNOWN_COND_ID;

    info = {};
    info.condId = uid;
    strncpy(info.name, name.c_str(), KERNELOBJECT_MAX_NAME_LENGTH + 1);
    info.attr = attr;
    // The condition shares the mutex's lock, so the mutex's delete is either done or not started
    info.mutexId = associated_mutex->was_deleted() ? -1 : associated_mutex->uid;
    info.numWaitThreads = static_cast<SceUInt32>(waiters.size());
    return SCE_KERNEL_OK;
}

LwCond::LwCond(SceUInt32 attr, const char *name, MutexPtr associated_mutex, Ptr<SceKernelLwCondWork> workarea)
    : WithUidClass(attr, name, std::move(associated_mutex))
    , workarea(workarea) {}

SceInt32 LwCond::info(SceKernelLwCondInfo &info) const {
    const auto guard = lock();
    if (!guard)
        return SCE_KERNEL_ERROR_UNKNOWN_LW_COND_ID;

    info = {};
    info.uid = uid;
    strncpy(info.name, name.c_str(), KERNELOBJECT_MAX_NAME_LENGTH + 1);
    info.attr = attr;
    info.pWork = workarea;
    info.pLwMutex = associated_mutex->workarea;
    info.numWaitThreads = static_cast<SceUInt32>(waiters.size());
    return SCE_KERNEL_OK;
}

// **************
// * Event Flag *
// **************

EventFlag::EventFlag(SceUInt32 attr, const char *name, SceUInt32 init_pattern)
    : attr(attr)
    , name(object_name(name))
    , init_pattern(init_pattern)
    , waiters(queue_order(attr))
    , flags(init_pattern) {}

SceInt32 EventFlag::check_create(const char *name, SceUInt32 attr) {
    if (const SceInt32 error = check_object_name(name, attr))
        return error;
    if (attr & ~(SCE_EVENT_WAITMULTIPLE | SCE_KERNEL_ATTR_TH_PRIO | SCE_KERNEL_ATTR_OPENABLE))
        return SCE_KERNEL_ERROR_ILLEGAL_ATTR;
    return SCE_KERNEL_OK;
}

SceInt32 EventFlag::check_wait(SceUInt32 pattern, SceUInt32 wait_mode) {
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
    QueuedThreadWait wait(*this, waiters, std::move(guard), thread, { SCE_KERNEL_WAITTYPE_EVENTFLAG, uid }, Request{ .wait_mode = wait_mode, .pattern = pattern, .out_bits = out_bits }, options);
    WaitResult r;
    while (true) {
        if (r = wait.run_callbacks(); !r)
            break;
        if (was_deleted()) {
            r = SCE_KERNEL_ERROR_WAIT_DELETE;
            break;
        }
        if (take(wait.node.request)) {
            drop_hold();
            r = SCE_KERNEL_OK;
            break;
        }
        // A wait that times out right away returns these bits
        if (out_bits)
            *out_bits = flags;
        // The steps of block(), with the outputs of a wait the waiter ends itself
        if (const std::optional<WaitResult> refused = wait.begin()) {
            r = *refused;
        } else {
            waiters.push(wait.node, wait.priority);
            // A timeout or a callback notify that takes the wait out writes the flags it left, and a
            // timeout gives the hold back, before the waiters behind are served. A thread exit does
            // neither.
            if (const std::optional<WaitResult> mine = wait.sleep()) {
                if (*mine) {
                    if (out_bits)
                        *out_bits = flags;
                    if (**mine == SCE_KERNEL_ERROR_WAIT_TIMEOUT)
                        drop_hold();
                }
                waiters.remove(wait.node);
                serve_locked();
            }
            r = wait.retire();
        }
        if (r && *r == SCE_KERNEL_ERROR_NOTIFY_CALLBACK)
            continue;
        // Run the callbacks notified during the wait. Only the thread exiting changes the result.
        if (r) {
            if (const WaitResult ran = wait.run_callbacks(); !ran)
                r = ran;
        }
        break;
    }
    // A deleted flag's bits can't change anymore. Write them out for a wait that the delete ended,
    // also when the delete came while the thread was running callbacks outside the queue.
    if (r && *r == SCE_KERNEL_ERROR_WAIT_DELETE && out_bits)
        *out_bits = flags;
    return guest_result(r);
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
    const Request request{ .wait_mode = wait_mode, .pattern = pattern, .out_bits = out_bits };
    if (take(request))
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
    serve_locked();

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

void EventFlag::serve_locked() {
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

    SceUInt32 nb_threads = 0;
    while (auto *const node = waiters.front()) {
        if (SceUInt32 *const out_bits = node->request.out_bits)
            *out_bits = pattern;
        drop_hold();
        waiters.complete(*node, SCE_KERNEL_ERROR_WAIT_CANCEL);
        ++nb_threads;
    }

    flags = pattern;

    if (num_wait_threads)
        *num_wait_threads = nb_threads;

    return SCE_KERNEL_OK;
}

SceInt32 EventFlag::info(SceKernelEventFlagInfo &info) const {
    const auto guard = lock();
    if (!guard)
        return SCE_KERNEL_ERROR_UNKNOWN_EVF_ID;

    info = {};
    info.evfId = uid;
    strncpy(info.name, name.c_str(), KERNELOBJECT_MAX_NAME_LENGTH + 1);
    info.attr = attr;
    info.initPattern = init_pattern;
    info.currentPattern = flags;
    info.numWaitThreads = static_cast<SceUInt32>(waiters.size());
    return SCE_KERNEL_OK;
}

// *************
// * Msg Pipe  *
// *************

// The create event carries the user data of the creating thread, same as for a simple event
MsgPipe::MsgPipe(SceUInt32 attr, const char *name, std::size_t buf_size, SceUID creator)
    : WithUidClass(attr, name, 0, system_event_user_data(creator))
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
    // Fire the delete event and end the event waits first, as for any event
    Event::on_delete(deleter);
    senders.complete_all(SCE_KERNEL_ERROR_WAIT_DELETE);
    receivers.complete_all(SCE_KERNEL_ERROR_WAIT_DELETE);
}

// The size of all the pieces together
static uint64_t total_size(std::span<const MsgPipeVector> pieces) {
    uint64_t total = 0;
    for (const MsgPipeVector &piece : pieces)
        total += piece.size;
    return total;
}

bool MsgPipe::Transfer::done() const {
    return full ? left == 0 : left < total;
}

SceSize MsgPipe::Transfer::moved() const {
    return total - left;
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

void MsgPipe::Transfer::advance(SceSize n) {
    offset += n;
    left -= n;
}

void MsgPipe::Transfer::report() const {
    if (size)
        *size = moved();
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
    SceSize at = receiver.peek ? std::min(receiver.total - receiver.left, held) : 0;
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

void MsgPipe::serve_locked() {
    // The first receiver goes first, so the pipe's bytes go out before those of waiting senders
    while (true) {
        if (auto *const receiver = receivers.front(); receiver && take(receiver->request))
            receivers.complete(*receiver);
        else if (auto *const sender = senders.front(); sender && give(sender->request))
            senders.complete(*sender);
        else
            break;
    }
    Event::serve_locked();
}

void MsgPipe::post(SceUInt32 done_bit, SceInt32 result, SceSize moved) {
    SceUInt32 bits = moved > 0 ? done_bit : 0;
    if (done_bit == SCE_KERNEL_EVENT_IN && moved > 0 && data_buffer.Used() > 0)
        bits |= SCE_KERNEL_EVENT_DATA_EXIST;
    // A suspended or stopped caller is not a pipe error
    if (result < 0 && result != SCE_KERNEL_ERROR_THREAD_STOPPED && result != SCE_KERNEL_ERROR_THREAD_SUSPENDED)
        bits |= SCE_KERNEL_EVENT_ERROR;
    if (done_bit == SCE_KERNEL_EVENT_OUT && data_buffer.Used() == 0)
        pattern &= ~SCE_KERNEL_EVENT_DATA_EXIST;
    // The number of bytes moved, or all ones on error
    const SceUInt32 size = moved | (result < 0 ? 0xFFFFFFFFU : 0);
    fire(bits, (static_cast<SceUInt64>(size) << 32) | static_cast<SceUInt32>(result), true);
}

SceInt32 MsgPipe::receive(ThreadState &thread, MemState &mem, SceUInt32 wait_mode, std::span<const MsgPipeVector> pieces, SceSize *size, WaitOptions &options) {
    if (!is_msg_pipe_mode(wait_mode))
        return SCE_KERNEL_ERROR_ILLEGAL_MODE;
    // DONT_WAIT behaves like a try, then runs callbacks if the call allows them
    if (wait_mode & SCE_KERNEL_MSG_PIPE_MODE_DONT_WAIT) {
        const SceInt32 result = try_receive(thread, mem, wait_mode, pieces, size);
        if (options.callbacks)
            thread.process_callbacks();
        return result;
    }
    const uint64_t total = total_size(pieces);
    if (total > data_buffer.Capacity())
        return SCE_KERNEL_ERROR_ILLEGAL_SIZE;

    return transfer(thread, false, Transfer{ .mem = mem, .pieces = pieces, .size = size, .total = static_cast<SceSize>(total), .full = (wait_mode & SCE_KERNEL_MSG_PIPE_MODE_FULL) != 0, .peek = (wait_mode & SCE_KERNEL_MSG_PIPE_MODE_DONT_REMOVE) != 0 }, options);
}

SceInt32 MsgPipe::send(ThreadState &thread, MemState &mem, SceUInt32 wait_mode, std::span<const MsgPipeVector> pieces, SceSize *size, WaitOptions &options) {
    if (!is_msg_pipe_mode(wait_mode))
        return SCE_KERNEL_ERROR_ILLEGAL_MODE;
    // DONT_WAIT behaves like a try, then runs callbacks if the call allows them
    if (wait_mode & SCE_KERNEL_MSG_PIPE_MODE_DONT_WAIT) {
        const SceInt32 result = try_send(thread, mem, wait_mode, pieces, size);
        if (options.callbacks)
            thread.process_callbacks();
        return result;
    }
    const uint64_t total = total_size(pieces);
    if (total > data_buffer.Capacity())
        return SCE_KERNEL_ERROR_ILLEGAL_SIZE;

    return transfer(thread, true, Transfer{ .mem = mem, .pieces = pieces, .size = size, .total = static_cast<SceSize>(total), .full = (wait_mode & SCE_KERNEL_MSG_PIPE_MODE_FULL) != 0 }, options);
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
    QueuedThreadWait wait(*this, queue, std::move(guard), thread, { SCE_KERNEL_WAITTYPE_MSG_PIPE, uid }, request, options);
    Transfer &mine = wait.node.request;
    WaitResult r;
    while (true) {
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
        r = wait.block(partly_served(queue));
        if (!r)
            return guest_result(r);
        if (*r != SCE_KERNEL_ERROR_NOTIFY_CALLBACK)
            break;
    }
    // The pipe event comes before the callbacks notified during the wait, even without a wait
    post(sending ? SCE_KERNEL_EVENT_IN : SCE_KERNEL_EVENT_OUT, *r, mine.moved());
    // Run the callbacks notified during the wait. Only the thread exiting changes the result.
    if (const WaitResult ran = wait.run_callbacks(); !ran)
        return guest_result(ran);
    return *r;
}

SceInt32 MsgPipe::try_receive(ThreadState &thread, MemState &mem, SceUInt32 wait_mode, std::span<const MsgPipeVector> pieces, SceSize *size) {
    if (!is_msg_pipe_mode(wait_mode))
        return SCE_KERNEL_ERROR_ILLEGAL_MODE;
    const uint64_t total = total_size(pieces);
    if (total > data_buffer.Capacity())
        return SCE_KERNEL_ERROR_ILLEGAL_SIZE;

    const auto guard = lock();
    if (!guard)
        return SCE_KERNEL_ERROR_UNKNOWN_MSG_PIPE_ID;
    Transfer receiver{ .mem = mem, .pieces = pieces, .size = size, .total = static_cast<SceSize>(total), .full = (wait_mode & SCE_KERNEL_MSG_PIPE_MODE_FULL) != 0, .peek = (wait_mode & SCE_KERNEL_MSG_PIPE_MODE_DONT_REMOVE) != 0 };
    // Behind the receivers already waiting, or short of bytes for all of a FULL receive, it fails
    // without writing a size
    const bool can_take = first_in_line(receivers, thread) && !(receiver.full && receiver.total > available());
    const SceInt32 result = can_take && take(receiver) ? SCE_KERNEL_OK : SCE_KERNEL_ERROR_MSG_PIPE_EMPTY;
    // Every try posts a pipe event, even a failed one
    post(SCE_KERNEL_EVENT_OUT, result, receiver.moved());
    return result;
}

SceInt32 MsgPipe::try_send(ThreadState &thread, MemState &mem, SceUInt32 wait_mode, std::span<const MsgPipeVector> pieces, SceSize *size) {
    if (!is_msg_pipe_mode(wait_mode))
        return SCE_KERNEL_ERROR_ILLEGAL_MODE;
    const uint64_t total = total_size(pieces);
    if (total > data_buffer.Capacity())
        return SCE_KERNEL_ERROR_ILLEGAL_SIZE;

    const auto guard = lock();
    if (!guard)
        return SCE_KERNEL_ERROR_UNKNOWN_MSG_PIPE_ID;
    Transfer sender{ .mem = mem, .pieces = pieces, .size = size, .total = static_cast<SceSize>(total), .full = (wait_mode & SCE_KERNEL_MSG_PIPE_MODE_FULL) != 0 };
    // Behind the senders already waiting, or short of room for all of a FULL send, it fails without
    // writing a size
    const bool can_give = first_in_line(senders, thread) && !(sender.full && sender.total > data_buffer.Free());
    const SceInt32 result = can_give && give(sender) ? SCE_KERNEL_OK : SCE_KERNEL_ERROR_MSG_PIPE_FULL;
    // Every try posts a pipe event, even a failed one
    post(SCE_KERNEL_EVENT_IN, result, sender.moved());
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

    info = {};
    info.mppId = uid;
    strncpy(info.name, name.c_str(), KERNELOBJECT_MAX_NAME_LENGTH + 1);
    info.attr = attr;
    info.bufSize = static_cast<SceInt32>(data_buffer.Capacity());
    info.freeSize = static_cast<SceInt32>(data_buffer.Free());
    info.numSendWaitThreads = static_cast<SceInt32>(senders.size());
    info.numReceiveWaitThreads = static_cast<SceInt32>(receivers.size());
    return SCE_KERNEL_OK;
}
