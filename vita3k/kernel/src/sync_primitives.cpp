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
#include <kernel/state.h>
#include <kernel/sync_primitives.h>

#include <kernel/types.h>
#include <util/align.h>
#include <util/log.h>

#include <algorithm>
#include <thread>

SyncPrimitive::SyncPrimitive(SceUInt32 attr, const char *name, const KernelObject *share_lock_with)
    : KernelObject(share_lock_with)
    , Waitable(lock_mutex())
    , attr(attr)
    , name(name, strnlen(name, KERNELOBJECT_MAX_NAME_LENGTH)) {}

SceInt32 SyncPrimitive::check_name(const char *name, SceUInt32 attr) {
    if (!name)
        return SCE_KERNEL_ERROR_ILLEGAL_ADDR;
    if ((strlen(name) > KERNELOBJECT_MAX_NAME_LENGTH) && (attr & SCE_KERNEL_ATTR_OPENABLE))
        return SCE_KERNEL_ERROR_UID_NAME_TOO_LONG;
    return SCE_KERNEL_OK;
}

// ************************
// * Multiple event waits *
// ************************

// Prototype only, never to be merged: whether VITA3K_PROTO_SCENARIO names scenario
static bool proto_scenario_is(std::string_view scenario) {
    static const char *const chosen = std::getenv("VITA3K_PROTO_SCENARIO");
    return chosen && scenario == chosen;
}

// Prototype only: holds the caller until reached() holds, polling, and aborts after 5 s so the
// scenario fails
static void proto_hold_until(std::string_view scenario, auto reached) {
    for (int i = 0; i < 5000; i++) {
        if (reached()) {
            LOG_INFO("proto {}: held until it happened", scenario);
            return;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    LOG_ERROR("proto {}: it never happened", scenario);
    std::abort();
}

EventPtr find_event(const KernelObjects &objects, SceUID uid) {
    if (SimpleEventPtr event = objects.find<SimpleEvent>(uid))
        return event;
    if (TimerPtr timer = objects.find<Timer>(uid))
        return timer;
    return objects.find<MsgPipe>(uid);
}

// The wait UID a thread in a multiple event wait reports, the UID of none of its events
constexpr SceUID MULTI_EVENTS_WAIT_ID = static_cast<SceUID>(0x80000000);
// The object UID of an entry whose UID names nothing, which no object has
constexpr SceUID NO_OBJECT = SCE_KERNEL_ERROR_UNKNOWN_EVENT_ID;

MultiEventWait::MultiEventWait(KernelState &kernel, ThreadStatePtr thread, std::span<const SceKernelWaitEvent> wait_list, bool any)
    : thread(std::move(thread))
    , any(any)
    , count(static_cast<SceUInt32>(wait_list.size())) {
    std::ranges::copy(wait_list, list.begin());
    for (SceUInt32 i = 0; i < count; i++) {
        const SceUID uid = list[i].eventId;
        events[i] = find_event(kernel.objects, uid);
        if (const auto object = kernel.objects.find(uid))
            objects[i] = object->uid;
        else
            objects[i] = kernel.get_thread(uid) ? uid : NO_OBJECT;
        recipients[i].wait = this;
        recipients[i].index = i;
        // Set in place, since a token can't be copied
        tokens[i].thread = this->thread;
        tokens[i].entry = { list[i].eventPattern, &recipients[i] };
    }
}

WaitResult MultiEventWait::run(SceUInt32 *time_left, bool callbacks) {
    const Deadline deadline = deadline_from(time_left);
    while (true) {
        // Before any event is looked up
        if (callbacks && thread->has_pending_callbacks()) {
            if (const WaitResult r = thread->run_callbacks(); !r)
                return r;
        }
        const uint64_t generation = thread->begin_scan();
        const Entries queued = scan(generation);
        // Prototype only: the first arm waits until a callback is pending
        if (proto_scenario_is("notify_during_scan") && !proto_held_scan) {
            proto_held_scan = true;
            proto_hold_until("notify_during_scan", [&] {
                const std::lock_guard<std::mutex> lock(thread->mutex);
                return thread->callbacks_notified();
            });
        }
        const bool sleeps = arm(generation, queued, callbacks, deadline);
        if (sleeps)
            sleep(generation, queued, deadline);
        // Prototype only: a wait a notify ended does no cleanup until its events have no waiter left
        if (proto_scenario_is("notify_then_set") && thread->proto_ended_with(generation, SCE_KERNEL_ERROR_NOTIFY_CALLBACK)) {
            proto_hold_until("notify_then_set", [&] {
                for (SceUInt32 i = 0; i < count; i++) {
                    SceKernelEventInfo info{};
                    if (queued.test(i) && events[i]->info(info) == SCE_KERNEL_OK && info.numWaitThreads != 0)
                        return false;
                }
                return true;
            });
        }
        // Prototype only: a wait held until its thread was exiting ends for the exit, not with an
        // event it took or a timeout
        if (proto_held_for_exit) {
            if (!thread->proto_ended_for_exit(generation)) {
                LOG_ERROR("proto exit_before_look: multiple wait generation {} ended with something else than the exit", generation);
                std::abort();
            }
            LOG_INFO("proto exit_before_look: multiple wait generation {} ended for the exit and took nothing", generation);
        }
        const ThreadState::WaitEnd end = thread->unregister_wait(generation);
        dequeue(queued);
        if (!end.result)
            return end.result;
        // The scan's results alone write no time left
        if (time_left && (sleeps || *end.result == SCE_KERNEL_ERROR_WAIT_TIMEOUT))
            *time_left = time_left_until(deadline, end.at);
        if (*end.result == SCE_KERNEL_ERROR_NOTIFY_CALLBACK)
            continue;
        // The callbacks notified during a sleep run before the call returns
        if (sleeps && callbacks) {
            if (const WaitResult ran = thread->run_callbacks(); !ran)
                return ran;
        }
        return end.result;
    }
}

bool MultiEventWait::EntryRecipient::accept(uint64_t generation, const EventResult &result) {
    return wait->end_entry(generation, { index, result.result, result.bits, result.user_data });
}

bool MultiEventWait::end_entry(uint64_t generation, const Ended &ended) {
    const std::lock_guard<std::mutex> lock(thread->mutex);
    if (!thread->registration.takes_results(generation) || finished.test(ended.index))
        return false;
    finished.set(ended.index);
    results[num_results++] = ended;
    // The entry that ends an OR wait, or the last of an AND wait, ends a waiting registration
    if (thread->registration.waiting() && (any || (waiting_on & ~finished).none()))
        thread->end_registration(SCE_KERNEL_OK);
    return true;
}

SceUInt32 MultiEventWait::copy_results(std::span<SceKernelResultEvent> out) {
    const auto listed = std::span{ objects }.first(count);
    for (SceUInt32 i = 0; i < num_results; i++) {
        const Ended &ended = results[i];
        // Every entry reports the UID of the first entry that names the same object
        const auto first = std::ranges::find(listed, objects[ended.index]) - listed.begin();
        out[i] = { list[first].eventId, ended.result, ended.bits, 0, ended.user_data };
    }
    return num_results;
}

MultiEventWait::Entries MultiEventWait::scan(uint64_t generation) {
    Entries queued;
    // In OR mode an entry that ends stops the queueing, but the scan still records the set ones
    bool queueing = true;
    for (SceUInt32 i = 0; i < count; i++) {
        {
            const std::lock_guard<std::mutex> lock(thread->mutex);
            if (finished.test(i))
                continue;
        }
        tokens[i].generation = generation;
        const SceInt32 scanned = events[i] ? events[i]->scan_token(tokens[i], queueing) : SCE_KERNEL_ERROR_UNKNOWN_EVENT_ID;
        if (scanned == SCE_KERNEL_ERROR_UNKNOWN_EVENT_ID)
            end_entry(generation, { i, static_cast<SceInt32>(SCE_KERNEL_ERROR_UNKNOWN_EVENT_ID), SCE_KERNEL_EVENT_ERROR, 0 });
        if (scanned != SCE_KERNEL_ERROR_EVENT_COND)
            queueing = queueing && !any;
        else if (queueing)
            queued.set(i);
        // Prototype only: the scan waits after the first entry's token until another party ended it
        if (proto_scenario_is("set_during_scan") && i == 0 && queued.test(0)) {
            proto_hold_until("set_during_scan", [&] {
                const std::lock_guard<std::mutex> lock(thread->mutex);
                return finished.test(0);
            });
        }
    }
    return queued;
}

bool MultiEventWait::arm(uint64_t generation, const Entries &queued, bool callbacks, Deadline deadline) {
    const std::lock_guard<std::mutex> lock(thread->mutex);
    assert(thread->registration.scanning() && thread->registration.current(generation));
    // A token an event ended after it was queued waits no more
    waiting_on = queued & ~finished;
    const std::size_t left = waiting_on.count();
    if (left == 0 || (any && left < count)) {
        thread->end_registration(SCE_KERNEL_OK);
        return false;
    }
    // A deadline gone before the sleep ends the call without the final callback check
    if (std::chrono::steady_clock::now() >= deadline) {
        thread->end_registration(SCE_KERNEL_ERROR_WAIT_TIMEOUT);
        return false;
    }
    // A callback pending since the call started doesn't keep the thread from sleeping
    thread->arm({ SCE_KERNEL_WAITTYPE_MP_EVENTS, MULTI_EVENTS_WAIT_ID, callbacks });
    return true;
}

void MultiEventWait::sleep(uint64_t generation, const Entries &queued, Deadline deadline) {
    while (true) {
        // A timer's event fires only when something looks at it, so the thread wakes for the next one
        // of the timers it is the first waiter of
        Deadline wake = deadline;
        for (SceUInt32 i = 0; i < count; i++) {
            if (queued.test(i))
                wake = std::min(wake, events[i]->watch_until(tokens[i]));
        }
        const WaitResult parked = thread->park(wake);
        const bool looks = parked && *parked == SCE_KERNEL_ERROR_WAIT_TIMEOUT;
        // Prototype only: the first look comes 50 ms after the deadline, as from a late host
        if (proto_scenario_is("late_timer_wake") && looks && !proto_held_wake) {
            proto_held_wake = true;
            std::this_thread::sleep_until(deadline + std::chrono::milliseconds(50));
            LOG_INFO("proto late_timer_wake: woke 50 ms after the deadline");
        }
        // Prototype only: the first look waits until the thread is exiting
        if (proto_scenario_is("exit_before_look") && looks && !proto_held_wake) {
            proto_held_wake = true;
            thread->proto_hold("exit_before_look", generation);
            proto_held_for_exit = true;
        }
        // The park's result may be old by now, so the registration decides before the wait fires
        // anything: a wait that ended, or whose thread is exiting, never takes a timer's event
        const WaitResult state = thread->check_wait();
        if (!state) {
            thread->end_wait(generation, state);
            return;
        }
        if (*state == SCE_KERNEL_OK)
            return;
        // Woken for a timer's event, its new schedule, a new first waiter or the deadline. After a late
        // wake only the timer events due by the deadline still end the wait.
        const Deadline now = std::chrono::steady_clock::now();
        fire_due(queued, std::min(now, deadline));
        if (now >= deadline) {
            thread->end_wait(generation, SCE_KERNEL_ERROR_WAIT_TIMEOUT);
            return;
        }
    }
}

void MultiEventWait::fire_due(const Entries &queued, Deadline until) {
    // One event at a time, the earliest first, so an OR wait ends with the one that came due first. A
    // timer that doesn't fire after all, by a later read of its clock, waits for the next look.
    Entries passed;
    while (true) {
        SceUInt32 earliest = count;
        Deadline earliest_at = until;
        for (SceUInt32 i = 0; i < count; i++) {
            if (!queued.test(i) || passed.test(i))
                continue;
            const Deadline at = events[i]->next_fire();
            if (at <= earliest_at) {
                earliest = i;
                earliest_at = at;
            }
        }
        if (earliest == count)
            return;
        if (!events[earliest]->update(until))
            passed.set(earliest);
    }
}

void MultiEventWait::dequeue(const Entries &queued) {
    for (SceUInt32 i = 0; i < count; i++) {
        if (queued.test(i))
            events[i]->remove_token(tokens[i]);
    }
}

// *****************
// * Simple events *
// *****************

Event::Event(SceUInt32 attr, const char *name, SceUInt32 init_pattern, SceUInt64 init_user_data)
    : SyncPrimitive(attr, name)
    , auto_reset(attr & SCE_KERNEL_EVENT_ATTR_AUTO_RESET)
    , waiters(attr)
    , pattern(init_pattern | SCE_KERNEL_EVENT_CREATE)
    , last_user_data(init_user_data) {}

// The create event comes with the creating thread's user data, like the other system events
SimpleEvent::SimpleEvent(SceUInt32 attr, const char *name, SceUInt32 init_pattern, SceUID creator)
    : WithUidClass(attr, name, init_pattern, system_event_user_data(creator)) {}

SceInt32 SimpleEvent::check_create(const char *name, SceUInt32 attr, SceUInt32 init_pattern) {
    if (const SceInt32 error = check_name(name, attr))
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

bool Event::complete(WaitQueue<WaitEntry>::Waiter &waiter, SceInt32 result, SceUInt32 bits) {
    return waiter.entry.recipient->accept(waiter.generation, { result, bits, result == SCE_KERNEL_OK ? last_user_data : 0 });
}

namespace {

// The outputs of a plain event wait, which gets the bits once its wait is ended
class PlainEventRecipient final : public EventRecipient {
public:
    PlainEventRecipient(ThreadState &thread, SceUInt32 *result_pattern, SceUInt64 *user_data)
        : thread(thread)
        , result_pattern(result_pattern)
        , user_data(user_data) {}

    bool accept(uint64_t generation, const EventResult &result) override {
        // Every claim takes its waiter off the queue under the event's lock, so a queued plain wait
        // still waits
        [[maybe_unused]] const bool ended = thread.end_wait(generation, result.result);
        assert(ended);
        if (result.result == SCE_KERNEL_OK) {
            if (result_pattern)
                *result_pattern = result.bits;
            if (user_data)
                *user_data = result.user_data;
        }
        return true;
    }

private:
    ThreadState &thread;
    SceUInt32 *const result_pattern;
    SceUInt64 *const user_data;
};

} // namespace

bool Event::depart_locked(ThreadState &thread, uint64_t generation, Departure reason) {
    return depart_queued(waiters, thread, generation, reason);
}

std::size_t Event::end_waits(SceInt32 result) {
    std::size_t ended = 0;
    waiters.remove_if([&](auto &waiter) {
        ended += complete(waiter, result, SCE_KERNEL_EVENT_ERROR);
        return true;
    });
    return ended;
}

SceInt32 Event::scan_token(Token &token, bool queue) {
    const auto guard = lock();
    if (!guard)
        return SCE_KERNEL_ERROR_UNKNOWN_EVENT_ID;
    catch_up();
    if (const SceUInt32 bits = pattern & token.entry.pattern) {
        if (complete(token, SCE_KERNEL_OK, bits))
            reset_taken(token.entry.pattern);
        return SCE_KERNEL_OK;
    }
    if (queue) {
        token.priority = token.thread->get_priority();
        waiters.push(token);
    }
    return SCE_KERNEL_ERROR_EVENT_COND;
}

void Event::remove_token(Token &token) {
    const auto guard = lock();
    if (!guard)
        return;
    // A token that leaves without the bits lets the waits behind it have them
    waiters.remove(token);
    serve_locked();
}

Deadline Event::next_fire() {
    const auto guard = lock();
    return guard ? event_deadline() : Deadline::max();
}

Deadline Event::watch_until(const Token &token) {
    const auto guard = lock();
    if (!guard || waiters.front() != &token)
        return Deadline::max();
    return event_deadline();
}

bool Event::update(Deadline until) {
    const auto guard = lock();
    return guard && fire_by(until);
}

bool Event::fire_by(Deadline until) {
    return event_deadline() <= until && catch_up();
}

SceInt32 Event::try_take(SceUInt32 wait_pattern, SceUInt32 *result_pattern, SceUInt64 *user_data) {
    if (!(pattern & wait_pattern))
        return SCE_KERNEL_ERROR_EVENT_COND;

    if (result_pattern)
        *result_pattern = pattern & wait_pattern;

    reset_taken(wait_pattern);

    if (user_data)
        *user_data = last_user_data;

    return SCE_KERNEL_OK;
}

void Event::reset_taken(SceUInt32 wait_pattern) {
    // A pipe's DATA_EXIST says whether its ring holds bytes, so only the ring changes it
    if (auto_reset)
        pattern &= ~(wait_pattern & ~SCE_KERNEL_EVENT_DATA_EXIST);
}

SceInt32 Event::wait(const ThreadStatePtr &thread, SceUInt32 wait_pattern, SceUInt32 *result_pattern, SceUInt64 *user_data, SceUInt32 *timeout, bool callbacks) {
    auto guard = lock();
    if (!guard)
        return SCE_KERNEL_ERROR_UNKNOWN_EVENT_ID;

    PlainEventRecipient recipient(*thread, result_pattern, user_data);
    // A wait for pattern 0 matches nothing, so it lasts until its timeout, a cancel or a delete. If
    // the bits are taken right after a callback notify, the wait returns NOTIFY_CALLBACK.
    const WaitResult r = wait_loop(*this, waiters, guard, thread, { SCE_KERNEL_WAITTYPE_EVENT, uid, callbacks }, { wait_pattern, &recipient }, timeout, CallbacksCheck::after_wait, [&](WaitEntry &, SceInt32 last) -> std::optional<SceInt32> {
        // A timer's events that are already due fire before the take, so the waiter can't get an old
        // pulse
        catch_up();
        if (try_take(wait_pattern, result_pattern, user_data) == SCE_KERNEL_OK)
            return last;
        return std::nullopt;
    });
    return wait_result(r, result_pattern);
}

SceInt32 Event::wait_result(const WaitResult &r, SceUInt32 *result_pattern) {
    const SceInt32 err = guest_result(r);
    // A wait ended by a delete or a cancel gets the error event and no user data. If the wait got
    // the bits, they are already written, also when it returns NOTIFY_CALLBACK.
    if (err < 0 && err != SCE_KERNEL_ERROR_WAIT_TIMEOUT && err != SCE_KERNEL_ERROR_NOTIFY_CALLBACK && result_pattern)
        *result_pattern = SCE_KERNEL_EVENT_ERROR;
    return err;
}

SceInt32 Event::poll(SceUInt32 wait_pattern, SceUInt32 *result_pattern, SceUInt64 *user_data) {
    const auto guard = lock();
    if (!guard)
        return SCE_KERNEL_ERROR_UNKNOWN_EVENT_ID;
    catch_up();
    if (wait_pattern == 0)
        return SCE_KERNEL_ERROR_INVALID_ARGUMENT;

    return try_take(wait_pattern, result_pattern, user_data);
}

SceInt32 SimpleEvent::set_or_pulse(SceUInt32 set_pattern, SceUInt64 user_data, bool is_set) {
    const auto guard = lock();
    if (!guard)
        return SCE_KERNEL_ERROR_UNKNOWN_EVENT_ID;
    if (set_pattern & ~SCE_KERNEL_EVENT_USER_DEFINED_MASK)
        return SCE_KERNEL_ERROR_ILLEGAL_PATTERN;

    fire(set_pattern, user_data, is_set);
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
    serve_locked();

    // A pulse clears its bits once the waiters had them, also the ones set before it, and keeps its
    // user data
    if (!is_set)
        pattern &= ~set_pattern;
}

void Event::serve_locked() {
    waiters.remove_if([&](auto &waiter) {
        const SceUInt32 bits = pattern & waiter.entry.pattern;
        if (!bits)
            return false;
        // A token of a wait that takes no more results leaves without the bits, for the waits behind it
        if (complete(waiter, SCE_KERNEL_OK, bits))
            reset_taken(waiter.entry.pattern);
        return true;
    });
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

    // The entries of multiple event waits count as waiters too
    const auto nb_threads = static_cast<SceUInt32>(end_waits(SCE_KERNEL_ERROR_WAIT_CANCEL));
    if (num_wait_threads)
        *num_wait_threads = nb_threads;

    return SCE_KERNEL_OK;
}

SceInt32 Event::cancel(SceUInt32 set_pattern, SceUInt64 user_data, SceUInt32 *num_wait_threads) {
    const auto guard = lock();
    if (!guard)
        return SCE_KERNEL_ERROR_UNKNOWN_EVENT_ID;
    catch_up();
    if (set_pattern & ~SCE_KERNEL_EVENT_USER_DEFINED_MASK)
        return SCE_KERNEL_ERROR_ILLEGAL_PATTERN;

    // The entries of multiple event waits count as waiters too
    const auto nb_threads = static_cast<SceUInt32>(end_waits(SCE_KERNEL_ERROR_WAIT_CANCEL));
    // Every bit is replaced, the create event too, and no waiter is left to get them
    pattern = set_pattern;
    last_user_data = user_data;
    if (num_wait_threads)
        *num_wait_threads = nb_threads;

    return SCE_KERNEL_OK;
}

SceInt32 Event::get_pattern(SceUInt32 &result) {
    const auto guard = lock();
    if (!guard)
        return SCE_KERNEL_ERROR_UNKNOWN_EVENT_ID;
    catch_up();
    result = pattern;
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
    : WithUidClass(attr, name, 0, system_event_user_data(creator)) {}

// A timer event is due at least this long after it was queued or after the event before it
constexpr uint64_t TIMER_EVENT_FLOOR = 200;

bool Timer::catch_up() {
    const uint64_t now = get_current_time();
    if (next_event > now)
        return false;
    fire(SCE_KERNEL_EVENT_TIMER, 0, !is_pulse);
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
    // The first waiter sleeps until the old event time, so wake it to see the new one
    if (const auto *const first = waiters.front())
        first->thread->wake();
}

Deadline Timer::event_deadline() const {
    if (next_event == std::numeric_limits<uint64_t>::max())
        return Deadline::max();
    // An event that is due already gives a time that passed, so a bounded update still fires it
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

Deadline Timer::recheck_locked(const ThreadState &thread, Deadline until) {
    while (fire_by(until)) {
    }
    // A single wait, so the thread has one entry here. Only the first waiter watches the clock, and
    // serve_locked() wakes the one that becomes the first.
    const auto *const first = waiters.front();
    return first && first->thread.get() == &thread ? event_deadline() : Deadline::max();
}

void Timer::serve_locked() {
    Event::serve_locked();
    // The first waiter may be a new one, which now has to wake for the next event
    if (const auto *const first = waiters.front())
        first->thread->wake();
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

SceInt32 Timer::timer_info(SceKernelTimerInfo &info) {
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

SceInt32 Timer::cancel(SceUInt32 *num_wait_threads) {
    const auto guard = lock();
    if (!guard)
        return SCE_KERNEL_ERROR_UNKNOWN_TIMER_ID;
    catch_up();

    // The entries of multiple event waits count as waiters too
    const auto nb_threads = static_cast<SceUInt32>(end_waits(SCE_KERNEL_ERROR_WAIT_CANCEL));
    // The event set with sceKernelSetTimerEvent is canceled too, but the timer keeps running with
    // the event's interval and type
    armed = false;
    is_repeat = false;
    next_event = std::numeric_limits<uint64_t>::max();
    if (num_wait_threads)
        *num_wait_threads = nb_threads;

    return SCE_KERNEL_OK;
}

// *********
// * Mutex *
// *********

Mutex::Mutex(SceUInt32 attr, const char *name, SceInt32 init_count, ThreadStatePtr thread, Ptr<SceKernelLwMutexWork> workarea)
    : SyncPrimitive(attr, name)
    , init_count(init_count)
    , workarea(workarea)
    , waiters(attr)
    , lock_count(init_count)
    , owner(init_count > 0 && thread ? thread->id : 0) {}

SceInt32 Mutex::check_create(const char *name, SceUInt32 attr, SceInt32 init_count, bool kernel) {
    if (const SceInt32 error = check_name(name, attr))
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
    // The waits of its conditions end first, then its own, as the firmware's delete does. The
    // conditions share the mutex's lock, which is held here.
    for (auto it = conditions.begin(); it != conditions.end();) {
        Condvar &cond = *it;
        it = conditions.erase(it);
        cond.end_for_mutex_delete();
    }
    waiters.end_all(SCE_KERNEL_ERROR_WAIT_DELETE);
    owner = 0;
}

bool Mutex::depart_locked(ThreadState &thread, uint64_t generation, Departure reason) {
    return depart_queued(waiters, thread, generation, reason);
}

SceInt32 Mutex::try_take(MemState &mem, const ThreadStatePtr &thread, SceInt32 count) {
    bool is_recursive = (attr & SCE_KERNEL_MUTEX_ATTR_RECURSIVE);

    // Already owned
    if (owner) {
        // Owned by ourselves
        if (owner == thread->id) {
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
    owner = thread->id;
    update_workarea(mem);

    return SCE_KERNEL_OK;
}

void Mutex::update_workarea(MemState &mem) {
    if (!lightweight())
        return;
    workarea.get(mem)->lockCount = lock_count;
    workarea.get(mem)->owner = owner;
}

SceInt32 Mutex::acquire(MemState &mem, const ThreadStatePtr &thread, SceInt32 count, SceUInt32 *timeout, bool callbacks) {
    auto guard = lock();
    if (!guard)
        return lightweight() ? SCE_KERNEL_ERROR_UNKNOWN_LW_MUTEX_ID : SCE_KERNEL_ERROR_UNKNOWN_MUTEX_ID;
    if (const SceInt32 error = check_count(count))
        return error;
    // The owner's own lock takes or fails at once, without callbacks
    if (owner == thread->id)
        return try_take(mem, thread, count);

    const WaitResult r = wait_loop(*this, waiters, guard, thread, { lightweight() ? SCE_KERNEL_WAITTYPE_LW_MUTEX : SCE_KERNEL_WAITTYPE_MUTEX, uid, callbacks }, { count }, timeout, CallbacksCheck::after_wait, [&](WaitEntry &, SceInt32) {
        return take_if_free(mem, thread, count);
    });
    // A deleted mutex has no owner, and its work area may be freed already
    if (owner == thread->id)
        update_workarea(mem);
    return guest_result(r);
}

SceInt32 Mutex::relock(MemState &mem, const ThreadStatePtr &thread, WaitTarget target) {
    auto guard = lock();
    if (!guard)
        return lightweight() ? SCE_KERNEL_ERROR_UNKNOWN_LW_MUTEX_ID : SCE_KERNEL_ERROR_UNKNOWN_MUTEX_ID;
    // The owner's own relock takes or fails at once, like its lock
    if (owner == thread->id)
        return try_take(mem, thread, 1);

    const WaitResult r = wait_loop(*this, waiters, guard, thread, target, { 1 }, nullptr, CallbacksCheck::on_delete, [&](WaitEntry &, SceInt32) {
        return take_if_free(mem, thread, 1);
    });
    if (owner == thread->id)
        update_workarea(mem);
    return guest_result(r);
}

std::optional<SceInt32> Mutex::take_if_free(MemState &mem, const ThreadStatePtr &thread, SceInt32 count) {
    // A heavy mutex also needs no waiter ahead of this thread. A mutex acquired by this thread's
    // callbacks still blocks this wait.
    if (!owner && (lightweight() || waiters.first_for(thread->get_priority())))
        return try_take(mem, thread, count);
    return std::nullopt;
}

SceInt32 Mutex::try_acquire(MemState &mem, const ThreadStatePtr &thread, SceInt32 count) {
    const auto guard = lock();
    if (!guard)
        return lightweight() ? SCE_KERNEL_ERROR_UNKNOWN_LW_MUTEX_ID : SCE_KERNEL_ERROR_UNKNOWN_MUTEX_ID;
    if (const SceInt32 error = check_count(count))
        return error;

    return try_take(mem, thread, count);
}

SceInt32 Mutex::release(MemState &mem, const ThreadStatePtr &thread, SceInt32 unlock_count) {
    const auto guard = lock();
    // Deleted meanwhile, which a lightweight mutex reports as not owned
    if (!guard)
        return lightweight() ? SCE_KERNEL_ERROR_LW_MUTEX_NOT_OWNED : SCE_KERNEL_ERROR_UNKNOWN_MUTEX_ID;
    return release_held(mem, thread, unlock_count);
}

SceInt32 Mutex::release_held(MemState &mem, const ThreadStatePtr &thread, SceInt32 unlock_count) {
    if (was_deleted())
        return lightweight() ? SCE_KERNEL_ERROR_LW_MUTEX_NOT_OWNED : SCE_KERNEL_ERROR_UNKNOWN_MUTEX_ID;
    if (const SceInt32 error = check_count(unlock_count))
        return error;
    if (thread->id != owner)
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
    if (auto *waiter = waiters.front()) {
        const SceInt32 count = waiter->entry.lock_count;
        const SceUID id = waiter->thread->id;
        // The head gets the mutex once its wait is ended
        waiters.end(*waiter);
        lock_count = count;
        owner = id;
    }
}

SceInt32 Mutex::cancel(const ThreadStatePtr &thread, SceInt32 new_count, SceUInt32 *num_wait_threads) {
    const auto guard = lock();
    if (!guard)
        return lightweight() ? SCE_KERNEL_ERROR_UNKNOWN_LW_MUTEX_ID : SCE_KERNEL_ERROR_UNKNOWN_MUTEX_ID;

    // Any negative count restores the lock count the mutex was created with
    if (new_count < 0)
        new_count = init_count;
    if (new_count > 1 && !(attr & SCE_KERNEL_MUTEX_ATTR_RECURSIVE))
        return SCE_KERNEL_ERROR_ILLEGAL_COUNT;

    const auto nb_threads = static_cast<SceUInt32>(waiters.end_all(SCE_KERNEL_ERROR_WAIT_CANCEL));
    lock_count = new_count;
    owner = new_count > 0 ? thread->id : 0;
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
    : WithUidClass(attr, name)
    , waiters(attr) {}

SceInt32 RWLock::check_create(const char *name, SceUInt32 attr) {
    if (const SceInt32 error = check_name(name, attr))
        return error;
    if (attr & ~(SCE_KERNEL_RW_LOCK_ATTR_RECURSIVE | SCE_KERNEL_RW_LOCK_ATTR_CEILING | SCE_KERNEL_ATTR_TH_PRIO | SCE_KERNEL_ATTR_OPENABLE))
        return SCE_KERNEL_ERROR_ILLEGAL_ATTR;
    return SCE_KERNEL_OK;
}

void RWLock::on_delete(SceUID) {
    waiters.end_all(SCE_KERNEL_ERROR_WAIT_DELETE);
}

bool RWLock::depart_locked(ThreadState &thread, uint64_t generation, Departure reason) {
    return depart_queued(waiters, thread, generation, reason);
}

bool RWLock::proto_departure_reached() {
    // The reader behind the writer that left got in next to the read lock already held
    return waiters.empty() && lock_count == 2 && !write_owner;
}

SceInt32 RWLock::try_take(const ThreadStatePtr &thread, bool is_write) {
    // The writer may take the write lock again only if the lock is recursive
    if (is_write && write_owner == thread->id) {
        if (!(attr & SCE_KERNEL_RW_LOCK_ATTR_RECURSIVE))
            return SCE_KERNEL_ERROR_RW_LOCK_RECURSIVE;
        lock_count++;
        return SCE_KERNEL_OK;
    }

    // A waiting writer goes before new readers
    const bool writer_waits = waiters.find_if([](auto &waiter) { return waiter.entry.is_write; }) != nullptr;
    if (!is_write && writer_waits)
        return SCE_KERNEL_ERROR_RW_LOCK_FAILED_TO_LOCK;

    // Readers share the lock, but a reader and a writer don't, even in the same thread
    if (write_owner || (is_write && lock_count > 0))
        return SCE_KERNEL_ERROR_RW_LOCK_FAILED_TO_LOCK;

    lock_count++;
    if (is_write)
        write_owner = thread->id;
    return SCE_KERNEL_OK;
}

SceInt32 RWLock::acquire(const ThreadStatePtr &thread, bool is_write, SceUInt32 *timeout, bool callbacks) {
    auto guard = lock();
    if (!guard)
        return SCE_KERNEL_ERROR_UNKNOWN_RW_LOCK_ID;
    // The writer locking again succeeds or fails at once, without callbacks. A reader asking for a
    // write lock waits, since readers aren't tracked individually.
    if (write_owner == thread->id)
        return try_take(thread, is_write);

    const WaitResult r = wait_loop(*this, waiters, guard, thread, { SCE_KERNEL_WAITTYPE_RW_LOCK, uid, callbacks }, { is_write }, timeout, CallbacksCheck::after_wait, [&](WaitEntry &, SceInt32) -> std::optional<SceInt32> {
        // A writer needs the lock to be free. A reader needs no writer and an empty queue. A lock
        // taken by this thread's callbacks counts like anyone else's.
        if (write_owner || (is_write ? lock_count > 0 : !waiters.empty()))
            return std::nullopt;
        lock_count++;
        if (is_write)
            write_owner = thread->id;
        return SCE_KERNEL_OK;
    });
    return guest_result(r);
}

SceInt32 RWLock::try_acquire(const ThreadStatePtr &thread, bool is_write) {
    const auto guard = lock();
    if (!guard)
        return SCE_KERNEL_ERROR_UNKNOWN_RW_LOCK_ID;

    return try_take(thread, is_write);
}

SceInt32 RWLock::try_acquire_read(const ThreadStatePtr &thread) {
    return try_acquire(thread, false);
}

SceInt32 RWLock::try_acquire_write(const ThreadStatePtr &thread) {
    return try_acquire(thread, true);
}

SceInt32 RWLock::acquire_read(const ThreadStatePtr &thread, SceUInt32 *timeout, bool callbacks) {
    return acquire(thread, false, timeout, callbacks);
}

SceInt32 RWLock::acquire_write(const ThreadStatePtr &thread, SceUInt32 *timeout, bool callbacks) {
    return acquire(thread, true, timeout, callbacks);
}

SceInt32 RWLock::release_read(const ThreadStatePtr &thread) {
    return release(thread, false);
}

SceInt32 RWLock::release_write(const ThreadStatePtr &thread) {
    return release(thread, true);
}

SceInt32 RWLock::release(const ThreadStatePtr &thread, bool is_write) {
    const auto guard = lock();
    if (!guard)
        return SCE_KERNEL_ERROR_UNKNOWN_RW_LOCK_ID;

    if (is_write) {
        // Only the writer may write unlock
        if (write_owner != thread->id)
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
    info.numReadWaitThreads = static_cast<SceUInt32>(waiters.count_if([](const auto &waiter) { return !waiter.entry.is_write; }));
    info.numWriteWaitThreads = static_cast<SceUInt32>(waiters.count_if([](const auto &waiter) { return waiter.entry.is_write; }));
    return SCE_KERNEL_OK;
}

void RWLock::serve_locked() {
    if (write_owner)
        return;

    // Without a waiting writer every waiting reader gets in, also next to readers that hold the lock.
    // A waiting writer goes first, even before readers that waited longer, once nobody holds it.
    auto *const writer = waiters.find_if([](auto &waiter) { return waiter.entry.is_write; });
    if (!writer) {
        lock_count += static_cast<SceInt32>(waiters.end_all(SCE_KERNEL_OK));
    } else if (lock_count == 0) {
        const SceUID id = writer->thread->id;
        // The writer gets the lock once its wait is ended
        waiters.end(*writer);
        write_owner = id;
        lock_count = 1;
    }
}

SceInt32 RWLock::cancel(const ThreadStatePtr &thread, SceUInt32 *num_read_wait_threads, SceUInt32 *num_write_wait_threads, SceInt32 flag) {
    const auto guard = lock();
    if (!guard)
        return SCE_KERNEL_ERROR_UNKNOWN_RW_LOCK_ID;

    SceUInt32 nb_writers = 0;
    waiters.remove_if([&](auto &waiter) {
        if (!waiter.entry.is_write)
            return false;
        waiters.claim(waiter, SCE_KERNEL_ERROR_WAIT_CANCEL);
        ++nb_writers;
        return true;
    });
    const auto nb_readers = static_cast<SceUInt32>(waiters.end_all(SCE_KERNEL_ERROR_WAIT_CANCEL));

    // The lock is reset, and only the calling thread may hold it afterwards
    const bool write_locked = flag & SCE_KERNEL_RW_LOCK_CANCEL_WITH_WRITE_LOCK;
    lock_count = write_locked ? 1 : 0;
    write_owner = write_locked ? thread->id : 0;

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
    , waiters(attr)
    , val(init_val) {}

SceInt32 Semaphore::check_create(const char *name, SceUInt32 attr, SceInt32 init_val, SceInt32 max_val) {
    if (const SceInt32 error = check_name(name, attr))
        return error;
    if (attr & ~(SCE_KERNEL_ATTR_TH_PRIO | SCE_KERNEL_ATTR_OPENABLE))
        return SCE_KERNEL_ERROR_ILLEGAL_ATTR;
    if (init_val < 0 || max_val <= 0 || init_val > max_val)
        return SCE_KERNEL_ERROR_ILLEGAL_COUNT;
    return SCE_KERNEL_OK;
}

void Semaphore::on_delete(SceUID) {
    waiters.end_all(SCE_KERNEL_ERROR_WAIT_DELETE);
}

SceInt32 Semaphore::wait(const ThreadStatePtr &thread, SceInt32 need_count, SceUInt32 *timeout, bool callbacks) {
    auto guard = lock();
    if (!guard)
        return SCE_KERNEL_ERROR_UNKNOWN_SEMA_ID;
    if (need_count <= 0 || need_count > max)
        return SCE_KERNEL_ERROR_ILLEGAL_COUNT;

    const WaitResult r = wait_loop(*this, waiters, guard, thread, { SCE_KERNEL_WAITTYPE_SEMAPHORE, uid, callbacks }, { need_count }, timeout, CallbacksCheck::after_wait, [&](WaitEntry &, SceInt32) -> std::optional<SceInt32> {
        if (try_take(thread, need_count))
            return SCE_KERNEL_OK;
        return std::nullopt;
    });
    return guest_result(r);
}

bool Semaphore::depart_locked(ThreadState &thread, uint64_t generation, Departure reason) {
    return depart_queued(waiters, thread, generation, reason);
}

bool Semaphore::proto_departure_reached() {
    return val >= 1 && waiters.empty();
}

SceInt32 Semaphore::poll(const ThreadStatePtr &thread, SceInt32 need_count) {
    const auto guard = lock();
    if (!guard)
        return SCE_KERNEL_ERROR_UNKNOWN_SEMA_ID;
    if (need_count <= 0 || need_count > max)
        return SCE_KERNEL_ERROR_ILLEGAL_COUNT;

    return try_take(thread, need_count) ? SCE_KERNEL_OK : SCE_KERNEL_ERROR_SEMA_ZERO;
}

bool Semaphore::try_take(const ThreadStatePtr &thread, SceInt32 need_count) {
    // A waiter queued ahead goes first, even one the count doesn't cover yet
    if (val < need_count || !waiters.first_for(thread->get_priority()))
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
    while (auto *waiter = waiters.front()) {
        if (val < waiter->entry.need_count)
            break;
        val -= waiter->entry.need_count;
        waiters.end(*waiter);
    }
}

SceInt32 Semaphore::cancel(SceInt32 set_count, SceUInt32 *num_wait_threads) {
    const auto guard = lock();
    if (!guard)
        return SCE_KERNEL_ERROR_UNKNOWN_SEMA_ID;

    if (set_count > max)
        return SCE_KERNEL_ERROR_ILLEGAL_COUNT;

    const auto nb_threads = static_cast<SceUInt32>(waiters.end_all(SCE_KERNEL_ERROR_WAIT_CANCEL));
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

// A condition's waits and its mutex's state are always used together, and the mutex's delete ends
// the condition's waits, so the condition shares the mutex's lock
Condvar::Condvar(SceUInt32 attr, const char *name, MutexPtr associated_mutex)
    : SyncPrimitive(attr, name, associated_mutex.get())
    , associated_mutex(std::move(associated_mutex))
    , waiters(attr) {}

Condvar::~Condvar() {
    // One that was never deleted is still in its mutex's conditions
    const auto guard = lock();
    if (guard)
        detach();
}

SceInt32 Condvar::attach() {
    const auto guard = lock();
    // The mutex's lock is this one, so its delete can't run meanwhile
    if (associated_mutex->was_deleted())
        return lightweight() ? SCE_KERNEL_ERROR_UNKNOWN_LW_MUTEX_ID : SCE_KERNEL_ERROR_UNKNOWN_MUTEX_ID;
    associated_mutex->conditions.push_back(*this);
    return SCE_KERNEL_OK;
}

void Condvar::detach() {
    // Not in the list once the mutex's delete took it out
    associated_mutex->conditions.remove(*this);
}

void Condvar::end_for_mutex_delete() {
    mutex_deleted = true;
    // wait() maps this to the error of the mutex's kind
    waiters.end_all(SCE_KERNEL_ERROR_WAIT_DELETE_MUTEX);
}

void Condvar::on_delete(SceUID) {
    // wait() maps this to the condition's own delete error
    waiters.end_all(SCE_KERNEL_ERROR_WAIT_DELETE);
    detach();
}

SceInt32 Condvar::wait(MemState &mem, const ThreadStatePtr &thread, SceUInt32 *timeout, bool callbacks) {
    auto guard = lock();
    if (!guard)
        return lightweight() ? SCE_KERNEL_ERROR_UNKNOWN_LW_COND_ID : SCE_KERNEL_ERROR_UNKNOWN_COND_ID;

    // The mutex's lock is the condition's, held here
    if (const SceInt32 error = associated_mutex->release_held(mem, thread, 1))
        return error;

    // Condition waits have nothing to take, they only wait for a signal
    const WaitResult r = wait_loop(*this, waiters, guard, thread, { lightweight() ? SCE_KERNEL_WAITTYPE_LW_COND_SIGNAL : SCE_KERNEL_WAITTYPE_COND_SIGNAL, uid, callbacks }, std::monostate{}, timeout, CallbacksCheck::on_delete, [](std::monostate &, SceInt32) -> std::optional<SceInt32> {
        return std::nullopt;
    });
    // Deleted during the wait, or while its callbacks ran
    if (r && *r == SCE_KERNEL_ERROR_WAIT_DELETE)
        return lightweight() ? SCE_KERNEL_ERROR_WAIT_DELETE_LW_COND : SCE_KERNEL_ERROR_WAIT_DELETE_COND;
    // The mutex was deleted while the wait was queued, so it ends without a relock
    if (r && *r == SCE_KERNEL_ERROR_WAIT_DELETE_MUTEX)
        return lightweight() ? SCE_KERNEL_ERROR_WAIT_DELETE_LW_MUTEX : SCE_KERNEL_ERROR_WAIT_DELETE_MUTEX;
    // A timed out wait still takes the mutex back
    const bool timed_out = r && *r == SCE_KERNEL_ERROR_WAIT_TIMEOUT;
    if (!r || (*r != SCE_KERNEL_OK && !timed_out))
        return guest_result(r);
    // Prototype only: the waiter lets go of the condition's lock after the claim ended its wait,
    // until another thread deleted the condition
    if (proto_scenario_is("cond_delete_after_claim")) {
        bool deleted = false;
        guard.unlock();
        for (int i = 0; i < 5000 && !deleted; i++) {
            guard.lock();
            deleted = was_deleted();
            guard.unlock();
            if (!deleted)
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
        if (!deleted) {
            LOG_ERROR("proto cond_delete_after_claim: the condition was never deleted");
            std::abort();
        }
        guard.lock();
        LOG_INFO("proto cond_delete_after_claim: held after the claim until the condition was deleted, result {:#x}", static_cast<uint32_t>(*r));
    }
    // A condition deleted since the claim that ended the wait leaves the mutex alone, and the wait
    // keeps what the claim gave it
    if (was_deleted())
        return *r;

    guard.unlock();
    // Taking the mutex back is still part of the condition variable wait, a wait on the mutex with no
    // timeout that leaves the time left as it was at the signal
    const SceInt32 result = associated_mutex->relock(mem, thread, { lightweight() ? SCE_KERNEL_WAITTYPE_LW_COND_LW_MUTEX : SCE_KERNEL_WAITTYPE_COND_MUTEX, associated_mutex->uid, callbacks });
    if (timed_out && result == SCE_KERNEL_OK)
        return SCE_KERNEL_ERROR_WAIT_TIMEOUT;
    // A delete during the relock gives the mutex's delete error. A heavy mutex deleted while the
    // waiter wasn't queued on the condition, after a signal or out for its callback, gives the
    // unknown mutex error at the relock. On a Vita a lightweight one leaves the waiter blocked
    // instead, which isn't copied, so it gives the delete error too.
    if (result == SCE_KERNEL_ERROR_WAIT_DELETE || result == SCE_KERNEL_ERROR_UNKNOWN_LW_MUTEX_ID)
        return lightweight() ? SCE_KERNEL_ERROR_WAIT_DELETE_LW_MUTEX : SCE_KERNEL_ERROR_WAIT_DELETE_MUTEX;
    // only heavy mutexes can be canceled
    if (result == SCE_KERNEL_ERROR_WAIT_CANCEL)
        return SCE_KERNEL_ERROR_WAIT_CANCEL_MUTEX;
    return result;
}

SceInt32 Condvar::signal(SignalTarget target) {
    const auto guard = lock();
    if (!guard)
        return lightweight() ? SCE_KERNEL_ERROR_UNKNOWN_LW_COND_ID : SCE_KERNEL_ERROR_UNKNOWN_COND_ID;

    if (target.type == SignalTarget::Type::Specific) {
        // Search for specified waiting thread
        auto *waiter = waiters.find_if([&](auto &w) { return w.thread->id == target.thread_id; });
        if (!waiter)
            return SCE_KERNEL_ERROR_ILLEGAL_THREAD_ID;
        waiters.end(*waiter);
    } else if (target.type == SignalTarget::Type::Any) {
        // Ends the first waiter's wait in queue order
        if (auto *waiter = waiters.front())
            waiters.end(*waiter);
    } else {
        waiters.end_all(SCE_KERNEL_OK);
    }

    return SCE_KERNEL_OK;
}

bool Condvar::depart_locked(ThreadState &thread, uint64_t generation, Departure reason) {
    return depart_queued(waiters, thread, generation, reason);
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
    info.mutexId = mutex_deleted ? -1 : associated_mutex->uid;
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
    : WithUidClass(attr, name)
    , init_pattern(init_pattern)
    , waiters(attr)
    , flags(init_pattern) {}

SceInt32 EventFlag::check_create(const char *name, SceUInt32 attr) {
    if (const SceInt32 error = check_name(name, attr))
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
    waiters.end_all(SCE_KERNEL_ERROR_WAIT_DELETE);
    holder = 0;
    holds = 0;
}

bool EventFlag::depart_locked(ThreadState &thread, uint64_t generation, Departure reason) {
    return depart_queued(waiters, thread, generation, reason, [&](auto &waiter) {
        if (waiter.entry.out_bits)
            *waiter.entry.out_bits = flags;
        if (reason == Departure::timeout)
            drop_hold();
        // Prototype only: what departure_before_cleanup checks while the waiter is held
        if (reason == Departure::callback) {
            proto_out_bits = waiter.entry.out_bits;
            proto_departed_flags = flags;
        }
    });
}

bool EventFlag::proto_departure_reached() {
    // The waiter left the queue with its hold and the flags of its departure in its local out bits,
    // and a set came after
    return waiters.empty() && holds > 0 && proto_out_bits && *proto_out_bits == proto_departed_flags && flags != proto_departed_flags;
}

void EventFlag::drop_hold() {
    if (holds > 0 && --holds == 0)
        holder = 0;
}

bool EventFlag::held_by_other(SceUID thread_id) const {
    return holds > 0 && holder != thread_id;
}

SceInt32 EventFlag::try_take(SceUID thread_id, SceUInt32 pattern, SceUInt32 wait_mode, SceUInt32 *out_bits) {
    if (held_by_other(thread_id))
        return SCE_KERNEL_ERROR_EVF_MULTI;

    bool condition;
    if (wait_mode & SCE_EVENT_WAITOR) {
        condition = flags & pattern;
    } else {
        condition = (flags & pattern) == pattern;
    }

    if (out_bits) {
        *out_bits = flags;
    }

    if (!condition)
        return SCE_KERNEL_ERROR_EVF_COND;

    if (wait_mode & SCE_EVENT_WAITCLEAR) {
        flags = 0;
    }

    if (wait_mode & SCE_EVENT_WAITCLEAR_PAT) {
        flags &= ~pattern;
    }

    return SCE_KERNEL_OK;
}

SceInt32 EventFlag::wait(const ThreadStatePtr &thread, SceUInt32 pattern, SceUInt32 wait_mode, SceUInt32 *out_bits, SceUInt32 *timeout, bool callbacks) {
    auto guard = lock();
    if (!guard)
        return SCE_KERNEL_ERROR_UNKNOWN_EVF_ID;
    if (const SceInt32 error = check_wait(pattern, wait_mode))
        return error;
    if (held_by_other(thread->id))
        return SCE_KERNEL_ERROR_EVF_MULTI;

    // A flag without SCE_EVENT_WAITMULTIPLE is held while the wait lasts, take() releases it
    if (!(attr & SCE_EVENT_WAITMULTIPLE)) {
        holder = thread->id;
        ++holds;
    }
    const WaitResult r = wait_loop(*this, waiters, guard, thread, { SCE_KERNEL_WAITTYPE_EVENTFLAG, uid, callbacks }, { wait_mode, pattern, out_bits }, timeout, CallbacksCheck::after_wait, [&](WaitEntry &entry, SceInt32) -> std::optional<SceInt32> {
        if (matches(entry)) {
            take(entry);
            return SCE_KERNEL_OK;
        }
        // A wait whose deadline is already gone returns these bits
        if (entry.out_bits)
            *entry.out_bits = flags;
        return std::nullopt;
    });
    // A deleted flag's bits can't change any more, so a wait that a delete ended gets them when it
    // returns, also one deleted while its callbacks ran out of the queue
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

    return try_take(thread_id, pattern, wait_mode, out_bits);
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

bool EventFlag::matches(const WaitEntry &wait) const {
    return (wait.wait_mode & SCE_EVENT_WAITOR) ? (flags & wait.pattern) != 0 : (flags & wait.pattern) == wait.pattern;
}

void EventFlag::take(const WaitEntry &wait) {
    if (wait.out_bits)
        *wait.out_bits = flags;
    if (wait.wait_mode & SCE_EVENT_WAITCLEAR)
        flags = 0;
    if (wait.wait_mode & SCE_EVENT_WAITCLEAR_PAT)
        flags &= ~wait.pattern;
    drop_hold();
}

void EventFlag::serve_locked() {
    waiters.remove_if([&](auto &waiter) {
        if (!matches(waiter.entry))
            return false;
        waiters.claim(waiter, SCE_KERNEL_OK);
        take(waiter.entry);
        return true;
    });
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
    waiters.remove_if([&](auto &waiter) {
        waiters.claim(waiter, SCE_KERNEL_ERROR_WAIT_CANCEL);
        if (waiter.entry.out_bits)
            *waiter.entry.out_bits = pattern;
        drop_hold();
        ++nb_threads;
        return true;
    });

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

// The create event comes with the creating thread's user data, like a simple event's
MsgPipe::MsgPipe(SceUInt32 attr, const char *name, std::size_t buf_size, SceUID creator)
    : WithUidClass(attr, name, 0, system_event_user_data(creator))
    , senders(attr & SCE_KERNEL_MSG_PIPE_ATTR_TH_PRIO_S ? WaitQueue<Sender>::Order::priority : WaitQueue<Sender>::Order::fifo)
    , receivers(attr & SCE_KERNEL_MSG_PIPE_ATTR_TH_PRIO_R ? WaitQueue<Receiver>::Order::priority : WaitQueue<Receiver>::Order::fifo)
    // A pipe's buffer is a whole number of 4 KiB pages
    , data_buffer(align(buf_size, 0x1000)) {}

SceInt32 MsgPipe::check_create(const char *name, SceUInt32 type, SceUInt32 attr, SceSize buf_size) {
    if (const SceInt32 error = check_name(name, attr))
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
    // The delete event and the end of the event waits come first, as for any event
    Event::on_delete(deleter);
    senders.end_all(SCE_KERNEL_ERROR_WAIT_DELETE);
    receivers.end_all(SCE_KERNEL_ERROR_WAIT_DELETE);
}

bool MsgPipe::depart_locked(ThreadState &thread, uint64_t generation, Departure reason) {
    // A wait is in one queue, so at most one of them has it
    return depart_queued(senders, thread, generation, reason) || depart_queued(receivers, thread, generation, reason) || Event::depart_locked(thread, generation, reason);
}

// The size of all the pieces together
static uint64_t total_size(std::span<const MsgPipeVector> pieces) {
    uint64_t total = 0;
    for (const MsgPipeVector &piece : pieces)
        total += piece.size;
    return total;
}

MsgPipe::Transfer::Transfer(MemState &mem, std::span<const MsgPipeVector> pieces, SceSize *size, SceSize *moved, SceSize total, bool full)
    : pieces(pieces)
    , mem(&mem)
    , size(size)
    , moved(moved)
    , total(total)
    , left(total)
    , full(full) {}

MsgPipe::Receiver::Receiver(MemState &mem, std::span<const MsgPipeVector> pieces, SceSize *size, SceSize *moved, SceSize total, bool full, bool peek)
    : Transfer(mem, pieces, size, moved, total, full)
    , peek(peek) {}

bool MsgPipe::Transfer::done() const {
    return full ? left == 0 : left < total;
}

bool MsgPipe::Transfer::keeps_place() const {
    return left != total;
}

std::span<char> MsgPipe::Transfer::run(SceSize n) {
    // Steps over the pieces already moved, and empty ones
    while (offset == pieces[piece].size) {
        ++piece;
        offset = 0;
    }
    const MsgPipeVector &current = pieces[piece];
    return std::span{ current.base.cast<char>().get(*mem), current.size }.subspan(offset, std::min(n, current.size - offset));
}

void MsgPipe::Transfer::advance(SceSize n) {
    offset += n;
    left -= n;
}

void MsgPipe::Transfer::report() const {
    if (size)
        *size = total - left;
    *moved = total - left;
}

void MsgPipe::copy(Sender &sender, Receiver &receiver, SceSize n) {
    // A copy of the sender moves through its pieces, and a peek leaves the sender where it was
    Sender from = sender;
    while (n > 0) {
        const std::span<char> in = from.run(n);
        const std::span<char> out = receiver.run(static_cast<SceSize>(in.size()));
        std::ranges::copy(in.first(out.size()), out.begin());
        const SceSize moved = static_cast<SceSize>(out.size());
        from.advance(moved);
        receiver.advance(moved);
        n -= moved;
    }
    if (!receiver.peek)
        sender = from;
}

void MsgPipe::fill(Sender &sender) {
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

void MsgPipe::drain(Receiver &receiver) {
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
    for (const auto &sender : senders)
        bytes += sender.entry.left;
    return bytes;
}

bool MsgPipe::give(Sender &sender) {
    // A send reaches the first waiting receiver even with no bytes, which ends an ASAP receive
    while (auto *const receiver = receivers.front()) {
        Receiver &to = receiver->entry;
        copy(sender, to, std::min(to.left, sender.left));
        to.report();
        // A FULL receive that still wants bytes stays the first one, and keeps waiting
        if (to.full && to.left > 0)
            break;
        receivers.end(*receiver);
        if (sender.left == 0)
            break;
    }
    fill(sender);
    sender.report();
    return sender.done();
}

bool MsgPipe::take(Receiver &receiver) {
    drain(receiver);
    // Each waiting sender gives its bytes to the receiver while it wants some, then puts its rest
    // into the pipe, up to the first sender that stays
    while (auto *const sender = senders.front()) {
        Sender &from = sender->entry;
        copy(from, receiver, std::min(receiver.left, from.left));
        // A peek frees nothing
        if (receiver.peek)
            break;
        fill(from);
        from.report();
        if (!from.done())
            break;
        senders.end(*sender);
    }
    receiver.report();
    return receiver.done();
}

void MsgPipe::serve_locked() {
    // The first receiver goes first, so the pipe's bytes go out before those of waiting senders
    while (true) {
        if (auto *const receiver = receivers.front(); receiver && take(receiver->entry))
            receivers.end(*receiver);
        else if (auto *const sender = senders.front(); sender && give(sender->entry))
            senders.end(*sender);
        else
            break;
    }
    Event::serve_locked();
}

void MsgPipe::post(SceUInt32 done_bit, SceInt32 result, SceSize moved) {
    SceUInt32 bits = moved > 0 ? done_bit : 0;
    if (done_bit == SCE_KERNEL_EVENT_IN && moved > 0 && data_buffer.Used() > 0)
        bits |= SCE_KERNEL_EVENT_DATA_EXIST;
    // A suspend or a stop of the caller isn't the pipe's error
    if (result < 0 && result != SCE_KERNEL_ERROR_THREAD_STOPPED && result != SCE_KERNEL_ERROR_THREAD_SUSPENDED)
        bits |= SCE_KERNEL_EVENT_ERROR;
    if (done_bit == SCE_KERNEL_EVENT_OUT && data_buffer.Used() == 0)
        pattern &= ~SCE_KERNEL_EVENT_DATA_EXIST;
    // The size moved, all ones for an error
    const SceUInt32 size = moved | (result < 0 ? 0xFFFFFFFFU : 0);
    fire(bits, (static_cast<SceUInt64>(size) << 32) | static_cast<SceUInt32>(result), true);
}

template <typename Side>
bool MsgPipe::goes_first(WaitQueue<Side> &side, int priority) {
    // Nothing goes ahead of a head that moved part of its bytes
    const auto *const head = side.front();
    return side.first_for(priority) && (!head || !head->entry.keeps_place());
}

SceInt32 MsgPipe::receive(const ThreadStatePtr &thread, MemState &mem, SceUInt32 wait_mode, std::span<const MsgPipeVector> pieces, SceSize *size, SceUInt32 *timeout, bool callbacks) {
    if (!is_msg_pipe_mode(wait_mode))
        return SCE_KERNEL_ERROR_ILLEGAL_MODE;
    // DONT_WAIT ends like a try, and a wait with callbacks runs them after it
    if (wait_mode & SCE_KERNEL_MSG_PIPE_MODE_DONT_WAIT) {
        const SceInt32 result = try_receive(thread, mem, wait_mode, pieces, size);
        if (callbacks)
            thread->process_callbacks();
        return result;
    }
    const uint64_t total = total_size(pieces);
    if (total > data_buffer.Capacity())
        return SCE_KERNEL_ERROR_ILLEGAL_SIZE;

    auto guard = lock();
    if (!guard)
        return SCE_KERNEL_ERROR_UNKNOWN_MSG_PIPE_ID;
    SceSize moved = 0;
    Receiver receiver(mem, pieces, size, &moved, static_cast<SceSize>(total), wait_mode & SCE_KERNEL_MSG_PIPE_MODE_FULL, wait_mode & SCE_KERNEL_MSG_PIPE_MODE_DONT_REMOVE);
    // A receive of nothing ends at once, before any callback runs, with no event
    if (receiver.total == 0) {
        receiver.report();
        return SCE_KERNEL_OK;
    }

    const auto take_bytes = [&](Receiver &transfer, SceInt32) -> std::optional<SceInt32> {
        // Don't take anything while other receivers are ahead in the queue. The size is written
        // after every attempt.
        const bool done = goes_first(receivers, thread->get_priority()) && take(transfer);
        transfer.report();
        if (done)
            return SCE_KERNEL_OK;
        return std::nullopt;
    };
    const WaitResult r = wait_loop(*this, receivers, guard, thread, { SCE_KERNEL_WAITTYPE_MSG_PIPE, uid, callbacks }, receiver, timeout, CallbacksCheck::after_take_or_wait, take_bytes, [&](SceInt32 result) {
        // The receive's end is an event of the pipe, posted before the callbacks notified during it
        // run
        post(SCE_KERNEL_EVENT_OUT, result, moved);
    });
    return guest_result(r);
}

SceInt32 MsgPipe::send(const ThreadStatePtr &thread, MemState &mem, SceUInt32 wait_mode, std::span<const MsgPipeVector> pieces, SceSize *size, SceUInt32 *timeout, bool callbacks) {
    if (!is_msg_pipe_mode(wait_mode))
        return SCE_KERNEL_ERROR_ILLEGAL_MODE;
    // DONT_WAIT ends like a try, and a wait with callbacks runs them after it
    if (wait_mode & SCE_KERNEL_MSG_PIPE_MODE_DONT_WAIT) {
        const SceInt32 result = try_send(thread, mem, wait_mode, pieces, size);
        if (callbacks)
            thread->process_callbacks();
        return result;
    }
    const uint64_t total = total_size(pieces);
    if (total > data_buffer.Capacity())
        return SCE_KERNEL_ERROR_ILLEGAL_SIZE;

    auto guard = lock();
    if (!guard)
        return SCE_KERNEL_ERROR_UNKNOWN_MSG_PIPE_ID;
    SceSize moved = 0;
    Sender sender(mem, pieces, size, &moved, static_cast<SceSize>(total), wait_mode & SCE_KERNEL_MSG_PIPE_MODE_FULL);
    // A send of nothing ends at once, before any callback runs, with no event
    if (sender.total == 0) {
        sender.report();
        return SCE_KERNEL_OK;
    }

    const auto give_bytes = [&](Sender &transfer, SceInt32) -> std::optional<SceInt32> {
        // Don't send anything while other senders are ahead in the queue. The size is written
        // after every attempt.
        const bool done = goes_first(senders, thread->get_priority()) && give(transfer);
        transfer.report();
        if (done)
            return SCE_KERNEL_OK;
        return std::nullopt;
    };
    const WaitResult r = wait_loop(*this, senders, guard, thread, { SCE_KERNEL_WAITTYPE_MSG_PIPE, uid, callbacks }, sender, timeout, CallbacksCheck::after_take_or_wait, give_bytes, [&](SceInt32 result) {
        // The send's end is an event of the pipe, posted before the callbacks notified during it run
        post(SCE_KERNEL_EVENT_IN, result, moved);
    });
    return guest_result(r);
}

SceInt32 MsgPipe::try_receive(const ThreadStatePtr &thread, MemState &mem, SceUInt32 wait_mode, std::span<const MsgPipeVector> pieces, SceSize *size) {
    if (!is_msg_pipe_mode(wait_mode))
        return SCE_KERNEL_ERROR_ILLEGAL_MODE;
    const uint64_t total = total_size(pieces);
    if (total > data_buffer.Capacity())
        return SCE_KERNEL_ERROR_ILLEGAL_SIZE;

    const auto guard = lock();
    if (!guard)
        return SCE_KERNEL_ERROR_UNKNOWN_MSG_PIPE_ID;
    SceSize moved = 0;
    Receiver receiver(mem, pieces, size, &moved, static_cast<SceSize>(total), wait_mode & SCE_KERNEL_MSG_PIPE_MODE_FULL, wait_mode & SCE_KERNEL_MSG_PIPE_MODE_DONT_REMOVE);
    // Behind the receivers already waiting, or short of bytes for all of a FULL receive, it fails
    // without writing a size
    const bool can_take = goes_first(receivers, thread->get_priority()) && !(receiver.full && receiver.total > available());
    const SceInt32 result = can_take && take(receiver) ? SCE_KERNEL_OK : SCE_KERNEL_ERROR_MSG_PIPE_EMPTY;
    // Every try is an event of the pipe
    post(SCE_KERNEL_EVENT_OUT, result, moved);
    return result;
}

SceInt32 MsgPipe::try_send(const ThreadStatePtr &thread, MemState &mem, SceUInt32 wait_mode, std::span<const MsgPipeVector> pieces, SceSize *size) {
    if (!is_msg_pipe_mode(wait_mode))
        return SCE_KERNEL_ERROR_ILLEGAL_MODE;
    const uint64_t total = total_size(pieces);
    if (total > data_buffer.Capacity())
        return SCE_KERNEL_ERROR_ILLEGAL_SIZE;

    const auto guard = lock();
    if (!guard)
        return SCE_KERNEL_ERROR_UNKNOWN_MSG_PIPE_ID;
    SceSize moved = 0;
    Sender sender(mem, pieces, size, &moved, static_cast<SceSize>(total), wait_mode & SCE_KERNEL_MSG_PIPE_MODE_FULL);
    // Behind the senders already waiting, or short of room for all of a FULL send, it fails without
    // writing a size
    const bool can_give = goes_first(senders, thread->get_priority()) && !(sender.full && sender.total > data_buffer.Free());
    const SceInt32 result = can_give && give(sender) ? SCE_KERNEL_OK : SCE_KERNEL_ERROR_MSG_PIPE_FULL;
    // Every try is an event of the pipe
    post(SCE_KERNEL_EVENT_IN, result, moved);
    return result;
}

SceInt32 MsgPipe::cancel(SceUInt32 *num_send_wait_threads, SceUInt32 *num_receive_wait_threads) {
    const auto guard = lock();
    if (!guard)
        return SCE_KERNEL_ERROR_UNKNOWN_MSG_PIPE_ID;

    const auto nb_senders = static_cast<SceUInt32>(senders.end_all(SCE_KERNEL_ERROR_WAIT_CANCEL));
    const auto nb_receivers = static_cast<SceUInt32>(receivers.end_all(SCE_KERNEL_ERROR_WAIT_CANCEL));
    // The ring empties with no event, and each woken waiter posts its own as it returns
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
