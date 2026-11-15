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

#include <chrono>
#include <cstdint>
#include <mutex>

// The process time in microseconds: system time minus a base while the process runs, and the
// frozen time while it is suspended
class ProcessClock {
public:
    // The system time in microseconds, the one host clock every kernel time read comes from
    static uint64_t system_now() {
        return std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now().time_since_epoch()).count();
    }

    uint64_t now() const {
        const std::lock_guard<std::mutex> guard(lock);
        return running ? system_now() - base : base;
    }

    // The system time a process time falls at if the process runs from now on
    uint64_t to_system(uint64_t process_time) const {
        const std::lock_guard<std::mutex> guard(lock);
        return running ? process_time + base : system_now() + (process_time - base);
    }

    // Starts the process time at 0
    void start() {
        const std::lock_guard<std::mutex> guard(lock);
        base = system_now();
        running = true;
    }

    bool suspended() const {
        const std::lock_guard<std::mutex> guard(lock);
        return !running;
    }

    // Freezes the process time for a process suspend
    void suspend() {
        const std::lock_guard<std::mutex> guard(lock);
        running = false;
        base = system_now() - base;
    }

    // Lets the process time go on from where it froze
    void resume() {
        const std::lock_guard<std::mutex> guard(lock);
        running = true;
        base = system_now() - base;
    }

private:
    mutable std::mutex lock;
    // The system time the process time counts from while running, the frozen time while suspended
    uint64_t base = 0;
    bool running = false;
};
