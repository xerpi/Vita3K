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

#include <cstdint>

// Where a thread's guest code stopped, which a hold that finds it there reports
enum class StopPoint : uint8_t {
    // In its own code
    guest_code,
    // On its way back from a call into the kernel
    syscall,
};

// How guest code goes on after a check
enum class GuestResume : uint8_t {
    run,
    // One instruction, for the debugger
    step,
    // Not now: the level returned, the thread is ending, or it doesn't run
    leave,
};

// The kernel side of a thread's guest code. The CPU backend calls it with no lock held, from the
// host thread that runs the code.
class GuestBoundary {
public:
    // Checks whether guest code may go on. where is where it stopped, debugger_stop says a finished
    // step or a breakpoint stopped it, and returned says the level's guest function returned. It
    // blocks while something holds the thread.
    virtual GuestResume resume_guest(StopPoint where, bool debugger_stop, bool returned) = 0;
    // Runs the import with this NID, which the guest code called
    virtual void kernel_call(uint32_t nid) = 0;
    // Runs the guest's abort handler for an abort the CPU noted
    virtual void guest_abort() = 0;

protected:
    ~GuestBoundary() = default;
};
