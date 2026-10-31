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
#include <mem/state.h>
#include <mem/util.h>

#include <map>

struct KernelState;

struct Breakpoint {
    bool thumb_mode;
    unsigned char data[4];
};

struct Debugger {
    explicit Debugger(KernelState &kernel);

    bool watch_import_calls = false;
    bool watch_code = false;
    bool watch_memory = false;

    bool log_imports = false;
    bool log_exports = false;
    bool dump_elfs = false;

    void add_breakpoint(MemState &mem, uint32_t addr, bool thumb_mode);
    void remove_breakpoint(MemState &mem, uint32_t addr);
    // Applies watch_memory to every thread
    void update_watches();
    void deinit();

private:
    std::mutex mutex;
    KernelState &parent;
    std::map<Address, Breakpoint> breakpoints;
};
