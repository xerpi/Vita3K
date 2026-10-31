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

#include <kernel/debugger.h>
#include <kernel/state.h>

constexpr unsigned char THUMB_BREAKPOINT[2] = { 0x00, 0xBE };
constexpr unsigned char ARM_BREAKPOINT[4] = { 0x70, 0x00, 0x20, 0xE1 };

void Debugger::add_breakpoint(MemState &mem, uint32_t addr, bool thumb_mode) {
    const auto lock = std::lock_guard(mutex);
    uint8_t *const code = Ptr<uint8_t>(addr).get(mem);
    const size_t size = thumb_mode ? sizeof(THUMB_BREAKPOINT) : sizeof(ARM_BREAKPOINT);
    Breakpoint bk{ thumb_mode };
    std::memcpy(bk.data, code, size);
    std::memcpy(code, thumb_mode ? THUMB_BREAKPOINT : ARM_BREAKPOINT, size);
    breakpoints.emplace(addr, bk);
    parent.invalidate_jit_cache(addr, 4);
}

void Debugger::remove_breakpoint(MemState &mem, uint32_t addr) {
    const auto lock = std::lock_guard(mutex);
    if (breakpoints.contains(addr)) {
        auto last = breakpoints[addr];
        std::memcpy(Ptr<uint8_t>(addr).get(mem), &last.data, last.thumb_mode ? sizeof(THUMB_BREAKPOINT) : sizeof(ARM_BREAKPOINT));
        breakpoints.erase(addr);
        parent.invalidate_jit_cache(addr, 4);
    }
}

Debugger::Debugger(KernelState &kernel)
    : parent(kernel) {
}

void Debugger::update_watches() {
    parent.set_memory_watch(watch_memory);
}

void Debugger::deinit() {
    std::lock_guard<std::mutex> lock(mutex);
    breakpoints.clear();
}
