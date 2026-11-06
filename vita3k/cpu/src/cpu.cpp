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

#include <cpu/disasm/functions.h>
#include <cpu/impl/dynarmic_cpu.h>
#include <cpu/state.h>
#include <mem/ptr.h>
#include <util/types.h>

#include <memory>
#include <string>

CPUState::CPUState(MemState &mem, SceUID thread_id)
    : thread_id(thread_id)
    , mem(mem) {}

CPUStatePtr init_cpu(MemState &mem, SceUID thread_id, std::size_t processor_id, CPUOptions options) {
    CPUStatePtr cpu = std::make_unique<DynarmicCPU>(mem, thread_id, processor_id, options);
    if (!init(cpu->disasm))
        return nullptr;
    return cpu;
}

void CPUState::set_pc(uint32_t val) {
    if (val & 1) {
        set_cpsr(get_cpsr() | 0x20);
        val = val & 0xFFFFFFFE;
    } else {
        set_cpsr(get_cpsr() & 0xFFFFFFDF);
        val = val & 0xFFFFFFFC;
    }
    set_reg(15, val);
}

void CPUState::note_abort(Address addr) {
    abort_fault_addr.store(addr);
    abort_pending.store(true);
    request_stop();
}

std::optional<Address> CPUState::take_abort() {
    if (!abort_pending.exchange(false))
        return std::nullopt;
    return abort_fault_addr.load();
}

std::string CPUState::disassemble(uint64_t at, bool thumb, uint16_t *insn_size) {
    const uint8_t *const code = Ptr<const uint8_t>(static_cast<Address>(at)).get(mem);
    const size_t buffer_size = GiB(4) - at;
    return ::disassemble(disasm, code, buffer_size, at, thumb, insn_size);
}

std::string CPUState::disassemble(uint64_t at, uint16_t *insn_size) {
    return disassemble(at, is_thumb_mode(), insn_size);
}
