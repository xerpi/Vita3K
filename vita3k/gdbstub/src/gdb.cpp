#include <memory>

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

#include <emuenv/state.h>
#include <util/log.h>

#include <gdbstub/functions.h>
#include <gdbstub/state.h>

#include <cpu/state.h>

#include <array>
#include <bit>
#include <cerrno>
#include <iterator>
#include <kernel/state.h>
#include <mem/state.h>
#include <optional>
#include <span>
#include <sstream>

// Sockets
#ifdef _WIN32
#include <winsock.h>
#else
#include <arpa/inet.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <unistd.h>
#endif

#define LOG_GDB_LEVEL 0

#if LOG_GDB_LEVEL >= 1
#define LOG_GDB LOG_INFO
#else
#define LOG_GDB(a, ...)
#endif

#if LOG_GDB_LEVEL >= 2
#define LOG_GDB_DEBUG LOG_INFO
#else
#define LOG_GDB_DEBUG(a, ...)
#endif

// Credit to jfhs for their GDB stub for RPCS3 which this stub is based on.

typedef char PacketData[1200];

// The longest packet the stub waits for before it drops it
constexpr size_t MAX_PACKET_SIZE = 16384;

struct PacketCommand {
    char *data{};
    int64_t length = -1;

    int64_t begin_index = -1;
    int64_t end_index = -1;
    int64_t content_length = -1;

    char *content_start{};
    char *checksum_start{};

    uint8_t checksum = 0;

    bool is_valid = false;
};

typedef std::function<std::string(EmuEnvState &state, PacketCommand &command)> PacketFunction;

struct PacketFunctionBundle {
    std::string_view name;
    PacketFunction function;
};

static std::string content_string(PacketCommand &command) {
    return std::string(command.content_start, static_cast<unsigned long>(command.content_length));
}

// Hexes
static std::string be_hex(uint32_t value) {
    return fmt::format("{:0>8x}", std::byteswap(value));
}

static std::string to_hex(uint32_t value) {
    return fmt::format("{:0>8x}", value);
}

static std::string to_hex(SceUID value) {
    return fmt::format("{:0>8x}", value);
}

static uint32_t parse_hex(const std::string &hex) {
    return static_cast<uint32_t>(std::strtoul(hex.c_str(), nullptr, 16));
}

// A register value gdb sends, as hex bytes in the target's little endian order
static uint32_t parse_reg(const std::string &hex) {
    return std::byteswap(parse_hex(hex));
}

static uint8_t make_checksum(const char *data, int64_t length) {
    size_t sum = 0;

    for (int64_t a = 0; a < length; a++) {
        sum += data[a];
    }

    return static_cast<uint8_t>(sum % 256);
}

static PacketCommand parse_command(char *data, int64_t length) {
    PacketCommand command = {};

    command.data = data;
    command.length = length;

    // TODO: Use std::find() to find packet begin and end.
    if (length > 1)
        command.begin_index = 1;
    for (int64_t a = 0; a < length; a++) {
        if (data[a] == '#')
            command.end_index = a;
    }

    command.is_valid = command.begin_index != -1 && command.end_index != -1
        && command.end_index > command.begin_index && command.end_index + 2 < length;
    if (!command.is_valid)
        return command;

    command.content_start = command.data + command.begin_index;
    command.content_length = command.end_index - command.begin_index;

    command.checksum_start = command.data + command.end_index + 1;
    command.checksum = static_cast<uint8_t>(parse_hex(std::string(command.checksum_start, 2)));

    command.is_valid = make_checksum(command.content_start, command.content_length) == command.checksum;
    if (!command.is_valid)
        return command;

    return command;
}

static int64_t server_reply(GDBState &state, const char *data, int64_t length) {
    uint8_t checksum = make_checksum(data, length);
    std::string packet_data = fmt::format("${}#{:0>2x}", std::string(data, length), checksum);
    size_t sent = 0;
    while (sent < packet_data.size()) {
        const int64_t n = send(state.client_socket, packet_data.data() + sent, static_cast<int>(packet_data.size() - sent), 0);
        // A signal can interrupt send before it writes anything, so try again
        if (n < 0 && errno == EINTR)
            continue;
        if (n <= 0)
            return n;
        sent += static_cast<size_t>(n);
    }
    return static_cast<int64_t>(sent);
}

static int64_t server_reply(GDBState &state, const char *text) {
    return server_reply(state, text, strlen(text));
}

static int64_t server_ack(GDBState &state, char ack = '+') {
    return send(state.client_socket, &ack, 1, 0);
}

static std::string cmd_supported(EmuEnvState &state, PacketCommand &command) {
    return "multiprocess-;swbreak+;hwbreak-;qRelocInsn-;fork-events-;vfork-events-;"
           "exec-events-;vContSupported+;QThreadEvents-;no-resumed-;qXfer:features:read+";
}

// Register numbers in the target description and in remote packets.
constexpr uint32_t REG_CPSR = 16;
constexpr uint32_t REG_D0 = 17;
constexpr uint32_t VFP_DOUBLE_REGS = 32;
constexpr uint32_t REG_FPSCR = REG_D0 + VFP_DOUBLE_REGS;
constexpr uint32_t REG_Q0 = REG_FPSCR + 1;
constexpr uint32_t NEON_QUAD_REGS = 16;

// Core, VFP and NEON registers. gdb shows s0-s31 from the d registers by itself but not q0-q15, so
// the q registers are listed too.
static std::string make_target_xml() {
    std::string xml = R"(<?xml version="1.0"?>
<!DOCTYPE target SYSTEM "gdb-target.dtd">
<target version="1.0">
<architecture>arm</architecture>
<feature name="org.gnu.gdb.arm.core">
)";
    for (int r = 0; r < 13; r++)
        fmt::format_to(std::back_inserter(xml), R"(<reg name="r{}" bitsize="32" type="uint32"/>)", r);
    xml += fmt::format(R"(<reg name="sp" bitsize="32" type="data_ptr"/>
<reg name="lr" bitsize="32" type="uint32"/>
<reg name="pc" bitsize="32" type="code_ptr"/>
<reg name="cpsr" bitsize="32" type="uint32" regnum="{}"/>
</feature>
<feature name="org.gnu.gdb.arm.vfp">
)",
        REG_CPSR);
    for (uint32_t d = 0; d < VFP_DOUBLE_REGS; d++)
        fmt::format_to(std::back_inserter(xml), R"(<reg name="d{}" bitsize="64" type="ieee_double" regnum="{}"/>)", d, REG_D0 + d);
    xml += fmt::format(R"(<reg name="fpscr" bitsize="32" type="int" group="float" regnum="{}"/>
</feature>
<feature name="org.gnu.gdb.arm.neon">
<vector id="v4u32" type="uint32" count="4"/>
)",
        REG_FPSCR);
    for (uint32_t q = 0; q < NEON_QUAD_REGS; q++)
        fmt::format_to(std::back_inserter(xml), R"(<reg name="q{}" bitsize="128" type="v4u32" regnum="{}"/>)", q, REG_Q0 + q);
    xml += "</feature>\n</target>";
    return xml;
}

static const std::string TARGET_XML = make_target_xml();

// qXfer:features:read:target.xml:OFFSET,LENGTH returns the part of TARGET_XML gdb asks for, with m
// before it while more follows and l before the last part
static std::string cmd_target_xml(EmuEnvState &state, PacketCommand &command) {
    const std::string content = content_string(command);
    const size_t comma = content.find(',');
    const size_t colon = content.rfind(':', comma);
    if (colon == std::string::npos || comma == std::string::npos)
        return "E00";
    const size_t offset = parse_hex(content.substr(colon + 1, comma - colon - 1));
    const size_t length = parse_hex(content.substr(comma + 1));
    if (offset >= TARGET_XML.size())
        return "l";
    const std::string part = TARGET_XML.substr(offset, length);
    return (offset + part.size() < TARGET_XML.size() ? "m" : "l") + part;
}

static std::string cmd_reply_empty(EmuEnvState &state, PacketCommand &command) {
    return "";
}

// This function is not thread safe
static SceUID select_thread(EmuEnvState &state, int thread_id) {
    if (thread_id == 0) {
        if (state.kernel.threads.empty())
            return -1;
        return state.kernel.threads.begin()->first;
    }
    return thread_id;
}

static std::string cmd_set_current_thread(EmuEnvState &state, PacketCommand &command) {
    const auto guard = std::lock_guard(state.kernel.mutex);
    int32_t thread_id = parse_hex(std::string(
        command.content_start + 2, static_cast<unsigned long>(command.content_length - 2)));

    switch (command.content_start[1]) {
    case 'c':
        LOG_GDB("GDB Server Deprecated Continue Option 'c'");
        // state.gdb.current_continue_thread = select_thread(state, thread_id);
        break;
    case 'g':
        state.gdb.current_thread = select_thread(state, thread_id);
        break;
    default:
        LOG_GDB("GDB Server Unknown Set Current Thread OP. {}", command.content_start[1]);
        break;
    }

    return "OK";
}

static std::string cmd_get_current_thread(EmuEnvState &state, PacketCommand &command) {
    return "QC" + to_hex(state.gdb.current_thread);
}

// How many 32-bit words a register has in p and P packets
static size_t reg_words(uint32_t reg) {
    if (reg >= REG_D0 && reg < REG_D0 + VFP_DOUBLE_REGS)
        return 2;
    if (reg >= REG_Q0 && reg < REG_Q0 + NEON_QUAD_REGS)
        return 4;
    return 1;
}

// The context word that a register starts at, or nothing for an unknown register
static std::optional<uint32_t> reg_first_word(uint32_t reg) {
    if (reg <= 15)
        return reg;
    if (reg == REG_CPSR)
        return CPUContext::WORD_CPSR;
    if (reg >= REG_D0 && reg < REG_D0 + VFP_DOUBLE_REGS)
        return CPUContext::WORD_S0 + (reg - REG_D0) * 2;
    if (reg == REG_FPSCR)
        return CPUContext::WORD_FPSCR;
    if (reg >= REG_Q0 && reg < REG_Q0 + NEON_QUAD_REGS)
        return CPUContext::WORD_S0 + (reg - REG_Q0) * 4;
    return std::nullopt;
}

// The thread gdb selected, or null
static GuestThreadPtr selected_thread(EmuEnvState &state) {
    const auto guard = std::lock_guard(state.kernel.mutex);
    const auto it = state.kernel.threads.find(state.gdb.current_thread);
    return it == state.kernel.threads.end() ? nullptr : it->second;
}

static std::string cmd_read_registers(EmuEnvState &state, PacketCommand &command) {
    const GuestThreadPtr thread = selected_thread(state);
    if (!thread)
        return "E00";
    CPUContext context;
    if (!thread->debugger_context(context))
        return "E01";

    // gdb reads the registers after cpsr one by one with p
    std::string str;
    str.reserve((CPUContext::WORD_CPSR + 1) * 8);
    for (uint32_t i = 0; i <= CPUContext::WORD_CPSR; i++)
        str += be_hex(context.word(i));
    return str;
}

static std::string cmd_write_registers(EmuEnvState &state, PacketCommand &command) {
    const GuestThreadPtr thread = selected_thread(state);
    if (!thread)
        return "E00";

    // r0 to r15 and cpsr, as many as the packet has
    const std::string content = content_string(command).substr(1);
    std::array<uint32_t, CPUContext::WORD_CPSR + 1> words;
    const size_t count = std::min(words.size(), content.size() / 8);
    for (size_t i = 0; i < count; i++)
        words[i] = parse_reg(content.substr(i * 8, 8));
    if (!thread->set_debugger_regs(0, std::span(words).first(count)))
        return "E01";
    return "OK";
}

static std::string cmd_read_register(EmuEnvState &state, PacketCommand &command) {
    const GuestThreadPtr thread = selected_thread(state);
    if (!thread)
        return "E00";
    CPUContext context;
    if (!thread->debugger_context(context))
        return "E01";

    const std::string content = content_string(command);
    const uint32_t reg = parse_hex(content.substr(1));
    const std::optional<uint32_t> first = reg_first_word(reg);
    if (!first) {
        LOG_GDB("GDB Server Queried Invalid Register {}", reg);
        return be_hex(0);
    }
    // A double register is two words and a quad register four, the low one first
    std::string result;
    for (uint32_t i = 0; i < reg_words(reg); i++)
        result += be_hex(context.word(*first + i));
    return result;
}

static std::string cmd_write_register(EmuEnvState &state, PacketCommand &command) {
    const GuestThreadPtr thread = selected_thread(state);
    if (!thread)
        return "E00";

    const std::string content = content_string(command);
    const size_t equal_index = content.find('=');
    const uint32_t reg = parse_hex(content.substr(1, equal_index - 1));
    const std::string value = content.substr(equal_index + 1);
    const size_t count = reg_words(reg);
    if (value.size() != count * 8)
        return "E00";
    const std::optional<uint32_t> first = reg_first_word(reg);
    if (!first) {
        LOG_GDB("GDB Server Modified Invalid Register {}", reg);
        return "OK";
    }
    // A double or quad register is written whole, the low word first
    std::array<uint32_t, 4> words;
    for (size_t i = 0; i < count; i++)
        words[i] = parse_reg(value.substr(i * 8, 8));
    if (!thread->set_debugger_regs(*first, std::span(words).first(count)))
        return "E01";
    return "OK";
}

static bool check_memory_region(Address address, Address length, MemState &mem) {
    // Every page the range touches, with the end in 64 bits so a range at the top of the address
    // space doesn't wrap around to an empty one
    const uint64_t end = static_cast<uint64_t>(address) + length;
    return address && end < (uint64_t{ 1 } << 32) && is_valid_addr_range(mem, address, static_cast<Address>(end));
}

static std::string cmd_read_memory(EmuEnvState &state, PacketCommand &command) {
    const std::string content = content_string(command);
    const size_t pos = content.find(',');

    const std::string first = content.substr(1, pos - 1);
    const std::string second = content.substr(pos + 1);
    const uint32_t address = parse_hex(first);
    const uint32_t length = parse_hex(second);

    if (!check_memory_region(address, length, state.mem))
        return "EAA";

    std::string str;
    str.reserve(length * 2);

    for (uint32_t a = 0; a < length; a++) {
        fmt::format_to(std::back_inserter(str), "{:02x}", *Ptr<uint8_t>(address + a).get(state.mem));
    }

    return str;
}

static std::string cmd_write_memory(EmuEnvState &state, PacketCommand &command) {
    const std::string content = content_string(command);
    const size_t pos_first = content.find(',');
    const size_t pos_second = content.find(':');

    const std::string first = content.substr(1, pos_first - 1);
    const std::string second = content.substr(pos_first + 1, pos_second - pos_first);
    const uint32_t address = parse_hex(first);
    const uint32_t length = parse_hex(second);
    const std::string hex_data = content.substr(pos_second + 1);

    if (!check_memory_region(address, length, state.mem))
        return "EAA";

    for (uint32_t a = 0; a < length; a++) {
        *Ptr<uint8_t>(address + a).get(state.mem) = static_cast<uint8_t>(parse_hex(hex_data.substr(a * 2, 2)));
    }

    return "OK";
}

// server_next() might not be able to tell the difference between the end of the packet ($) and 0x24 ($).
// Thus, cmd_write_binary is disabled.
static std::string cmd_write_binary(EmuEnvState &state, PacketCommand &command) {
    const std::string content = content_string(command);
    const size_t pos_first = content.find(',');
    const size_t pos_second = content.find(':');

    const std::string first = content.substr(1, pos_first - 1);
    const std::string second = content.substr(pos_first + 1, pos_second - pos_first);
    const uint32_t address = parse_hex(first);
    const uint32_t length = parse_hex(second);
    const char *data = command.content_start + pos_second + 1;

    if (!check_memory_region(address, length, state.mem))
        return "EAA";

    for (uint32_t a = 0; a < length; a++) {
        *Ptr<uint8_t>(address + a).get(state.mem) = data[a];
    }

    return "OK";
}

static std::string cmd_detach(EmuEnvState &state, PacketCommand &command) { return "OK"; }

static std::string log_stack_traceback(EmuEnvState &state, Address sp) {
    std::string str;
    for (Address addr = sp; addr <= sp + 1024; addr += 4) {
        if (!Ptr<uint32_t>(addr).valid(state.mem))
            continue;
        const Address value = *Ptr<uint32_t>(addr).get(state.mem);
        if (const auto mod = state.kernel.find_module_by_addr(value))
            fmt::format_to(std::back_inserter(str), "0x{:X} (module: {})\n", value, mod->module_name);
    }
    return str;
}

// The stop reply for thread, or an error once it is deleted
static std::string stop_reply(const GuestThread &thread) {
    if (thread.get_status() == ThreadStatus::deleted)
        return "E01";
    return "T05thread:" + to_hex(thread.state.id) + ";";
}

static std::vector<GuestThreadPtr> list_threads(EmuEnvState &state) {
    const auto guard = std::lock_guard(state.kernel.mutex);
    std::vector<GuestThreadPtr> threads;
    for (const auto &[_, thread] : state.kernel.threads)
        threads.push_back(thread);
    return threads;
}

static std::string cmd_continue(EmuEnvState &state, PacketCommand &command) {
    const std::string content = content_string(command);
    constexpr auto watch_delay = std::chrono::milliseconds(100);

    uint64_t index = 5;
    uint64_t next = 0;
    do {
        next = content.find(';', index + 1);
        std::string text = content.substr(index + 1, next - index - 1);

        const char cmd = text[0];
        switch (cmd) {
        case 's':
        case 'S': {
            // The action names the thread to step, else it's the current one
            const size_t colon = text.find(':');
            SceUID thread_id = colon == std::string::npos ? 0 : static_cast<SceUID>(parse_hex(text.substr(colon + 1)));
            if (thread_id <= 0)
                thread_id = state.gdb.current_thread;
            const GuestThreadPtr thread = state.kernel.get_thread(thread_id);
            if (!thread)
                return "E00";

            // A thread that isn't at a stop, or that something else holds, can't step, and gdb takes
            // the error as a stop
            if (!thread->step())
                return "E01";

            state.gdb.current_thread = thread_id;
            return stop_reply(*thread);
        }
        case 'c':
        case 'C': {
            // Resume the world. A thread something else holds stays stopped.
            state.kernel.set_threads_suspended(false);

            // Wait until some thread hits a breakpoint, and copy its registers while it's stopped
            GuestThreadPtr stopped;
            CPUContext context;
            while (!stopped) {
                auto lock = std::unique_lock(state.kernel.mutex);

                if (state.gdb.server_die)
                    return "";
                for (const auto &[_, thread] : state.kernel.threads) {
                    if (thread->debugger_context(context)) {
                        stopped = thread;
                        break;
                    }
                }

                lock.unlock();

                std::this_thread::sleep_for(std::chrono::milliseconds(watch_delay));
            }

            LOG_INFO("GDB Breakpoint trigger (thread name: {}, thread_id: {})", stopped->state.name, stopped->state.id);
            LOG_INFO("PC: {} LR: {}", context.get_pc(), context.get_lr());
            LOG_INFO("{}", log_stack_traceback(state, context.get_sp()));

            // Hold every thread for the debugger and wait until each one is at a stop or ended
            state.kernel.set_threads_suspended(true);
            for (const GuestThreadPtr &thread : list_threads(state))
                thread->wait_for_debugger_stop();

            state.gdb.current_thread = stopped->state.id;
            return stop_reply(*stopped);
        }
        default:
            LOG_GDB("Unsupported vCont command '{}'", cmd);
            break;
        }

        index = next;
    } while (next != std::string::npos);

    return "";
}

static std::string cmd_continue_supported(EmuEnvState &state, PacketCommand &command) {
    return "vCont;c;C;s;S;t;r";
}

static std::string cmd_thread_alive(EmuEnvState &state, PacketCommand &command) {
    const auto guard = std::lock_guard(state.kernel.mutex);
    const std::string content = content_string(command);
    const int32_t thread_id = parse_hex(content.substr(1));

    // A deleted thread stays in the map until its host thread ends
    const auto it = state.kernel.threads.find(thread_id);
    if (it != state.kernel.threads.end() && it->second->get_status() != ThreadStatus::deleted)
        return "OK";

    return "E00";
}

static std::string cmd_kill(EmuEnvState &state, PacketCommand &command) {
    return "OK";
}

static std::string cmd_die(EmuEnvState &state, PacketCommand &command) {
    state.gdb.server_die = true;
    return "";
}

static std::string cmd_attached(EmuEnvState &state, PacketCommand &command) { return "1"; }

static std::string cmd_thread_status(EmuEnvState &state, PacketCommand &command) { return "T0"; }

static std::string cmd_reason(EmuEnvState &state, PacketCommand &command) { return "S05"; }

// The thread list reply for thread_info_index, without the deleted threads that wait for their host
// thread to end
static std::string thread_info_reply(EmuEnvState &state) {
    const auto guard = std::lock_guard(state.kernel.mutex);
    int index = state.gdb.thread_info_index;
    for (const auto &[id, thread] : state.kernel.threads) {
        if (thread->get_status() != ThreadStatus::deleted && index-- == 0)
            return 'm' + to_hex(id);
    }
    return "l";
}

static std::string cmd_get_first_thread(EmuEnvState &state, PacketCommand &command) {
    state.gdb.thread_info_index = 0;
    return thread_info_reply(state);
}

static std::string cmd_get_next_thread(EmuEnvState &state, PacketCommand &command) {
    ++state.gdb.thread_info_index;
    return thread_info_reply(state);
}

static std::string cmd_add_breakpoint(EmuEnvState &state, PacketCommand &command) {
    const std::string content = content_string(command);

    const uint64_t first = content.find(',');
    const uint64_t second = content.find(',', first + 1);
    const uint32_t type = static_cast<uint32_t>(std::stol(content.substr(1, first - 1)));
    const uint32_t address = parse_hex(content.substr(first + 1, second - 1 - first));
    const uint32_t kind = static_cast<uint32_t>(std::stol(content.substr(second + 1, content.size() - second - 1)));

    LOG_GDB("GDB Server New Breakpoint at {} ({}, {}).", log_hex(address), type, kind);

    // kind is 2 for a 16-bit Thumb instruction, 3 for a 32-bit one and 4 for ARM. A 16-bit BKPT
    // over the first halfword stops at a 32-bit Thumb instruction too.
    // https://sourceware.org/gdb/current/onlinedocs/gdb/ARM-Breakpoint-Kinds.html#ARM-Breakpoint-Kinds
    state.kernel.debugger.add_breakpoint(state.mem, address, kind == 2 || kind == 3);

    return "OK";
}

static std::string cmd_remove_breakpoint(EmuEnvState &state, PacketCommand &command) {
    const std::string content = content_string(command);

    const uint64_t first = content.find(',');
    const uint64_t second = content.find(',', first + 1);
    const uint32_t type = static_cast<uint32_t>(std::stol(content.substr(1, first - 1)));
    const uint32_t address = parse_hex(content.substr(first + 1, second - 1 - first));
    const uint32_t kind = static_cast<uint32_t>(std::stol(content.substr(second + 1, content.size() - second - 1)));

    LOG_GDB("GDB Server Removed Breakpoint at {} ({}, {}).", log_hex(address), type, kind);
    state.kernel.debugger.remove_breakpoint(state.mem, address);

    return "OK";
}

static std::string cmd_deprecated(EmuEnvState &state, PacketCommand &command) {
    LOG_GDB("GDB Server: Deprecated Packet. {}", content_string(command));

    return "";
}

static std::string cmd_unimplemented(EmuEnvState &state, PacketCommand &command) {
    LOG_GDB("GDB Server: Unimplemented Packet. {}", content_string(command));

    return "";
}

const static PacketFunctionBundle functions[] = {
    // General
    { "!", cmd_unimplemented },
    { "?", cmd_reason },
    { "H", cmd_set_current_thread },
    { "T", cmd_thread_alive },
    { "i", cmd_unimplemented },
    { "I", cmd_unimplemented },
    { "A", cmd_unimplemented },
    { "bc", cmd_unimplemented },
    { "bs", cmd_unimplemented },
    { "t", cmd_unimplemented },

    // Read/Write
    { "p", cmd_read_register },
    { "P", cmd_write_register },
    { "g", cmd_read_registers },
    { "G", cmd_write_registers },
    { "m", cmd_read_memory },
    { "M", cmd_write_memory },
    { "X", cmd_unimplemented }, // change cmd_unimplemented to cmd_write_binary to enable binary downloading

    // Query Packets
    { "qfThreadInfo", cmd_get_first_thread },
    { "qsThreadInfo", cmd_get_next_thread },
    { "qSupported", cmd_supported },
    { "qXfer:features:read:target.xml:", cmd_target_xml },
    { "qAttached", cmd_attached },
    { "qTStatus", cmd_thread_status },
    { "qC", cmd_get_current_thread },
    { "q", cmd_unimplemented },
    { "Q", cmd_unimplemented },

    // Shutdown
    { "d", cmd_unimplemented },
    { "r", cmd_unimplemented },
    { "R", cmd_unimplemented },
    { "k", cmd_die },

    // Control Packets
    { "vCont?", cmd_continue_supported },
    { "vCont", cmd_continue },
    { "vKill", cmd_kill },
    { "vMustReplyEmpty", cmd_reply_empty },
    { "v", cmd_unimplemented },

    // Breakpoints
    { "z", cmd_remove_breakpoint },
    { "Z", cmd_add_breakpoint },

    // Deprecated
    { "b", cmd_deprecated },
    { "B", cmd_deprecated },
    { "c", cmd_deprecated },
    { "C", cmd_deprecated },
    { "s", cmd_deprecated },
    { "S", cmd_deprecated },
};

static bool command_begins_with(PacketCommand &command, const std::string_view small_str) {
    // If the command's content is shorter than small_str, it can't match
    if (static_cast<size_t>(command.content_length) < small_str.size())
        return false;

    return std::memcmp(command.content_start, small_str.data(), small_str.size()) == 0;
}

static int64_t server_next(EmuEnvState &state) {
    PacketData buffer;

    // Wait for the server to close or a packet to be received.
    fd_set readSet;
    timeval timeout = { 1, 0 };
    do {
        readSet = { 0 };
        FD_SET(state.gdb.client_socket, &readSet);
    } while (select(state.gdb.client_socket + 1, &readSet, nullptr, nullptr, &timeout) < 1 && !state.gdb.server_die);
    if (state.gdb.server_die)
        return -1;

    const int64_t length = recv(state.gdb.client_socket, buffer, sizeof(buffer), 0);
    if (length <= 0) {
        LOG_GDB("GDB Server Connection Closed");
        return -1;
    }
    state.gdb.input_buffer.append(buffer, static_cast<size_t>(length));
    auto &input = state.gdb.input_buffer;
    while (!input.empty()) {
        switch (input.front()) {
        case '+': {
            input.erase(0, 1);
            break; // Cool.
        }
        case '-': {
            input.erase(0, 1);
            LOG_GDB("GDB Server Transmission Error");
            server_reply(state.gdb, state.gdb.last_reply.c_str());
            break;
        }
        case '$': {
            const size_t end = input.find('#');
            // Wait for the rest of a packet split across reads, up to a limit
            if (end == std::string::npos || input.size() < end + 3) {
                if (input.size() > MAX_PACKET_SIZE) {
                    LOG_GDB("GDB Server Packet Too Long");
                    input.clear();
                    server_ack(state.gdb, '-');
                }
                return length;
            }
            const size_t packet_size = end + 3;
            PacketCommand command = parse_command(input.data(), static_cast<int64_t>(packet_size));
            if (command.is_valid) {
                server_ack(state.gdb, '+');
                bool found_command = false;
                for (const auto &function : functions) {
                    if (command_begins_with(command, function.name)) {
                        found_command = true;
                        LOG_GDB("GDB Server Recognized Command as {}. {}", function.name,
                            std::string(command.content_start, command.content_length));
                        state.gdb.last_reply = function.function(state, command);
                        if (state.gdb.server_die)
                            break;
                        server_reply(state.gdb, state.gdb.last_reply.c_str());
                        break;
                    }
                }
                if (!found_command) {
                    LOG_GDB("GDB Server Unrecognized Command. {}", std::string(command.content_start, command.content_length));
                    state.gdb.last_reply = "";
                    server_reply(state.gdb, state.gdb.last_reply.c_str());
                }
            } else {
                server_ack(state.gdb, '-');
                LOG_GDB("GDB Server Invalid Command. {}", input.substr(0, packet_size));
            }
            input.erase(0, packet_size);
            break;
        }
        default:
            input.erase(0, 1);
            break;
        }
        if (state.gdb.server_die)
            break;
    }

    return length;
}

static void server_listen(EmuEnvState &state) {
    state.gdb.client_socket = accept(state.gdb.listen_socket, nullptr, nullptr);
    state.gdb.input_buffer.clear();

    if (state.gdb.client_socket == -1) {
        LOG_GDB("GDB Server Failed: Could not accept socket.");
        return;
    }

    LOG_INFO("GDB Server Received Connection");

    // A reply goes out at once, without waiting for the ack of the '+' sent before it
    const int nodelay = 1;
    setsockopt(state.gdb.client_socket, IPPROTO_TCP, TCP_NODELAY, reinterpret_cast<const char *>(&nodelay), sizeof(nodelay));

    int64_t status;

    do {
        status = server_next(state);
    } while (status >= 0 && !state.gdb.server_die);

    server_close(state);
}

void server_open(EmuEnvState &state) {
    LOG_GDB("Starting GDB Server...");

#ifdef _WIN32
    int32_t err = WSAStartup(MAKEWORD(2, 2), &state.gdb.wsaData);
    if (err) {
        LOG_ERROR("GDB Server Failed: Could not start WSA service.");
        return;
    }
#endif

    state.gdb.listen_socket = socket(AF_INET, SOCK_STREAM, 0);
    if (state.gdb.listen_socket == -1) {
        LOG_ERROR("GDB Server Failed: Could not create socket.");
        return;
    }

#ifndef _WIN32
    // An earlier session's connection keeps the port in TIME_WAIT for a while after it ends
    const int reuse = 1;
    setsockopt(state.gdb.listen_socket, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));
#endif

    sockaddr_in socket_address{};
    socket_address.sin_family = AF_INET;
    socket_address.sin_port = htons(GDB_SERVER_PORT);
#ifdef _WIN32
    socket_address.sin_addr.S_un.S_addr = htonl(INADDR_ANY);
#else
    socket_address.sin_addr.s_addr = htonl(INADDR_ANY);
#endif

    if (bind(state.gdb.listen_socket, (sockaddr *)&socket_address, sizeof(socket_address)) == -1) {
        LOG_ERROR("GDB Server Failed: Could not bind socket.");
        return;
    }

    if (listen(state.gdb.listen_socket, 1) == -1) {
        LOG_ERROR("GDB Server Failed: Could not listen on socket.");
        return;
    }

    state.gdb.server_thread = std::make_shared<std::thread>(server_listen, std::ref(state));

    LOG_INFO("GDB Server is listening on port {}", GDB_SERVER_PORT);
}

void server_close(EmuEnvState &state) {
#ifdef _WIN32
    closesocket(state.gdb.listen_socket);
    WSACleanup();
#else
    close(state.gdb.listen_socket);
#endif

    state.gdb.server_die = true;
    if (state.gdb.server_thread && state.gdb.server_thread->get_id() != std::this_thread::get_id())
        state.gdb.server_thread->join();
}
