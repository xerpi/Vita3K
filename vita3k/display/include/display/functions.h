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

#include <kernel/thread/thread_state.h>

#include <cstdint>

struct DisplayState;
struct EmuEnvState;
struct DisplayFrameInfo;

void start_sync_thread(EmuEnvState &emuenv);
// Blocks the thread until the vblank count reaches the target count
void wait_vblank(DisplayState &display, const ThreadStatePtr &wait_thread, uint64_t target_vcount, bool is_cb);
// Blocks the thread for a number of vblanks. If since_setbuf is set they count from the last
// sceDisplaySetFrameBuf, otherwise from the vblank where the thread's last display wait ended.
void wait_display_vblank(DisplayState &display, const ThreadStatePtr &wait_thread, int vcount, bool since_setbuf, bool is_cb);
// if the result is not nullptr, contain the predicted frame (pointer needs to be freed later)
DisplayFrameInfo *predict_next_image(EmuEnvState &emuenv, Address sync_object);
void update_prediction(EmuEnvState &emuenv, DisplayFrameInfo &frame);
