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

#include "SceThreadmgr.h"
#include <modules/module_parent.h>

#include <kernel/callback.h>
#include <kernel/state.h>
#include <kernel/sync_primitives.h>
#include <kernel/types.h>
#include <packages/functions.h>

#include <util/lock_and_find.h>

#include <chrono>
#include <thread>

#include <util/tracy.h>
TRACY_MODULE_NAME(SceThreadmgr);

inline static uint64_t get_current_time() {
    return std::chrono::duration_cast<std::chrono::microseconds>(
        std::chrono::high_resolution_clock::now().time_since_epoch())
        .count();
}

// Small helpers used by HLE wrappers to instantiate sync objects.
namespace {

template <typename T, typename Map, typename... Args>
SceUID create_sync_object(KernelState &kernel, Map &map, Args &&...args) {
    const SceUID uid = kernel.get_next_uid();
    auto obj = std::make_shared<T>(uid, std::forward<Args>(args)...);
    std::lock_guard<std::mutex> lock(kernel.mutex);
    map.emplace(uid, std::move(obj));
    return uid;
}

template <typename T, typename Map>
SceUID find_sync_object_by_name(KernelState &kernel, Map &map, const char *name) {
    std::lock_guard<std::mutex> lock(kernel.mutex);
    for (const auto &[uid, obj] : map) {
        if (std::strcmp(obj->name, name) == 0)
            return uid;
    }
    return SCE_KERNEL_ERROR_UID_CANNOT_FIND_BY_NAME;
}

template <typename T, typename Map>
SceInt32 delete_sync_object(KernelState &kernel, Map &map, SceUID uid, SceInt32 unknown_err) {
    auto obj = lock_and_find(uid, map, kernel.mutex);
    if (!obj)
        return unknown_err;
    obj->mark_deleted();
    std::lock_guard<std::mutex> lock(kernel.mutex);
    map.erase(uid);
    return SCE_KERNEL_OK;
}

} // namespace

EXPORT(SceInt32, __sceKernelCreateLwMutex, Ptr<SceKernelLwMutexWork> workarea, const char *name, SceUInt32 attr, Ptr<SceKernelCreateLwMutex_opt> opt) {
    TRACY_FUNC(__sceKernelCreateLwMutex, workarea, name, attr, opt);
    if (!name)
        return RET_ERROR(SCE_KERNEL_ERROR_INVALID_ARGUMENT);
    const auto *opts = opt.get(emuenv.mem);
    const SceInt32 init_count = opts->init_count;
    const bool recursive = (attr & SCE_KERNEL_MUTEX_ATTR_RECURSIVE) != 0;
    if (init_count < 0 || (!recursive && init_count > 1))
        return RET_ERROR(SCE_KERNEL_ERROR_INVALID_ARGUMENT);
    if ((attr & SCE_KERNEL_ATTR_OPENABLE) && std::strlen(name) > SCE_UID_NAMELEN)
        return RET_ERROR(SCE_KERNEL_ERROR_UID_NAME_TOO_LONG);

    const ThreadStatePtr initial_owner = emuenv.kernel.get_thread(thread_id);
    const SceUID uid = create_sync_object<LwMutex>(emuenv.kernel, emuenv.kernel.lwmutexes,
        name, attr, init_count, initial_owner, workarea);
    auto *wa = workarea.get(emuenv.mem);
    wa->owner = (init_count > 0 && initial_owner) ? initial_owner->id : 0;
    wa->uid = uid;
    wa->attr = attr;
    wa->lockCount = init_count;
    return SCE_KERNEL_OK;
}

EXPORT(SceInt32, _sceKernelCancelEvent, SceUID event_id, SceUInt32 *pNumWaitThreads) {
    TRACY_FUNC(_sceKernelCancelEvent, event_id, pNumWaitThreads);
    auto ev = lock_and_find(event_id, emuenv.kernel.simple_events, emuenv.kernel.mutex);
    if (!ev)
        return RET_ERROR(SCE_KERNEL_ERROR_UNKNOWN_EVENT_ID);
    const SceUInt32 n = ev->cancel();
    if (pNumWaitThreads)
        *pNumWaitThreads = n;
    return SCE_KERNEL_OK;
}

EXPORT(SceInt32, _sceKernelCancelEventFlag, SceUID event_id, SceUInt pattern, SceUInt32 *num_wait_thread) {
    TRACY_FUNC(_sceKernelCancelEventFlag, event_id, pattern, num_wait_thread);
    auto ef = lock_and_find(event_id, emuenv.kernel.eventflags, emuenv.kernel.mutex);
    if (!ef)
        return RET_ERROR(SCE_KERNEL_ERROR_UNKNOWN_EVF_ID);
    const SceUInt32 n = ef->cancel(pattern);
    if (num_wait_thread)
        *num_wait_thread = n;
    return SCE_KERNEL_OK;
}

EXPORT(int, _sceKernelCancelEventWithSetPattern) {
    TRACY_FUNC(_sceKernelCancelEventWithSetPattern);
    return UNIMPLEMENTED();
}

EXPORT(SceInt32, _sceKernelCancelMsgPipe, SceUID msgPipeId, SceUInt32 *pNumSendWaitThreads, SceUInt32 *pNumReceiveWaitThreads) {
    TRACY_FUNC(_sceKernelCancelMsgPipe, msgPipeId, pNumSendWaitThreads, pNumReceiveWaitThreads);
    auto pipe = lock_and_find(msgPipeId, emuenv.kernel.msgpipes, emuenv.kernel.mutex);
    if (!pipe)
        return RET_ERROR(SCE_KERNEL_ERROR_UNKNOWN_MSG_PIPE_ID);
    const auto counts = pipe->cancel();
    if (pNumSendWaitThreads)
        *pNumSendWaitThreads = counts.send;
    if (pNumReceiveWaitThreads)
        *pNumReceiveWaitThreads = counts.recv;
    return SCE_KERNEL_OK;
}

EXPORT(SceInt32, _sceKernelCancelMutex, SceUID mutexId, SceInt32 newCount, SceUInt32 *pNumWaitThreads) {
    TRACY_FUNC(_sceKernelCancelMutex, mutexId, newCount, pNumWaitThreads);
    if (newCount < -1)
        return RET_ERROR(SCE_KERNEL_ERROR_INVALID_ARGUMENT);
    const MutexPtr mutex = lock_and_find(mutexId, emuenv.kernel.mutexes, emuenv.kernel.mutex);
    if (!mutex)
        return RET_ERROR(SCE_KERNEL_ERROR_UNKNOWN_MUTEX_ID);
    const SceUInt32 n = mutex->cancel(emuenv.kernel.get_thread(thread_id), newCount);
    if (pNumWaitThreads)
        *pNumWaitThreads = n;
    return SCE_KERNEL_OK;
}

EXPORT(SceInt32, _sceKernelCancelRWLock, SceUID rwLockId, SceUInt32 *pNumReadWaitThreads, SceUInt32 *pNumWriteWaitThreads, SceInt32 flag) {
    TRACY_FUNC(_sceKernelCancelRWLock, rwLockId, pNumReadWaitThreads, pNumWriteWaitThreads, flag);
    auto rwlock = lock_and_find(rwLockId, emuenv.kernel.rwlocks, emuenv.kernel.mutex);
    if (!rwlock)
        return RET_ERROR(SCE_KERNEL_ERROR_UNKNOWN_RW_LOCK_ID);
    const auto counts = rwlock->cancel(emuenv.kernel.get_thread(thread_id), flag);
    if (pNumReadWaitThreads)
        *pNumReadWaitThreads = counts.read;
    if (pNumWriteWaitThreads)
        *pNumWriteWaitThreads = counts.write;
    return SCE_KERNEL_OK;
}

EXPORT(int, _sceKernelCancelSema, SceUID semaId, SceInt32 setCount, SceUInt32 *pNumWaitThreads) {
    TRACY_FUNC(_sceKernelCancelSema, semaId, setCount, pNumWaitThreads);
    auto s = lock_and_find(semaId, emuenv.kernel.semaphores, emuenv.kernel.mutex);
    if (!s)
        return RET_ERROR(SCE_KERNEL_ERROR_UNKNOWN_SEMA_ID);
    const SceUInt32 n = s->cancel(setCount);
    if (pNumWaitThreads)
        *pNumWaitThreads = n;
    return SCE_KERNEL_OK;
}

EXPORT(int, _sceKernelCancelTimer) {
    TRACY_FUNC(_sceKernelCancelTimer);
    return UNIMPLEMENTED();
}

EXPORT(SceUID, _sceKernelCreateCond, const char *pName, SceUInt32 attr, SceUID mutexId, const SceKernelCondOptParam *pOptParam) {
    TRACY_FUNC(_sceKernelCreateCond, pName, attr, mutexId, pOptParam);
    if ((attr & SCE_KERNEL_ATTR_OPENABLE) && std::strlen(pName) > SCE_UID_NAMELEN)
        return RET_ERROR(SCE_KERNEL_ERROR_UID_NAME_TOO_LONG);
    auto mu = lock_and_find(mutexId, emuenv.kernel.mutexes, emuenv.kernel.mutex);
    if (!mu)
        return RET_ERROR(SCE_KERNEL_ERROR_UNKNOWN_MUTEX_ID);
    return create_sync_object<Condvar>(emuenv.kernel, emuenv.kernel.condvars,
        pName, attr, mu);
}

EXPORT(SceUID, _sceKernelCreateEventFlag, const char *pName, SceUInt32 attr, SceUInt32 initPattern, const SceKernelEventFlagOptParam *pOptParam) {
    TRACY_FUNC(_sceKernelCreateEventFlag, pName, attr, initPattern, pOptParam);
    if ((attr & SCE_KERNEL_ATTR_OPENABLE) && std::strlen(pName) > SCE_UID_NAMELEN)
        return RET_ERROR(SCE_KERNEL_ERROR_UID_NAME_TOO_LONG);
    return create_sync_object<EventFlag>(emuenv.kernel, emuenv.kernel.eventflags,
        pName, attr, initPattern);
}

EXPORT(int, _sceKernelCreateLwCond, Ptr<SceKernelLwCondWork> workarea, const char *name, SceUInt attr, Ptr<SceKernelCreateLwCond_opt> opt) {
    TRACY_FUNC(_sceKernelCreateLwCond, workarea, name, attr, opt);
    if ((attr & SCE_KERNEL_ATTR_OPENABLE) && std::strlen(name) > SCE_UID_NAMELEN)
        return RET_ERROR(SCE_KERNEL_ERROR_UID_NAME_TOO_LONG);
    const auto assoc_mutex_uid = opt.get(emuenv.mem)->workarea_mutex.get(emuenv.mem)->uid;
    auto mu = lock_and_find(assoc_mutex_uid, emuenv.kernel.lwmutexes, emuenv.kernel.mutex);
    if (!mu)
        return RET_ERROR(SCE_KERNEL_ERROR_UNKNOWN_LW_MUTEX_ID);
    const SceUID uid = create_sync_object<LwCondVar>(emuenv.kernel, emuenv.kernel.lwcondvars,
        name, attr, mu);
    workarea.get(emuenv.mem)->uid = uid;
    return SCE_KERNEL_OK;
}

EXPORT(int, _sceKernelCreateMsgPipeWithLR) {
    TRACY_FUNC(_sceKernelCreateMsgPipeWithLR);
    return UNIMPLEMENTED();
}

EXPORT(int, _sceKernelCreateMutex, const char *name, SceUInt attr, int init_count, SceKernelMutexOptParam *opt_param) {
    TRACY_FUNC(_sceKernelCreateMutex, name, attr, init_count, opt_param);
    if ((attr & SCE_KERNEL_ATTR_OPENABLE) && std::strlen(name) > SCE_UID_NAMELEN)
        return RET_ERROR(SCE_KERNEL_ERROR_UID_NAME_TOO_LONG);
    return create_sync_object<Mutex>(emuenv.kernel, emuenv.kernel.mutexes,
        name, attr, init_count, emuenv.kernel.get_thread(thread_id));
}

EXPORT(SceUID, _sceKernelCreateRWLock, const char *name, SceUInt32 attr, SceKernelMutexOptParam *opt_param) {
    TRACY_FUNC(_sceKernelCreateRWLock, name, attr, opt_param);
    if ((attr & SCE_KERNEL_ATTR_OPENABLE) && std::strlen(name) > SCE_UID_NAMELEN)
        return RET_ERROR(SCE_KERNEL_ERROR_UID_NAME_TOO_LONG);
    return create_sync_object<RWLock>(emuenv.kernel, emuenv.kernel.rwlocks,
        name, attr);
}

EXPORT(int, _sceKernelCreateSema, const char *name, SceUInt attr, int initVal, Ptr<SceKernelCreateSema_opt> opt) {
    TRACY_FUNC(_sceKernelCreateSema, name, attr, initVal, opt);
    if ((attr & SCE_KERNEL_ATTR_OPENABLE) && std::strlen(name) > SCE_UID_NAMELEN)
        return RET_ERROR(SCE_KERNEL_ERROR_UID_NAME_TOO_LONG);
    return create_sync_object<Semaphore>(emuenv.kernel, emuenv.kernel.semaphores,
        name, attr, initVal, opt.get(emuenv.mem)->maxVal);
}

EXPORT(int, _sceKernelCreateSema_16XX, const char *name, SceUInt attr, int initVal, Ptr<SceKernelCreateSema_opt> opt) {
    TRACY_FUNC(_sceKernelCreateSema_16XX, name, attr, initVal, opt);
    if ((attr & SCE_KERNEL_ATTR_OPENABLE) && std::strlen(name) > SCE_UID_NAMELEN)
        return RET_ERROR(SCE_KERNEL_ERROR_UID_NAME_TOO_LONG);
    return create_sync_object<Semaphore>(emuenv.kernel, emuenv.kernel.semaphores,
        name, attr, initVal, opt.get(emuenv.mem)->maxVal);
}

EXPORT(SceUID, _sceKernelCreateSimpleEvent, const char *name, SceUInt32 attr, SceUInt32 init_pattern, const SceKernelSimpleEventOptParam *pOptParam) {
    TRACY_FUNC(_sceKernelCreateSimpleEvent, name, attr, init_pattern, pOptParam);
    if ((attr & SCE_KERNEL_ATTR_OPENABLE) && std::strlen(name) > SCE_UID_NAMELEN)
        return RET_ERROR(SCE_KERNEL_ERROR_UID_NAME_TOO_LONG);
    return create_sync_object<SimpleEvent>(emuenv.kernel, emuenv.kernel.simple_events,
        name, attr, init_pattern);
}

EXPORT(int, _sceKernelCreateTimer, const char *name, SceUInt32 attr, const uint32_t *opt_params) {
    TRACY_FUNC(_sceKernelCreateTimer, name, attr, opt_params);
    if ((attr & SCE_KERNEL_ATTR_OPENABLE) && std::strlen(name) > SCE_UID_NAMELEN)
        return RET_ERROR(SCE_KERNEL_ERROR_UID_NAME_TOO_LONG);
    return create_sync_object<Timer>(emuenv.kernel, emuenv.kernel.timers,
        name, attr);
}

EXPORT(int, _sceKernelDeleteLwCond, Ptr<SceKernelLwCondWork> workarea) {
    TRACY_FUNC(_sceKernelDeleteLwCond, workarea);
    return delete_sync_object<LwCondVar>(emuenv.kernel, emuenv.kernel.lwcondvars,
        workarea.get(emuenv.mem)->uid, SCE_KERNEL_ERROR_UNKNOWN_LW_COND_ID);
}

EXPORT(int, _sceKernelDeleteLwMutex, Ptr<SceKernelLwMutexWork> workarea) {
    TRACY_FUNC(_sceKernelDeleteLwMutex, workarea);
    if (!workarea)
        return SCE_KERNEL_ERROR_ILLEGAL_ADDR;
    return delete_sync_object<LwMutex>(emuenv.kernel, emuenv.kernel.lwmutexes,
        workarea.get(emuenv.mem)->uid, SCE_KERNEL_ERROR_UNKNOWN_LW_MUTEX_ID);
}

EXPORT(int, _sceKernelExitCallback) {
    TRACY_FUNC(_sceKernelExitCallback);
    return UNIMPLEMENTED();
}

EXPORT(SceInt32, _sceKernelGetCallbackInfo, SceUID callbackId, SceKernelCallbackInfo *pInfo) {
    TRACY_FUNC(_sceKernelGetCallbackInfo, callbackId, pInfo);
    const CallbackPtr cb = lock_and_find(callbackId, emuenv.kernel.callbacks, emuenv.kernel.mutex);

    if (!cb)
        return RET_ERROR(SCE_KERNEL_ERROR_UNKNOWN_CALLBACK_ID);

    if (!pInfo)
        return RET_ERROR(SCE_KERNEL_ERROR_ILLEGAL_ADDR); // TODO check result

    if (pInfo->size != sizeof(*pInfo))
        return RET_ERROR(SCE_KERNEL_ERROR_INVALID_ARGUMENT_SIZE);

    pInfo->callbackId = callbackId;
    std::strcpy(pInfo->name, cb->get_name());
    pInfo->attr = 0;
    pInfo->threadId = cb->get_owner_thread_id();
    pInfo->callbackFunc = cb->get_callback_function();
    const Callback::Info snap = cb->info();
    pInfo->notifyId = snap.notifier_id;
    pInfo->notifyCount = static_cast<SceInt32>(snap.num_notifications);
    pInfo->notifyArg = snap.notify_arg;
    pInfo->pCommon = cb->get_user_common_ptr();

    return SCE_KERNEL_OK;
}

EXPORT(SceInt32, _sceKernelGetCondInfo, SceUID condId, Ptr<SceKernelCondInfo> pInfo) {
    TRACY_FUNC(_sceKernelGetCondInfo, condId, pInfo);
    const CondvarPtr condvar = lock_and_find(condId, emuenv.kernel.condvars, emuenv.kernel.mutex);
    if (!condvar)
        return RET_ERROR(SCE_KERNEL_ERROR_UNKNOWN_EVF_ID);

    SceKernelCondInfo *info = pInfo.get(emuenv.mem);
    if (!info)
        return RET_ERROR(SCE_KERNEL_ERROR_ILLEGAL_ADDR);

    if (info->size != sizeof(*info))
        return RET_ERROR(SCE_KERNEL_ERROR_INVALID_ARGUMENT_SIZE);

    info->condId = condId;
    std::strcpy(info->name, condvar->name);
    info->attr = condvar->attr;
    info->mutexId = condvar->mutex_obj()->uid;
    info->numWaitThreads = condvar->num_waiters();

    return SCE_KERNEL_OK;
}

EXPORT(SceInt32, _sceKernelGetEventFlagInfo, SceUID evfId, Ptr<SceKernelEventFlagInfo> pInfo) {
    TRACY_FUNC(_sceKernelGetEventFlagInfo, evfId, pInfo);
    const EventFlagPtr eventflag = lock_and_find(evfId, emuenv.kernel.eventflags, emuenv.kernel.mutex);
    if (!eventflag)
        return RET_ERROR(SCE_KERNEL_ERROR_UNKNOWN_EVF_ID);

    SceKernelEventFlagInfo *info = pInfo.get(emuenv.mem);
    if (!info)
        return RET_ERROR(SCE_KERNEL_ERROR_ILLEGAL_ADDR);

    if (info->size != sizeof(*info))
        return RET_ERROR(SCE_KERNEL_ERROR_INVALID_ARGUMENT_SIZE);

    info->evfId = evfId;
    std::strcpy(info->name, eventflag->name);
    info->attr = eventflag->attr;
    info->initPattern = eventflag->current_pattern(); // Todo, give only current pattern
    info->currentPattern = eventflag->current_pattern();
    info->numWaitThreads = eventflag->num_waiters();

    return SCE_KERNEL_OK;
}

EXPORT(int, _sceKernelGetEventInfo) {
    TRACY_FUNC(_sceKernelGetEventInfo);
    return UNIMPLEMENTED();
}

EXPORT(SceInt32, _sceKernelGetEventPattern, SceUID event_id, SceUInt32 *get_pattern) {
    TRACY_FUNC(_sceKernelGetEventPattern, event_id, get_pattern);
    const SimpleEventPtr event = lock_and_find(event_id, emuenv.kernel.simple_events, emuenv.kernel.mutex);
    if (!event)
        return RET_ERROR(SCE_KERNEL_ERROR_UNKNOWN_EVENT_ID);
    if (!get_pattern)
        return RET_ERROR(SCE_KERNEL_ERROR_ILLEGAL_ADDR);

    *get_pattern = event->current_pattern();
    return SCE_KERNEL_OK;
}

EXPORT(int, _sceKernelGetLwCondInfo) {
    TRACY_FUNC(_sceKernelGetLwCondInfo);
    return UNIMPLEMENTED();
}

EXPORT(int, _sceKernelGetLwCondInfoById) {
    TRACY_FUNC(_sceKernelGetLwCondInfoById);
    return UNIMPLEMENTED();
}

EXPORT(int, _sceKernelGetLwMutexInfoById, SceUID lightweight_mutex_id, Ptr<SceKernelLwMutexInfo> info, SceSize size) {
    TRACY_FUNC(_sceKernelGetLwMutexInfoById, lightweight_mutex_id, info, size);
    SceKernelLwMutexInfo *info_data = info.get(emuenv.mem);
    SceSize info_size = info_data->size;
    SceKernelLwMutexInfo info_data_local;
    if (info_size < sizeof(SceKernelLwMutexInfo)) {
        info_data = &info_data_local;
        info_data_local.size = info_size;
    }
    LwMutexPtr mutex = lock_and_find(lightweight_mutex_id, emuenv.kernel.lwmutexes, emuenv.kernel.mutex);
    if (!mutex)
        return RET_ERROR(SCE_KERNEL_ERROR_UNKNOWN_LW_MUTEX_ID);

    info_data->uid = lightweight_mutex_id;
    std::strcpy(info_data->name, mutex->name);
    info_data->attr = mutex->attr;
    info_data->pWork = mutex->get_workarea();
    info_data->initCount = mutex->initial_count();
    info_data->currentCount = mutex->current_lock_count();
    info_data->currentOwnerId = mutex->current_owner_id();
    info_data->numWaitThreads = static_cast<SceUInt32>(mutex->num_waiters());
    if (info_size < sizeof(SceKernelLwMutexInfo))
        memcpy(info.get(emuenv.mem), &info_data_local, info_size);
    else
        info_data->size = sizeof(SceKernelLwMutexInfo);
    return SCE_KERNEL_OK;
}

EXPORT(int, _sceKernelGetMsgPipeInfo) {
    TRACY_FUNC(_sceKernelGetMsgPipeInfo);
    return UNIMPLEMENTED();
}

EXPORT(int, _sceKernelGetMutexInfo, SceUID mutexId, SceKernelMutexInfo *pInfo) {
    TRACY_FUNC(_sceKernelGetMutexInfo, mutexId, pInfo);
    if (!pInfo)
        return RET_ERROR(SCE_KERNEL_ERROR_INVALID_ARGUMENT);
    SceKernelMutexInfo *info_data = pInfo;
    SceSize info_size = info_data->size;
    SceKernelMutexInfo info_data_local;
    if (info_size < sizeof(*pInfo)) {
        info_data = &info_data_local;
        info_data_local.size = info_size;
    }
    const MutexPtr mutex = lock_and_find(mutexId, emuenv.kernel.mutexes, emuenv.kernel.mutex);
    if (!mutex)
        return RET_ERROR(SCE_KERNEL_ERROR_UNKNOWN_MUTEX_ID);
    info_data->mutexId = mutexId;
    std::strcpy(info_data->name, mutex->name);
    info_data->attr = mutex->attr;
    info_data->initCount = mutex->initial_count();
    info_data->currentCount = mutex->current_lock_count();
    if (mutex->current_owner_id() != 0) {
        info_data->currentOwnerId = mutex->current_owner_id();
    } else {
        info_data->currentOwnerId = 0;
    }
    info_data->numWaitThreads = mutex->num_waiters();
    if (info_size < sizeof(*pInfo)) {
        memcpy(pInfo, &info_data_local, info_size);
    } else {
        info_data->size = sizeof(*pInfo);
    }
    return SCE_KERNEL_OK;
}

EXPORT(int, _sceKernelGetRWLockInfo, SceUID rwlockId, SceKernelRWLockInfo *info) {
    TRACY_FUNC(_sceKernelGetRWLockInfo, rwlockId, info);
    if (!info)
        return RET_ERROR(SCE_KERNEL_ERROR_INVALID_ARGUMENT);
    if (info->size < sizeof(SceKernelRWLockInfo))
        return RET_ERROR(SCE_KERNEL_ERROR_INVALID_ARGUMENT);
    const RWLockPtr rwlock = lock_and_find(rwlockId, emuenv.kernel.rwlocks, emuenv.kernel.mutex);
    if (!rwlock)
        return RET_ERROR(SCE_KERNEL_ERROR_UNKNOWN_RW_LOCK_ID);
    const std::lock_guard<std::mutex> rwlock_lock(rwlock->mutex);
    info->rwLockId = rwlock->uid;
    std::strcpy(info->name, rwlock->name);
    info->attr = rwlock->attr;
    info->lockCount = 0;
    info->writeOwnerId = 0;
    info->numReadWaitThreads = 0;
    info->numWriteWaitThreads = 0;
    return UNIMPLEMENTED();
}

EXPORT(SceInt32, _sceKernelGetSemaInfo, SceUID semaId, Ptr<SceKernelSemaInfo> pInfo) {
    TRACY_FUNC(_sceKernelGetSemaInfo, semaId, pInfo);
    const SemaphorePtr semaphore = lock_and_find(semaId, emuenv.kernel.semaphores, emuenv.kernel.mutex);
    if (!semaphore)
        return RET_ERROR(SCE_KERNEL_ERROR_UNKNOWN_SEMA_ID);

    SceKernelSemaInfo *info = pInfo.get(emuenv.mem);
    if (!info)
        return RET_ERROR(SCE_KERNEL_ERROR_ILLEGAL_ADDR);

    if (info->size != sizeof(*info))
        return RET_ERROR(SCE_KERNEL_ERROR_INVALID_ARGUMENT_SIZE);

    info->attr = semaphore->attr;
    info->currentCount = semaphore->current_value();
    info->initCount = semaphore->initial_value();
    info->maxCount = semaphore->max_value();
    std::strcpy(info->name, semaphore->name);
    info->semaId = semaId;
    info->numWaitThreads = semaphore->num_waiters();

    return SCE_KERNEL_OK;
}

EXPORT(int, _sceKernelGetSystemInfo) {
    TRACY_FUNC(_sceKernelGetSystemInfo);
    return UNIMPLEMENTED();
}

EXPORT(int, _sceKernelGetSystemTime) {
    TRACY_FUNC(_sceKernelGetSystemTime);
    return UNIMPLEMENTED();
}

EXPORT(int, _sceKernelGetThreadContextForVM, SceUID threadId, Ptr<SceKernelThreadCpuRegisterInfo> pCpuRegisterInfo, Ptr<SceKernelThreadVfpRegisterInfo> pVfpRegisterInfo) {
    TRACY_FUNC(_sceKernelGetThreadContextForVM, threadId, pCpuRegisterInfo, pVfpRegisterInfo);
    STUBBED("Stub");

    const ThreadStatePtr thread = emuenv.kernel.get_thread(threadId);
    if (!thread)
        return RET_ERROR(SCE_KERNEL_ERROR_UNKNOWN_THREAD_ID);

    const auto context = save_context(*thread->cpu);
    SceKernelThreadCpuRegisterInfo *infoCpu = pCpuRegisterInfo.get(emuenv.mem);
    if (infoCpu) {
        if (infoCpu->size != sizeof(*infoCpu))
            return RET_ERROR(SCE_KERNEL_ERROR_INVALID_ARGUMENT_SIZE);

        infoCpu->cpsr = context.cpsr;
        memcpy(infoCpu->reg, context.cpu_registers.data(), 16 * 4);
        infoCpu->sb = 100000; // Todo
        infoCpu->st = 100000; // Todo
        infoCpu->teehbr = 100000; // Todo
        infoCpu->tpidrurw = read_tpidruro(*thread->cpu);
    }

    SceKernelThreadVfpRegisterInfo *infoVfp = pVfpRegisterInfo.get(emuenv.mem);
    if (infoVfp) {
        if (infoVfp->size != sizeof(*infoVfp))
            return RET_ERROR(SCE_KERNEL_ERROR_INVALID_ARGUMENT_SIZE);

        infoVfp->fpscr = context.fpscr;
        memcpy(infoVfp->reg, context.fpu_registers.data(), 64 * 4);
    }

    return SCE_KERNEL_OK;
}

EXPORT(SceInt32, _sceKernelGetThreadCpuAffinityMask, SceUID thid) {
    TRACY_FUNC(_sceKernelGetThreadCpuAffinityMask, thid);
    const ThreadStatePtr thread = emuenv.kernel.get_thread(thid ? thid : thread_id);

    if (!thread)
        return RET_ERROR(SCE_KERNEL_ERROR_UNKNOWN_THREAD_ID);
    if (thread->affinity_mask == 0)
        return SCE_KERNEL_CPU_MASK_USER_ALL;

    return thread->affinity_mask;
}

EXPORT(int, _sceKernelGetThreadEventInfo) {
    TRACY_FUNC(_sceKernelGetThreadEventInfo);
    return UNIMPLEMENTED();
}

EXPORT(int, _sceKernelGetThreadExitStatus, SceUID thid, SceInt32 *pExitStatus) {
    TRACY_FUNC(_sceKernelGetThreadExitStatus, thid, pExitStatus);
    const ThreadStatePtr thread = emuenv.kernel.get_thread(thid ? thid : thread_id);
    if (!thread) {
        return SCE_KERNEL_ERROR_UNKNOWN_THREAD_ID;
    }
    if (thread->status != ThreadStatus::dormant) {
        return SCE_KERNEL_ERROR_NOT_DORMANT;
    }
    if (pExitStatus) {
        *pExitStatus = thread->returned_value;
    }
    return 0;
}

EXPORT(SceInt32, _sceKernelGetThreadInfo, SceUID threadId, Ptr<SceKernelThreadInfo> pInfo) {
    TRACY_FUNC(_sceKernelGetThreadInfo, threadId, pInfo);
    STUBBED("STUB");

    const ThreadStatePtr thread = emuenv.kernel.get_thread(threadId ? threadId : thread_id);
    if (!thread)
        return RET_ERROR(SCE_KERNEL_ERROR_UNKNOWN_THREAD_ID);

    SceKernelThreadInfo *info = pInfo.get(emuenv.mem);
    if (!info)
        return RET_ERROR(SCE_KERNEL_ERROR_ILLEGAL_ADDR);

    if (info->size != sizeof(*info))
        return RET_ERROR(SCE_KERNEL_ERROR_INVALID_ARGUMENT_SIZE);

    // TODO: SCE_KERNEL_ERROR_ILLEGAL_CONTEXT check

    {
        std::lock_guard lock(thread->mutex);
        std::strcpy(info->name, thread->name);
        info->stack = Ptr<void>(thread->stack.get());
        info->stackSize = thread->stack_size;
        info->initPriority = thread->priority; // Todo Give only current priority
        info->currentPriority = thread->priority;
        info->initCpuAffinityMask = thread->affinity_mask; // Todo Give init affinity
        info->currentCpuAffinityMask = thread->affinity_mask;
        info->entry = SceKernelThreadEntry(thread->entry_point);
        info->status = thread->vita_status_locked();
        const bool waiting = thread->status == ThreadStatus::waiting;
        info->waitType = waiting ? thread->wait_info.type : 0;
        info->waitId = waiting ? thread->wait_info.uid : 0;
        if (thread->status == ThreadStatus::dormant) {
            info->exitStatus = thread->returned_value;
        }
        info->runClocks = rtc_get_ticks(emuenv.kernel.base_tick.tick) - thread->start_tick;
    }
    return SCE_KERNEL_OK;
}

EXPORT(int, _sceKernelGetThreadRunStatus) {
    TRACY_FUNC(_sceKernelGetThreadRunStatus);
    return UNIMPLEMENTED();
}

EXPORT(int, _sceKernelGetTimerBase) {
    TRACY_FUNC(_sceKernelGetTimerBase);
    return UNIMPLEMENTED();
}

EXPORT(int, _sceKernelGetTimerEventRemainingTime) {
    TRACY_FUNC(_sceKernelGetTimerEventRemainingTime);
    return UNIMPLEMENTED();
}

EXPORT(int, _sceKernelGetTimerInfo) {
    TRACY_FUNC(_sceKernelGetTimerInfo);
    return UNIMPLEMENTED();
}

EXPORT(int, _sceKernelGetTimerTime) {
    TRACY_FUNC(_sceKernelGetTimerTime);
    return UNIMPLEMENTED();
}

static SceInt32 lock_lw_mutex(EmuEnvState &emuenv, SceUID thread_id, const char *export_name,
    Ptr<SceKernelLwMutexWork> workarea, int lock_count, SceUInt32 *timeout, bool cb) {
    if (!workarea)
        return RET_ERROR(SCE_KERNEL_ERROR_INVALID_ARGUMENT);

    const auto lwmutexid = workarea.get(emuenv.mem)->uid;
    auto mutex = lock_and_find(lwmutexid, emuenv.kernel.lwmutexes, emuenv.kernel.mutex);
    if (!mutex)
        return RET_ERROR(SCE_KERNEL_ERROR_UNKNOWN_LW_MUTEX_ID);
    const Deadline deadline = deadline_from(timeout);
    return unwrap_or_bail(mutex->lock(emuenv.kernel.get_thread(thread_id), lock_count, deadline, emuenv.mem, cb), timeout, deadline);
}

EXPORT(int, _sceKernelLockLwMutex, Ptr<SceKernelLwMutexWork> workarea, int lock_count, unsigned int *ptimeout) {
    TRACY_FUNC(_sceKernelLockLwMutex, workarea, lock_count, ptimeout);
    return lock_lw_mutex(emuenv, thread_id, export_name, workarea, lock_count, ptimeout, false);
}

EXPORT(SceInt32, _sceKernelLockLwMutexCB, Ptr<SceKernelLwMutexWork> workarea, SceInt32 lock_count, SceUInt32 *pTimeout) {
    TRACY_FUNC(_sceKernelLockLwMutexCB, workarea, lock_count, pTimeout);
    return lock_lw_mutex(emuenv, thread_id, export_name, workarea, lock_count, pTimeout, true);
}

EXPORT(int, _sceKernelLockMutex, SceUID mutexid, int lock_count, unsigned int *timeout) {
    TRACY_FUNC(_sceKernelLockMutex, mutexid, lock_count, timeout);
    auto mutex = lock_and_find(mutexid, emuenv.kernel.mutexes, emuenv.kernel.mutex);
    if (!mutex)
        return RET_ERROR(SCE_KERNEL_ERROR_UNKNOWN_MUTEX_ID);
    const Deadline deadline = deadline_from(timeout);
    return unwrap_or_bail(mutex->lock(emuenv.kernel.get_thread(thread_id), lock_count, deadline, emuenv.mem, false), timeout, deadline);
}

EXPORT(SceInt32, _sceKernelLockMutexCB, SceUID mutexId, SceInt32 lockCount, SceUInt32 *pTimeout) {
    TRACY_FUNC(_sceKernelLockMutexCB, mutexId, lockCount, pTimeout);
    auto mutex = lock_and_find(mutexId, emuenv.kernel.mutexes, emuenv.kernel.mutex);
    if (!mutex)
        return RET_ERROR(SCE_KERNEL_ERROR_UNKNOWN_MUTEX_ID);
    const Deadline deadline = deadline_from(pTimeout);
    return unwrap_or_bail(mutex->lock(emuenv.kernel.get_thread(thread_id), lockCount, deadline, emuenv.mem, true), pTimeout, deadline);
}

EXPORT(SceInt32, _sceKernelLockReadRWLock, SceUID lock_id, SceUInt32 *timeout) {
    TRACY_FUNC(_sceKernelLockReadRWLock, lock_id, timeout);
    auto rwlock = lock_and_find(lock_id, emuenv.kernel.rwlocks, emuenv.kernel.mutex);
    if (!rwlock)
        return RET_ERROR(SCE_KERNEL_ERROR_UNKNOWN_RW_LOCK_ID);
    const Deadline deadline = deadline_from(timeout);
    return unwrap_or_bail(rwlock->lock(emuenv.kernel.get_thread(thread_id), false, deadline, false), timeout, deadline);
}

EXPORT(SceInt32, _sceKernelLockReadRWLockCB, SceUID lock_id, SceUInt32 *timeout) {
    TRACY_FUNC(_sceKernelLockReadRWLockCB, lock_id, timeout);
    auto rwlock = lock_and_find(lock_id, emuenv.kernel.rwlocks, emuenv.kernel.mutex);
    if (!rwlock)
        return RET_ERROR(SCE_KERNEL_ERROR_UNKNOWN_RW_LOCK_ID);
    const Deadline deadline = deadline_from(timeout);
    return unwrap_or_bail(rwlock->lock(emuenv.kernel.get_thread(thread_id), false, deadline, true), timeout, deadline);
}

EXPORT(SceInt32, _sceKernelLockWriteRWLock, SceUID lock_id, SceUInt32 *timeout) {
    TRACY_FUNC(_sceKernelLockWriteRWLock, lock_id, timeout);
    auto rwlock = lock_and_find(lock_id, emuenv.kernel.rwlocks, emuenv.kernel.mutex);
    if (!rwlock)
        return RET_ERROR(SCE_KERNEL_ERROR_UNKNOWN_RW_LOCK_ID);
    const Deadline deadline = deadline_from(timeout);
    return unwrap_or_bail(rwlock->lock(emuenv.kernel.get_thread(thread_id), true, deadline, false), timeout, deadline);
}

EXPORT(SceInt32, _sceKernelLockWriteRWLockCB, SceUID lock_id, SceUInt32 *timeout) {
    TRACY_FUNC(_sceKernelLockWriteRWLockCB, lock_id, timeout);
    auto rwlock = lock_and_find(lock_id, emuenv.kernel.rwlocks, emuenv.kernel.mutex);
    if (!rwlock)
        return RET_ERROR(SCE_KERNEL_ERROR_UNKNOWN_RW_LOCK_ID);
    const Deadline deadline = deadline_from(timeout);
    return unwrap_or_bail(rwlock->lock(emuenv.kernel.get_thread(thread_id), true, deadline, true), timeout, deadline);
}

EXPORT(int, _sceKernelPMonThreadGetCounter) {
    TRACY_FUNC(_sceKernelPMonThreadGetCounter);
    return UNIMPLEMENTED();
}

EXPORT(int, _sceKernelPollEvent, SceUID event_id, SceUInt32 bit_pattern, SceUInt32 *result_pattern, SceUInt64 *user_data) {
    TRACY_FUNC(_sceKernelPollEvent, event_id, bit_pattern, result_pattern, user_data);
    auto ev = lock_and_find(event_id, emuenv.kernel.simple_events, emuenv.kernel.mutex);
    if (!ev) {
        auto timer = lock_and_find(event_id, emuenv.kernel.timers, emuenv.kernel.mutex);
        if (!timer)
            return RET_ERROR(SCE_KERNEL_ERROR_UNKNOWN_EVENT_ID);
        return timer->wait_or_poll(emuenv.kernel.get_thread(thread_id), result_pattern, user_data, false, false);
    }
    return unwrap_or_bail(ev->wait_or_poll(emuenv.kernel.get_thread(thread_id), bit_pattern, result_pattern, user_data, deadline_from(nullptr), false, false));
}

EXPORT(int, _sceKernelPollEventFlag, SceUID event_id, unsigned int flags, unsigned int wait, unsigned int *outBits) {
    TRACY_FUNC(_sceKernelPollEventFlag, event_id, flags, wait, outBits);
    auto ef = lock_and_find(event_id, emuenv.kernel.eventflags, emuenv.kernel.mutex);
    if (!ef)
        return RET_ERROR(SCE_KERNEL_ERROR_UNKNOWN_EVF_ID);
    return ef->poll(flags, wait, outBits);
}

EXPORT(int, _sceKernelPulseEventWithNotifyCallback) {
    TRACY_FUNC(_sceKernelPulseEventWithNotifyCallback);
    return UNIMPLEMENTED();
}

EXPORT(int, _sceKernelReceiveMsgPipeVector) {
    TRACY_FUNC(_sceKernelReceiveMsgPipeVector);
    return UNIMPLEMENTED();
}

EXPORT(int, _sceKernelReceiveMsgPipeVectorCB) {
    TRACY_FUNC(_sceKernelReceiveMsgPipeVectorCB);
    return UNIMPLEMENTED();
}

EXPORT(int, _sceKernelRegisterThreadEventHandler, const char *name, SceUID thread_mask, SceUInt32 mask, sceKernelRegisterThreadEventHandlerOpt *opt) {
    TRACY_FUNC(_sceKernelRegisterThreadEventHandler, name, thread_mask, mask, opt);
    if (!opt)
        return RET_ERROR(SCE_KERNEL_ERROR_INVALID_ARGUMENT);

    if (mask & SCE_KERNEL_THREAD_EVENT_TYPE_START) {
        if (emuenv.kernel.thread_event_start)
            LOG_WARN("Multiple thread handlers are not supported");

        emuenv.kernel.thread_event_start = opt->handler;
        emuenv.kernel.thread_event_start_arg = opt->common;
    }

    if (mask & SCE_KERNEL_THREAD_EVENT_TYPE_END) {
        if (emuenv.kernel.thread_event_end)
            LOG_WARN("Multiple thread handlers are not supported");

        emuenv.kernel.thread_event_end = opt->handler;
        emuenv.kernel.thread_event_end_arg = opt->common;
    }

    return SCE_KERNEL_OK;
}

EXPORT(int, _sceKernelSendMsgPipeVector) {
    TRACY_FUNC(_sceKernelSendMsgPipeVector);
    return UNIMPLEMENTED();
}

EXPORT(int, _sceKernelSendMsgPipeVectorCB) {
    TRACY_FUNC(_sceKernelSendMsgPipeVectorCB);
    return UNIMPLEMENTED();
}

EXPORT(int, _sceKernelSetEventWithNotifyCallback) {
    TRACY_FUNC(_sceKernelSetEventWithNotifyCallback);
    return UNIMPLEMENTED();
}

EXPORT(int, _sceKernelSetThreadContextForVM, SceUID threadId, Ptr<SceKernelThreadCpuRegisterInfo> pCpuRegisterInfo, Ptr<SceKernelThreadVfpRegisterInfo> pVfpRegisterInfo) {
    TRACY_FUNC(_sceKernelSetThreadContextForVM, threadId, pCpuRegisterInfo, pVfpRegisterInfo);
    const ThreadStatePtr thread = emuenv.kernel.get_thread(threadId);
    if (!thread)
        return RET_ERROR(SCE_KERNEL_ERROR_UNKNOWN_THREAD_ID);

    SceKernelThreadCpuRegisterInfo *infoCpu = pCpuRegisterInfo.get(emuenv.mem);
    if (infoCpu) {
        if (infoCpu->size != sizeof(*infoCpu))
            return RET_ERROR(SCE_KERNEL_ERROR_INVALID_ARGUMENT_SIZE);

        // Todo
    }

    SceKernelThreadVfpRegisterInfo *infoVfp = pVfpRegisterInfo.get(emuenv.mem);
    if (infoVfp) {
        if (infoVfp->size != sizeof(*infoVfp))
            return RET_ERROR(SCE_KERNEL_ERROR_INVALID_ARGUMENT_SIZE);

        // Todo
    }

    return UNIMPLEMENTED();
}

EXPORT(SceInt32, _sceKernelSetTimerEvent, SceUID timer_id, SceInt32 type, Ptr<SceKernelSysClock> pInterval, SceInt32 fRepeat) {
    TRACY_FUNC(_sceKernelSetTimerEvent, timer_id, type, pInterval, fRepeat);
    auto timer = lock_and_find(timer_id, emuenv.kernel.timers, emuenv.kernel.mutex);
    if (!timer)
        return RET_ERROR(SCE_KERNEL_ERROR_UNKNOWN_TIMER_ID);
    SceKernelSysClock *interval = pInterval.get(emuenv.mem);
    return timer->set(type, interval, fRepeat);
}

EXPORT(int, _sceKernelSetTimerTime) {
    TRACY_FUNC(_sceKernelSetTimerTime);
    return UNIMPLEMENTED();
}

EXPORT(int, _sceKernelSignalLwCond, Ptr<SceKernelLwCondWork> workarea) {
    TRACY_FUNC(_sceKernelSignalLwCond, workarea);
    SceUID condid = workarea.get(emuenv.mem)->uid;
    auto cv = lock_and_find(condid, emuenv.kernel.lwcondvars, emuenv.kernel.mutex);
    if (!cv)
        return RET_ERROR(SCE_KERNEL_ERROR_UNKNOWN_LW_COND_ID);
    return cv->signal(CondvarSignalTarget(CondvarSignalTarget::Type::Any));
}

EXPORT(int, _sceKernelSignalLwCondAll, Ptr<SceKernelLwCondWork> workarea) {
    TRACY_FUNC(_sceKernelSignalLwCondAll, workarea);
    SceUID condid = workarea.get(emuenv.mem)->uid;
    auto cv = lock_and_find(condid, emuenv.kernel.lwcondvars, emuenv.kernel.mutex);
    if (!cv)
        return RET_ERROR(SCE_KERNEL_ERROR_UNKNOWN_LW_COND_ID);
    return cv->signal(CondvarSignalTarget(CondvarSignalTarget::Type::All));
}

EXPORT(int, _sceKernelSignalLwCondTo) {
    TRACY_FUNC(_sceKernelSignalLwCondTo);
    return UNIMPLEMENTED();
}

EXPORT(int, _sceKernelStartThread, SceUID thid, SceSize arglen, Ptr<void> argp) {
    TRACY_FUNC(_sceKernelStartThread, thid, arglen, argp);
    auto thread = emuenv.kernel.get_thread(thid);

    if (!thread) {
        return RET_ERROR(SCE_KERNEL_ERROR_UNKNOWN_THREAD_ID);
    }

    {
        std::lock_guard lock(thread->mutex);
        if (thread->status == ThreadStatus::running) {
            return RET_ERROR(SCE_KERNEL_ERROR_RUNNING);
        }
    }

    const int res = thread->start(arglen, argp, true);
    if (res < 0) {
        return RET_ERROR(res);
    }
    return res;
}

EXPORT(int, _sceKernelTryReceiveMsgPipeVector) {
    TRACY_FUNC(_sceKernelTryReceiveMsgPipeVector);
    return UNIMPLEMENTED();
}

EXPORT(int, _sceKernelTrySendMsgPipeVector) {
    TRACY_FUNC(_sceKernelTrySendMsgPipeVector);
    return UNIMPLEMENTED();
}

EXPORT(int, _sceKernelUnlockLwMutex) {
    TRACY_FUNC(_sceKernelUnlockLwMutex);
    return UNIMPLEMENTED();
}

EXPORT(SceInt32, _sceKernelWaitCond, SceUID condId, SceUInt32 *pTimeout) {
    TRACY_FUNC(_sceKernelWaitCond, condId, pTimeout);
    auto cv = lock_and_find(condId, emuenv.kernel.condvars, emuenv.kernel.mutex);
    if (!cv)
        return RET_ERROR(SCE_KERNEL_ERROR_UNKNOWN_COND_ID);
    const Deadline deadline = deadline_from(pTimeout);
    return unwrap_or_bail(cv->wait(emuenv.kernel.get_thread(thread_id), deadline, emuenv.mem, false), pTimeout, deadline);
}

EXPORT(SceInt32, _sceKernelWaitCondCB, SceUID condId, SceUInt32 *pTimeout) {
    TRACY_FUNC(_sceKernelWaitCondCB, condId, pTimeout);
    auto cv = lock_and_find(condId, emuenv.kernel.condvars, emuenv.kernel.mutex);
    if (!cv)
        return RET_ERROR(SCE_KERNEL_ERROR_UNKNOWN_COND_ID);
    const Deadline deadline = deadline_from(pTimeout);
    return unwrap_or_bail(cv->wait(emuenv.kernel.get_thread(thread_id), deadline, emuenv.mem, true), pTimeout, deadline);
}

EXPORT(SceInt32, _sceKernelWaitEvent, SceUID event_id, SceUInt32 bit_pattern, SceUInt32 *result_pattern, SceUInt64 *user_data, SceUInt32 *timeout) {
    TRACY_FUNC(_sceKernelWaitEvent, event_id, bit_pattern, result_pattern, user_data, timeout);
    auto ev = lock_and_find(event_id, emuenv.kernel.simple_events, emuenv.kernel.mutex);
    if (!ev) {
        auto timer = lock_and_find(event_id, emuenv.kernel.timers, emuenv.kernel.mutex);
        if (!timer)
            return RET_ERROR(SCE_KERNEL_ERROR_UNKNOWN_EVENT_ID);
        if (timeout)
            LOG_WARN_ONCE("Ignoring timer timeout");
        return timer->wait_or_poll(emuenv.kernel.get_thread(thread_id), result_pattern, user_data, true, false);
    }
    const Deadline deadline = deadline_from(timeout);
    return unwrap_or_bail(ev->wait_or_poll(emuenv.kernel.get_thread(thread_id), bit_pattern, result_pattern, user_data, deadline, true, false), timeout, deadline);
}

EXPORT(SceInt32, _sceKernelWaitEventCB, SceUID event_id, SceUInt32 bit_pattern, SceUInt32 *result_pattern, SceUInt64 *user_data, SceUInt32 *timeout) {
    TRACY_FUNC(_sceKernelWaitEventCB, event_id, bit_pattern, result_pattern, user_data, timeout);
    auto ev = lock_and_find(event_id, emuenv.kernel.simple_events, emuenv.kernel.mutex);
    if (!ev) {
        auto timer = lock_and_find(event_id, emuenv.kernel.timers, emuenv.kernel.mutex);
        if (!timer)
            return RET_ERROR(SCE_KERNEL_ERROR_UNKNOWN_EVENT_ID);
        if (timeout)
            LOG_WARN_ONCE("Ignoring timer timeout");
        return timer->wait_or_poll(emuenv.kernel.get_thread(thread_id), result_pattern, user_data, true, true);
    }
    const Deadline deadline = deadline_from(timeout);
    return unwrap_or_bail(ev->wait_or_poll(emuenv.kernel.get_thread(thread_id), bit_pattern, result_pattern, user_data, deadline, true, true), timeout, deadline);
}

EXPORT(SceInt32, _sceKernelWaitEventFlag, SceUID evfId, SceUInt32 bitPattern, SceUInt32 waitMode, SceUInt32 *pResultPat, SceUInt32 *pTimeout) {
    TRACY_FUNC(_sceKernelWaitEventFlag, evfId, bitPattern, waitMode, pResultPat, pTimeout);
    auto ef = lock_and_find(evfId, emuenv.kernel.eventflags, emuenv.kernel.mutex);
    if (!ef)
        return RET_ERROR(SCE_KERNEL_ERROR_UNKNOWN_EVF_ID);
    const Deadline deadline = deadline_from(pTimeout);
    return unwrap_or_bail(ef->wait(emuenv.kernel.get_thread(thread_id), bitPattern, waitMode, pResultPat, deadline, false), pTimeout, deadline);
}

EXPORT(SceInt32, _sceKernelWaitEventFlagCB, SceUID evfId, SceUInt32 bitPattern, SceUInt32 waitMode, SceUInt32 *pResultPat, SceUInt32 *pTimeout) {
    TRACY_FUNC(_sceKernelWaitEventFlagCB, evfId, bitPattern, waitMode, pResultPat, pTimeout);
    auto ef = lock_and_find(evfId, emuenv.kernel.eventflags, emuenv.kernel.mutex);
    if (!ef)
        return RET_ERROR(SCE_KERNEL_ERROR_UNKNOWN_EVF_ID);
    const Deadline deadline = deadline_from(pTimeout);
    return unwrap_or_bail(ef->wait(emuenv.kernel.get_thread(thread_id), bitPattern, waitMode, pResultPat, deadline, true), pTimeout, deadline);
}

EXPORT(int, _sceKernelWaitException) {
    TRACY_FUNC(_sceKernelWaitException);
    return UNIMPLEMENTED();
}

EXPORT(int, _sceKernelWaitExceptionCB) {
    TRACY_FUNC(_sceKernelWaitExceptionCB);
    return UNIMPLEMENTED();
}

EXPORT(int, _sceKernelWaitLwCond, Ptr<SceKernelLwCondWork> workarea, SceUInt32 *timeout) {
    TRACY_FUNC(_sceKernelWaitLwCond, workarea, timeout);
    const auto cond_id = workarea.get(emuenv.mem)->uid;
    auto cv = lock_and_find(cond_id, emuenv.kernel.lwcondvars, emuenv.kernel.mutex);
    if (!cv)
        return RET_ERROR(SCE_KERNEL_ERROR_UNKNOWN_LW_COND_ID);
    const Deadline deadline = deadline_from(timeout);
    return unwrap_or_bail(cv->wait(emuenv.kernel.get_thread(thread_id), deadline, emuenv.mem, false), timeout, deadline);
}

EXPORT(SceInt32, _sceKernelWaitLwCondCB, Ptr<SceKernelLwCondWork> pWork, SceUInt32 *pTimeout) {
    TRACY_FUNC(_sceKernelWaitLwCondCB, pWork, pTimeout);
    const auto cond_id = pWork.get(emuenv.mem)->uid;
    auto cv = lock_and_find(cond_id, emuenv.kernel.lwcondvars, emuenv.kernel.mutex);
    if (!cv)
        return RET_ERROR(SCE_KERNEL_ERROR_UNKNOWN_LW_COND_ID);
    const Deadline deadline = deadline_from(pTimeout);
    return unwrap_or_bail(cv->wait(emuenv.kernel.get_thread(thread_id), deadline, emuenv.mem, true), pTimeout, deadline);
}

EXPORT(int, _sceKernelWaitMultipleEvents) {
    TRACY_FUNC(_sceKernelWaitMultipleEvents);
    return UNIMPLEMENTED();
}

EXPORT(int, _sceKernelWaitMultipleEventsCB) {
    TRACY_FUNC(_sceKernelWaitMultipleEventsCB);
    return UNIMPLEMENTED();
}

EXPORT(SceInt32, _sceKernelWaitSema, SceUID semaId, SceInt32 needCount, SceUInt32 *pTimeout) {
    TRACY_FUNC(_sceKernelWaitSema, semaId, needCount, pTimeout);
    auto sema = lock_and_find(semaId, emuenv.kernel.semaphores, emuenv.kernel.mutex);
    if (!sema)
        return RET_ERROR(SCE_KERNEL_ERROR_UNKNOWN_SEMA_ID);
    const Deadline deadline = deadline_from(pTimeout);
    return unwrap_or_bail(sema->wait_for(emuenv.kernel.get_thread(thread_id), needCount, deadline, false), pTimeout, deadline);
}

EXPORT(SceInt32, _sceKernelWaitSemaCB, SceUID semaId, SceInt32 needCount, SceUInt32 *pTimeout) {
    TRACY_FUNC(_sceKernelWaitSemaCB, semaId, needCount, pTimeout);
    auto sema = lock_and_find(semaId, emuenv.kernel.semaphores, emuenv.kernel.mutex);
    if (!sema)
        return RET_ERROR(SCE_KERNEL_ERROR_UNKNOWN_SEMA_ID);
    const Deadline deadline = deadline_from(pTimeout);
    return unwrap_or_bail(sema->wait_for(emuenv.kernel.get_thread(thread_id), needCount, deadline, true), pTimeout, deadline);
}

static WaitResult wait_signal(ThreadStatePtr thread, Deadline deadline,
    bool cb) {
    const WaitInfo info{
        .type = cb ? SCE_KERNEL_WAITTYPE_SIGNAL_CB : SCE_KERNEL_WAITTYPE_SIGNAL,
        .reason = "signal",
    };
    thread->enter_wait(info);
    while (true) {
        bool run_callbacks = false;
        bool should_bail = false;
        bool signal_pending = false;
        {
            std::lock_guard lock(thread->mutex);
            should_bail = thread->stop_requested_locked();
            if (!should_bail && thread->signal_pending) {
                thread->signal_pending = false;
                signal_pending = true;
            }
            if (!should_bail && !signal_pending && cb && thread->callbacks_pending) {
                thread->callbacks_pending = false;
                run_callbacks = true;
            }
        }
        if (should_bail) {
            thread->leave_wait();
            return std::unexpected{ ExitSignal{} };
        }
        if (signal_pending) {
            thread->leave_wait();
            return SCE_KERNEL_OK;
        }
        if (run_callbacks) {
            thread->leave_wait();
            while (true) {
                thread->process_callbacks();
                bool callbacks_pending = false;
                bool post_callback_signal = false;
                {
                    std::lock_guard lock(thread->mutex);
                    if (thread->stop_requested_locked())
                        return std::unexpected{ ExitSignal{} };
                    if (thread->signal_pending) {
                        thread->signal_pending = false;
                        post_callback_signal = true;
                    } else if (cb && thread->callbacks_pending) {
                        thread->callbacks_pending = false;
                        callbacks_pending = true;
                    }
                }
                if (post_callback_signal)
                    return SCE_KERNEL_OK;
                if (!callbacks_pending)
                    break;
            }
            thread->enter_wait(info);
            continue;
        }
        if (thread->park_until(deadline) == ThreadState::ParkResult::timed_out) {
            thread->leave_wait();
            return SCE_KERNEL_ERROR_WAIT_TIMEOUT;
        }
    }
}

EXPORT(int, _sceKernelWaitSignal, uint32_t unknown, uint32_t delay, uint32_t timeout) {
    TRACY_FUNC(_sceKernelWaitSignal, unknown, delay, timeout);
    return unwrap_or_bail(wait_signal(emuenv.kernel.get_thread(thread_id), deadline_from(&timeout), false));
}

EXPORT(int, _sceKernelWaitSignalCB, uint32_t unknown, uint32_t delay, uint32_t timeout) {
    TRACY_FUNC(_sceKernelWaitSignalCB, unknown, delay, timeout);
    return unwrap_or_bail(wait_signal(emuenv.kernel.get_thread(thread_id), deadline_from(&timeout), true));
}

static void writeback_remaining(SceUInt *timeout, Deadline deadline) {
    if (!timeout || deadline == Deadline::max())
        return;
    const auto remaining = std::chrono::duration_cast<std::chrono::microseconds>(
        deadline - std::chrono::steady_clock::now())
                               .count();
    *timeout = (remaining <= 0) ? 0 : static_cast<SceUInt>(remaining);
}

static WaitResult wait_thread_end(ThreadStatePtr waiter, ThreadStatePtr target,
    int *stat, SceUInt *timeout, bool cb) {
    std::unique_lock<std::mutex> tlock(target->mutex);
    if (target->status == ThreadStatus::dormant) {
        if (stat)
            *stat = static_cast<SceInt32>(target->returned_value);
        return SCE_KERNEL_OK;
    }

    const WaitInfo info{
        .type = cb ? SCE_KERNEL_WAITTYPE_WAITTHEND_CB : SCE_KERNEL_WAITTYPE_WAITTHEND,
        .uid = target->id,
        .reason = "thread_end",
    };

    WaitThreadEndJoinerEntry entry;

    const auto deadline = deadline_from(timeout);
    const WaitResult res = target->wait_thread_end_joiners.enqueue_and_wait(tlock, waiter, entry, deadline, cb, info);

    if (res && *res == SCE_KERNEL_OK) {
        if (stat)
            *stat = entry.returned_value;
        writeback_remaining(timeout, deadline);
    } else if (res && *res == SCE_KERNEL_ERROR_WAIT_TIMEOUT && timeout) {
        *timeout = 0;
    }
    return res;
}

EXPORT(int, _sceKernelWaitThreadEnd, SceUID thid, int *stat, SceUInt *timeout) {
    TRACY_FUNC(_sceKernelWaitThreadEnd, thid, stat, timeout);
    auto target = emuenv.kernel.get_thread(thid);
    if (!target)
        return RET_ERROR(SCE_KERNEL_ERROR_UNKNOWN_THREAD_ID);
    return unwrap_or_bail(wait_thread_end(emuenv.kernel.get_thread(thread_id), target, stat, timeout, false));
}

EXPORT(int, _sceKernelWaitThreadEndCB, SceUID thid, int *stat, SceUInt *timeout) {
    TRACY_FUNC(_sceKernelWaitThreadEndCB, thid, stat, timeout);
    auto target = emuenv.kernel.get_thread(thid);
    if (!target)
        return RET_ERROR(SCE_KERNEL_ERROR_UNKNOWN_THREAD_ID);
    return unwrap_or_bail(wait_thread_end(emuenv.kernel.get_thread(thread_id), target, stat, timeout, true));
}

EXPORT(SceInt32, sceKernelCancelCallback, SceUID callbackId) {
    TRACY_FUNC(sceKernelCancelCallback, callbackId);
    const CallbackPtr cb = lock_and_find(callbackId, emuenv.kernel.callbacks, emuenv.kernel.mutex);

    if (!cb)
        return RET_ERROR(SCE_KERNEL_ERROR_UNKNOWN_CALLBACK_ID);
    cb->cancel();

    return SCE_KERNEL_OK;
}

EXPORT(int, sceKernelChangeActiveCpuMask) {
    TRACY_FUNC(sceKernelChangeActiveCpuMask);
    return UNIMPLEMENTED();
}

EXPORT(SceInt32, sceKernelChangeThreadCpuAffinityMask, SceUID thid, SceInt32 affinity_mask) {
    TRACY_FUNC(sceKernelChangeThreadCpuAffinityMask, thid, affinity_mask);
    const ThreadStatePtr thread = emuenv.kernel.get_thread(thid ? thid : thread_id);

    if (!thread)
        return RET_ERROR(SCE_KERNEL_ERROR_UNKNOWN_THREAD_ID);

    const SceInt32 old_affinity = thread->affinity_mask;

    if (affinity_mask & ~SCE_KERNEL_CPU_MASK_USER_ALL)
        return RET_ERROR(SCE_KERNEL_ERROR_ILLEGAL_CPU_AFFINITY_MASK);

    thread->affinity_mask = affinity_mask;
    thread->tls.get_ptr<int>().get(emuenv.mem)[TLS_CPU_AFFINITY_MASK] = affinity_mask;
    return old_affinity;
}

EXPORT(SceInt32, sceKernelChangeThreadPriority2, SceUID thid, SceInt32 priority) {
    TRACY_FUNC(sceKernelChangeThreadPriority2, thid, priority);
    const ThreadStatePtr thread = emuenv.kernel.get_thread(thid ? thid : thread_id);
    if (!thread)
        return RET_ERROR(SCE_KERNEL_ERROR_UNKNOWN_THREAD_ID);

    const SceInt32 old_priority = thread->priority;

    if (priority == SCE_KERNEL_CURRENT_THREAD_PRIORITY) {
        priority = emuenv.kernel.get_thread(thread_id)->priority;
    }

    if (priority >= SCE_KERNEL_HIGHEST_DEFAULT_PRIORITY
        && priority <= SCE_KERNEL_LOWEST_DEFAULT_PRIORITY) {
        priority = SCE_KERNEL_GAME_DEFAULT_PRIORITY_ACTUAL + (priority - SCE_KERNEL_DEFAULT_PRIORITY);
    }

    if (priority < SCE_KERNEL_HIGHEST_PRIORITY_USER || priority > SCE_KERNEL_LOWEST_PRIORITY_USER)
        return RET_ERROR(SCE_KERNEL_ERROR_ILLEGAL_PRIORITY);

    thread->priority = priority;
    thread->tls.get_ptr<int>().get(emuenv.mem)[TLS_CURRENT_PRIORITY] = priority;

    return old_priority;
}

EXPORT(SceInt32, sceKernelChangeThreadPriority, SceUID thid, SceInt32 priority) {
    TRACY_FUNC(sceKernelChangeThreadPriority, thid, priority);
    auto err = CALL_EXPORT(sceKernelChangeThreadPriority2, thid, priority);
    if (err < 0)
        return err;

    return SCE_KERNEL_OK;
}

EXPORT(int, sceKernelChangeThreadVfpException, SceInt32 clearMask, SceInt32 setMask) {
    TRACY_FUNC(sceKernelChangeThreadVfpException, clearMask, setMask);
    if (((clearMask | setMask) & 0xf7ffff60) != 0 || (clearMask & setMask) != 0) {
        return RET_ERROR(SCE_KERNEL_ERROR_INVALID_ARGUMENT);
    }
    const ThreadStatePtr thread = emuenv.kernel.get_thread(thread_id);
    if (!thread)
        return RET_ERROR(SCE_KERNEL_ERROR_UNKNOWN_THREAD_ID);
    int &vfp_exception = thread->tls.get_ptr<int>().get(emuenv.mem)[TLS_VFP_EXCEPTION];
    int old_exception = vfp_exception;
    vfp_exception = setMask | (vfp_exception & ~clearMask);
    STUBBED("");
    return old_exception;
}

EXPORT(SceInt32, sceKernelCheckCallback) {
    TRACY_FUNC(sceKernelCheckCallback);
    return emuenv.kernel.get_thread(thread_id)->process_callbacks();
}

EXPORT(int, sceKernelCheckWaitableStatus) {
    TRACY_FUNC(sceKernelCheckWaitableStatus);
    return UNIMPLEMENTED();
}

EXPORT(SceInt32, sceKernelClearEvent, SceUID event_id, SceUInt32 clear_pattern) {
    TRACY_FUNC(sceKernelClearEvent, event_id, clear_pattern);
    auto ev = lock_and_find(event_id, emuenv.kernel.simple_events, emuenv.kernel.mutex);
    if (!ev)
        return RET_ERROR(SCE_KERNEL_ERROR_UNKNOWN_EVENT_ID);
    return ev->clear(clear_pattern);
}

EXPORT(SceInt32, sceKernelClearEventFlag, SceUID evfId, SceUInt32 bitPattern) {
    TRACY_FUNC(sceKernelClearEventFlag, evfId, bitPattern);
    auto ef = lock_and_find(evfId, emuenv.kernel.eventflags, emuenv.kernel.mutex);
    if (!ef)
        return RET_ERROR(SCE_KERNEL_ERROR_UNKNOWN_EVF_ID);
    return ef->clear(bitPattern);
}

EXPORT(int, sceKernelCloseCond) {
    TRACY_FUNC(sceKernelCloseCond);
    return UNIMPLEMENTED();
}

EXPORT(int, sceKernelCloseEventFlag, SceUID evfId) {
    TRACY_FUNC(sceKernelCloseEventFlag, evfId);
    return UNIMPLEMENTED();
}

EXPORT(int, sceKernelCloseMsgPipe) {
    TRACY_FUNC(sceKernelCloseMsgPipe);
    return UNIMPLEMENTED();
}

EXPORT(int, sceKernelCloseMutex) {
    TRACY_FUNC(sceKernelCloseMutex);
    return UNIMPLEMENTED();
}

EXPORT(int, sceKernelCloseMutex_089) {
    TRACY_FUNC(sceKernelCloseMutex_089);
    return UNIMPLEMENTED();
}

EXPORT(int, sceKernelCloseRWLock) {
    TRACY_FUNC(sceKernelCloseRWLock);
    return UNIMPLEMENTED();
}

EXPORT(int, sceKernelCloseSema) {
    TRACY_FUNC(sceKernelCloseSema);
    return UNIMPLEMENTED();
}

EXPORT(int, sceKernelCloseSimpleEvent) {
    TRACY_FUNC(sceKernelCloseSimpleEvent);
    return UNIMPLEMENTED();
}

EXPORT(int, sceKernelCloseTimer) {
    TRACY_FUNC(sceKernelCloseTimer);
    return STUBBED("References not implemented.");
}

EXPORT(SceUID, sceKernelCreateCallback, char *name, SceUInt32 attr, Ptr<SceKernelCallbackFunction> callbackFunc, Ptr<void> pCommon) {
    TRACY_FUNC(sceKernelCreateCallback, name, attr, callbackFunc, pCommon);
    if (attr || !callbackFunc.address())
        return RET_ERROR(SCE_KERNEL_ERROR_ILLEGAL_ATTR);

    ThreadStatePtr thread = emuenv.kernel.get_thread(thread_id);
    auto cb = std::make_shared<Callback>(thread_id, name, callbackFunc, pCommon);
    std::lock_guard lock(emuenv.kernel.mutex);
    SceUID cb_uid = emuenv.kernel.get_next_uid();
    emuenv.kernel.callbacks.emplace(cb_uid, cb);
    thread->callbacks.push_back(cb);
    return cb_uid;
}

EXPORT(int, sceKernelCreateThreadForUser, const char *name, SceKernelThreadEntry entry, int init_priority, SceKernelCreateThread_opt *options) {
    TRACY_FUNC(sceKernelCreateThreadForUser, name, entry, init_priority, options);
    if (options->cpu_affinity_mask & ~SCE_KERNEL_CPU_MASK_USER_ALL) {
        return RET_ERROR(SCE_KERNEL_ERROR_INVALID_CPU_AFFINITY);
    }

    const ThreadStatePtr thread = emuenv.kernel.create_thread(emuenv.mem, name, entry.cast<void>(), init_priority, options->cpu_affinity_mask, options->stack_size, options->option.get(emuenv.mem));
    if (!thread)
        return RET_ERROR(SCE_KERNEL_ERROR_ERROR);
    return thread->id;
}

static WaitResult delay_thread(ThreadStatePtr thread, Deadline deadline,
    bool cb) {
    const WaitInfo info{
        .type = cb ? SCE_KERNEL_WAITTYPE_DELAY_CB : SCE_KERNEL_WAITTYPE_DELAY,
        .reason = "delay",
    };
    thread->enter_wait(info);
    while (true) {
        const auto park_result = thread->park_until(deadline);

        bool run_callbacks = false;
        bool should_bail = false;
        {
            std::lock_guard lock(thread->mutex);
            should_bail = thread->stop_requested_locked();
            if (!should_bail && cb && thread->callbacks_pending) {
                thread->callbacks_pending = false;
                run_callbacks = true;
            }
        }
        if (should_bail) {
            thread->leave_wait();
            return std::unexpected{ ExitSignal{} };
        }
        if (park_result == ThreadState::ParkResult::timed_out) {
            thread->leave_wait();
            return SCE_KERNEL_OK;
        }
        if (run_callbacks) {
            thread->leave_wait();
            while (true) {
                thread->process_callbacks();
                bool callbacks_pending = false;
                {
                    std::lock_guard lock(thread->mutex);
                    if (thread->stop_requested_locked())
                        return std::unexpected{ ExitSignal{} };
                    if (cb && thread->callbacks_pending) {
                        thread->callbacks_pending = false;
                        callbacks_pending = true;
                    }
                }
                if (!callbacks_pending)
                    break;
            }
            thread->enter_wait(info);
        }
        // Otherwise spurious wake. Re-park against the same deadline.
    }
}

EXPORT(int, sceKernelDelayThread, SceUInt delay) {
    TRACY_FUNC(sceKernelDelayThread, delay);
    if (delay == 0)
        return RET_ERROR(SCE_KERNEL_ERROR_INVALID_ARGUMENT);
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::microseconds(delay);
    return unwrap_or_bail(delay_thread(emuenv.kernel.get_thread(thread_id), deadline, false));
}

EXPORT(int, sceKernelDelayThread200, SceUInt delay) {
    TRACY_FUNC(sceKernelDelayThread200, delay);
    if (delay < 201)
        delay = 201;
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::microseconds(delay);
    return unwrap_or_bail(delay_thread(emuenv.kernel.get_thread(thread_id), deadline, false));
}

EXPORT(int, sceKernelDelayThreadCB, SceUInt delay) {
    TRACY_FUNC(sceKernelDelayThreadCB, delay);
    if (delay == 0)
        return RET_ERROR(SCE_KERNEL_ERROR_INVALID_ARGUMENT);
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::microseconds(delay);
    return unwrap_or_bail(delay_thread(emuenv.kernel.get_thread(thread_id), deadline, true));
}

EXPORT(int, sceKernelDelayThreadCB200, SceUInt delay) {
    TRACY_FUNC(sceKernelDelayThreadCB200, delay);
    if (delay < 201)
        delay = 201;
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::microseconds(delay);
    return unwrap_or_bail(delay_thread(emuenv.kernel.get_thread(thread_id), deadline, true));
}

EXPORT(int, sceKernelDeleteCallback, SceUID callbackId) {
    TRACY_FUNC(sceKernelDeleteCallback, callbackId);
    const CallbackPtr cb = lock_and_find(callbackId, emuenv.kernel.callbacks, emuenv.kernel.mutex);
    if (!cb)
        return RET_ERROR(SCE_KERNEL_ERROR_UNKNOWN_CALLBACK_ID);
    const auto owner = emuenv.kernel.get_thread(cb->get_owner_thread_id());
    std::lock_guard lock(emuenv.kernel.mutex);
    emuenv.kernel.callbacks.erase(callbackId);
    if (owner)
        std::erase(owner->callbacks, cb);
    return 0;
}

EXPORT(int, sceKernelDeleteCond, SceUID condition_variable_id) {
    TRACY_FUNC(sceKernelDeleteCond, condition_variable_id);
    return delete_sync_object<Condvar>(emuenv.kernel, emuenv.kernel.condvars, condition_variable_id, SCE_KERNEL_ERROR_UNKNOWN_COND_ID);
}

EXPORT(int, sceKernelDeleteEventFlag, SceUID event_id) {
    TRACY_FUNC(sceKernelDeleteEventFlag, event_id);
    return delete_sync_object<EventFlag>(emuenv.kernel, emuenv.kernel.eventflags, event_id, SCE_KERNEL_ERROR_UNKNOWN_EVF_ID);
}

EXPORT(SceInt32, sceKernelDeleteMsgPipe, SceUID msgPipeId) {
    TRACY_FUNC(sceKernelDeleteMsgPipe, msgPipeId);
    auto pipe = lock_and_find(msgPipeId, emuenv.kernel.msgpipes, emuenv.kernel.mutex);
    if (!pipe)
        return RET_ERROR(SCE_KERNEL_ERROR_UNKNOWN_MSG_PIPE_ID);
    pipe->mark_deleted_and_drain();
    std::lock_guard<std::mutex> _l(emuenv.kernel.mutex);
    emuenv.kernel.msgpipes.erase(msgPipeId);
    return (SceInt32)SCE_KERNEL_OK;
}

EXPORT(int, sceKernelDeleteMutex, SceUID mutexid) {
    TRACY_FUNC(sceKernelDeleteMutex, mutexid);
    return delete_sync_object<Mutex>(emuenv.kernel, emuenv.kernel.mutexes, mutexid, SCE_KERNEL_ERROR_UNKNOWN_MUTEX_ID);
}

EXPORT(SceInt32, sceKernelDeleteRWLock, SceUID lock_id) {
    TRACY_FUNC(sceKernelDeleteRWLock, lock_id);
    return delete_sync_object<RWLock>(emuenv.kernel, emuenv.kernel.rwlocks, lock_id, SCE_KERNEL_ERROR_UNKNOWN_RW_LOCK_ID);
}

EXPORT(int, sceKernelDeleteSema, SceUID semaid) {
    TRACY_FUNC(sceKernelDeleteSema, semaid);
    return delete_sync_object<Semaphore>(emuenv.kernel, emuenv.kernel.semaphores, semaid, SCE_KERNEL_ERROR_UNKNOWN_SEMA_ID);
}

EXPORT(int, sceKernelDeleteSimpleEvent, SceUID event_id) {
    TRACY_FUNC(sceKernelDeleteSimpleEvent, event_id);
    return delete_sync_object<SimpleEvent>(emuenv.kernel, emuenv.kernel.simple_events, event_id, SCE_KERNEL_ERROR_UNKNOWN_EVENT_ID);
}

EXPORT(int, sceKernelDeleteThread, SceUID thid) {
    TRACY_FUNC(sceKernelDeleteThread, thid);
    const ThreadStatePtr thread = emuenv.kernel.get_thread(thid);
    if (!thread)
        return RET_ERROR(SCE_KERNEL_ERROR_UNKNOWN_THREAD_ID);
    {
        std::lock_guard lock(thread->mutex);
        if (thread->status != ThreadStatus::dormant)
            return RET_ERROR(SCE_KERNEL_ERROR_NOT_DORMANT);
    }
    thread->request_host_thread_exit();
    emuenv.kernel.wait_thread_deleted(thid);
    return 0;
}

EXPORT(int, sceKernelDeleteTimer, SceUID timer_handle) {
    TRACY_FUNC(sceKernelDeleteTimer, timer_handle);
    return delete_sync_object<Timer>(emuenv.kernel, emuenv.kernel.timers, timer_handle, SCE_KERNEL_ERROR_UNKNOWN_TIMER_ID);
}

EXPORT(int, sceKernelExitDeleteThread, int status) {
    TRACY_FUNC(sceKernelExitDeleteThread, status);
    const ThreadStatePtr thread = emuenv.kernel.get_thread(thread_id);
    thread->exit_delete(status);

    return status;
}

EXPORT(SceInt32, sceKernelGetCallbackCount, SceUID callbackId) {
    TRACY_FUNC(sceKernelGetCallbackCount, callbackId);
    const CallbackPtr cb = lock_and_find(callbackId, emuenv.kernel.callbacks, emuenv.kernel.mutex);

    if (!cb)
        return RET_ERROR(SCE_KERNEL_ERROR_UNKNOWN_CALLBACK_ID);

    return cb->info().num_notifications;
}

EXPORT(int, sceKernelGetMsgPipeCreatorId) {
    TRACY_FUNC(sceKernelGetMsgPipeCreatorId);
    return UNIMPLEMENTED();
}

EXPORT(int, sceKernelGetProcessId) {
    TRACY_FUNC(sceKernelGetProcessId);
    STUBBED("pid: 1");
    return 1;
}

EXPORT(uint64_t, sceKernelGetSystemTimeWide) {
    TRACY_FUNC(sceKernelGetSystemTimeWide);
    return get_current_time();
}

EXPORT(SceInt32, sceKernelGetThreadCpuAffinityMask, SceUID thid) {
    TRACY_FUNC(sceKernelGetThreadCpuAffinityMask, thid);
    return CALL_EXPORT(_sceKernelGetThreadCpuAffinityMask, thid);
}

EXPORT(int, sceKernelGetThreadStackFreeSize) {
    TRACY_FUNC(sceKernelGetThreadStackFreeSize);
    return UNIMPLEMENTED();
}

EXPORT(Ptr<void>, sceKernelGetThreadTLSAddr, SceUID thid, int key) {
    TRACY_FUNC(sceKernelGetThreadTLSAddr, thid, key);
    return emuenv.kernel.get_thread_tls_addr(emuenv.mem, thid, key);
}

EXPORT(int, sceKernelGetThreadmgrUIDClass) {
    TRACY_FUNC(sceKernelGetThreadmgrUIDClass);
    return UNIMPLEMENTED();
}

EXPORT(uint64_t, sceKernelGetTimerBaseWide, SceUID timer_handle) {
    TRACY_FUNC(sceKernelGetTimerBaseWide, timer_handle);
    const TimerPtr timer_info = lock_and_find(timer_handle, emuenv.kernel.timers, emuenv.kernel.mutex);

    if (!timer_info)
        return RET_ERROR(SCE_KERNEL_ERROR_UNKNOWN_TIMER_ID);

    return timer_info->start_time();
}

EXPORT(uint64_t, sceKernelGetTimerTimeWide, SceUID timer_handle) {
    TRACY_FUNC(sceKernelGetTimerTimeWide, timer_handle);
    const TimerPtr timer_info = lock_and_find(timer_handle, emuenv.kernel.timers, emuenv.kernel.mutex);

    if (!timer_info)
        return RET_ERROR(SCE_KERNEL_ERROR_UNKNOWN_TIMER_ID);

    return get_current_time() - timer_info->start_time();
}

EXPORT(SceInt32, sceKernelNotifyCallback, SceUID callbackId, SceInt32 notifyArg) {
    TRACY_FUNC(sceKernelNotifyCallback, callbackId, notifyArg);
    const CallbackPtr cb = lock_and_find(callbackId, emuenv.kernel.callbacks, emuenv.kernel.mutex);
    if (!cb)
        return RET_ERROR(SCE_KERNEL_ERROR_UNKNOWN_CALLBACK_ID);

    cb->notify(emuenv.kernel, SCE_UID_INVALID_UID, notifyArg);

    return SCE_KERNEL_OK;
}

EXPORT(SceUID, sceKernelOpenCond, const char *pName) {
    TRACY_FUNC(sceKernelOpenCond, pName);
    return find_sync_object_by_name<Condvar>(emuenv.kernel, emuenv.kernel.condvars, pName);
}

EXPORT(SceUID, sceKernelOpenEventFlag, const char *pName) {
    TRACY_FUNC(sceKernelOpenEventFlag, pName);
    return find_sync_object_by_name<EventFlag>(emuenv.kernel, emuenv.kernel.eventflags, pName);
}

EXPORT(SceUID, sceKernelOpenMsgPipe, const char *pName) {
    TRACY_FUNC(sceKernelOpenMsgPipe, pName);
    return find_sync_object_by_name<MsgPipe>(emuenv.kernel, emuenv.kernel.msgpipes, pName);
}

EXPORT(SceUID, sceKernelOpenMutex, const char *pName) {
    TRACY_FUNC(sceKernelOpenMutex, pName);
    return find_sync_object_by_name<Mutex>(emuenv.kernel, emuenv.kernel.mutexes, pName);
}

EXPORT(int, sceKernelOpenMutex_089) {
    TRACY_FUNC(sceKernelOpenMutex_089);
    return UNIMPLEMENTED();
}

EXPORT(SceUID, sceKernelOpenRWLock, const char *pName) {
    TRACY_FUNC(sceKernelOpenRWLock, pName);
    return find_sync_object_by_name<RWLock>(emuenv.kernel, emuenv.kernel.rwlocks, pName);
}

EXPORT(SceUID, sceKernelOpenSema, const char *pName) {
    TRACY_FUNC(sceKernelOpenSema, pName);
    return find_sync_object_by_name<Semaphore>(emuenv.kernel, emuenv.kernel.semaphores, pName);
}

EXPORT(SceUID, sceKernelOpenSimpleEvent, const char *pName) {
    TRACY_FUNC(sceKernelOpenSimpleEvent, pName);
    return find_sync_object_by_name<SimpleEvent>(emuenv.kernel, emuenv.kernel.simple_events, pName);
}

EXPORT(SceUID, sceKernelOpenTimer, const char *pName) {
    TRACY_FUNC(sceKernelOpenTimer, pName);
    return find_sync_object_by_name<Timer>(emuenv.kernel, emuenv.kernel.timers, pName);
}

EXPORT(SceInt32, sceKernelPollSema, SceUID semaid, SceInt32 needCount) {
    TRACY_FUNC(sceKernelPollSema, semaid, needCount);
    if (needCount < 0)
        return RET_ERROR(SCE_KERNEL_ERROR_INVALID_ARGUMENT);
    const SemaphorePtr semaphore = lock_and_find(semaid, emuenv.kernel.semaphores, emuenv.kernel.mutex);
    if (!semaphore)
        return RET_ERROR(SCE_KERNEL_ERROR_UNKNOWN_SEMA_ID);
    return semaphore->poll(needCount);
}

EXPORT(SceInt32, sceKernelPulseEvent, SceUID event_id, SceUInt32 set_pattern, SceUInt64 user_data) {
    TRACY_FUNC(sceKernelPulseEvent, event_id, set_pattern, user_data);
    auto ev = lock_and_find(event_id, emuenv.kernel.simple_events, emuenv.kernel.mutex);
    if (!ev)
        return RET_ERROR(SCE_KERNEL_ERROR_UNKNOWN_EVENT_ID);
    return ev->set_or_pulse(set_pattern, user_data, false);
}

EXPORT(int, sceKernelRegisterCallbackToEvent) {
    TRACY_FUNC(sceKernelRegisterCallbackToEvent);
    return UNIMPLEMENTED();
}

EXPORT(int, sceKernelResumeThreadForVM, SceUID threadId) {
    TRACY_FUNC(sceKernelResumeThreadForVM, threadId);
    STUBBED("STUB");

    const ThreadStatePtr thread = emuenv.kernel.get_thread(threadId);
    if (!thread)
        return RET_ERROR(SCE_KERNEL_ERROR_UNKNOWN_THREAD_ID);

    thread->resume();

    return 0;
}

EXPORT(int, sceKernelSendSignal, SceUID target_thread_id) {
    TRACY_FUNC(sceKernelSendSignal, target_thread_id);
    const auto thread = emuenv.kernel.get_thread(target_thread_id);
    {
        std::lock_guard lock(thread->mutex);
        if (thread->signal_pending)
            return SCE_KERNEL_ERROR_ALREADY_SENT;
        thread->signal_pending = true;
    }
    thread->unpark();
    return SCE_KERNEL_OK;
}

EXPORT(SceInt32, sceKernelSetEvent, SceUID event_id, SceUInt32 set_pattern, SceUInt64 user_data) {
    TRACY_FUNC(sceKernelSetEvent, event_id, set_pattern, user_data);
    auto ev = lock_and_find(event_id, emuenv.kernel.simple_events, emuenv.kernel.mutex);
    if (!ev)
        return RET_ERROR(SCE_KERNEL_ERROR_UNKNOWN_EVENT_ID);
    return ev->set_or_pulse(set_pattern, user_data, true);
}

EXPORT(SceInt32, sceKernelSetEventFlag, SceUID evfId, SceUInt32 bitPattern) {
    TRACY_FUNC(sceKernelSetEventFlag, evfId, bitPattern);
    auto ef = lock_and_find(evfId, emuenv.kernel.eventflags, emuenv.kernel.mutex);
    if (!ef)
        return RET_ERROR(SCE_KERNEL_ERROR_UNKNOWN_EVF_ID);
    return ef->set(bitPattern);
}

EXPORT(int, sceKernelSetTimerTimeWide, SceUID timer_handle, SceUInt64 time) {
    TRACY_FUNC(sceKernelSetTimerTimeWide, timer_handle, time);
    const TimerPtr timer_info = lock_and_find(timer_handle, emuenv.kernel.timers, emuenv.kernel.mutex);
    if (!timer_info)
        return RET_ERROR(SCE_KERNEL_ERROR_UNKNOWN_TIMER_ID);

    const auto old_time = timer_info->start_time();
    timer_info->set_start_time(time);
    return old_time;
}

EXPORT(int, sceKernelSignalCond, SceUID condid) {
    TRACY_FUNC(sceKernelSignalCond, condid);
    auto cv = lock_and_find(condid, emuenv.kernel.condvars, emuenv.kernel.mutex);
    if (!cv)
        return RET_ERROR(SCE_KERNEL_ERROR_UNKNOWN_COND_ID);
    return cv->signal(CondvarSignalTarget(CondvarSignalTarget::Type::Any));
}

EXPORT(int, sceKernelSignalCondAll, SceUID condid) {
    TRACY_FUNC(sceKernelSignalCondAll, condid);
    auto cv = lock_and_find(condid, emuenv.kernel.condvars, emuenv.kernel.mutex);
    if (!cv)
        return RET_ERROR(SCE_KERNEL_ERROR_UNKNOWN_COND_ID);
    return cv->signal(CondvarSignalTarget(CondvarSignalTarget::Type::All));
}

EXPORT(int, sceKernelSignalCondTo, SceUID condid, SceUID thread_target) {
    TRACY_FUNC(sceKernelSignalCondTo, condid, thread_target);
    auto cv = lock_and_find(condid, emuenv.kernel.condvars, emuenv.kernel.mutex);
    if (!cv)
        return RET_ERROR(SCE_KERNEL_ERROR_UNKNOWN_COND_ID);
    return cv->signal(CondvarSignalTarget(CondvarSignalTarget::Type::Specific, thread_target));
}

EXPORT(int, sceKernelSignalSema, SceUID semaid, int signal) {
    TRACY_FUNC(sceKernelSignalSema, semaid, signal);
    auto sema = lock_and_find(semaid, emuenv.kernel.semaphores, emuenv.kernel.mutex);
    if (!sema)
        return RET_ERROR(SCE_KERNEL_ERROR_UNKNOWN_SEMA_ID);
    return sema->signal(signal);
}

EXPORT(int, sceKernelStartTimer, SceUID timer_handle) {
    TRACY_FUNC(sceKernelStartTimer, timer_handle);
    auto timer = lock_and_find(timer_handle, emuenv.kernel.timers, emuenv.kernel.mutex);
    if (!timer)
        return RET_ERROR(SCE_KERNEL_ERROR_UNKNOWN_TIMER_ID);
    return timer->start();
}

EXPORT(int, sceKernelStopTimer, SceUID timer_handle) {
    TRACY_FUNC(sceKernelStopTimer, timer_handle);
    auto timer = lock_and_find(timer_handle, emuenv.kernel.timers, emuenv.kernel.mutex);
    if (!timer)
        return RET_ERROR(SCE_KERNEL_ERROR_UNKNOWN_TIMER_ID);
    return timer->stop();
}

EXPORT(int, sceKernelSuspendThreadForVM, SceUID threadId) {
    TRACY_FUNC(sceKernelSuspendThreadForVM, threadId);
    STUBBED("STUB");

    const ThreadStatePtr thread = emuenv.kernel.get_thread(threadId);
    if (!thread)
        return RET_ERROR(SCE_KERNEL_ERROR_UNKNOWN_THREAD_ID);

    thread->suspend();

    return 0;
}

EXPORT(int, sceKernelTryLockMutex, SceUID mutexid, int lock_count) {
    TRACY_FUNC(sceKernelTryLockMutex, mutexid, lock_count);
    auto mutex = lock_and_find(mutexid, emuenv.kernel.mutexes, emuenv.kernel.mutex);
    if (!mutex)
        return RET_ERROR(SCE_KERNEL_ERROR_UNKNOWN_MUTEX_ID);
    return mutex->try_lock(emuenv.kernel.get_thread(thread_id), lock_count, emuenv.mem);
}

EXPORT(SceInt32, sceKernelTryLockReadRWLock, SceUID lock_id) {
    TRACY_FUNC(sceKernelTryLockReadRWLock, lock_id);
    auto rwlock = lock_and_find(lock_id, emuenv.kernel.rwlocks, emuenv.kernel.mutex);
    if (!rwlock)
        return RET_ERROR(SCE_KERNEL_ERROR_UNKNOWN_RW_LOCK_ID);
    return rwlock->try_lock(emuenv.kernel.get_thread(thread_id), false);
}

EXPORT(SceInt32, sceKernelTryLockWriteRWLock, SceUID lock_id) {
    TRACY_FUNC(sceKernelTryLockWriteRWLock, lock_id);
    auto rwlock = lock_and_find(lock_id, emuenv.kernel.rwlocks, emuenv.kernel.mutex);
    if (!rwlock)
        return RET_ERROR(SCE_KERNEL_ERROR_UNKNOWN_RW_LOCK_ID);
    return rwlock->try_lock(emuenv.kernel.get_thread(thread_id), true);
}

EXPORT(int, sceKernelUnlockMutex, SceUID mutexid, int unlock_count) {
    TRACY_FUNC(sceKernelUnlockMutex, mutexid, unlock_count);
    auto mutex = lock_and_find(mutexid, emuenv.kernel.mutexes, emuenv.kernel.mutex);
    if (!mutex)
        return RET_ERROR(SCE_KERNEL_ERROR_UNKNOWN_MUTEX_ID);
    return mutex->unlock(emuenv.kernel.get_thread(thread_id), unlock_count, emuenv.mem);
}

EXPORT(int, sceKernelUnlockReadRWLock, SceUID lock_id) {
    TRACY_FUNC(sceKernelUnlockReadRWLock, lock_id);
    auto rwlock = lock_and_find(lock_id, emuenv.kernel.rwlocks, emuenv.kernel.mutex);
    if (!rwlock)
        return RET_ERROR(SCE_KERNEL_ERROR_UNKNOWN_RW_LOCK_ID);
    return rwlock->unlock(emuenv.kernel.get_thread(thread_id), false);
}

EXPORT(int, sceKernelUnlockWriteRWLock, SceUID lock_id) {
    TRACY_FUNC(sceKernelUnlockWriteRWLock, lock_id);
    auto rwlock = lock_and_find(lock_id, emuenv.kernel.rwlocks, emuenv.kernel.mutex);
    if (!rwlock)
        return RET_ERROR(SCE_KERNEL_ERROR_UNKNOWN_RW_LOCK_ID);
    return rwlock->unlock(emuenv.kernel.get_thread(thread_id), true);
}

EXPORT(int, sceKernelUnregisterCallbackFromEvent) {
    TRACY_FUNC(sceKernelUnregisterCallbackFromEvent);
    return UNIMPLEMENTED();
}

EXPORT(int, sceKernelUnregisterCallbackFromEventAll) {
    TRACY_FUNC(sceKernelUnregisterCallbackFromEventAll);
    return UNIMPLEMENTED();
}

EXPORT(int, sceKernelUnregisterThreadEventHandler) {
    TRACY_FUNC(sceKernelUnregisterThreadEventHandler);
    return UNIMPLEMENTED();
}

EXPORT(int, sceKernelWaitThreadEndCB_089) {
    TRACY_FUNC(sceKernelWaitThreadEndCB_089);
    return UNIMPLEMENTED();
}

EXPORT(int, sceKernelWaitThreadEnd_089) {
    TRACY_FUNC(sceKernelWaitThreadEnd_089);
    return UNIMPLEMENTED();
}
