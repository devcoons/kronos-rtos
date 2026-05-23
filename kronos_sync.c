/*!
@file   kronos_sync.c
@brief  Source file of KronOS (RTOS) synchronization functionalities
@t.odo  -
*/

/******************************************************************************
* Includes
******************************************************************************/

#include "kronos_internal.h"

#include <stddef.h>

/******************************************************************************
* Preprocessor Definitions & Macros
******************************************************************************/

#define KRONOS_MUTEX_INITIALIZED_MAGIC      0x4B4D5458UL
#define KRONOS_SEMAPHORE_INITIALIZED_MAGIC 0x4B53454DUL

#ifndef KRONOS_SYNC_REGISTRY_SIZE
#define KRONOS_SYNC_REGISTRY_SIZE 16U
#endif

/******************************************************************************
* Enumerations, Structures & Variables
******************************************************************************/

static kronos_mutex_t *g_registeredMutexes[KRONOS_SYNC_REGISTRY_SIZE];
static kronos_semaphore_t *g_registeredSemaphores[KRONOS_SYNC_REGISTRY_SIZE];

/******************************************************************************
* Declaration | Static Functions
******************************************************************************/

static int32_t kronos_sync_wake_waiter(kronos_wait_reason_e waitReason, void *waitObject);
static void kronos_sync_set_outcome(kronos_service_outcome_t *outcome, kronos_status_e status, uint32_t switchRequired);
static kronos_status_e kronos_sync_register_mutex(kronos_mutex_t *mutex);
static kronos_status_e kronos_sync_register_semaphore(kronos_semaphore_t *semaphore);
static uint32_t kronos_sync_mutex_is_registered(const kronos_mutex_t *mutex);
static uint32_t kronos_sync_semaphore_is_registered(const kronos_semaphore_t *semaphore);
static uint32_t kronos_sync_mutex_is_valid(const kronos_mutex_t *mutex);
static uint32_t kronos_sync_semaphore_is_valid(const kronos_semaphore_t *semaphore);
static uint32_t kronos_sync_has_waiter(kronos_wait_reason_e waitReason, const void *waitObject);
static void kronos_sync_release_owned_mutex(kronos_mutex_t *mutex, kronos_task_id_t taskId);

/******************************************************************************
* Definition | Static Functions
******************************************************************************/

static int32_t kronos_sync_wake_waiter(kronos_wait_reason_e waitReason, void *waitObject)
{
    int32_t waitingTaskIndex;

    waitingTaskIndex = Kronos_TaskFindWaiting(waitReason, waitObject, g_currentTask);
    if (waitingTaskIndex < 0)
    {
        return -1;
    }

    Kronos_TaskUnblock(&g_tasks[waitingTaskIndex]);
    if (g_schedulerSuspendDepth != 0U)
    {
        g_schedulerSwitchPending = 1U;
    }

    return waitingTaskIndex;
}

static void kronos_sync_set_outcome(kronos_service_outcome_t *outcome, kronos_status_e status, uint32_t switchRequired)
{
    if (outcome != NULL)
    {
        outcome->status = status;
        outcome->switch_required = switchRequired;
    }
}

static kronos_status_e kronos_sync_register_mutex(kronos_mutex_t *mutex)
{
    uint32_t slotIndex;

    for (slotIndex = 0U; slotIndex < KRONOS_SYNC_REGISTRY_SIZE; ++slotIndex)
    {
        if (g_registeredMutexes[slotIndex] == mutex)
        {
            return KRONOS_STATUS_OK;
        }
    }

    for (slotIndex = 0U; slotIndex < KRONOS_SYNC_REGISTRY_SIZE; ++slotIndex)
    {
        if (g_registeredMutexes[slotIndex] == NULL)
        {
            g_registeredMutexes[slotIndex] = mutex;
            return KRONOS_STATUS_OK;
        }
    }

    return KRONOS_STATUS_NO_MEMORY;
}

static kronos_status_e kronos_sync_register_semaphore(kronos_semaphore_t *semaphore)
{
    uint32_t slotIndex;

    for (slotIndex = 0U; slotIndex < KRONOS_SYNC_REGISTRY_SIZE; ++slotIndex)
    {
        if (g_registeredSemaphores[slotIndex] == semaphore)
        {
            return KRONOS_STATUS_OK;
        }
    }

    for (slotIndex = 0U; slotIndex < KRONOS_SYNC_REGISTRY_SIZE; ++slotIndex)
    {
        if (g_registeredSemaphores[slotIndex] == NULL)
        {
            g_registeredSemaphores[slotIndex] = semaphore;
            return KRONOS_STATUS_OK;
        }
    }

    return KRONOS_STATUS_NO_MEMORY;
}

static uint32_t kronos_sync_mutex_is_registered(const kronos_mutex_t *mutex)
{
    uint32_t slotIndex;

    for (slotIndex = 0U; slotIndex < KRONOS_SYNC_REGISTRY_SIZE; ++slotIndex)
    {
        if (g_registeredMutexes[slotIndex] == mutex)
        {
            return 1U;
        }
    }

    return 0U;
}

static uint32_t kronos_sync_semaphore_is_registered(const kronos_semaphore_t *semaphore)
{
    uint32_t slotIndex;

    for (slotIndex = 0U; slotIndex < KRONOS_SYNC_REGISTRY_SIZE; ++slotIndex)
    {
        if (g_registeredSemaphores[slotIndex] == semaphore)
        {
            return 1U;
        }
    }

    return 0U;
}

static uint32_t kronos_sync_mutex_is_valid(const kronos_mutex_t *mutex)
{
    return (mutex != NULL) &&
           (kronos_sync_mutex_is_registered(mutex) != 0U) &&
           (mutex->initialized == KRONOS_MUTEX_INITIALIZED_MAGIC) &&
           (((mutex->lock_count == 0U) && (mutex->owner_task_id == KRONOS_INVALID_TASK_ID)) ||
            ((mutex->lock_count != 0U) && (mutex->owner_task_id < g_numTasks)));
}

static uint32_t kronos_sync_semaphore_is_valid(const kronos_semaphore_t *semaphore)
{
    return (semaphore != NULL) &&
           (kronos_sync_semaphore_is_registered(semaphore) != 0U) &&
           (semaphore->initialized == KRONOS_SEMAPHORE_INITIALIZED_MAGIC) &&
           (semaphore->max_count != 0U) &&
           (semaphore->count <= semaphore->max_count);
}

static uint32_t kronos_sync_has_waiter(kronos_wait_reason_e waitReason, const void *waitObject)
{
    return (Kronos_TaskFindWaiting(waitReason, waitObject, g_currentTask) >= 0) ? 1U : 0U;
}

static void kronos_sync_release_owned_mutex(kronos_mutex_t *mutex, kronos_task_id_t taskId)
{
    int32_t waitingTaskIndex;

    if ((mutex == NULL) || (mutex->owner_task_id != taskId) || (mutex->lock_count == 0U))
    {
        return;
    }

    mutex->owner_task_id = KRONOS_INVALID_TASK_ID;
    mutex->lock_count = 0U;

    waitingTaskIndex = kronos_sync_wake_waiter(KRONOS_WAIT_MUTEX, mutex);
    if (waitingTaskIndex >= 0)
    {
        mutex->owner_task_id = (uint32_t)waitingTaskIndex;
        mutex->lock_count = 1U;
    }
}

/******************************************************************************
* Definition | Public Functions
******************************************************************************/

void Kronos_SyncResetState(void)
{
    uint32_t slotIndex;

    for (slotIndex = 0U; slotIndex < KRONOS_SYNC_REGISTRY_SIZE; ++slotIndex)
    {
        g_registeredMutexes[slotIndex] = NULL;
        g_registeredSemaphores[slotIndex] = NULL;
    }
}

void Kronos_SyncCleanupTask(kronos_task_id_t taskId)
{
    uint32_t slotIndex;

    if (taskId >= g_numTasks)
    {
        return;
    }

    for (slotIndex = 0U; slotIndex < KRONOS_SYNC_REGISTRY_SIZE; ++slotIndex)
    {
        if (kronos_sync_mutex_is_valid(g_registeredMutexes[slotIndex]) != 0U)
        {
            kronos_sync_release_owned_mutex(g_registeredMutexes[slotIndex], taskId);
        }
    }
}

kronos_status_e RTOS_MutexInit(kronos_mutex_t *mutex)
{
    kronos_status_e status;

    if (mutex == NULL)
    {
        return KRONOS_STATUS_INVALID_ARGUMENT;
    }

    if ((kronos_sync_mutex_is_registered(mutex) != 0U) &&
        (mutex->initialized == KRONOS_MUTEX_INITIALIZED_MAGIC) &&
        ((mutex->lock_count != 0U) || (kronos_sync_has_waiter(KRONOS_WAIT_MUTEX, mutex) != 0U)))
    {
        return KRONOS_STATUS_IN_USE;
    }

    mutex->owner_task_id = KRONOS_INVALID_TASK_ID;
    mutex->lock_count = 0U;
    mutex->initialized = KRONOS_MUTEX_INITIALIZED_MAGIC;

    status = kronos_sync_register_mutex(mutex);
    if (status != KRONOS_STATUS_OK)
    {
        mutex->initialized = 0U;
    }

    return status;
}

kronos_status_e RTOS_MutexLock(kronos_mutex_t *mutex)
{
    if (kronos_sync_mutex_is_valid(mutex) == 0U)
    {
        return KRONOS_STATUS_INVALID_ARGUMENT;
    }

    if ((g_schedulerStarted == 0U) || (g_currentTask >= g_numTasks))
    {
        return KRONOS_STATUS_ERROR;
    }

    Kronos_SchedulerRequestService(KRONOS_SERVICE_MUTEX_LOCK, mutex, 0U);
    return g_taskRuntime[g_currentTask].service_result;
}

kronos_status_e RTOS_MutexUnlock(kronos_mutex_t *mutex)
{
    if (kronos_sync_mutex_is_valid(mutex) == 0U)
    {
        return KRONOS_STATUS_INVALID_ARGUMENT;
    }

    if ((g_schedulerStarted == 0U) || (g_currentTask >= g_numTasks))
    {
        return KRONOS_STATUS_ERROR;
    }

    Kronos_SchedulerRequestService(KRONOS_SERVICE_MUTEX_UNLOCK, mutex, 0U);
    return g_taskRuntime[g_currentTask].service_result;
}

kronos_status_e RTOS_SemaphoreInit(kronos_semaphore_t *semaphore, uint32_t initialCount, uint32_t maxCount)
{
    kronos_status_e status;

    if ((semaphore == NULL) || (maxCount == 0U) || (initialCount > maxCount))
    {
        return KRONOS_STATUS_INVALID_ARGUMENT;
    }

    if ((kronos_sync_semaphore_is_registered(semaphore) != 0U) &&
        (semaphore->initialized == KRONOS_SEMAPHORE_INITIALIZED_MAGIC) &&
        (kronos_sync_has_waiter(KRONOS_WAIT_SEMAPHORE, semaphore) != 0U))
    {
        return KRONOS_STATUS_IN_USE;
    }

    semaphore->count = initialCount;
    semaphore->max_count = maxCount;
    semaphore->initialized = KRONOS_SEMAPHORE_INITIALIZED_MAGIC;

    status = kronos_sync_register_semaphore(semaphore);
    if (status != KRONOS_STATUS_OK)
    {
        semaphore->initialized = 0U;
    }

    return status;
}

kronos_status_e RTOS_SemaphoreTake(kronos_semaphore_t *semaphore)
{
    if (kronos_sync_semaphore_is_valid(semaphore) == 0U)
    {
        return KRONOS_STATUS_INVALID_ARGUMENT;
    }

    if ((g_schedulerStarted == 0U) || (g_currentTask >= g_numTasks))
    {
        return KRONOS_STATUS_ERROR;
    }

    Kronos_SchedulerRequestService(KRONOS_SERVICE_SEMAPHORE_TAKE, semaphore, 0U);
    return g_taskRuntime[g_currentTask].service_result;
}

kronos_status_e RTOS_SemaphoreGive(kronos_semaphore_t *semaphore)
{
    if (kronos_sync_semaphore_is_valid(semaphore) == 0U)
    {
        return KRONOS_STATUS_INVALID_ARGUMENT;
    }

    if ((g_schedulerStarted == 0U) || (g_currentTask >= g_numTasks))
    {
        return KRONOS_STATUS_ERROR;
    }

    Kronos_SchedulerRequestService(KRONOS_SERVICE_SEMAPHORE_GIVE, semaphore, 0U);
    return g_taskRuntime[g_currentTask].service_result;
}

void Kronos_SyncMutexLock(TCB_t *currentTcb, kronos_mutex_t *mutex, kronos_service_outcome_t *outcome)
{
    if ((currentTcb == NULL) || (kronos_sync_mutex_is_valid(mutex) == 0U))
    {
        kronos_sync_set_outcome(outcome, KRONOS_STATUS_INVALID_ARGUMENT, 0U);
        return;
    }

    if ((mutex->lock_count != 0U) && (mutex->owner_task_id == g_currentTask))
    {
        if (mutex->lock_count == UINT32_MAX)
        {
            kronos_sync_set_outcome(outcome, KRONOS_STATUS_OVERFLOW, 0U);
            return;
        }

        mutex->lock_count++;
        kronos_sync_set_outcome(outcome, KRONOS_STATUS_OK, 0U);
        return;
    }

    if (mutex->lock_count == 0U)
    {
        mutex->owner_task_id = g_currentTask;
        mutex->lock_count = 1U;
        kronos_sync_set_outcome(outcome, KRONOS_STATUS_OK, 0U);
        return;
    }

    if (g_schedulerSuspendDepth != 0U)
    {
        kronos_sync_set_outcome(outcome, KRONOS_STATUS_SCHEDULER_SUSPENDED, 0U);
        return;
    }

    Kronos_TaskBlock(currentTcb, KRONOS_WAIT_MUTEX, mutex);
    kronos_sync_set_outcome(outcome, KRONOS_STATUS_OK, 0U);
}

void Kronos_SyncMutexUnlock(TCB_t *currentTcb, kronos_mutex_t *mutex, kronos_service_outcome_t *outcome)
{
    int32_t waitingTaskIndex;

    if ((currentTcb == NULL) || (kronos_sync_mutex_is_valid(mutex) == 0U))
    {
        kronos_sync_set_outcome(outcome, KRONOS_STATUS_INVALID_ARGUMENT, 0U);
        return;
    }

    if ((mutex->lock_count == 0U) || (mutex->owner_task_id != g_currentTask))
    {
        kronos_sync_set_outcome(outcome, KRONOS_STATUS_NOT_OWNER, 0U);
        return;
    }

    mutex->lock_count--;
    if (mutex->lock_count != 0U)
    {
        kronos_sync_set_outcome(outcome, KRONOS_STATUS_OK, 0U);
        return;
    }

    mutex->owner_task_id = KRONOS_INVALID_TASK_ID;

    waitingTaskIndex = kronos_sync_wake_waiter(KRONOS_WAIT_MUTEX, mutex);
    if (waitingTaskIndex >= 0)
    {
        mutex->owner_task_id = (uint32_t)waitingTaskIndex;
        mutex->lock_count = 1U;
        kronos_sync_set_outcome(outcome, KRONOS_STATUS_OK, 1U);
        return;
    }

    kronos_sync_set_outcome(outcome, KRONOS_STATUS_OK, 0U);
}

void Kronos_SyncSemaphoreTake(TCB_t *currentTcb, kronos_semaphore_t *semaphore, kronos_service_outcome_t *outcome)
{
    if ((currentTcb == NULL) || (kronos_sync_semaphore_is_valid(semaphore) == 0U))
    {
        kronos_sync_set_outcome(outcome, KRONOS_STATUS_INVALID_ARGUMENT, 0U);
        return;
    }

    if (semaphore->count > 0U)
    {
        semaphore->count--;
        kronos_sync_set_outcome(outcome, KRONOS_STATUS_OK, 0U);
        return;
    }

    if (g_schedulerSuspendDepth != 0U)
    {
        kronos_sync_set_outcome(outcome, KRONOS_STATUS_SCHEDULER_SUSPENDED, 0U);
        return;
    }

    Kronos_TaskBlock(currentTcb, KRONOS_WAIT_SEMAPHORE, semaphore);
    kronos_sync_set_outcome(outcome, KRONOS_STATUS_OK, 0U);
}

void Kronos_SyncSemaphoreGive(kronos_semaphore_t *semaphore, kronos_service_outcome_t *outcome)
{
    if (kronos_sync_semaphore_is_valid(semaphore) == 0U)
    {
        kronos_sync_set_outcome(outcome, KRONOS_STATUS_INVALID_ARGUMENT, 0U);
        return;
    }

    if (kronos_sync_wake_waiter(KRONOS_WAIT_SEMAPHORE, semaphore) >= 0)
    {
        kronos_sync_set_outcome(outcome, KRONOS_STATUS_OK, 1U);
        return;
    }

    if (semaphore->count >= semaphore->max_count)
    {
        kronos_sync_set_outcome(outcome, KRONOS_STATUS_OVERFLOW, 0U);
        return;
    }

    semaphore->count++;
    kronos_sync_set_outcome(outcome, KRONOS_STATUS_OK, 0U);
}

/******************************************************************************
* EOF - NO CODE AFTER THIS LINE
******************************************************************************/
