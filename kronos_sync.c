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

/******************************************************************************
* Declaration | Static Functions
******************************************************************************/

static int32_t kronos_sync_wake_waiter(kronos_wait_reason_e waitReason, void *waitObject);
static void kronos_sync_set_outcome(kronos_service_outcome_t *outcome, int32_t status, uint32_t switchRequired);

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

static void kronos_sync_set_outcome(kronos_service_outcome_t *outcome, int32_t status, uint32_t switchRequired)
{
    if (outcome != NULL)
    {
        outcome->status = status;
        outcome->switch_required = switchRequired;
    }
}

/******************************************************************************
* Definition | Public Functions
******************************************************************************/

int32_t RTOS_MutexInit(kronos_mutex_t *mutex)
{
    if (mutex == NULL)
    {
        return KRONOS_STATUS_INVALID_ARGUMENT;
    }

    mutex->owner_task_id = KRONOS_INVALID_TASK_ID;
    mutex->lock_count = 0U;

    return KRONOS_STATUS_OK;
}

int32_t RTOS_MutexLock(kronos_mutex_t *mutex)
{
    if (mutex == NULL)
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

int32_t RTOS_MutexUnlock(kronos_mutex_t *mutex)
{
    if (mutex == NULL)
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

int32_t RTOS_SemaphoreInit(kronos_semaphore_t *semaphore, uint32_t initialCount, uint32_t maxCount)
{
    if ((semaphore == NULL) || (maxCount == 0U) || (initialCount > maxCount))
    {
        return KRONOS_STATUS_INVALID_ARGUMENT;
    }

    semaphore->count = initialCount;
    semaphore->max_count = maxCount;

    return KRONOS_STATUS_OK;
}

int32_t RTOS_SemaphoreTake(kronos_semaphore_t *semaphore)
{
    if (semaphore == NULL)
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

int32_t RTOS_SemaphoreGive(kronos_semaphore_t *semaphore)
{
    if (semaphore == NULL)
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
    if ((currentTcb == NULL) || (mutex == NULL))
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

    if ((currentTcb == NULL) || (mutex == NULL))
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
    if ((currentTcb == NULL) || (semaphore == NULL) || (semaphore->max_count == 0U) || (semaphore->count > semaphore->max_count))
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
    if ((semaphore == NULL) || (semaphore->max_count == 0U) || (semaphore->count > semaphore->max_count))
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