/*!
@file   kronos_scheduler.c
@brief  Source file of KronOS (RTOS) Scheduler functionalities
@t.odo  -

---------------------------------------------------------------------------
GNU Affero General Public License v3.0

Copyright (c) 2024 Ioannis D. (devcoons)

This program is free software: you can redistribute it and/or modify it
under the terms of the GNU Affero General Public License as published by
the Free Software Foundation, either version 3 of the License.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
GNU Affero General Public License for more details.

You should have received a copy of the GNU Affero General Public License
along with this program. If not, see <https://www.gnu.org/licenses/>.

For commercial use, including proprietary or for-profit applications,
a separate license is required. Contact:

- GitHub: [https://github.com/devcoons](https://github.com/devcoons)
- Email: i_-_-_s@outlook.com
*/

/******************************************************************************
* Includes
******************************************************************************/

#include "kronos_internal.h"

#include <stddef.h>

/******************************************************************************
* Enumerations, Structures & Variables
******************************************************************************/

volatile uint32_t g_tickCount = 0U;
uint32_t g_schedulerStarted = 0U;
volatile uint32_t g_schedulerSuspendDepth = 0U;
volatile uint32_t g_schedulerSwitchPending = 0U;

/******************************************************************************
* Declaration | Static Functions
******************************************************************************/

static void kronos_scheduler_store_current_context(uint32_t *savedStackPtr);
static uint32_t *kronos_scheduler_keep_current_running(uint32_t *savedStackPtr);
static void kronos_scheduler_clear_service_state(kronos_task_runtime_t *runtimeState);
static void kronos_scheduler_init_outcome(kronos_service_outcome_t *outcome);
static int32_t kronos_scheduler_select_next_ready_task(uint32_t startTask);
static uint32_t *kronos_scheduler_switch_or_continue(uint32_t *savedStackPtr);

/******************************************************************************
* Definition | Static Functions
******************************************************************************/

static void kronos_scheduler_store_current_context(uint32_t *savedStackPtr)
{
    g_tasks[g_currentTask].stack_top_ptr = savedStackPtr;
    Kronos_TaskUpdateStackMetrics(&g_tasks[g_currentTask], savedStackPtr);
}

static uint32_t *kronos_scheduler_keep_current_running(uint32_t *savedStackPtr)
{
    g_tasks[g_currentTask].task_state = TASK_STATE_RUNNING;
    return savedStackPtr;
}

static void kronos_scheduler_clear_service_state(kronos_task_runtime_t *runtimeState)
{
    runtimeState->service_request = KRONOS_SERVICE_NONE;
    runtimeState->service_object_ptr = NULL;
    runtimeState->service_parameter = 0U;
}

static void kronos_scheduler_init_outcome(kronos_service_outcome_t *outcome)
{
    if (outcome != NULL)
    {
        outcome->status = KRONOS_STATUS_OK;
        outcome->switch_required = 0U;
    }
}

static int32_t kronos_scheduler_select_next_ready_task(uint32_t startTask)
{
    int32_t nextTaskIndex;

    nextTaskIndex = Kronos_TaskFindNextReady(TASK_KIND_APPLICATION, startTask);
    if (nextTaskIndex < 0)
    {
        nextTaskIndex = Kronos_TaskFindNextReady(TASK_KIND_SYSTEM, startTask);
    }
    if (nextTaskIndex < 0)
    {
        nextTaskIndex = Kronos_TaskFindNextReady(TASK_KIND_IDLE, startTask);
    }

    return nextTaskIndex;
}

static uint32_t *kronos_scheduler_switch_or_continue(uint32_t *savedStackPtr)
{
    if ((g_schedulerSuspendDepth != 0U) && (g_tasks[g_currentTask].task_state == TASK_STATE_RUNNING))
    {
        g_schedulerSwitchPending = 1U;
        return kronos_scheduler_keep_current_running(savedStackPtr);
    }

    Scheduler_RoundRobin();
    return g_tasks[g_currentTask].stack_top_ptr;
}

/******************************************************************************
* Definition | Public Functions
******************************************************************************/

void Kronos_SchedulerResetState(void)
{
    g_tickCount = 0U;
    g_schedulerStarted = 0U;
    g_schedulerSuspendDepth = 0U;
    g_schedulerSwitchPending = 0U;
}

void Kronos_SchedulerRequestService(kronos_service_e serviceRequest, void *serviceObject, uint32_t serviceParameter)
{
    kronos_task_runtime_t *runtimeState;

    if ((g_schedulerStarted == 0U) || (g_currentTask >= g_numTasks))
    {
        return;
    }

    runtimeState = &g_taskRuntime[g_currentTask];
    runtimeState->service_request = serviceRequest;
    runtimeState->service_object_ptr = serviceObject;
    runtimeState->service_parameter = serviceParameter;
    runtimeState->service_result = KRONOS_STATUS_OK;

    Kronos_PortRequestYield();
}

void RTOS_Start(void)
{
    if (g_numTasks == 0U)
    {
        return;
    }

    Kronos_PortInitScheduler(TICK_FREQ_HZ);
    Kronos_TaskActivate(Kronos_TaskSelectStartTask());
    Kronos_PortStartScheduler();

    for (;;)
    {
    }
}

void RTOS_Delay(uint32_t ms)
{
    uint32_t startTick;

    if (ms == 0U)
    {
        RTOS_ForceSwitch();
        return;
    }

    if (g_schedulerSuspendDepth != 0U)
    {
        startTick = g_tickCount;
        while ((g_tickCount - startTick) < ms)
        {
        }
        return;
    }

    Kronos_SchedulerRequestService(KRONOS_SERVICE_DELAY, NULL, ms);
}

void RTOS_SuspendScheduler(void)
{
    Kronos_SchedulerRequestService(KRONOS_SERVICE_SUSPEND_SCHEDULER, NULL, 0U);
}

void RTOS_ResumeScheduler(void)
{
    Kronos_SchedulerRequestService(KRONOS_SERVICE_RESUME_SCHEDULER, NULL, 0U);
}

void RTOS_ForceSwitch(void)
{
    Kronos_SchedulerRequestService(KRONOS_SERVICE_FORCE_SWITCH, NULL, 0U);
}

void Scheduler_RoundRobin(void)
{
    int32_t nextTaskIndex;

    if (g_numTasks == 0U)
    {
        return;
    }

    if (g_tasks[g_currentTask].task_state == TASK_STATE_RUNNING)
    {
        g_tasks[g_currentTask].task_state = TASK_STATE_READY;
    }

    nextTaskIndex = kronos_scheduler_select_next_ready_task(g_currentTask);
    if (nextTaskIndex >= 0)
    {
        Kronos_TaskActivate((uint32_t)nextTaskIndex);
        return;
    }

    if (g_tasks[g_currentTask].task_state == TASK_STATE_READY)
    {
        Kronos_TaskActivate(g_currentTask);
    }
}

void Kronos_CoreOnTick(void)
{
    uint32_t taskIndex;

    g_tickCount++;

    if (g_schedulerStarted == 0U)
    {
        return;
    }

    for (taskIndex = 0U; taskIndex < g_numTasks; ++taskIndex)
    {
        if (g_tasks[taskIndex].task_state == TASK_STATE_TIME_DELAY)
        {
            if (g_tasks[taskIndex].remaining_delay > 0U)
            {
                g_tasks[taskIndex].remaining_delay--;
            }

            if (g_tasks[taskIndex].remaining_delay == 0U)
            {
                g_tasks[taskIndex].task_state = TASK_STATE_READY;
            }
        }
    }

    if (g_schedulerSuspendDepth != 0U)
    {
        g_schedulerSwitchPending = 1U;
        return;
    }

    Kronos_PortPendContextSwitch();
}

kronos_stack_check_e Kronos_CoreCheckCurrentTaskStack(uint32_t *exceptionStackPtr)
{
    return Kronos_TaskCheckStack(&g_tasks[g_currentTask], exceptionStackPtr);
}

uint32_t *Kronos_CorePrepareFirstTask(void)
{
    g_schedulerStarted = 1U;
    Kronos_TaskUpdateStackMetrics(&g_tasks[g_currentTask], g_tasks[g_currentTask].stack_top_ptr);
    return g_tasks[g_currentTask].stack_top_ptr;
}

uint32_t *Kronos_CoreHandleSupervisorCall(uint32_t *savedStackPtr)
{
    TCB_t *currentTcb;
    const kronos_channel_request_t *channelRequest;
    kronos_task_runtime_t *runtimeState;
    kronos_service_outcome_t outcome;

    currentTcb = &g_tasks[g_currentTask];
    runtimeState = &g_taskRuntime[g_currentTask];
    kronos_scheduler_store_current_context(savedStackPtr);
    kronos_scheduler_init_outcome(&outcome);
    channelRequest = (const kronos_channel_request_t *)runtimeState->service_object_ptr;

    switch (runtimeState->service_request)
    {
        case KRONOS_SERVICE_DELAY:
            if (runtimeState->service_parameter > 0U)
            {
                currentTcb->remaining_delay = runtimeState->service_parameter;
                currentTcb->task_state = TASK_STATE_TIME_DELAY;
            }
            kronos_scheduler_clear_service_state(runtimeState);
            runtimeState->service_result = KRONOS_STATUS_OK;
            return kronos_scheduler_switch_or_continue(savedStackPtr);

        case KRONOS_SERVICE_SUSPEND_SCHEDULER:
            if (g_schedulerSuspendDepth < UINT32_MAX)
            {
                g_schedulerSuspendDepth++;
            }
            kronos_scheduler_clear_service_state(runtimeState);
            runtimeState->service_result = KRONOS_STATUS_OK;
            return kronos_scheduler_keep_current_running(savedStackPtr);

        case KRONOS_SERVICE_RESUME_SCHEDULER:
            kronos_scheduler_clear_service_state(runtimeState);
            if (g_schedulerSuspendDepth == 0U)
            {
                runtimeState->service_result = KRONOS_STATUS_ERROR;
                return kronos_scheduler_keep_current_running(savedStackPtr);
            }

            g_schedulerSuspendDepth--;
            runtimeState->service_result = KRONOS_STATUS_OK;
            if ((g_schedulerSuspendDepth == 0U) && (g_schedulerSwitchPending != 0U))
            {
                g_schedulerSwitchPending = 0U;
                return kronos_scheduler_switch_or_continue(savedStackPtr);
            }
            return kronos_scheduler_keep_current_running(savedStackPtr);

        case KRONOS_SERVICE_MUTEX_LOCK:
            Kronos_SyncMutexLock(currentTcb, (kronos_mutex_t *)runtimeState->service_object_ptr, &outcome);
            break;

        case KRONOS_SERVICE_MUTEX_UNLOCK:
            Kronos_SyncMutexUnlock(currentTcb, (kronos_mutex_t *)runtimeState->service_object_ptr, &outcome);
            break;

        case KRONOS_SERVICE_SEMAPHORE_TAKE:
            Kronos_SyncSemaphoreTake(currentTcb, (kronos_semaphore_t *)runtimeState->service_object_ptr, &outcome);
            break;

        case KRONOS_SERVICE_SEMAPHORE_GIVE:
            Kronos_SyncSemaphoreGive((kronos_semaphore_t *)runtimeState->service_object_ptr, &outcome);
            break;

        case KRONOS_SERVICE_INGRESS_RECEIVE:
            if (channelRequest == NULL)
            {
                outcome.status = KRONOS_STATUS_INVALID_ARGUMENT;
            }
            else
            {
                Kronos_ChannelsIngressReceive(channelRequest->message_ptr, &outcome);
            }
            break;

        case KRONOS_SERVICE_INGRESS_WAIT:
            Kronos_ChannelsIngressWait(&outcome);
            break;

        case KRONOS_SERVICE_EGRESS_SEND:
            Kronos_ChannelsEgressSend(channelRequest, &outcome);
            break;

        case KRONOS_SERVICE_EGRESS_BROADCAST:
            Kronos_ChannelsEgressBroadcast(channelRequest, &outcome);
            break;

        case KRONOS_SERVICE_FORCE_SWITCH:
        case KRONOS_SERVICE_NONE:
        default:
            kronos_scheduler_clear_service_state(runtimeState);
            runtimeState->service_result = KRONOS_STATUS_OK;
            return kronos_scheduler_switch_or_continue(savedStackPtr);
    }

    kronos_scheduler_clear_service_state(runtimeState);
    runtimeState->service_result = outcome.status;

    if ((currentTcb->task_state != TASK_STATE_RUNNING) || (outcome.switch_required != 0U))
    {
        return kronos_scheduler_switch_or_continue(savedStackPtr);
    }

    return kronos_scheduler_keep_current_running(savedStackPtr);
}

uint32_t *Kronos_CoreSwitchTask(uint32_t *savedStackPtr)
{
    kronos_scheduler_store_current_context(savedStackPtr);
    return kronos_scheduler_switch_or_continue(savedStackPtr);
}

uint32_t *Kronos_CoreQuarantineCurrentTask(uint32_t *faultStackPtr, uint32_t faultFlags, uint32_t faultAddress)
{
    int32_t nextTaskIndex;

    Kronos_TaskQuarantine(&g_tasks[g_currentTask], faultStackPtr, faultFlags, faultAddress);

    nextTaskIndex = kronos_scheduler_select_next_ready_task(g_currentTask);
    if (nextTaskIndex >= 0)
    {
        Kronos_TaskActivate((uint32_t)nextTaskIndex);
    }

    return g_tasks[g_currentTask].stack_top_ptr;
}

/******************************************************************************
* EOF - NO CODE AFTER THIS LINE
******************************************************************************/