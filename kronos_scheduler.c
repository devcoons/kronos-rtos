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
static kronos_status_e kronos_scheduler_create_task(const kronos_task_create_request_t *request);
static kronos_status_e kronos_scheduler_run_driver_init(const kronos_driver_init_request_t *request);
static void kronos_scheduler_round_robin(void);

/******************************************************************************
* Definition | Static Functions
******************************************************************************/

static void kronos_scheduler_store_current_context(uint32_t *savedStackPtr)
{
    g_tasks[g_currentTask].stack_top_ptr = savedStackPtr;
    kronos_task_update_stack_metrics(&g_tasks[g_currentTask], savedStackPtr);
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

    nextTaskIndex = kronos_task_find_next_ready(TASK_KIND_APPLICATION, startTask);
    if (nextTaskIndex < 0)
    {
        nextTaskIndex = kronos_task_find_next_ready(TASK_KIND_SYSTEM, startTask);
    }
    if (nextTaskIndex < 0)
    {
        nextTaskIndex = kronos_task_find_next_ready(TASK_KIND_IDLE, startTask);
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

    kronos_scheduler_round_robin();
    return g_tasks[g_currentTask].stack_top_ptr;
}

static kronos_status_e kronos_scheduler_create_task(const kronos_task_create_request_t *request)
{
    if (request == NULL)
    {
        return KRONOS_STATUS_INVALID_ARGUMENT;
    }

    return kronos_task_create_internal(request->task_function,
                                     request->stack_words,
                                     request->task_name,
                                     TASK_KIND_APPLICATION,
                                     NULL);
}

static kronos_status_e kronos_scheduler_run_driver_init(const kronos_driver_init_request_t *request)
{
    if ((request == NULL) || (request->init_function == NULL))
    {
        return KRONOS_STATUS_INVALID_ARGUMENT;
    }

    return request->init_function(request->context);
}

/******************************************************************************
* Definition | Public Functions
******************************************************************************/

void kronos_scheduler_reset_state(void)
{
    g_tickCount = 0U;
    g_schedulerStarted = 0U;
    g_schedulerSuspendDepth = 0U;
    g_schedulerSwitchPending = 0U;
}

void kronos_scheduler_request_service(kronos_service_e serviceRequest, void *serviceObject, uint32_t serviceParameter)
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

    kronos_port_request_yield();
}

void Kronos_Start(void)
{
    if (g_numTasks == 0U)
    {
        return;
    }

    kronos_port_init_scheduler(TICK_FREQ_HZ);
    kronos_task_activate(kronos_task_select_start_task());
    kronos_port_start_scheduler();

    for (;;)
    {
    }
}

void Kronos_Delay(uint32_t ms)
{
    uint32_t startTick;

    if (ms == 0U)
    {
        Kronos_ForceSwitch();
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

    kronos_scheduler_request_service(KRONOS_SERVICE_DELAY, NULL, ms);
}

void Kronos_SuspendScheduler(void)
{
    kronos_scheduler_request_service(KRONOS_SERVICE_SUSPEND_SCHEDULER, NULL, 0U);
}

void Kronos_ResumeScheduler(void)
{
    kronos_scheduler_request_service(KRONOS_SERVICE_RESUME_SCHEDULER, NULL, 0U);
}

void Kronos_ForceSwitch(void)
{
    kronos_scheduler_request_service(KRONOS_SERVICE_FORCE_SWITCH, NULL, 0U);
}

kronos_status_e Kronos_DriverInit(kronos_driver_init_fn_t initFunction, void *context)
{
    kronos_driver_init_request_t request;

    if (initFunction == NULL)
    {
        return KRONOS_STATUS_INVALID_ARGUMENT;
    }

    if ((g_schedulerStarted == 0U) || (g_currentTask >= g_numTasks))
    {
        return initFunction(context);
    }

    request.init_function = initFunction;
    request.context = context;

    kronos_scheduler_request_service(KRONOS_SERVICE_DRIVER_INIT, &request, 0U);
    return g_taskRuntime[g_currentTask].service_result;
}

static void kronos_scheduler_round_robin(void)
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
        kronos_task_activate((uint32_t)nextTaskIndex);
        return;
    }

    if (g_tasks[g_currentTask].task_state == TASK_STATE_READY)
    {
        kronos_task_activate(g_currentTask);
    }
}

void kronos_core_on_tick(void)
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

    kronos_port_pend_context_switch();
}

kronos_stack_check_e kronos_core_check_current_task_stack(uint32_t *exceptionStackPtr)
{
    return kronos_task_check_stack(&g_tasks[g_currentTask], exceptionStackPtr);
}

uint32_t *kronos_core_prepare_first_task(void)
{
    g_schedulerStarted = 1U;
    kronos_task_update_stack_metrics(&g_tasks[g_currentTask], g_tasks[g_currentTask].stack_top_ptr);
    return g_tasks[g_currentTask].stack_top_ptr;
}

uint32_t *kronos_core_handle_supervisor_call(uint32_t *savedStackPtr)
{
    TCB_t *currentTcb;
    const kronos_channel_request_t *channelRequest;
    const kronos_task_create_request_t *taskCreateRequest;
    const kronos_driver_init_request_t *driverInitRequest;
    kronos_task_runtime_t *runtimeState;
    kronos_service_outcome_t outcome;
    uint32_t serviceParameter;

    currentTcb = &g_tasks[g_currentTask];
    runtimeState = &g_taskRuntime[g_currentTask];
    kronos_scheduler_store_current_context(savedStackPtr);
    kronos_scheduler_init_outcome(&outcome);
    channelRequest = (const kronos_channel_request_t *)runtimeState->service_object_ptr;
    taskCreateRequest = (const kronos_task_create_request_t *)runtimeState->service_object_ptr;
    driverInitRequest = (const kronos_driver_init_request_t *)runtimeState->service_object_ptr;
    serviceParameter = runtimeState->service_parameter;

    switch (runtimeState->service_request)
    {
        case KRONOS_SERVICE_DELAY:
            if (serviceParameter > 0U)
            {
                currentTcb->remaining_delay = serviceParameter;
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
            kronos_sync_mutex_lock(currentTcb, (kronos_mutex_t *)runtimeState->service_object_ptr, &outcome);
            break;

        case KRONOS_SERVICE_MUTEX_UNLOCK:
            kronos_sync_mutex_unlock(currentTcb, (kronos_mutex_t *)runtimeState->service_object_ptr, &outcome);
            break;

        case KRONOS_SERVICE_SEMAPHORE_TAKE:
            kronos_sync_semaphore_take(currentTcb, (kronos_semaphore_t *)runtimeState->service_object_ptr, &outcome);
            break;

        case KRONOS_SERVICE_SEMAPHORE_GIVE:
            kronos_sync_semaphore_give((kronos_semaphore_t *)runtimeState->service_object_ptr, &outcome);
            break;

        case KRONOS_SERVICE_INGRESS_RECEIVE:
            if (channelRequest == NULL)
            {
                outcome.status = KRONOS_STATUS_INVALID_ARGUMENT;
            }
            else
            {
                kronos_channels_ingress_receive(channelRequest->message_ptr, &outcome);
            }
            break;

        case KRONOS_SERVICE_INGRESS_WAIT:
            kronos_channels_ingress_wait(&outcome);
            break;

        case KRONOS_SERVICE_EGRESS_SEND:
            kronos_channels_egress_send(channelRequest, &outcome);
            break;

        case KRONOS_SERVICE_EGRESS_BROADCAST:
            kronos_channels_egress_broadcast(channelRequest, &outcome);
            break;

        case KRONOS_SERVICE_TASK_CREATE:
            outcome.status = kronos_scheduler_create_task(taskCreateRequest);
            outcome.switch_required = 0U;
            break;

        case KRONOS_SERVICE_TASK_DELETE:
        {
            int32_t taskIndex;

            taskIndex = kronos_task_find_by_name((const char *)runtimeState->service_object_ptr);
            outcome.status = kronos_task_delete_internal((kronos_task_id_t)taskIndex);
            outcome.switch_required = ((outcome.status == KRONOS_STATUS_OK) && ((uint32_t)taskIndex == g_currentTask)) ? 1U : 0U;
            break;
        }

        case KRONOS_SERVICE_TASK_PAUSE:
            outcome.status = kronos_task_pause_internal((kronos_task_id_t)kronos_task_find_by_name((const char *)runtimeState->service_object_ptr));
            outcome.switch_required = 0U;
            break;

        case KRONOS_SERVICE_TASK_RESUME:
            outcome.status = kronos_task_resume_internal((kronos_task_id_t)kronos_task_find_by_name((const char *)runtimeState->service_object_ptr));
            outcome.switch_required = (outcome.status == KRONOS_STATUS_OK) ? 1U : 0U;
            break;

        case KRONOS_SERVICE_DRIVER_INIT:
            outcome.status = kronos_scheduler_run_driver_init(driverInitRequest);
            outcome.switch_required = 0U;
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

uint32_t *kronos_core_switch_task(uint32_t *savedStackPtr)
{
    kronos_scheduler_store_current_context(savedStackPtr);
    return kronos_scheduler_switch_or_continue(savedStackPtr);
}

uint32_t *kronos_core_quarantine_current_task(uint32_t *faultStackPtr, uint32_t faultFlags, uint32_t faultAddress)
{
    int32_t nextTaskIndex;

    kronos_task_quarantine(&g_tasks[g_currentTask], faultStackPtr, faultFlags, faultAddress);

    nextTaskIndex = kronos_scheduler_select_next_ready_task(g_currentTask);
    if (nextTaskIndex >= 0)
    {
        kronos_task_activate((uint32_t)nextTaskIndex);
    }

    return g_tasks[g_currentTask].stack_top_ptr;
}

/******************************************************************************
* EOF - NO CODE AFTER THIS LINE
******************************************************************************/
