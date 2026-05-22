/*!
@file   kronos_scheduler.c
@brief  Source file of KronOS (RTOS) Scheduler functionalities

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

#include "kronos_internal.h"

volatile uint32_t g_tickCount = 0U;
uint32_t g_schedulerStarted = 0U;

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

void Kronos_SchedulerResetState(void)
{
    g_tickCount = 0U;
    g_schedulerStarted = 0U;
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
    if (ms > 0U)
    {
        g_tasks[g_currentTask].remaining_delay = ms;
        g_tasks[g_currentTask].last_delay_decr = g_tickCount;
        g_tasks[g_currentTask].task_state = TASK_STATE_TIME_DELAY;
    }

    Kronos_PortRequestYield();
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
                g_tasks[taskIndex].last_delay_decr = g_tickCount;
                g_tasks[taskIndex].task_state = TASK_STATE_READY;
            }
        }
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

uint32_t *Kronos_CoreSwitchTask(uint32_t *savedStackPtr)
{
    g_tasks[g_currentTask].stack_top_ptr = savedStackPtr;
    Kronos_TaskUpdateStackMetrics(&g_tasks[g_currentTask], savedStackPtr);
    Scheduler_RoundRobin();
    return g_tasks[g_currentTask].stack_top_ptr;
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