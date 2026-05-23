/*!
@file   kronos_tasks.c
@brief  Source file of KronOS (RTOS) Task functionalities
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
#include <string.h>

/******************************************************************************
* Preprocessor Definitions & Macros
******************************************************************************/

#define KRONOS_STACK_FILL_BYTE 0xCCU

/******************************************************************************
* Enumerations, Structures & Variables
******************************************************************************/

TCB_t g_tasks[MAX_TASKS];
uint32_t g_numTasks = 0U;
uint32_t g_currentTask = 0U;
kronos_task_runtime_t g_taskRuntime[MAX_TASKS];

uint32_t g_stackPoolWordsUsed = 0U;

static uint32_t g_taskStackPool[STACK_POOL_SIZE_WORDS] KRONOS_PORT_STACK_POOL_SECTION;

/******************************************************************************
* Declaration | Static Functions
******************************************************************************/

static inline void kronos_wait_for_interrupt(void);
static void kronos_idle_task(void);
static void kronos_task_exit(void);
static void kronos_task_entry(void (*taskFunction)(void));
static uint32_t *kronos_allocate_stack_region(uint32_t slotWords);
static uint32_t kronos_clamp_warning_margin(uint32_t requestedStackWords);
static uint32_t kronos_calculate_stack_used_words(const TCB_t *tcb, const uint32_t *stackPtr);
static uint32_t kronos_task_get_index(const TCB_t *tcb);

/******************************************************************************
* Definition | Static Functions
******************************************************************************/

static inline void kronos_wait_for_interrupt(void)
{
    __asm volatile ("wfi");
}

static void kronos_idle_task(void)
{
    for (;;)
    {
        kronos_wait_for_interrupt();
    }
}

static void kronos_task_exit(void)
{
    g_tasks[g_currentTask].task_state = TASK_STATE_STOPPED;

    for (;;)
    {
        Kronos_PortRequestYield();
    }
}

static void kronos_task_entry(void (*taskFunction)(void))
{
    taskFunction();
    kronos_task_exit();
}

static uint32_t *kronos_allocate_stack_region(uint32_t slotWords)
{
    uint32_t alignedOffset;

    alignedOffset = (g_stackPoolWordsUsed + slotWords - 1U) & ~(slotWords - 1U);
    if ((alignedOffset + slotWords) > STACK_POOL_SIZE_WORDS)
    {
        return NULL;
    }

    g_stackPoolWordsUsed = alignedOffset + slotWords;
    return &g_taskStackPool[alignedOffset];
}

static uint32_t kronos_clamp_warning_margin(uint32_t requestedStackWords)
{
    uint32_t warningWords;

    warningWords = Kronos_PortGetStackWarningMarginWords();
    if (warningWords >= requestedStackWords)
    {
        warningWords = requestedStackWords / 2U;
    }

    if (warningWords == 0U)
    {
        warningWords = 1U;
    }

    return warningWords;
}

static uint32_t kronos_calculate_stack_used_words(const TCB_t *tcb, const uint32_t *stackPtr)
{
    uintptr_t stackStart;
    uintptr_t stackEnd;
    uintptr_t currentStack;

    if ((tcb == NULL) || (stackPtr == NULL) || (tcb->stack_slot_start_ptr == NULL) || (tcb->stack_slot_end_ptr == NULL))
    {
        return 0U;
    }

    stackStart = (uintptr_t)tcb->stack_slot_start_ptr;
    stackEnd = (uintptr_t)tcb->stack_slot_end_ptr;
    currentStack = (uintptr_t)stackPtr;

    if (currentStack <= stackStart)
    {
        return tcb->stack_slot_words;
    }

    if (currentStack >= stackEnd)
    {
        return 0U;
    }

    return (uint32_t)((stackEnd - currentStack) / sizeof(uint32_t));
}

static uint32_t kronos_task_get_index(const TCB_t *tcb)
{
    return (uint32_t)(tcb - &g_tasks[0]);
}

/******************************************************************************
* Definition | Public Functions
******************************************************************************/

void Kronos_TasksResetState(void)
{
    memset(g_tasks, 0, sizeof(g_tasks));
    memset(g_taskRuntime, 0, sizeof(g_taskRuntime));
    g_numTasks = 0U;
    g_currentTask = 0U;
    g_stackPoolWordsUsed = 0U;

    (void)Kronos_TaskCreateInternal(kronos_idle_task, IDLE_TASK_STACK_SIZE_WORDS, "kronos_idle", TASK_KIND_IDLE);
}

int32_t Kronos_TaskCreateInternal(void (*taskFunction)(void), uint32_t stackWords, const char *taskName, task_kind_e taskKind)
{
    TCB_t *tcb;
    uint32_t *stackBase;
    uint32_t *stackCursor;
    uint32_t effectiveStackWords;
    uint32_t slotWords;
    uint32_t warningWords;
    uint32_t regIndex;
    int32_t taskIndex;

    if ((taskFunction == NULL) || (taskName == NULL) || (taskName[0] == '\0') || (g_numTasks >= MAX_TASKS))
    {
        return -1;
    }

    if (Kronos_TaskFindByName(taskName) >= 0)
    {
        return -1;
    }

    effectiveStackWords = (stackWords == 0U) ? DEFAULT_STACK_SIZE_WORDS : stackWords;
    effectiveStackWords = (effectiveStackWords + 1U) & ~1U;

    if (effectiveStackWords < MIN_STACK_SIZE_WORDS)
    {
        return -1;
    }

    slotWords = Kronos_PortComputeTaskSlotWords(effectiveStackWords);
    stackBase = kronos_allocate_stack_region(slotWords);
    if (stackBase == NULL)
    {
        return -1;
    }

    taskIndex = (int32_t)g_numTasks;
    tcb = &g_tasks[g_numTasks];

    memset(tcb, 0, sizeof(*tcb));
    memset(stackBase, (int)KRONOS_STACK_FILL_BYTE, slotWords * sizeof(uint32_t));

    warningWords = kronos_clamp_warning_margin(effectiveStackWords);

    tcb->stack_slot_start_ptr = stackBase;
    tcb->stack_slot_end_ptr = stackBase + slotWords;
    tcb->stack_limit_ptr = tcb->stack_slot_end_ptr - effectiveStackWords;
    tcb->stack_warning_ptr = tcb->stack_limit_ptr + warningWords;
    tcb->task_state = TASK_STATE_READY;
    tcb->task_kind = taskKind;
    tcb->task_fn = taskFunction;
    tcb->task_name = taskName;
    tcb->requested_stack_words = effectiveStackWords;
    tcb->stack_slot_words = slotWords;
    tcb->stack_warning_words = warningWords;
    tcb->stack_reserved_words = slotWords - effectiveStackWords;
    tcb->fault_flags = KRONOS_TASK_FAULT_NONE;
    tcb->fault_address = 0U;

    stackCursor = tcb->stack_slot_end_ptr;
    stackCursor = (uint32_t *)((uintptr_t)stackCursor & ~((uintptr_t)0x7UL));

    *(--stackCursor) = 0x01000000UL;
    *(--stackCursor) = (uint32_t)(uintptr_t)kronos_task_entry;
    *(--stackCursor) = (uint32_t)(uintptr_t)kronos_task_exit;
    *(--stackCursor) = 0U;
    *(--stackCursor) = 0U;
    *(--stackCursor) = 0U;
    *(--stackCursor) = 0U;
    *(--stackCursor) = (uint32_t)(uintptr_t)taskFunction;

    for (regIndex = 0U; regIndex < 8U; ++regIndex)
    {
        *(--stackCursor) = 0U;
    }

    tcb->stack_top_ptr = stackCursor;
    Kronos_TaskUpdateStackMetrics(tcb, stackCursor);

    g_numTasks++;

    return taskIndex;
}

int32_t RTOS_TaskPause(uint32_t taskId)
{
    if ((taskId >= g_numTasks) || (taskId == g_currentTask))
    {
        return -1;
    }

    if (g_tasks[taskId].task_kind == TASK_KIND_IDLE)
    {
        return -1;
    }

    if (g_tasks[taskId].task_state == TASK_STATE_PAUSED)
    {
        return 0;
    }

    if ((g_tasks[taskId].task_state != TASK_STATE_READY) &&
        (g_tasks[taskId].task_state != TASK_STATE_TIME_DELAY))
    {
        return -1;
    }

    g_tasks[taskId].task_state = TASK_STATE_PAUSED;
    return 0;
}

int32_t RTOS_TaskResume(uint32_t taskId)
{
    if (taskId >= g_numTasks)
    {
        return -1;
    }

    if (g_tasks[taskId].task_state != TASK_STATE_PAUSED)
    {
        return -1;
    }

    if (g_tasks[taskId].remaining_delay > 0U)
    {
        g_tasks[taskId].task_state = TASK_STATE_TIME_DELAY;
    }
    else
    {
        g_tasks[taskId].task_state = TASK_STATE_READY;
        if (g_schedulerSuspendDepth != 0U)
        {
            g_schedulerSwitchPending = 1U;
        }
    }

    return 0;
}

void Kronos_TaskUpdateStackMetrics(TCB_t *tcb, const uint32_t *stackPtr)
{
    uint32_t stackUsedWords;
    uint32_t freeWords;

    stackUsedWords = kronos_calculate_stack_used_words(tcb, stackPtr);
    freeWords = (stackUsedWords >= tcb->requested_stack_words) ? 0U : (tcb->requested_stack_words - stackUsedWords);

    tcb->stack_used_words = stackUsedWords;
    tcb->stack_free_words = freeWords;
    if (stackUsedWords > tcb->stack_high_watermark_words)
    {
        tcb->stack_high_watermark_words = stackUsedWords;
    }
}

kronos_stack_check_e Kronos_TaskCheckStack(TCB_t *tcb, const uint32_t *stackPtr)
{
    Kronos_TaskUpdateStackMetrics(tcb, stackPtr);

    if ((stackPtr == NULL) || ((uintptr_t)stackPtr < (uintptr_t)tcb->stack_slot_start_ptr))
    {
        tcb->fault_flags |= KRONOS_TASK_FAULT_STACK_OVERFLOW;
        return KRONOS_STACK_CHECK_OVERFLOW;
    }

    if ((uintptr_t)stackPtr < (uintptr_t)tcb->stack_limit_ptr)
    {
        tcb->fault_flags |= KRONOS_TASK_FAULT_STACK_OVERFLOW;
        return KRONOS_STACK_CHECK_OVERFLOW;
    }

    if ((uintptr_t)stackPtr < (uintptr_t)tcb->stack_warning_ptr)
    {
        tcb->fault_flags |= KRONOS_TASK_FAULT_STACK_WARNING;
        return KRONOS_STACK_CHECK_WARNING;
    }

    return KRONOS_STACK_CHECK_OK;
}

void Kronos_TaskQuarantine(TCB_t *tcb, uint32_t *faultStackPtr, uint32_t faultFlags, uint32_t faultAddress)
{
    kronos_task_runtime_t *runtimeState;
    uint32_t normalizedFaultFlags;
    uintptr_t faultAddressValue;
    uintptr_t stackStart;
    uintptr_t stackEnd;

    runtimeState = &g_taskRuntime[kronos_task_get_index(tcb)];
    normalizedFaultFlags = faultFlags;
    faultAddressValue = (uintptr_t)faultAddress;
    stackStart = (uintptr_t)tcb->stack_slot_start_ptr;
    stackEnd = (uintptr_t)tcb->stack_slot_end_ptr;

    if ((faultStackPtr != NULL) && ((uintptr_t)faultStackPtr < (uintptr_t)tcb->stack_limit_ptr))
    {
        normalizedFaultFlags |= KRONOS_TASK_FAULT_STACK_OVERFLOW;
    }

    if ((faultAddressValue >= stackStart) && (faultAddressValue < stackEnd))
    {
        normalizedFaultFlags |= KRONOS_TASK_FAULT_STACK_OVERFLOW;
    }

    tcb->fault_flags |= normalizedFaultFlags;
    tcb->fault_address = faultAddress;
    tcb->remaining_delay = 0U;
    runtimeState->wait_reason = KRONOS_WAIT_NONE;
    runtimeState->wait_object_ptr = NULL;
    runtimeState->service_request = KRONOS_SERVICE_NONE;
    runtimeState->service_object_ptr = NULL;
    runtimeState->service_parameter = 0U;
    runtimeState->service_result = KRONOS_STATUS_ERROR;

    if ((normalizedFaultFlags & KRONOS_TASK_FAULT_STACK_OVERFLOW) != 0U)
    {
        tcb->task_state = TASK_STATE_ERROR_STACK_OVERFLOW;
        tcb->stack_top_ptr = tcb->stack_slot_start_ptr;
        tcb->stack_used_words = tcb->stack_slot_words;
        tcb->stack_free_words = 0U;
        if (tcb->stack_high_watermark_words < tcb->stack_slot_words)
        {
            tcb->stack_high_watermark_words = tcb->stack_slot_words;
        }
    }
    else
    {
        tcb->task_state = TASK_STATE_ERROR;
        if ((faultStackPtr != NULL) && ((uintptr_t)faultStackPtr >= stackStart) && ((uintptr_t)faultStackPtr < stackEnd))
        {
            tcb->stack_top_ptr = faultStackPtr;
            Kronos_TaskUpdateStackMetrics(tcb, faultStackPtr);
        }
    }
}

int32_t Kronos_TaskFindNextReady(task_kind_e taskKind, uint32_t startTask)
{
    uint32_t candidateTask;
    uint32_t searchedTasks;

    candidateTask = startTask;

    for (searchedTasks = 0U; searchedTasks < g_numTasks; ++searchedTasks)
    {
        candidateTask = (candidateTask + 1U) % g_numTasks;

        if ((g_tasks[candidateTask].task_state == TASK_STATE_READY) &&
            (g_tasks[candidateTask].task_kind == taskKind))
        {
            return (int32_t)candidateTask;
        }
    }

    return -1;
}

int32_t Kronos_TaskFindByName(const char *taskName)
{
    uint32_t taskIndex;

    if ((taskName == NULL) || (taskName[0] == '\0'))
    {
        return -1;
    }

    for (taskIndex = 0U; taskIndex < g_numTasks; ++taskIndex)
    {
        if ((g_tasks[taskIndex].task_name != NULL) && (strcmp(g_tasks[taskIndex].task_name, taskName) == 0))
        {
            return (int32_t)taskIndex;
        }
    }

    return -1;
}

int32_t Kronos_TaskFindWaiting(kronos_wait_reason_e waitReason, const void *waitObject, uint32_t startTask)
{
    uint32_t candidateTask;
    uint32_t searchedTasks;

    candidateTask = startTask;

    for (searchedTasks = 0U; searchedTasks < g_numTasks; ++searchedTasks)
    {
        candidateTask = (candidateTask + 1U) % g_numTasks;

        if ((g_tasks[candidateTask].task_state == TASK_STATE_WAITING) &&
            (g_taskRuntime[candidateTask].wait_reason == waitReason) &&
            (g_taskRuntime[candidateTask].wait_object_ptr == waitObject))
        {
            return (int32_t)candidateTask;
        }
    }

    return -1;
}

void Kronos_TaskBlock(TCB_t *tcb, kronos_wait_reason_e waitReason, void *waitObject)
{
    kronos_task_runtime_t *runtimeState;

    if (tcb == NULL)
    {
        return;
    }

    runtimeState = &g_taskRuntime[kronos_task_get_index(tcb)];
    runtimeState->wait_reason = waitReason;
    runtimeState->wait_object_ptr = waitObject;
    tcb->task_state = TASK_STATE_WAITING;
}

void Kronos_TaskUnblock(TCB_t *tcb)
{
    kronos_task_runtime_t *runtimeState;

    if (tcb == NULL)
    {
        return;
    }

    runtimeState = &g_taskRuntime[kronos_task_get_index(tcb)];
    runtimeState->wait_reason = KRONOS_WAIT_NONE;
    runtimeState->wait_object_ptr = NULL;
    tcb->task_state = TASK_STATE_READY;
}

uint32_t Kronos_TaskSelectStartTask(void)
{
    int32_t taskIndex;

    taskIndex = Kronos_TaskFindNextReady(TASK_KIND_APPLICATION, g_numTasks - 1U);
    if (taskIndex >= 0)
    {
        return (uint32_t)taskIndex;
    }

    taskIndex = Kronos_TaskFindNextReady(TASK_KIND_SYSTEM, g_numTasks - 1U);
    if (taskIndex >= 0)
    {
        return (uint32_t)taskIndex;
    }

    taskIndex = Kronos_TaskFindNextReady(TASK_KIND_IDLE, g_numTasks - 1U);
    if (taskIndex >= 0)
    {
        return (uint32_t)taskIndex;
    }

    return 0U;
}

void Kronos_TaskActivate(uint32_t taskIndex)
{
    g_currentTask = taskIndex;
    g_tasks[g_currentTask].task_state = TASK_STATE_RUNNING;
    Kronos_PortConfigureTaskStackRegion(g_tasks[g_currentTask].stack_slot_start_ptr, g_tasks[g_currentTask].stack_slot_words);
}

void Kronos_TaskUpdateAllStats(void)
{
    uint32_t taskIndex;

    for (taskIndex = 0U; taskIndex < g_numTasks; ++taskIndex)
    {
        const uint32_t *stackPtr;

        if ((taskIndex == g_currentTask) && (g_tasks[taskIndex].task_state == TASK_STATE_RUNNING))
        {
            stackPtr = Kronos_PortGetLiveProcessStackPointer();
        }
        else
        {
            stackPtr = g_tasks[taskIndex].stack_top_ptr;
        }

        (void)Kronos_TaskCheckStack(&g_tasks[taskIndex], stackPtr);
    }
}

/******************************************************************************
* EOF - NO CODE AFTER THIS LINE
******************************************************************************/