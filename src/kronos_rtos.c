/*!
@file   kronos_core.c
@brief  Source file of KronOS (RTOS) Core functionalities
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
* Preprocessor Definitions & Macros
******************************************************************************/

/******************************************************************************
* Includes
******************************************************************************/

#include "stm32l4xx.h"       /* For SysTick, NVIC, SCB, etc. */
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <stddef.h>
#include "kronos_rtos.h"

/******************************************************************************
* Enumerations, structures & Variables
******************************************************************************/

TCB_t     g_tasks[MAX_TASKS];
uint32_t  g_numTasks    = 0;
uint32_t  g_currentTask = 0;

/******************************************************************************
* Declaration | Static Functions
******************************************************************************/
/**
 * @brief  A default system task for KronOS.
 *         Could be used for idle or housekeeping.
 */
static void kronos_system_task(void);

/******************************************************************************
* Definition  | Static Functions
******************************************************************************/
/**
 * @brief kronos_system_task
 * A simple "do-nothing" loop. This could serve as an idle task.
 */
static void kronos_system_task(void)
{
    while(1)
    {
        __NOP();
        __NOP();
        __NOP();
        __NOP();
        __NOP();
        __NOP();
        __NOP();
        __NOP();
    }
}

/******************************************************************************
* Definition  | Public Functions
******************************************************************************/
/**
 * @brief  Initialize the KronOS RTOS (basic init).
 */
void RTOS_Init(void)
{
    g_numTasks    = 0;
    g_currentTask = 0;
    RTOS_CreateTask(kronos_system_task);
}

/**
 * @brief  Create a new task.
 *         Sets up a new task's stack so it can be restored by PendSV on context switch.
 *
 * @param  taskFunction   Pointer to the task's entry function.
 */
void RTOS_CreateTask(void (*taskFunction)(void))
{
    if (g_numTasks >= MAX_TASKS)
        return;

    TCB_t *tcb = &g_tasks[g_numTasks];

    memset(tcb->stack, 0xCC, STACK_SIZE * sizeof(uint32_t));

    uint32_t *stackTop = &(tcb->stack[STACK_SIZE - 1]);
    stackTop = (uint32_t *)((uint32_t)stackTop & ~0x7UL);

    *(--stackTop) = 0xDEADBEEF;              // Optional scratch or alignment
    tcb->stack_bot_ptr = stackTop;           // Just for reference
    *(--stackTop) = 0x01000000;              // xPSR (Thumb bit set)
    *(--stackTop) = (uint32_t)taskFunction;  // PC = Task entry
    *(--stackTop) = 0;                       // LR
    *(--stackTop) = 0;                       // R12
    *(--stackTop) = 0;                       // R3
    *(--stackTop) = 0;                       // R2
    *(--stackTop) = 0;                       // R1
    *(--stackTop) = 0;                       // R0

    for (int i = 0; i < 8; i++)
    {
        *(--stackTop) = 0;
    }

    tcb->stack_top_ptr = stackTop;
    tcb->task_fn       = taskFunction;
    tcb->task_state    = TASK_STATE_READY;

    g_numTasks++;
}

/**
 * @brief  Start the KronOS RTOS scheduler.
 *         Configures SysTick, sets PSP to the first task, and jumps to it.
 */
void RTOS_Start(void)
{
    SysTick->LOAD = (CPU_FREQ_HZ / TICK_FREQ_HZ) - 1;
    SysTick->VAL  = 0; 
    SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk |
                    SysTick_CTRL_TICKINT_Msk   |
                    SysTick_CTRL_ENABLE_Msk; 


    NVIC_SetPriority(PendSV_IRQn, 0xFF);

    __set_PSP((uint32_t)g_tasks[0].stack_bot_ptr);
    __ISB();
    __set_CONTROL(__get_CONTROL() | 0x02);
    __ISB();

    __enable_irq();

    g_tasks[0].task_state = TASK_STATE_RUNNING;

    __asm volatile("BX %0" : : "r" ((uint32_t)g_tasks[0].task_fn));
}

/**
 * @brief  A simple delay function. The current task is set to TIME_DELAY state
 *         and the scheduler is invoked via SVC.
 *
 * @param  ms  Delay in milliseconds.
 */
void RTOS_Delay(uint32_t ms)
{
    g_tasks[g_currentTask].remaining_delay = ms;
    g_tasks[g_currentTask].last_delay_decr = HAL_GetTick();
    g_tasks[g_currentTask].task_state      = TASK_STATE_TIME_DELAY;

    __asm volatile("SVC #0");
}

/**
 * @brief  Simple round-robin scheduler.
 *         Checks for tasks in TIME_DELAY state, updates their timers,
 *         and selects the next READY task to run.
 */
void Scheduler_RoundRobin(void)
{
    static uint32_t i;
    static uint32_t hal_tick;

    if (g_tasks[g_currentTask].task_state == TASK_STATE_RUNNING)
    {
        g_tasks[g_currentTask].task_state = TASK_STATE_READY;
    }

    hal_tick = HAL_GetTick();

    for (i = 0; i < g_numTasks; i++)
    {
        if (g_tasks[i].task_state == TASK_STATE_TIME_DELAY)
        {
            if (g_tasks[i].remaining_delay < (hal_tick - g_tasks[i].last_delay_decr))
            {
                g_tasks[i].remaining_delay = 0;
                g_tasks[i].last_delay_decr = 0;
                g_tasks[i].task_state      = TASK_STATE_READY;
            }
            else
            {
                g_tasks[i].remaining_delay -= (hal_tick - g_tasks[i].last_delay_decr);
                g_tasks[i].last_delay_decr  = hal_tick;
            }
        }
    }

    // Round-robin selection of the next READY task
    do
    {
        g_currentTask = ((g_currentTask + 1) >= g_numTasks) ? 0 : (g_currentTask + 1);
    }
    while (g_tasks[g_currentTask].task_state != TASK_STATE_READY);

    g_tasks[g_currentTask].task_state = TASK_STATE_RUNNING;
}

/**
 * @brief  PendSV_Handler: performs the context switch by saving/restoring
 *         the appropriate registers on the PSP.
 *         (Kept as-is for manual assembly context switching.)
 */
__attribute__((naked)) void PendSV_Handler(void)
{
    __asm volatile (
        "PUSH   {LR}            \n"
        "CPSID  i               \n" 
    );

    /* Save current task context (R4-R11) */
    __asm volatile (
        "MRS    R0, PSP         \n" 
        "STMDB  R0!, {R4-R11}   \n" 
    );
    __asm volatile (
        "STR    R0, [%0]        \n"
        :
        : "r" (&g_tasks[g_currentTask].stack_top_ptr)
        : "r0", "memory"
    );

    /* Call the scheduler to pick the next task */
    __asm volatile (
        "BL     Scheduler_RoundRobin \n"
    );

    /* Restore next task context */
    __asm volatile (
        "ISB                    \n"
        "LDR    R0, [%0]        \n" 
        "LDMIA  R0!, {R4-R11}   \n" 
        "MSR    PSP, R0         \n" 
        "ISB                    \n"
        "CPSIE  i               \n" 
        "POP    {LR}            \n"
        "BX     LR              \n"
        :
        : "r" (&g_tasks[g_currentTask].stack_top_ptr)
        : "r0", "memory"
    );
}

/**
 * @brief  SVC_Handler: triggered by "SVC #0" to yield control.
 *         (Kept as-is for manual assembly context switching.)
 */
__attribute__((naked)) void SVC_Handler(void)
{
    __asm volatile (
        "PUSH   {LR}            \n"
        "CPSID  i               \n"
    );

    /* Save current task context (R4-R11) */
    __asm volatile (
        "MRS    R0, PSP         \n" 
        "STMDB  R0!, {R4-R11}   \n" 
    );
    __asm volatile (
        "STR    R0, [%0]        \n" 
        :
        : "r" (&g_tasks[g_currentTask].stack_top_ptr)
        : "r0", "memory"
    );

    /* Call the scheduler to pick the next task */
    __asm volatile (
        "BL     Scheduler_RoundRobin \n"
    );

    /* Restore next task context */
    __asm volatile (
        "ISB                    \n"
        "LDR    R0, [%0]        \n" 
        "LDMIA  R0!, {R4-R11}   \n"
        "MSR    PSP, R0         \n" 
        "ISB                    \n"
        "CPSIE  i               \n" 
        "POP    {LR}            \n"
        "BX     LR              \n"
        :
        : "r" (&g_tasks[g_currentTask].stack_top_ptr)
        : "r0", "memory"
    );
}

/**
 * @brief  OSSysTick_Handler: Called every SysTick interrupt. Triggers PendSV
 *         to do the actual context switch.
 */
void OSSysTick_Handler(void)
{
    SCB->ICSR = SCB_ICSR_PENDSVSET_Msk;
}

/******************************************************************************
* EOF - NO CODE AFTER THIS LINE
******************************************************************************/
