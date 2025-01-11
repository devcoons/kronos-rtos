/*!
@file   kronos_core.h
@brief  Header file of KronOS (RTOS) Core functionalities
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
#ifndef KRONOS_CORE_H
#define KRONOS_CORE_H

/* --------------------- RTOS Configurations --------------------- */
#define MAX_TASKS       10
#define STACK_SIZE      48         // in 32-bit words
#define CPU_FREQ_HZ     80000000   // e.g., 80 MHz system clock
#define TICK_FREQ_HZ    1000       // 1ms tick

/******************************************************************************
 * Includes
******************************************************************************/
#include <stdint.h>
#include <string.h>

/******************************************************************************
 * Enumerations, structures & Variables
******************************************************************************/
/**
 * @brief Task states used by KronOS
 */
typedef enum
{
    TASK_STATE_UNKNOWN,
    TASK_STATE_READY,
    TASK_STATE_RUNNING,
    TASK_STATE_PAUSED,
    TASK_STATE_TIME_DELAY,
    TASK_STATE_WAITING,
    TASK_STATE_DELETING,
    TASK_STATE_STOPPED,
    TASK_STATE_ERROR,
    TASK_STATE_ERROR_STACK_OVERFLOW
} task_state_e;

/**
 * @brief Task Control Block (TCB)
 */
typedef struct
{
    uint32_t       *stack_top_ptr;  /**< Current top of stack (PSP) */
    uint32_t       *stack_bot_ptr;  /**< Bottom pointer for reference */
    task_state_e    task_state;
    uint32_t        remaining_delay;
    uint32_t        last_delay_decr;
    void            (*task_fn)(void);        /**< Pointer to the task function */
    uint32_t        stack[STACK_SIZE];       /**< Stack allocated for this task */
} TCB_t;

/* Global variables (extern) */
extern TCB_t     g_tasks[MAX_TASKS];
extern uint32_t  g_numTasks;
extern uint32_t  g_currentTask;

/******************************************************************************
* Declaration | Public Functions
******************************************************************************/
/**
 * @brief Initialize the KronOS RTOS
 */
void RTOS_Init(void);

/**
 * @brief Create a new task
 *
 * @param taskFunction  Pointer to the task entry function
 */
void RTOS_CreateTask(void (*taskFunction)(void));

/**
 * @brief Start the KronOS RTOS scheduler
 */
void RTOS_Start(void);

/**
 * @brief  A simple delay function. The current task is set to TIME_DELAY state
 */
void RTOS_Delay(uint32_t ms);

/**
 * @brief Perform round-robin scheduling
 */
void Scheduler_RoundRobin(void);

/**
 * @brief PendSV handler used by KronOS for context switching
 */
__attribute__((naked)) void OSPendSV_Handler(void);

/**
 * @brief SysTick handler to trigger context switches
 */
void OSSysTick_Handler(void);

/******************************************************************************
* EOF - NO CODE AFTER THIS LINE
******************************************************************************/
#endif /* KRONOS_CORE_H */
