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

/******************************************************************************
* Includes
******************************************************************************/

#include <stdint.h>

/******************************************************************************
* Enumerations, Structures & Variables
******************************************************************************/

#define MAX_APPLICATION_TASKS         10U
#define SYSTEM_TASK_COUNT             1U
#define MAX_TASKS                     (MAX_APPLICATION_TASKS + SYSTEM_TASK_COUNT)
#define DEFAULT_STACK_SIZE_WORDS      48U
#define MIN_STACK_SIZE_WORDS          32U
#define IDLE_TASK_STACK_SIZE_WORDS    64U
#define STACK_POOL_SIZE_WORDS         2048U
#define TICK_FREQ_HZ                  1000UL

#ifndef KRONOS_INGRESS_SLOT_COUNT
#define KRONOS_INGRESS_SLOT_COUNT     16U
#endif

#ifndef KRONOS_MESSAGE_MAX_PAYLOAD_BYTES
#define KRONOS_MESSAGE_MAX_PAYLOAD_BYTES 32U
#endif

#define KRONOS_TASK_FAULT_NONE            0UL
#define KRONOS_TASK_FAULT_STACK_WARNING   (1UL << 0)
#define KRONOS_TASK_FAULT_STACK_OVERFLOW  (1UL << 1)
#define KRONOS_TASK_FAULT_MEMORY_ACCESS   (1UL << 2)

#define KRONOS_INVALID_TASK_ID UINT32_MAX

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

typedef enum
{
    TASK_KIND_APPLICATION,
    TASK_KIND_SYSTEM,
    TASK_KIND_IDLE
} task_kind_e;

typedef enum
{
    KRONOS_STATUS_OK = 0,
    KRONOS_STATUS_ERROR = -1,
    KRONOS_STATUS_INVALID_ARGUMENT = -2,
    KRONOS_STATUS_NOT_OWNER = -3,
    KRONOS_STATUS_SCHEDULER_SUSPENDED = -4,
    KRONOS_STATUS_OVERFLOW = -5,
    KRONOS_STATUS_NOT_FOUND = -6,
    KRONOS_STATUS_EMPTY = -7,
    KRONOS_STATUS_TOO_LARGE = -8,
    KRONOS_STATUS_MAILBOX_FULL = -9,
    KRONOS_STATUS_INVALID_STATE = -10,
    KRONOS_STATUS_NO_MEMORY = -11,
    KRONOS_STATUS_IN_USE = -12
} kronos_status_e;

typedef uint32_t kronos_task_id_t;
typedef kronos_status_e (*kronos_driver_init_fn_t)(void *context);

typedef struct
{
    uint32_t owner_task_id;
    uint32_t lock_count;
    uint32_t initialized;
} kronos_mutex_t;

typedef struct
{
    uint32_t count;
    uint32_t max_count;
    uint32_t initialized;
} kronos_semaphore_t;

typedef struct
{
    uint32_t task_id;
    const char *task_name;
} kronos_ingress_t;

typedef struct
{
    uint32_t sequence;
    uint32_t message_id;
    uint32_t payload_size;
    uint32_t sender_task_id;
    uint32_t receiver_task_id;
    const char *sender_task_name;
    const char *receiver_task_name;
    uint8_t payload[KRONOS_MESSAGE_MAX_PAYLOAD_BYTES];
} kronos_mail_t;

typedef struct
{
    uint32_t *stack_top_ptr;
    uint32_t *stack_slot_start_ptr;
    uint32_t *stack_limit_ptr;
    uint32_t *stack_warning_ptr;
    uint32_t *stack_slot_end_ptr;
    task_state_e task_state;
    task_kind_e task_kind;
    uint32_t remaining_delay;
    void (*task_fn)(void);
    const char *task_name;
    uint32_t requested_stack_words;
    uint32_t stack_slot_words;
    uint32_t stack_warning_words;
    uint32_t stack_reserved_words;
    uint32_t stack_used_words;
    uint32_t stack_free_words;
    uint32_t stack_high_watermark_words;
    uint32_t fault_flags;
    uint32_t fault_address;
} TCB_t;

extern TCB_t g_tasks[MAX_TASKS];
extern uint32_t g_numTasks;
extern uint32_t g_currentTask;

/******************************************************************************
* Declaration | Public Functions
******************************************************************************/

void RTOS_Init(void);
kronos_status_e RTOS_CreateTask(void (*taskFunction)(void), uint32_t stackWords, const char *taskName);
kronos_status_e RTOS_TaskDelete(const char *taskName);
kronos_status_e RTOS_TaskPause(const char *taskName);
kronos_status_e RTOS_TaskResume(const char *taskName);
kronos_status_e RTOS_DriverInit(kronos_driver_init_fn_t initFunction, void *context);
kronos_status_e RTOS_IngressResolve(kronos_ingress_t *ingress, const char *taskName);
kronos_status_e RTOS_EgressSend(const kronos_ingress_t *ingress, uint32_t messageId, const void *payloadPtr, uint32_t payloadSize);
kronos_status_e RTOS_EgressSendByName(const char *taskName, uint32_t messageId, const void *payloadPtr, uint32_t payloadSize);
kronos_status_e RTOS_EgressBroadcast(uint32_t messageId, const void *payloadPtr, uint32_t payloadSize);
kronos_status_e RTOS_IngressReceive(kronos_mail_t *message);
kronos_status_e RTOS_IngressWait(void);
kronos_status_e RTOS_IngressReceiveWait(kronos_mail_t *message);
uint32_t RTOS_IngressPendingCount(void);
kronos_status_e RTOS_IngressPendingCountByName(const char *taskName, uint32_t *pendingCount);
kronos_status_e RTOS_MutexInit(kronos_mutex_t *mutex);
kronos_status_e RTOS_MutexLock(kronos_mutex_t *mutex);
kronos_status_e RTOS_MutexUnlock(kronos_mutex_t *mutex);
kronos_status_e RTOS_SemaphoreInit(kronos_semaphore_t *semaphore, uint32_t initialCount, uint32_t maxCount);
kronos_status_e RTOS_SemaphoreTake(kronos_semaphore_t *semaphore);
kronos_status_e RTOS_SemaphoreGive(kronos_semaphore_t *semaphore);
void RTOS_SuspendScheduler(void);
void RTOS_ResumeScheduler(void);
void RTOS_ForceSwitch(void);
void RTOS_Start(void);
void RTOS_Delay(uint32_t ms);
void Scheduler_RoundRobin(void);
const TCB_t *RTOS_GetTaskTable(void);
uint32_t RTOS_GetTaskCount(void);

#define RTOS_EgressSendStruct(targetTaskName, messageId, objectPtr) \
    RTOS_EgressSendByName((targetTaskName), (messageId), (objectPtr), (uint32_t)sizeof(*(objectPtr)))

#define RTOS_EgressSendStructToIngress(ingressPtr, messageId, objectPtr) \
    RTOS_EgressSend((ingressPtr), (messageId), (objectPtr), (uint32_t)sizeof(*(objectPtr)))

static inline void SuspendScheduler(void)
{
    RTOS_SuspendScheduler();
}

static inline void ResumeScheduler(void)
{
    RTOS_ResumeScheduler();
}

static inline void ForceSwitch(void)
{
    RTOS_ForceSwitch();
}

/******************************************************************************
* EOF - NO CODE AFTER THIS LINE
******************************************************************************/
#endif /* KRONOS_CORE_H */
