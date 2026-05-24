/*!
@file   kronos_internal.h
@brief  Header file of KronOS (RTOS) Internal functionalities
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

#ifndef KRONOS_INTERNAL_H
#define KRONOS_INTERNAL_H

/******************************************************************************
* Includes
******************************************************************************/

#include <stdint.h>

#include "kronos_core.h"
#include "kronos_kernel.h"
#include "kronos_port.h"

/******************************************************************************
* Enumerations, Structures & Variables
******************************************************************************/

typedef enum
{
	KRONOS_WAIT_NONE = 0,
	KRONOS_WAIT_MUTEX,
	KRONOS_WAIT_SEMAPHORE,
	KRONOS_WAIT_INGRESS
} kronos_wait_reason_e;

typedef enum
{
	KRONOS_SERVICE_NONE = 0,
	KRONOS_SERVICE_DELAY,
	KRONOS_SERVICE_FORCE_SWITCH,
	KRONOS_SERVICE_SUSPEND_SCHEDULER,
	KRONOS_SERVICE_RESUME_SCHEDULER,
	KRONOS_SERVICE_MUTEX_LOCK,
	KRONOS_SERVICE_MUTEX_UNLOCK,
	KRONOS_SERVICE_SEMAPHORE_TAKE,
	KRONOS_SERVICE_SEMAPHORE_GIVE,
	KRONOS_SERVICE_INGRESS_RECEIVE,
	KRONOS_SERVICE_INGRESS_WAIT,
	KRONOS_SERVICE_EGRESS_SEND,
	KRONOS_SERVICE_EGRESS_BROADCAST,
	KRONOS_SERVICE_TASK_CREATE,
	KRONOS_SERVICE_TASK_DELETE,
	KRONOS_SERVICE_TASK_PAUSE,
	KRONOS_SERVICE_TASK_RESUME,
	KRONOS_SERVICE_DRIVER_INIT
} kronos_service_e;

typedef struct
{
	kronos_status_e status;
	uint32_t switch_required;
} kronos_service_outcome_t;

typedef struct
{
	void (*task_function)(void);
	const char *task_name;
	uint32_t stack_words;
} kronos_task_create_request_t;

typedef struct
{
	kronos_driver_init_fn_t init_function;
	void *context;
} kronos_driver_init_request_t;

typedef struct
{
	const kronos_ingress_t *ingress_ptr;
	const char *task_name;
	const void *payload_ptr;
	kronos_mail_t *message_ptr;
	uint32_t message_id;
	uint32_t payload_size;
} kronos_channel_request_t;

typedef struct
{
	void *wait_object_ptr;
	void *service_object_ptr;
	uint32_t service_parameter;
	kronos_status_e service_result;
	kronos_wait_reason_e wait_reason;
	kronos_service_e service_request;
} kronos_task_runtime_t;

extern volatile uint32_t g_tickCount;
extern uint32_t g_schedulerStarted;
extern uint32_t g_stackPoolWordsUsed;
extern volatile uint32_t g_schedulerSuspendDepth;
extern volatile uint32_t g_schedulerSwitchPending;
extern kronos_task_runtime_t g_taskRuntime[MAX_TASKS];

/******************************************************************************
* Declaration | Internal Functions
******************************************************************************/

void kronos_tasks_reset_state(void);
kronos_status_e kronos_task_create_internal(void (*taskFunction)(void), uint32_t stackWords, const char *taskName, task_kind_e taskKind, kronos_task_id_t *taskId);
kronos_status_e kronos_task_delete_internal(kronos_task_id_t taskId);
kronos_status_e kronos_task_pause_internal(kronos_task_id_t taskId);
kronos_status_e kronos_task_resume_internal(kronos_task_id_t taskId);
void kronos_task_update_all_stats(void);
void kronos_task_update_stack_metrics(TCB_t *tcb, const uint32_t *stackPtr);
kronos_stack_check_e kronos_task_check_stack(TCB_t *tcb, const uint32_t *stackPtr);
void kronos_task_quarantine(TCB_t *tcb, uint32_t *faultStackPtr, uint32_t faultFlags, uint32_t faultAddress);
int32_t kronos_task_find_by_name(const char *taskName);
int32_t kronos_task_find_next_ready(task_kind_e taskKind, uint32_t startTask);
int32_t kronos_task_find_waiting(kronos_wait_reason_e waitReason, const void *waitObject, uint32_t startTask);
void kronos_task_block(TCB_t *tcb, kronos_wait_reason_e waitReason, void *waitObject);
void kronos_task_unblock(TCB_t *tcb);
uint32_t kronos_task_select_start_task(void);
void kronos_task_activate(uint32_t taskIndex);

void kronos_channels_reset_state(void);
void kronos_channels_cleanup_task(kronos_task_id_t taskId);

void kronos_scheduler_reset_state(void);
void kronos_scheduler_request_service(kronos_service_e serviceRequest, void *serviceObject, uint32_t serviceParameter);

void kronos_sync_reset_state(void);
void kronos_sync_cleanup_task(kronos_task_id_t taskId);
void kronos_sync_mutex_lock(TCB_t *currentTcb, kronos_mutex_t *mutex, kronos_service_outcome_t *outcome);
void kronos_sync_mutex_unlock(TCB_t *currentTcb, kronos_mutex_t *mutex, kronos_service_outcome_t *outcome);
void kronos_sync_semaphore_take(TCB_t *currentTcb, kronos_semaphore_t *semaphore, kronos_service_outcome_t *outcome);
void kronos_sync_semaphore_give(kronos_semaphore_t *semaphore, kronos_service_outcome_t *outcome);
void kronos_channels_ingress_receive(kronos_mail_t *message, kronos_service_outcome_t *outcome);
void kronos_channels_ingress_wait(kronos_service_outcome_t *outcome);
void kronos_channels_egress_send(const kronos_channel_request_t *request, kronos_service_outcome_t *outcome);
void kronos_channels_egress_broadcast(const kronos_channel_request_t *request, kronos_service_outcome_t *outcome);

/******************************************************************************
* EOF - NO CODE AFTER THIS LINE
******************************************************************************/
#endif /* KRONOS_INTERNAL_H */
