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
	KRONOS_SERVICE_EGRESS_BROADCAST
} kronos_service_e;

typedef struct
{
	int32_t status;
	uint32_t switch_required;
} kronos_service_outcome_t;

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
	int32_t service_result;
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

void Kronos_TasksResetState(void);
int32_t Kronos_TaskCreateInternal(void (*taskFunction)(void), uint32_t stackWords, const char *taskName, task_kind_e taskKind);
void Kronos_TaskUpdateAllStats(void);
void Kronos_TaskUpdateStackMetrics(TCB_t *tcb, const uint32_t *stackPtr);
kronos_stack_check_e Kronos_TaskCheckStack(TCB_t *tcb, const uint32_t *stackPtr);
void Kronos_TaskQuarantine(TCB_t *tcb, uint32_t *faultStackPtr, uint32_t faultFlags, uint32_t faultAddress);
int32_t Kronos_TaskFindByName(const char *taskName);
int32_t Kronos_TaskFindNextReady(task_kind_e taskKind, uint32_t startTask);
int32_t Kronos_TaskFindWaiting(kronos_wait_reason_e waitReason, const void *waitObject, uint32_t startTask);
void Kronos_TaskBlock(TCB_t *tcb, kronos_wait_reason_e waitReason, void *waitObject);
void Kronos_TaskUnblock(TCB_t *tcb);
uint32_t Kronos_TaskSelectStartTask(void);
void Kronos_TaskActivate(uint32_t taskIndex);

void Kronos_ChannelsResetState(void);

void Kronos_SchedulerResetState(void);
void Kronos_SchedulerRequestService(kronos_service_e serviceRequest, void *serviceObject, uint32_t serviceParameter);

void Kronos_SyncMutexLock(TCB_t *currentTcb, kronos_mutex_t *mutex, kronos_service_outcome_t *outcome);
void Kronos_SyncMutexUnlock(TCB_t *currentTcb, kronos_mutex_t *mutex, kronos_service_outcome_t *outcome);
void Kronos_SyncSemaphoreTake(TCB_t *currentTcb, kronos_semaphore_t *semaphore, kronos_service_outcome_t *outcome);
void Kronos_SyncSemaphoreGive(kronos_semaphore_t *semaphore, kronos_service_outcome_t *outcome);
void Kronos_ChannelsIngressReceive(kronos_mail_t *message, kronos_service_outcome_t *outcome);
void Kronos_ChannelsIngressWait(kronos_service_outcome_t *outcome);
void Kronos_ChannelsEgressSend(const kronos_channel_request_t *request, kronos_service_outcome_t *outcome);
void Kronos_ChannelsEgressBroadcast(const kronos_channel_request_t *request, kronos_service_outcome_t *outcome);

/******************************************************************************
* EOF - NO CODE AFTER THIS LINE
******************************************************************************/
#endif /* KRONOS_INTERNAL_H */