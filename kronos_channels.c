/*!
@file   kronos_channels.c
@brief  Source file of KronOS (RTOS) ingress and egress functionalities
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

#if (KRONOS_INGRESS_SLOT_COUNT == 0U)
#error "KRONOS_INGRESS_SLOT_COUNT must be greater than zero."
#endif

#if (KRONOS_MESSAGE_MAX_PAYLOAD_BYTES == 0U)
#error "KRONOS_MESSAGE_MAX_PAYLOAD_BYTES must be greater than zero."
#endif

#define KRONOS_MAIL_SLOT_INVALID UINT32_MAX

/******************************************************************************
* Enumerations, Structures & Variables
******************************************************************************/

typedef struct
{
    uint32_t head_slot_index;
    uint32_t tail_slot_index;
    uint32_t pending_count;
} kronos_ingress_state_t;

typedef struct
{
    uint32_t next_slot_index;
    uint32_t sequence;
    uint32_t sender_task_id;
    uint32_t receiver_task_id;
    uint32_t message_id;
    uint32_t payload_size;
    uint8_t payload[KRONOS_MESSAGE_MAX_PAYLOAD_BYTES];
} kronos_mail_slot_t;

static kronos_ingress_state_t g_ingressState[MAX_TASKS];
static kronos_mail_slot_t g_mailSlots[KRONOS_INGRESS_SLOT_COUNT];
static uint32_t g_mailFreeHead = KRONOS_MAIL_SLOT_INVALID;
static uint32_t g_mailFreeCount = 0U;
static uint32_t g_mailSequenceCounter = 0U;

/******************************************************************************
* Declaration | Static Functions
******************************************************************************/

static void kronos_channels_set_outcome(kronos_service_outcome_t *outcome, kronos_status_e status, uint32_t switchRequired);
static void kronos_channels_init_ingress_state(void);
static void kronos_channels_init_free_list(void);
static uint32_t kronos_channels_task_accepts_ingress(uint32_t taskIndex);
static uint32_t kronos_channels_task_waits_for_ingress(uint32_t taskIndex);
static int32_t kronos_channels_allocate_slot(void);
static void kronos_channels_release_slot(uint32_t slotIndex);
static void kronos_channels_queue_slot(uint32_t taskIndex, uint32_t slotIndex);
static int32_t kronos_channels_pop_slot(uint32_t taskIndex);
static void kronos_channels_copy_mail(kronos_mail_t *message, uint32_t slotIndex);
static int32_t kronos_channels_resolve_target_task(const kronos_channel_request_t *request);
static kronos_status_e kronos_channels_deliver(uint32_t taskIndex, const void *payloadPtr, uint32_t payloadSize, uint32_t messageId, kronos_service_outcome_t *outcome);
static uint32_t kronos_channels_count_broadcast_targets(void);
static void kronos_channels_cleanup_queue(uint32_t queueTaskIndex, kronos_task_id_t taskId);

/******************************************************************************
* Definition | Static Functions
******************************************************************************/

static void kronos_channels_set_outcome(kronos_service_outcome_t *outcome, kronos_status_e status, uint32_t switchRequired)
{
    if (outcome != NULL)
    {
        outcome->status = status;
        outcome->switch_required = switchRequired;
    }
}

static void kronos_channels_init_ingress_state(void)
{
    uint32_t taskIndex;

    for (taskIndex = 0U; taskIndex < MAX_TASKS; ++taskIndex)
    {
        g_ingressState[taskIndex].head_slot_index = KRONOS_MAIL_SLOT_INVALID;
        g_ingressState[taskIndex].tail_slot_index = KRONOS_MAIL_SLOT_INVALID;
        g_ingressState[taskIndex].pending_count = 0U;
    }
}

static void kronos_channels_init_free_list(void)
{
    uint32_t slotIndex;

    for (slotIndex = 0U; slotIndex < KRONOS_INGRESS_SLOT_COUNT; ++slotIndex)
    {
        g_mailSlots[slotIndex].next_slot_index = slotIndex + 1U;
        g_mailSlots[slotIndex].sequence = 0U;
        g_mailSlots[slotIndex].sender_task_id = KRONOS_INVALID_TASK_ID;
        g_mailSlots[slotIndex].receiver_task_id = KRONOS_INVALID_TASK_ID;
        g_mailSlots[slotIndex].message_id = 0U;
        g_mailSlots[slotIndex].payload_size = 0U;
        memset(g_mailSlots[slotIndex].payload, 0, sizeof(g_mailSlots[slotIndex].payload));
    }

    g_mailSlots[KRONOS_INGRESS_SLOT_COUNT - 1U].next_slot_index = KRONOS_MAIL_SLOT_INVALID;
    g_mailFreeHead = 0U;
    g_mailFreeCount = KRONOS_INGRESS_SLOT_COUNT;
    g_mailSequenceCounter = 0U;
}

static uint32_t kronos_channels_task_accepts_ingress(uint32_t taskIndex)
{
    task_state_e taskState;

    if (taskIndex >= g_numTasks)
    {
        return 0U;
    }

    if (g_tasks[taskIndex].task_kind == TASK_KIND_IDLE)
    {
        return 0U;
    }

    taskState = g_tasks[taskIndex].task_state;

    return (taskState != TASK_STATE_UNKNOWN) &&
           (taskState != TASK_STATE_DELETING) &&
           (taskState != TASK_STATE_STOPPED) &&
           (taskState != TASK_STATE_ERROR) &&
           (taskState != TASK_STATE_ERROR_STACK_OVERFLOW);
}

static uint32_t kronos_channels_task_waits_for_ingress(uint32_t taskIndex)
{
    if (taskIndex >= g_numTasks)
    {
        return 0U;
    }

    return (g_tasks[taskIndex].task_state == TASK_STATE_WAITING) &&
           (g_taskRuntime[taskIndex].wait_reason == KRONOS_WAIT_INGRESS) &&
           (g_taskRuntime[taskIndex].wait_object_ptr == &g_ingressState[taskIndex]);
}

static int32_t kronos_channels_allocate_slot(void)
{
    uint32_t slotIndex;

    if (g_mailFreeHead == KRONOS_MAIL_SLOT_INVALID)
    {
        return -1;
    }

    slotIndex = g_mailFreeHead;
    g_mailFreeHead = g_mailSlots[slotIndex].next_slot_index;
    g_mailSlots[slotIndex].next_slot_index = KRONOS_MAIL_SLOT_INVALID;
    g_mailFreeCount--;

    return (int32_t)slotIndex;
}

static void kronos_channels_release_slot(uint32_t slotIndex)
{
    if (slotIndex >= KRONOS_INGRESS_SLOT_COUNT)
    {
        return;
    }

    g_mailSlots[slotIndex].next_slot_index = g_mailFreeHead;
    g_mailSlots[slotIndex].sequence = 0U;
    g_mailSlots[slotIndex].sender_task_id = KRONOS_INVALID_TASK_ID;
    g_mailSlots[slotIndex].receiver_task_id = KRONOS_INVALID_TASK_ID;
    g_mailSlots[slotIndex].message_id = 0U;
    g_mailSlots[slotIndex].payload_size = 0U;
    memset(g_mailSlots[slotIndex].payload, 0, sizeof(g_mailSlots[slotIndex].payload));
    g_mailFreeHead = slotIndex;
    g_mailFreeCount++;
}

static void kronos_channels_queue_slot(uint32_t taskIndex, uint32_t slotIndex)
{
    kronos_ingress_state_t *ingressState;

    ingressState = &g_ingressState[taskIndex];

    g_mailSlots[slotIndex].next_slot_index = KRONOS_MAIL_SLOT_INVALID;

    if (ingressState->tail_slot_index == KRONOS_MAIL_SLOT_INVALID)
    {
        ingressState->head_slot_index = slotIndex;
        ingressState->tail_slot_index = slotIndex;
    }
    else
    {
        g_mailSlots[ingressState->tail_slot_index].next_slot_index = slotIndex;
        ingressState->tail_slot_index = slotIndex;
    }

    ingressState->pending_count++;

    if (kronos_channels_task_waits_for_ingress(taskIndex) != 0U)
    {
        Kronos_TaskUnblock(&g_tasks[taskIndex]);
    }
}

static int32_t kronos_channels_pop_slot(uint32_t taskIndex)
{
    kronos_ingress_state_t *ingressState;
    uint32_t slotIndex;

    ingressState = &g_ingressState[taskIndex];
    if (ingressState->head_slot_index == KRONOS_MAIL_SLOT_INVALID)
    {
        return -1;
    }

    slotIndex = ingressState->head_slot_index;
    ingressState->head_slot_index = g_mailSlots[slotIndex].next_slot_index;
    if (ingressState->head_slot_index == KRONOS_MAIL_SLOT_INVALID)
    {
        ingressState->tail_slot_index = KRONOS_MAIL_SLOT_INVALID;
    }

    ingressState->pending_count--;
    g_mailSlots[slotIndex].next_slot_index = KRONOS_MAIL_SLOT_INVALID;

    return (int32_t)slotIndex;
}

static void kronos_channels_copy_mail(kronos_mail_t *message, uint32_t slotIndex)
{
    memset(message, 0, sizeof(*message));

    message->sequence = g_mailSlots[slotIndex].sequence;
    message->message_id = g_mailSlots[slotIndex].message_id;
    message->payload_size = g_mailSlots[slotIndex].payload_size;
    message->sender_task_id = g_mailSlots[slotIndex].sender_task_id;
    message->receiver_task_id = g_mailSlots[slotIndex].receiver_task_id;

    if (message->sender_task_id < g_numTasks)
    {
        message->sender_task_name = g_tasks[message->sender_task_id].task_name;
    }

    if (message->receiver_task_id < g_numTasks)
    {
        message->receiver_task_name = g_tasks[message->receiver_task_id].task_name;
    }

    if (message->payload_size > 0U)
    {
        memcpy(message->payload, g_mailSlots[slotIndex].payload, message->payload_size);
    }
}

static int32_t kronos_channels_resolve_target_task(const kronos_channel_request_t *request)
{
    uint32_t taskIndex;

    if (request == NULL)
    {
        return -1;
    }

    if (request->ingress_ptr != NULL)
    {
        taskIndex = request->ingress_ptr->task_id;
        if ((taskIndex < g_numTasks) &&
            (request->ingress_ptr->task_name != NULL) &&
            (g_tasks[taskIndex].task_name != NULL) &&
            (strcmp(request->ingress_ptr->task_name, g_tasks[taskIndex].task_name) == 0))
        {
            return (int32_t)taskIndex;
        }

        return -1;
    }

    return Kronos_TaskFindByName(request->task_name);
}

static kronos_status_e kronos_channels_deliver(uint32_t taskIndex, const void *payloadPtr, uint32_t payloadSize, uint32_t messageId, kronos_service_outcome_t *outcome)
{
    uint32_t waitingForIngress;
    int32_t slotIndex;
    kronos_mail_slot_t *slot;

    if (payloadSize > KRONOS_MESSAGE_MAX_PAYLOAD_BYTES)
    {
        return KRONOS_STATUS_TOO_LARGE;
    }

    if ((payloadSize > 0U) && (payloadPtr == NULL))
    {
        return KRONOS_STATUS_INVALID_ARGUMENT;
    }

    if (kronos_channels_task_accepts_ingress(taskIndex) == 0U)
    {
        return KRONOS_STATUS_ERROR;
    }

    slotIndex = kronos_channels_allocate_slot();
    if (slotIndex < 0)
    {
        return KRONOS_STATUS_MAILBOX_FULL;
    }

    waitingForIngress = kronos_channels_task_waits_for_ingress(taskIndex);

    if (g_mailSequenceCounter == UINT32_MAX)
    {
        g_mailSequenceCounter = 1U;
    }
    else
    {
        g_mailSequenceCounter++;
    }

    slot = &g_mailSlots[(uint32_t)slotIndex];
    slot->sequence = g_mailSequenceCounter;
    slot->sender_task_id = g_currentTask;
    slot->receiver_task_id = taskIndex;
    slot->message_id = messageId;
    slot->payload_size = payloadSize;
    memset(slot->payload, 0, sizeof(slot->payload));
    if (payloadSize > 0U)
    {
        memcpy(slot->payload, payloadPtr, payloadSize);
    }

    kronos_channels_queue_slot(taskIndex, (uint32_t)slotIndex);

    if (waitingForIngress != 0U)
    {
        if (outcome != NULL)
        {
            outcome->switch_required = 1U;
        }
    }

    return KRONOS_STATUS_OK;
}

static uint32_t kronos_channels_count_broadcast_targets(void)
{
    uint32_t taskIndex;
    uint32_t targetCount;

    targetCount = 0U;

    for (taskIndex = 0U; taskIndex < g_numTasks; ++taskIndex)
    {
        if (kronos_channels_task_accepts_ingress(taskIndex) != 0U)
        {
            targetCount++;
        }
    }

    return targetCount;
}

static void kronos_channels_cleanup_queue(uint32_t queueTaskIndex, kronos_task_id_t taskId)
{
    kronos_ingress_state_t *ingressState;
    uint32_t previousSlotIndex;
    uint32_t currentSlotIndex;
    uint32_t nextSlotIndex;
    uint32_t removeSlot;

    if (queueTaskIndex >= MAX_TASKS)
    {
        return;
    }

    ingressState = &g_ingressState[queueTaskIndex];
    previousSlotIndex = KRONOS_MAIL_SLOT_INVALID;
    currentSlotIndex = ingressState->head_slot_index;

    while (currentSlotIndex != KRONOS_MAIL_SLOT_INVALID)
    {
        nextSlotIndex = g_mailSlots[currentSlotIndex].next_slot_index;
        removeSlot = ((g_mailSlots[currentSlotIndex].sender_task_id == taskId) ||
                      (g_mailSlots[currentSlotIndex].receiver_task_id == taskId)) ? 1U : 0U;

        if (removeSlot != 0U)
        {
            if (previousSlotIndex == KRONOS_MAIL_SLOT_INVALID)
            {
                ingressState->head_slot_index = nextSlotIndex;
            }
            else
            {
                g_mailSlots[previousSlotIndex].next_slot_index = nextSlotIndex;
            }

            if (ingressState->tail_slot_index == currentSlotIndex)
            {
                ingressState->tail_slot_index = previousSlotIndex;
            }

            if (ingressState->pending_count > 0U)
            {
                ingressState->pending_count--;
            }

            kronos_channels_release_slot(currentSlotIndex);
        }
        else
        {
            previousSlotIndex = currentSlotIndex;
        }

        currentSlotIndex = nextSlotIndex;
    }

    if (ingressState->head_slot_index == KRONOS_MAIL_SLOT_INVALID)
    {
        ingressState->tail_slot_index = KRONOS_MAIL_SLOT_INVALID;
        ingressState->pending_count = 0U;
    }
}

/******************************************************************************
* Definition | Public Functions
******************************************************************************/

kronos_status_e RTOS_IngressResolve(kronos_ingress_t *ingress, const char *taskName)
{
    int32_t taskIndex;

    if ((ingress == NULL) || (taskName == NULL) || (taskName[0] == '\0'))
    {
        return KRONOS_STATUS_INVALID_ARGUMENT;
    }

    taskIndex = Kronos_TaskFindByName(taskName);
    if (taskIndex < 0)
    {
        return KRONOS_STATUS_NOT_FOUND;
    }

    if (kronos_channels_task_accepts_ingress((uint32_t)taskIndex) == 0U)
    {
        return KRONOS_STATUS_ERROR;
    }

    ingress->task_id = (uint32_t)taskIndex;
    ingress->task_name = g_tasks[taskIndex].task_name;
    return KRONOS_STATUS_OK;
}

kronos_status_e RTOS_EgressSend(const kronos_ingress_t *ingress, uint32_t messageId, const void *payloadPtr, uint32_t payloadSize)
{
    kronos_channel_request_t request;

    if (ingress == NULL)
    {
        return KRONOS_STATUS_INVALID_ARGUMENT;
    }

    if ((g_schedulerStarted == 0U) || (g_currentTask >= g_numTasks))
    {
        return KRONOS_STATUS_ERROR;
    }

    request.ingress_ptr = ingress;
    request.task_name = NULL;
    request.payload_ptr = payloadPtr;
    request.message_ptr = NULL;
    request.message_id = messageId;
    request.payload_size = payloadSize;

    Kronos_SchedulerRequestService(KRONOS_SERVICE_EGRESS_SEND, &request, 0U);
    return g_taskRuntime[g_currentTask].service_result;
}

kronos_status_e RTOS_EgressSendByName(const char *taskName, uint32_t messageId, const void *payloadPtr, uint32_t payloadSize)
{
    kronos_channel_request_t request;

    if ((taskName == NULL) || (taskName[0] == '\0'))
    {
        return KRONOS_STATUS_INVALID_ARGUMENT;
    }

    if ((g_schedulerStarted == 0U) || (g_currentTask >= g_numTasks))
    {
        return KRONOS_STATUS_ERROR;
    }

    request.ingress_ptr = NULL;
    request.task_name = taskName;
    request.payload_ptr = payloadPtr;
    request.message_ptr = NULL;
    request.message_id = messageId;
    request.payload_size = payloadSize;

    Kronos_SchedulerRequestService(KRONOS_SERVICE_EGRESS_SEND, &request, 0U);
    return g_taskRuntime[g_currentTask].service_result;
}

kronos_status_e RTOS_EgressBroadcast(uint32_t messageId, const void *payloadPtr, uint32_t payloadSize)
{
    kronos_channel_request_t request;

    if ((g_schedulerStarted == 0U) || (g_currentTask >= g_numTasks))
    {
        return KRONOS_STATUS_ERROR;
    }

    request.ingress_ptr = NULL;
    request.task_name = NULL;
    request.payload_ptr = payloadPtr;
    request.message_ptr = NULL;
    request.message_id = messageId;
    request.payload_size = payloadSize;

    Kronos_SchedulerRequestService(KRONOS_SERVICE_EGRESS_BROADCAST, &request, 0U);
    return g_taskRuntime[g_currentTask].service_result;
}

kronos_status_e RTOS_IngressReceive(kronos_mail_t *message)
{
    kronos_channel_request_t request;

    if (message == NULL)
    {
        return KRONOS_STATUS_INVALID_ARGUMENT;
    }

    if ((g_schedulerStarted == 0U) || (g_currentTask >= g_numTasks))
    {
        return KRONOS_STATUS_ERROR;
    }

    request.ingress_ptr = NULL;
    request.task_name = NULL;
    request.payload_ptr = NULL;
    request.message_ptr = message;
    request.message_id = 0U;
    request.payload_size = 0U;

    Kronos_SchedulerRequestService(KRONOS_SERVICE_INGRESS_RECEIVE, &request, 0U);
    return g_taskRuntime[g_currentTask].service_result;
}

kronos_status_e RTOS_IngressWait(void)
{
    if ((g_schedulerStarted == 0U) || (g_currentTask >= g_numTasks))
    {
        return KRONOS_STATUS_ERROR;
    }

    Kronos_SchedulerRequestService(KRONOS_SERVICE_INGRESS_WAIT, NULL, 0U);
    return g_taskRuntime[g_currentTask].service_result;
}

kronos_status_e RTOS_IngressReceiveWait(kronos_mail_t *message)
{
    kronos_status_e status;

    if (message == NULL)
    {
        return KRONOS_STATUS_INVALID_ARGUMENT;
    }

    status = RTOS_IngressReceive(message);
    while (status == KRONOS_STATUS_EMPTY)
    {
        status = RTOS_IngressWait();
        if (status != KRONOS_STATUS_OK)
        {
            return status;
        }

        status = RTOS_IngressReceive(message);
    }

    return status;
}

uint32_t RTOS_IngressPendingCount(void)
{
    if (g_currentTask >= g_numTasks)
    {
        return 0U;
    }

    return g_ingressState[g_currentTask].pending_count;
}

kronos_status_e RTOS_IngressPendingCountByName(const char *taskName, uint32_t *pendingCount)
{
    int32_t taskIndex;

    if ((taskName == NULL) || (taskName[0] == '\0') || (pendingCount == NULL))
    {
        return KRONOS_STATUS_INVALID_ARGUMENT;
    }

    taskIndex = Kronos_TaskFindByName(taskName);
    if (taskIndex < 0)
    {
        return KRONOS_STATUS_NOT_FOUND;
    }

    *pendingCount = g_ingressState[(uint32_t)taskIndex].pending_count;
    return KRONOS_STATUS_OK;
}

void Kronos_ChannelsResetState(void)
{
    kronos_channels_init_ingress_state();
    kronos_channels_init_free_list();
}

void Kronos_ChannelsCleanupTask(kronos_task_id_t taskId)
{
    uint32_t taskIndex;

    if (taskId >= MAX_TASKS)
    {
        return;
    }

    for (taskIndex = 0U; taskIndex < g_numTasks; ++taskIndex)
    {
        kronos_channels_cleanup_queue(taskIndex, taskId);
    }
}

void Kronos_ChannelsIngressReceive(kronos_mail_t *message, kronos_service_outcome_t *outcome)
{
    int32_t slotIndex;

    if (message == NULL)
    {
        kronos_channels_set_outcome(outcome, KRONOS_STATUS_INVALID_ARGUMENT, 0U);
        return;
    }

    slotIndex = kronos_channels_pop_slot(g_currentTask);
    if (slotIndex < 0)
    {
        kronos_channels_set_outcome(outcome, KRONOS_STATUS_EMPTY, 0U);
        return;
    }

    kronos_channels_copy_mail(message, (uint32_t)slotIndex);
    kronos_channels_release_slot((uint32_t)slotIndex);
    kronos_channels_set_outcome(outcome, KRONOS_STATUS_OK, 0U);
}

void Kronos_ChannelsIngressWait(kronos_service_outcome_t *outcome)
{
    if (g_ingressState[g_currentTask].pending_count > 0U)
    {
        kronos_channels_set_outcome(outcome, KRONOS_STATUS_OK, 0U);
        return;
    }

    if (g_schedulerSuspendDepth != 0U)
    {
        kronos_channels_set_outcome(outcome, KRONOS_STATUS_SCHEDULER_SUSPENDED, 0U);
        return;
    }

    Kronos_TaskBlock(&g_tasks[g_currentTask], KRONOS_WAIT_INGRESS, &g_ingressState[g_currentTask]);
    kronos_channels_set_outcome(outcome, KRONOS_STATUS_OK, 0U);
}

void Kronos_ChannelsEgressSend(const kronos_channel_request_t *request, kronos_service_outcome_t *outcome)
{
    int32_t taskIndex;
    kronos_status_e status;

    if (request == NULL)
    {
        kronos_channels_set_outcome(outcome, KRONOS_STATUS_INVALID_ARGUMENT, 0U);
        return;
    }

    taskIndex = kronos_channels_resolve_target_task(request);
    if (taskIndex < 0)
    {
        kronos_channels_set_outcome(outcome, KRONOS_STATUS_NOT_FOUND, 0U);
        return;
    }

    status = kronos_channels_deliver((uint32_t)taskIndex,
                                     request->payload_ptr,
                                     request->payload_size,
                                     request->message_id,
                                     outcome);
    kronos_channels_set_outcome(outcome, status, (outcome != NULL) ? outcome->switch_required : 0U);
}

void Kronos_ChannelsEgressBroadcast(const kronos_channel_request_t *request, kronos_service_outcome_t *outcome)
{
    uint32_t targetCount;
    uint32_t taskIndex;
    kronos_status_e status;

    if (request == NULL)
    {
        kronos_channels_set_outcome(outcome, KRONOS_STATUS_INVALID_ARGUMENT, 0U);
        return;
    }

    if (request->payload_size > KRONOS_MESSAGE_MAX_PAYLOAD_BYTES)
    {
        kronos_channels_set_outcome(outcome, KRONOS_STATUS_TOO_LARGE, 0U);
        return;
    }

    if ((request->payload_size > 0U) && (request->payload_ptr == NULL))
    {
        kronos_channels_set_outcome(outcome, KRONOS_STATUS_INVALID_ARGUMENT, 0U);
        return;
    }

    targetCount = kronos_channels_count_broadcast_targets();
    if (targetCount == 0U)
    {
        kronos_channels_set_outcome(outcome, KRONOS_STATUS_NOT_FOUND, 0U);
        return;
    }

    if (g_mailFreeCount < targetCount)
    {
        kronos_channels_set_outcome(outcome, KRONOS_STATUS_MAILBOX_FULL, 0U);
        return;
    }

    kronos_channels_set_outcome(outcome, KRONOS_STATUS_OK, 0U);

    for (taskIndex = 0U; taskIndex < g_numTasks; ++taskIndex)
    {
        if (kronos_channels_task_accepts_ingress(taskIndex) != 0U)
        {
            status = kronos_channels_deliver(taskIndex,
                                             request->payload_ptr,
                                             request->payload_size,
                                             request->message_id,
                                             outcome);
            if (status != KRONOS_STATUS_OK)
            {
                kronos_channels_set_outcome(outcome, status, 0U);
                return;
            }
        }
    }
}

/******************************************************************************
* EOF - NO CODE AFTER THIS LINE
******************************************************************************/
