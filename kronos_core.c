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
* Includes
******************************************************************************/

#include "kronos_internal.h"

#include <stddef.h>

/******************************************************************************
* Definition | Public Functions
******************************************************************************/

void RTOS_Init(void)
{
    Kronos_SchedulerResetState();
    Kronos_TasksResetState();
    Kronos_SyncResetState();
    Kronos_ChannelsResetState();
    Kronos_TaskUpdateAllStats();
}

kronos_status_e RTOS_CreateTask(void (*taskFunction)(void), uint32_t stackWords, const char *taskName, kronos_task_id_t *taskId)
{
    kronos_task_create_request_t request;

    if (taskId == NULL)
    {
        return KRONOS_STATUS_INVALID_ARGUMENT;
    }

    *taskId = KRONOS_INVALID_TASK_ID;

    if (g_schedulerStarted == 0U)
    {
        return Kronos_TaskCreateInternal(taskFunction, stackWords, taskName, TASK_KIND_APPLICATION, taskId);
    }

    request.task_function = taskFunction;
    request.task_name = taskName;
    request.task_id_ptr = taskId;
    request.stack_words = stackWords;

    Kronos_SchedulerRequestService(KRONOS_SERVICE_TASK_CREATE, &request, 0U);
    return g_taskRuntime[g_currentTask].service_result;
}

const TCB_t *RTOS_GetTaskTable(void)
{
    return g_tasks;
}

uint32_t RTOS_GetTaskCount(void)
{
    return g_numTasks;
}

/******************************************************************************
* EOF - NO CODE AFTER THIS LINE
******************************************************************************/
