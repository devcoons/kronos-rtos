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

void Kronos_Init(void)
{
    kronos_scheduler_reset_state();
    kronos_tasks_reset_state();
    kronos_sync_reset_state();
    kronos_channels_reset_state();
    kronos_task_update_all_stats();
}

kronos_status_e Kronos_TaskCreate(void (*taskFunction)(void), uint32_t stackWords, const char *taskName)
{
    return Kronos_TaskCreateWithId(taskFunction, stackWords, taskName, NULL);
}

kronos_status_e Kronos_TaskCreateWithId(void (*taskFunction)(void), uint32_t stackWords, const char *taskName, kronos_task_id_t *taskId)
{
    kronos_task_create_request_t request;

    if (taskId != NULL)
    {
        *taskId = KRONOS_INVALID_TASK_ID;
    }

    if (g_schedulerStarted == 0U)
    {
        return kronos_task_create_internal(taskFunction, stackWords, taskName, TASK_KIND_APPLICATION, taskId);
    }

    request.task_function = taskFunction;
    request.task_name = taskName;
    request.stack_words = stackWords;
    request.task_id = taskId;

    kronos_scheduler_request_service(KRONOS_SERVICE_TASK_CREATE, &request, 0U);
    return g_taskRuntime[g_currentTask].service_result;
}

kronos_status_e Kronos_TaskGetIdByName(const char *taskName, kronos_task_id_t *taskId)
{
    int32_t taskIndex;

    if (taskId != NULL)
    {
        *taskId = KRONOS_INVALID_TASK_ID;
    }

    if ((taskName == NULL) || (taskName[0] == '\0') || (taskId == NULL))
    {
        return KRONOS_STATUS_INVALID_ARGUMENT;
    }

    taskIndex = kronos_task_find_by_name(taskName);
    if (taskIndex < 0)
    {
        return KRONOS_STATUS_NOT_FOUND;
    }

    *taskId = (kronos_task_id_t)taskIndex;
    return KRONOS_STATUS_OK;
}

const TCB_t *Kronos_GetTaskTable(void)
{
    return g_tasks;
}

uint32_t Kronos_GetTaskCount(void)
{
    return g_numTasks;
}

/******************************************************************************
* EOF - NO CODE AFTER THIS LINE
******************************************************************************/
