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
    kronos_task_create_request_t request;

    if (g_schedulerStarted == 0U)
    {
        return kronos_task_create_internal(taskFunction, stackWords, taskName, TASK_KIND_APPLICATION, NULL);
    }

    request.task_function = taskFunction;
    request.task_name = taskName;
    request.stack_words = stackWords;

    kronos_scheduler_request_service(KRONOS_SERVICE_TASK_CREATE, &request, 0U);
    return g_taskRuntime[g_currentTask].service_result;
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
