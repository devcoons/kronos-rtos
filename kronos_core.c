/*!
@file   kronos_core.c
@brief  Source file of KronOS (RTOS) Core functionalities

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

#include "kronos_internal.h"

void RTOS_Init(void)
{
    Kronos_SchedulerResetState();
    Kronos_TasksResetState();
    Kronos_TaskUpdateAllStats();
}

int32_t RTOS_CreateTask(void (*taskFunction)(void), uint32_t stackWords, const char *taskName)
{
    if (g_schedulerStarted != 0U)
    {
        return -1;
    }

    return Kronos_TaskCreateInternal(taskFunction, stackWords, taskName, TASK_KIND_APPLICATION);
}

const TCB_t *RTOS_GetTaskTable(void)
{
    return g_tasks;
}

uint32_t RTOS_GetTaskCount(void)
{
    return g_numTasks;
}