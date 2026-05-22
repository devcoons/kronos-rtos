/*!
@file   kronos_internal.h
@brief  Header file of KronOS (RTOS) Internal functionalities

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

#ifndef KRONOS_INTERNAL_H
#define KRONOS_INTERNAL_H

#include <stdint.h>

#include "kronos_core.h"
#include "kronos_kernel.h"
#include "kronos_port.h"

extern volatile uint32_t g_tickCount;
extern uint32_t g_schedulerStarted;
extern uint32_t g_stackPoolWordsUsed;

void Kronos_TasksResetState(void);
int32_t Kronos_TaskCreateInternal(void (*taskFunction)(void), uint32_t stackWords, const char *taskName, task_kind_e taskKind);
void Kronos_TaskUpdateAllStats(void);
void Kronos_TaskUpdateStackMetrics(TCB_t *tcb, const uint32_t *stackPtr);
kronos_stack_check_e Kronos_TaskCheckStack(TCB_t *tcb, const uint32_t *stackPtr);
void Kronos_TaskQuarantine(TCB_t *tcb, uint32_t *faultStackPtr, uint32_t faultFlags, uint32_t faultAddress);
int32_t Kronos_TaskFindNextReady(task_kind_e taskKind, uint32_t startTask);
uint32_t Kronos_TaskSelectStartTask(void);
void Kronos_TaskActivate(uint32_t taskIndex);

void Kronos_SchedulerResetState(void);

#endif /* KRONOS_INTERNAL_H */