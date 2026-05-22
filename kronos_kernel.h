/*!
@file   kronos_kernel.h
@brief  Header file of KronOS (RTOS) Kernel functionalities

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

#ifndef KRONOS_KERNEL_H
#define KRONOS_KERNEL_H

#include <stdint.h>

#include "kronos_core.h"

typedef enum
{
    KRONOS_STACK_CHECK_OK = 0,
    KRONOS_STACK_CHECK_WARNING = 1,
    KRONOS_STACK_CHECK_OVERFLOW = 2
} kronos_stack_check_e;

void Kronos_CoreOnTick(void);
kronos_stack_check_e Kronos_CoreCheckCurrentTaskStack(uint32_t *exceptionStackPtr);
uint32_t *Kronos_CorePrepareFirstTask(void);
uint32_t *Kronos_CoreSwitchTask(uint32_t *savedStackPtr);
uint32_t *Kronos_CoreQuarantineCurrentTask(uint32_t *faultStackPtr, uint32_t faultFlags, uint32_t faultAddress);

#endif /* KRONOS_KERNEL_H */