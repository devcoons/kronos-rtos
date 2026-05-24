/*!
@file   kronos_port.h
@brief  Header file of KronOS (RTOS) Port functionalities
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

#ifndef KRONOS_PORT_H
#define KRONOS_PORT_H

/******************************************************************************
* Includes
******************************************************************************/

#include <stdint.h>

/******************************************************************************
* Preprocessor Definitions & Macros
******************************************************************************/

#define KRONOS_PORT_STACK_WARNING_MARGIN_WORDS 16U
#define KRONOS_PORT_STACK_RECOVERY_WORDS       32U
#define KRONOS_PORT_STACK_POOL_ALIGNMENT_BYTES 8192U

#define KRONOS_PORT_STACK_POOL_SECTION \
    __attribute__((section(".kronos_task_stacks"), aligned(KRONOS_PORT_STACK_POOL_ALIGNMENT_BYTES)))

/******************************************************************************
* Declaration | Internal Functions
******************************************************************************/

void kronos_port_init_scheduler(uint32_t tickFrequencyHz);
void kronos_port_start_scheduler(void);
void kronos_port_request_yield(void);
void kronos_port_pend_context_switch(void);
void kronos_port_configure_task_stack_region(const uint32_t *stackBase, uint32_t stackWords);
uint32_t *kronos_port_get_live_process_stack_pointer(void);
uint32_t kronos_port_compute_task_slot_words(uint32_t requestedStackWords);
uint32_t kronos_port_get_default_clock_hz(void);
uint32_t kronos_port_get_stack_warning_margin_words(void);

/******************************************************************************
* EOF - NO CODE AFTER THIS LINE
******************************************************************************/
#endif /* KRONOS_PORT_H */
