# KronOS RTOS

KronOS is a small RTOS for STM32 microcontrollers, for these STM32 families:

- STM32L4
- STM32G4
- STM32L5
- STM32H7

The goal is to keep it simple:

- create tasks
- give each task its own stack size
- delay a task in milliseconds
- start a round-robin scheduler
- keep one internal idle task running when nothing else is ready

## What you can use today

The public API is in `kronos_core.h`.

Main functions:

- `RTOS_Init()`
- `RTOS_CreateTask()`
- `RTOS_Start()`
- `RTOS_Delay()`
- `RTOS_TaskPause()`
- `RTOS_TaskResume()`
- `RTOS_GetTaskTable()`
- `RTOS_GetTaskCount()`

## Very small example

```c
#include "kronos_core.h"

static void blink_task(void)
{
    for (;;)
    {
        /* Toggle your LED here */
        RTOS_Delay(250U);
    }
}

int main(void)
{
    /* Clock init, GPIO init, board init... */

    RTOS_Init();

    if (RTOS_CreateTask(blink_task, 64U, "blink") < 0)
    {
        for (;;)
        {
        }
    }

    RTOS_Start();

    for (;;)
    {
    }
}
```

Notes:

- Task functions must be `void task(void)`.
- A task should normally run in an infinite loop.
- `RTOS_Start()` does not return.
- The `stackWords` value is in 32-bit words, not bytes.
- Example: `64U` means 64 words = 256 bytes requested by the task.

## How to add KronOS to your own project

### 1. Copy the RTOS folder

Copy the full source tree into your project under:

```text
Libs/KrRtos
```

This is the easiest way, because it includes the public headers, internal headers, core sources, and STM32 port files.

### 2. Add the source files to your build

At minimum, add these core files:

```text
Libs/KrRtos/kronos_core.c
Libs/KrRtos/kronos_tasks.c
Libs/KrRtos/kronos_scheduler.c
```

For the port layer, the simple option is to add all files from:

```text
Libs/KrRtos/Port
```

That works because the preprocessor selects the active STM32 family.

If you prefer a smaller build, use:

- `kronos_port_stm32_armv7m.c` plus one family file for STM32L4, STM32G4, or STM32H7
or
- `kronos_port_stm32l5.c` for STM32L5

### 3. Add include paths

Your project must include these folders:

```text
Libs/KrRtos
Libs/KrRtos/Port
```

Application code usually only needs:

```c
#include "kronos_core.h"
```

### 4. Select the STM32 family

The port looks for one STM32 family define.

For example:

- `RTE_DEVICE_STARTUP_STM32L4XX`
- `RTE_DEVICE_STARTUP_STM32G4XX`
- `RTE_DEVICE_STARTUP_STM32L5XX`
- `RTE_DEVICE_STARTUP_STM32H7XX`

It also accepts the usual STM32 family macros like `STM32L4xx`, `STM32G4xx`, `STM32L5xx`, or `STM32H7xx`.

You only need one active family.

### 5. Make sure `SystemCoreClock` is valid

KronOS uses `SystemCoreClock` to configure SysTick.

Your project must provide:

```c
uint32_t SystemCoreClock;
```

In a normal STM32 project, this already exists in your system file.

Important:

- `SystemCoreClock` must match the real CPU clock before `RTOS_Start()` runs.
- If you change the clock, update `SystemCoreClock` first.
- KronOS does not require your `system_xxx.c` file to include RTOS headers.

### 6. Add the task stack section to the linker script

KronOS stores task stacks in a dedicated section called `.kronos_task_stacks`.

Your linker script must contain a section like this:

```ld
.kronos_task_stacks (NOLOAD) :
{
  . = ALIGN(8192);
  *(.kronos_task_stacks)
  *(.kronos_task_stacks*)
  . = ALIGN(32);
} >RAM
```

In this sample, the section is placed in `RAM` or `SRAM2`. The important part is that it is writable memory.

### 7. Make sure these exception handlers are not blocked by another file

The port provides these handlers:

- `SysTick_Handler`
- `PendSV_Handler`
- `SVC_Handler`
- `MemManage_Handler`

Your startup file should keep these handlers weak, or at least not define strong versions somewhere else.

If another file already owns these handlers, KronOS will not be able to run the scheduler correctly, so probably some integration will be required.

### 8. Create tasks before starting the scheduler

Typical order:

1. Initialize board and clock
2. Call `RTOS_Init()`
3. Create your application tasks
4. Call `RTOS_Start()`

Do not call `RTOS_CreateTask()` after the scheduler has started.

## Simple CMake example

```cmake
target_sources(app PRIVATE
    Libs/KrRtos/kronos_core.c
    Libs/KrRtos/kronos_tasks.c
    Libs/KrRtos/kronos_scheduler.c
    Libs/KrRtos/Port/kronos_port_stm32_armv7m.c
    Libs/KrRtos/Port/kronos_port_stm32l4.c
    Libs/KrRtos/Port/kronos_port_stm32g4.c
    Libs/KrRtos/Port/kronos_port_stm32l5.c
    Libs/KrRtos/Port/kronos_port_stm32h7.c
)

target_include_directories(app PRIVATE
    Libs/KrRtos
    Libs/KrRtos/Port
)
```

Then add the correct STM32 family define for your target.

## Things to remember

- There is always one internal idle task.
- Task stacks come from a fixed pool.
- If task creation fails, `RTOS_CreateTask()` returns a negative value.
- The default tick is `1000 Hz`, so `RTOS_Delay()` works in milliseconds in the current code.
- Stack size and limits can be changed in `Libs/KrRtos/kronos_core.h`.

If you want to add more kernel services later, this project is already split into core, task, scheduler, and port layers to make that easier.