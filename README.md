# KronOS RTOS

KronOS is a small RTOS for STM32 microcontrollers.

Supported STM32 families:

- STM32L4
- STM32G4
- STM32L5
- STM32H7

The current sample project targets the NUCLEO-L476RG and blinks the onboard LED on GPIOA pin 5.

## What KronOS provides today

KronOS is intentionally small, but it already includes the main kernel services needed for simple embedded applications:

- fixed round-robin scheduling
- bounded ready-task selection using per-kind ready queues
- tick handling based on an ordered delay list instead of scanning every task
- one internal idle task
- per-task stack sizing at task creation
- task creation while the scheduler is running
- task deletion while the scheduler is running
- delay in milliseconds
- task pause and resume
- mutexes
- counting semaphores
- scheduler suspend, resume, and forced context switch
- mailbox-style task messaging with ingress and egress APIs
- privileged atomic driver initialization from a task
- per-task stack tracking and fault quarantine support

The public API is in `Libs/KronosRTOS/kronos_core.h`.

Naming convention:

- Public application APIs use `Kronos_*`, for example `Kronos_TaskCreate()`.
- Internal kernel and port routines use `kronos_*`, for example `kronos_task_create_internal()`.
- STM32 interrupt handlers keep their CMSIS-required names, such as `SysTick_Handler()`.

## Main API groups

Task control:

- `Kronos_Init()`
- `Kronos_TaskCreate()`
- `Kronos_TaskCreateWithId()`
- `Kronos_TaskGetIdByName()`
- `Kronos_TaskDelete()`
- `Kronos_TaskDeleteById()`
- `Kronos_Start()`
- `Kronos_Delay()`
- `Kronos_TaskPause()`
- `Kronos_TaskPauseById()`
- `Kronos_TaskResume()`
- `Kronos_TaskResumeById()`
- `Kronos_DriverInit()`
- `Kronos_GetTaskTable()`
- `Kronos_GetTaskCount()`

Synchronization:

- `Kronos_MutexInit()`
- `Kronos_MutexLock()`
- `Kronos_MutexUnlock()`
- `Kronos_SemaphoreInit()`
- `Kronos_SemaphoreTake()`
- `Kronos_SemaphoreGive()`
- `Kronos_SuspendScheduler()`
- `Kronos_ResumeScheduler()`
- `Kronos_ForceSwitch()`

Mailboxes:

- `Kronos_IngressResolve()`
- `Kronos_EgressSend()`
- `Kronos_EgressSendByName()`
- `Kronos_EgressBroadcast()`
- `Kronos_IngressReceive()`
- `Kronos_IngressWait()`
- `Kronos_IngressReceiveWait()`
- `Kronos_IngressPendingCount()`
- `Kronos_IngressPendingCountByName()`

Important mailbox notes:

- Task names must be unique.
- Task names are the normal mailbox identifiers.
- Messages carry a message ID plus a byte payload.
- `Kronos_IngressWait()` moves the current task into an internal waiting state until new mail arrives.
- `Kronos_EgressBroadcast()` sends one copy to each ingress-capable task.

## Very small task example

```c
#include "kronos_core.h"

static void blink_task(void)
{
    for (;;)
    {
        /* Toggle your LED here */
        Kronos_Delay(250U);
    }
}

int main(void)
{
    /* Clock init, GPIO init, board init... */

    Kronos_Init();

    if (Kronos_TaskCreate(blink_task, 64U, "blink") != KRONOS_STATUS_OK)
    {
        for (;;)
        {
        }
    }

    Kronos_Start();

    for (;;)
    {
    }
}
```

Notes:

- Task functions must be `void task(void)`.
- A task should normally run forever. If it returns, KronOS deletes/stops it through the cleanup path.
- `Kronos_Start()` is not expected to return.
- `stackWords` is in 32-bit words, not bytes.
- `64U` means 64 words, which is 256 bytes.
- `Kronos_TaskCreate()` returns a status code only.
- Name-based task control APIs are convenient for setup and diagnostics.
- ID-based task control APIs avoid repeated name lookup in production paths.

## Scheduler Determinism

KronOS keeps runnable tasks in fixed FIFO ready queues grouped by task kind: application, system, and idle. Selecting the next task checks those queue heads in a fixed order, so ready-task selection is bounded and does not scan the task table.

Delayed tasks are kept in wake-tick order. `SysTick_Handler()` increments the global tick and only checks the head of the delay list, waking expired tasks until the first non-expired task is found. The tick path no longer loops through every task just to maintain delays.

Task name lookup is still a linear search, so production code should resolve names once with `Kronos_TaskGetIdByName()` or create tasks with `Kronos_TaskCreateWithId()`, then use the `ById` APIs in repeated control paths.

## Simple mailbox example

```c
#include "kronos_core.h"

typedef struct
{
    uint32_t counter;
} counter_mail_t;

static void producer_task(void)
{
    counter_mail_t message;

    message.counter = 0U;

    for (;;)
    {
        message.counter++;
        (void)Kronos_EgressSendStruct("consumer", 1U, &message);
        Kronos_Delay(1000U);
    }
}

static void consumer_task(void)
{
    kronos_mail_t mail;
    const counter_mail_t *payload;

    for (;;)
    {
        if (Kronos_IngressReceiveWait(&mail) == KRONOS_STATUS_OK)
        {
            payload = (const counter_mail_t *)mail.payload;
            (void)payload;
            /* Use payload->counter here. */
        }
    }
}
```

## Build the sample project

Use the configured CMake presets:

```powershell
cmake --preset Debug
cmake --build --preset Debug
```

This sample is already configured for the STM32 GNU toolchain through the preset files.

## How to add KronOS to your own project

### 1. Copy the RTOS folder

Copy the full RTOS tree into your project:

```text
Libs/KrRtos
```

### 2. Add the source files to your build

Add these common RTOS sources:

```text
Libs/KrRtos/kronos_core.c
Libs/KrRtos/kronos_tasks.c
Libs/KrRtos/kronos_scheduler.c
Libs/KrRtos/kronos_sync.c
Libs/KrRtos/kronos_channels.c
```

For the port layer:

- use `Libs/KrRtos/Port/kronos_port_stm32_armv7m.c` plus one family file for STM32L4, STM32G4, or STM32H7
- use `Libs/KrRtos/Port/kronos_port_stm32l5.c` for STM32L5

Family configuration sources:

- `Libs/KrRtos/Port/kronos_port_stm32l4.c`
- `Libs/KrRtos/Port/kronos_port_stm32g4.c`
- `Libs/KrRtos/Port/kronos_port_stm32h7.c`

### 3. Add include paths

Your project must include:

```text
Libs/KrRtos
Libs/KrRtos/Port
```

Application code usually only needs:

```c
#include "kronos_core.h"
```

### 4. Select one STM32 family

The port needs one active STM32 family define.

Examples:

- `RTE_DEVICE_STARTUP_STM32L4XX`
- `RTE_DEVICE_STARTUP_STM32G4XX`
- `RTE_DEVICE_STARTUP_STM32L5XX`
- `RTE_DEVICE_STARTUP_STM32H7XX`

The usual STM32 family macros also work:

- `STM32L4xx`
- `STM32G4xx`
- `STM32L5xx`
- `STM32H7xx`

Only one family should be active.

### 5. Make sure `SystemCoreClock` is valid

KronOS uses `SystemCoreClock` when it configures SysTick.

Your project must provide:

```c
uint32_t SystemCoreClock;
```

Important:

- `SystemCoreClock` must match the real CPU clock before `Kronos_Start()`.
- If the clock changes, update `SystemCoreClock` first.
- KronOS does not require your `system_xxx.c` file to include RTOS headers.

### 6. Add the task stack section to the linker script

KronOS stores task stacks in `.kronos_task_stacks`.

Your linker script needs a writable section such as:

```ld
.kronos_task_stacks (NOLOAD) :
{
  . = ALIGN(8192);
  *(.kronos_task_stacks)
  *(.kronos_task_stacks*)
  . = ALIGN(32);
} >RAM
```

In this sample, the section is placed in SRAM2.

### 7. Make sure the RTOS handlers stay available

The port provides these handlers:

- `SysTick_Handler`
- `PendSV_Handler`
- `SVC_Handler`
- `MemManage_Handler`

Your startup code should leave these handlers weak, or at least not replace them by accident.

### 8. Create tasks before starting the scheduler

Typical order:

1. Initialize board and clock.
2. Call `Kronos_Init()`.
3. Create initial application tasks.
4. Call `Kronos_Start()`.

Tasks can also create or delete application tasks after the scheduler starts. Runtime creation and deletion are performed through the kernel service path.

## Simple CMake example

```cmake
target_sources(app PRIVATE
    Libs/KrRtos/kronos_core.c
    Libs/KrRtos/kronos_tasks.c
    Libs/KrRtos/kronos_scheduler.c
    Libs/KrRtos/kronos_sync.c
    Libs/KrRtos/kronos_channels.c
    Libs/KrRtos/Port/kronos_port_stm32_armv7m.c
    Libs/KrRtos/Port/kronos_port_stm32l4.c
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
- If task creation fails, `Kronos_TaskCreate()` returns a `kronos_status_e` error code.
- Task operations such as delete, pause, and resume are name-based.
- Deleted tasks release pending mailbox messages. If a deleted task owns a registered mutex, KronOS releases it and wakes one waiter.
- Mutexes and semaphores must be initialized before use. KronOS rejects uninitialized sync objects.
- The default tick is `1000 Hz`, so `Kronos_Delay()` uses milliseconds in the current code.
- Stack sizing and limits can be adjusted in `Libs/KrRtos/kronos_core.h`.
- Mailboxes use a fixed global slot pool and a bounded payload size.

KronOS stays split into core, task, scheduler, synchronization, channel, and port layers so it is easier to maintain and port.
