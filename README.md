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

The public API is in `Libs/KrRtos/kronos_core.h`.

## Main API groups

Task control:

- `RTOS_Init()`
- `RTOS_CreateTask()`
- `RTOS_TaskDelete()`
- `RTOS_TaskDelete(taskName)`
- `RTOS_Start()`
- `RTOS_Delay()`
- `RTOS_TaskPause()`
- `RTOS_TaskPause(taskName)`
- `RTOS_TaskResume()`
- `RTOS_TaskResume(taskName)`
- `RTOS_DriverInit()`
- `RTOS_GetTaskTable()`
- `RTOS_GetTaskCount()`

Synchronization:

- `RTOS_MutexInit()`
- `RTOS_MutexLock()`
- `RTOS_MutexUnlock()`
- `RTOS_SemaphoreInit()`
- `RTOS_SemaphoreTake()`
- `RTOS_SemaphoreGive()`
- `RTOS_SuspendScheduler()`
- `RTOS_ResumeScheduler()`
- `RTOS_ForceSwitch()`

Mailboxes:

- `RTOS_IngressResolve()`
- `RTOS_EgressSend()`
- `RTOS_EgressSendByName()`
- `RTOS_EgressBroadcast()`
- `RTOS_IngressReceive()`
- `RTOS_IngressWait()`
- `RTOS_IngressReceiveWait()`
- `RTOS_IngressPendingCount()`
- `RTOS_IngressPendingCountByName()`

Important mailbox notes:

- Task names must be unique.
- Task names are the normal mailbox identifiers.
- Messages carry a message ID plus a byte payload.
- `RTOS_IngressWait()` moves the current task into an internal waiting state until new mail arrives.
- `RTOS_EgressBroadcast()` sends one copy to each ingress-capable task.

## Very small task example

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

    if (RTOS_CreateTask(blink_task, 64U, "blink") != KRONOS_STATUS_OK)
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
- A task should normally run forever. If it returns, KronOS deletes/stops it through the cleanup path.
- `RTOS_Start()` is not expected to return.
- `stackWords` is in 32-bit words, not bytes.
- `64U` means 64 words, which is 256 bytes.
- `RTOS_CreateTask()` returns a status code only.
- Task control APIs use task names, not task IDs.

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
        (void)RTOS_EgressSendStruct("consumer", 1U, &message);
        RTOS_Delay(1000U);
    }
}

static void consumer_task(void)
{
    kronos_mail_t mail;
    const counter_mail_t *payload;

    for (;;)
    {
        if (RTOS_IngressReceiveWait(&mail) == KRONOS_STATUS_OK)
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

- `SystemCoreClock` must match the real CPU clock before `RTOS_Start()`.
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
2. Call `RTOS_Init()`.
3. Create initial application tasks.
4. Call `RTOS_Start()`.

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
- If task creation fails, `RTOS_CreateTask()` returns a `kronos_status_e` error code.
- Task operations such as delete, pause, and resume are name-based.
- Deleted tasks release pending mailbox messages. If a deleted task owns a registered mutex, KronOS releases it and wakes one waiter.
- Mutexes and semaphores must be initialized before use. KronOS rejects uninitialized sync objects.
- The default tick is `1000 Hz`, so `RTOS_Delay()` uses milliseconds in the current code.
- Stack sizing and limits can be adjusted in `Libs/KrRtos/kronos_core.h`.
- Mailboxes use a fixed global slot pool and a bounded payload size.

KronOS stays split into core, task, scheduler, synchronization, channel, and port layers so it is easier to maintain and port.
