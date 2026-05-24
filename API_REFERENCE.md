# KronOS API Reference

This document describes the exposed KronOS RTOS API and the internal kernel and port interfaces.

Public APIs are intended for application code. Internal APIs are intended for the kernel and port layer. Do not call internal APIs from application tasks unless you are modifying KronOS itself.

## Table of Contents

- [Quick Model](#quick-model)
- [Status Codes](#status-codes)
- [Public API](#public-api)
  - [Task and Kernel Control](#task-and-kernel-control)
  - [Scheduler Control](#scheduler-control)
  - [Task Messaging](#task-messaging)
  - [Synchronization](#synchronization)
  - [Task Introspection](#task-introspection)
  - [Public Convenience Macros](#public-convenience-macros)
- [Port API](#port-api)
- [Internal API](#internal-api)
  - [Task Internals](#task-internals)
  - [Scheduler and Core Internals](#scheduler-and-core-internals)
  - [Channel Internals](#channel-internals)
  - [Synchronization Internals](#synchronization-internals)
- [Interrupt Handlers](#interrupt-handlers)
- [Data Structures](#data-structures)
  - [Public Data Structures](#public-data-structures)
  - [Internal Data Structures](#internal-data-structures)
  - [Port Data Structures](#port-data-structures)
- [Enumerations](#enumerations)
- [Constants and Configuration Macros](#constants-and-configuration-macros)
- [Global Variables](#global-variables)
- [Examples](#examples)

## Quick Model

KronOS is a small fixed round-robin RTOS for STM32 microcontrollers.

Core behavior:

- Initial tasks are usually created before the scheduler starts.
- One idle task is created internally by `Kronos_Init()`.
- The scheduler uses a 1 kHz tick by default.
- Ready tasks are selected from fixed FIFO queues by task kind, so ready selection is bounded.
- Delayed tasks are stored in wake-tick order, so the tick path checks only expired delay-list entries instead of scanning all tasks.
- Task stacks are allocated from a fixed stack pool.
- Each task has an ingress mailbox queue.
- Messages are copied into fixed-size global mail slots.
- Mutex, semaphore, delay, messaging, and scheduler control services are handled through supervisor calls.
- Tasks can be created and deleted after the scheduler starts.
- Driver initialization can be requested from a task and executed atomically in kernel context.
- The port layer configures SysTick, SVC, PendSV, MemManage, and MPU regions.

Normal application order:

```c
Kronos_Init();
{
    (void)Kronos_TaskCreate(task_a, 64U, "task_a");
    (void)Kronos_TaskCreate(task_b, 64U, "task_b");
}
Kronos_Start();
```

## Status Codes

Status-returning public APIs return `kronos_status_e`.

| Code | Value | Meaning |
| --- | ---: | --- |
| `KRONOS_STATUS_OK` | `0` | Operation completed successfully. |
| `KRONOS_STATUS_ERROR` | `-1` | Generic error or invalid runtime state. |
| `KRONOS_STATUS_INVALID_ARGUMENT` | `-2` | A pointer, name, count, or size argument is invalid. |
| `KRONOS_STATUS_NOT_OWNER` | `-3` | The current task tried to unlock a mutex it does not own. |
| `KRONOS_STATUS_SCHEDULER_SUSPENDED` | `-4` | The call would block, but the scheduler is suspended. |
| `KRONOS_STATUS_OVERFLOW` | `-5` | A counter would exceed its maximum value. |
| `KRONOS_STATUS_NOT_FOUND` | `-6` | A named task or target could not be found. |
| `KRONOS_STATUS_EMPTY` | `-7` | A mailbox has no pending message. |
| `KRONOS_STATUS_TOO_LARGE` | `-8` | A message payload is larger than `KRONOS_MESSAGE_MAX_PAYLOAD_BYTES`. |
| `KRONOS_STATUS_MAILBOX_FULL` | `-9` | No global mail slot is available for delivery. |
| `KRONOS_STATUS_INVALID_STATE` | `-10` | The object or task is valid but not in a state where the operation is allowed. |
| `KRONOS_STATUS_NO_MEMORY` | `-11` | No task slot, stack slot, mailbox slot, or registry slot is available. |
| `KRONOS_STATUS_IN_USE` | `-12` | The resource already exists or cannot be reinitialized while active. |

Functions that return counts or pointers keep their natural return type.

## Public API

Public APIs are declared in `kronos_core.h`. Application code normally includes only:

```c
#include "kronos_core.h"
```

### Task and Kernel Control

#### `Kronos_Init`

```c
void Kronos_Init(void);
```

Visibility: Public application API.

Inputs: None.

Outputs: None.

What it does: Resets scheduler state, task state, channel state, and stack statistics. It also creates the internal idle task.

How it works: Calls the scheduler, task, and channel reset routines. `kronos_tasks_reset_state()` creates the idle task named `"kronos_idle"`.

Return values: None.

Notes:

- Call this once before creating application tasks.
- Calling this while the scheduler is running is not a supported application flow.
- Existing tasks and mailboxes are reset.

Example:

```c
int main(void)
{
    board_init();
    Kronos_Init();
    (void)Kronos_TaskCreate(app_task, 64U, "app");
    Kronos_Start();
}
```

#### `Kronos_TaskCreate`

```c
kronos_status_e Kronos_TaskCreate(void (*taskFunction)(void),
                                uint32_t stackWords,
                                const char *taskName);
```

Visibility: Public application API.

Inputs:

| Input | Description |
| --- | --- |
| `taskFunction` | Task entry function. It must match `void task(void)`. |
| `stackWords` | Requested task stack size in 32-bit words. `0` uses `DEFAULT_STACK_SIZE_WORDS`. |
| `taskName` | Unique non-empty task name string. |

Outputs: Creates one task control block and one stack slot.

What it does: Creates an application task in the ready state.

How it works: Before the scheduler starts, the task is created directly. After the scheduler starts, the request is handled through SVC in kernel context. KronOS validates the task, rounds the stack size to an even word count, allocates or reuses a port-sized stack slot, builds the initial exception frame, places the task in `g_tasks`, and appends it to the ready queue for its task kind.

Return values:

| Return | Meaning |
| --- | --- |
| `KRONOS_STATUS_OK` | Task created successfully. |
| `KRONOS_STATUS_INVALID_ARGUMENT` | Null task function, null/empty name, or too-small stack. |
| `KRONOS_STATUS_IN_USE` | Another active task already uses the same name. |
| `KRONOS_STATUS_NO_MEMORY` | No task slot or stack pool space is available. |

Notes:

- Tasks can be created before or after `Kronos_Start()`.
- `stackWords` is words, not bytes. `64U` means 256 bytes.
- Task names must be unique.
- Deleted task slots may be reused.
- A task should normally run forever. If it returns, KronOS routes it through task deletion cleanup and stops it.

Example:

```c
static void sensor_task(void)
{
    for (;;)
    {
        read_sensor();
        Kronos_Delay(10U);
    }
}

if (Kronos_TaskCreate(sensor_task, 96U, "sensor") != KRONOS_STATUS_OK)
{
    error_handler();
}
```

#### `Kronos_TaskCreateWithId`

```c
kronos_status_e Kronos_TaskCreateWithId(void (*taskFunction)(void),
                                        uint32_t stackWords,
                                        const char *taskName,
                                        kronos_task_id_t *taskId);
```

Visibility: Public application API.

What it does: Creates an application task and optionally returns its task ID.

Use case: Prefer this over name-only creation when production code needs to control the task later without repeated name lookup.

Return values: Same as `Kronos_TaskCreate()`. If `taskId` is non-null, it is set to `KRONOS_INVALID_TASK_ID` before validation and to the created task ID on success.

Example:

```c
kronos_task_id_t worker_id;

if (Kronos_TaskCreateWithId(worker_task, 96U, "worker", &worker_id) != KRONOS_STATUS_OK)
{
    error_handler();
}
```

#### `Kronos_TaskGetIdByName`

```c
kronos_status_e Kronos_TaskGetIdByName(const char *taskName,
                                       kronos_task_id_t *taskId);
```

Visibility: Public application API.

What it does: Resolves a task name to a task ID.

Notes:

- Name lookup is linear over the active task table.
- Resolve names during setup or diagnostics; use `ById` APIs in repeated runtime paths.

Return values:

| Return | Meaning |
| --- | --- |
| `KRONOS_STATUS_OK` | Task ID resolved. |
| `KRONOS_STATUS_INVALID_ARGUMENT` | Null/empty name or null output pointer. |
| `KRONOS_STATUS_NOT_FOUND` | No active task has that name. |

#### `Kronos_TaskDelete`

```c
kronos_status_e Kronos_TaskDelete(const char *taskName);
```

Visibility: Public application API.

Inputs:

| Input | Description |
| --- | --- |
| `taskName` | Unique task name to delete. |

Outputs: Marks the named task stopped and cleans task-owned runtime resources.

What it does: Deletes an application task before or after the scheduler starts.

How it works: Before the scheduler starts, deletion runs directly. After the scheduler starts, the request is handled through SVC in kernel context. KronOS resolves the task name to the internal task slot, rejects the idle task, removes pending mailbox messages where the task is sender or receiver, releases any registered mutex owned by the task, clears wait/service state, and marks the task slot stopped for later reuse.

Return values:

| Return | Meaning |
| --- | --- |
| `KRONOS_STATUS_OK` | Task deleted. |
| `KRONOS_STATUS_NOT_FOUND` | Task name is invalid or the task is already stopped. |
| `KRONOS_STATUS_INVALID_ARGUMENT` | The task is the internal idle task. |

Notes:

- A task may delete itself. On successful self-delete, the call does not return to that task.
- Deleting a task does not compact the task table. The slot can later be reused.
- Mutex cleanup only applies to mutexes initialized with `Kronos_MutexInit()`.

Example:

```c
if (Kronos_TaskDelete("sensor") != KRONOS_STATUS_OK)
{
    error_handler();
}
```

#### `Kronos_TaskDeleteById`

```c
kronos_status_e Kronos_TaskDeleteById(kronos_task_id_t taskId);
```

Visibility: Public application API.

What it does: Deletes an application task by task ID.

Use case: Production control paths that already have a task ID from `Kronos_TaskCreateWithId()` or `Kronos_TaskGetIdByName()`.

Return values: Same deletion status values as `Kronos_TaskDelete()`, except lookup is by ID.

#### `Kronos_Start`

```c
void Kronos_Start(void);
```

Visibility: Public application API.

Inputs: None.

Outputs: Starts the scheduler and does not return in normal operation.

What it does: Starts SysTick and switches execution to the first selected task.

How it works: Initializes the port scheduler with `TICK_FREQ_HZ`, selects the first ready task, activates it, and asks the port layer to start the scheduler through SVC.

Return values: None.

Notes:

- The current implementation creates an idle task during `Kronos_Init()`, so there is normally at least one task.
- `Kronos_Start()` is not expected to return.
- Application tasks must already be created.

Example:

```c
Kronos_Init();
{
    (void)Kronos_TaskCreate(control_task, 128U, "control");
}
Kronos_Start();
```

#### `Kronos_Delay`

```c
void Kronos_Delay(uint32_t ms);
```

Visibility: Public application API.

Inputs:

| Input | Description |
| --- | --- |
| `ms` | Delay time in scheduler ticks. With the default 1 kHz tick, this is milliseconds. |

Outputs: May block the current task until the delay expires.

What it does: Delays the current task or yields when `ms == 0`.

How it works: For a non-zero delay, KronOS requests the delay service. The scheduler stores the delay in `remaining_delay`, moves the task to `TASK_STATE_TIME_DELAY`, inserts it into the ordered delay list, and selects another ready task. SysTick checks the delay-list head and wakes expired tasks without scanning the whole task table.

Return values: None.

Notes:

- If `ms == 0`, this is a forced yield.
- If the scheduler is suspended, the function busy-waits on `g_tickCount` instead of blocking the task.
- Do not use long delays while the scheduler is suspended.

Example:

```c
for (;;)
{
    toggle_led();
    Kronos_Delay(500U);
}
```

#### `Kronos_TaskPause`

```c
kronos_status_e Kronos_TaskPause(const char *taskName);
```

Visibility: Public application API.

Inputs:

| Input | Description |
| --- | --- |
| `taskName` | Unique task name to pause. |

Outputs: Changes the named task state to `TASK_STATE_PAUSED`.

What it does: Pauses another task.

How it works: Before the scheduler starts, the task is paused directly. After the scheduler starts, the request is handled through SVC. KronOS resolves the task name, rejects the current task and idle task, removes a ready task from its ready queue or a delayed task from the delay list, then changes it to paused.

Return values:

| Return | Meaning |
| --- | --- |
| `KRONOS_STATUS_OK` | Task is paused, or it was already paused. |
| `KRONOS_STATUS_INVALID_ARGUMENT` | Invalid task, current task, or idle task. |
| `KRONOS_STATUS_INVALID_STATE` | Task is not ready, delayed, or already paused. |

Notes:

- A task cannot pause itself with this API.
- Pausing a delayed task preserves its remaining delay counter.

Example:

```c
void stop_worker_temporarily(void)
{
    (void)Kronos_TaskPause("worker");
}
```

#### `Kronos_TaskPauseById`

```c
kronos_status_e Kronos_TaskPauseById(kronos_task_id_t taskId);
```

Visibility: Public application API.

What it does: Pauses another task by task ID.

Return values: Same pause status values as `Kronos_TaskPause()`, except lookup is by ID.

#### `Kronos_TaskResume`

```c
kronos_status_e Kronos_TaskResume(const char *taskName);
```

Visibility: Public application API.

Inputs:

| Input | Description |
| --- | --- |
| `taskName` | Unique task name returned by the application-level naming convention. |

Outputs: Moves a paused task back to ready or delayed state.

What it does: Resumes a paused task.

How it works: Before the scheduler starts, the task is resumed directly. After the scheduler starts, the request is handled through SVC. KronOS resolves the task name and, if the paused task still has `remaining_delay > 0`, it is reinserted into the delay list. Otherwise it becomes ready and is appended to its ready queue. If the scheduler is suspended, a pending switch is recorded.

Return values:

| Return | Meaning |
| --- | --- |
| `KRONOS_STATUS_OK` | Task resumed. |
| `KRONOS_STATUS_NOT_FOUND` | Task name is invalid or the task is stopped. |
| `KRONOS_STATUS_INVALID_STATE` | Task is not paused. |

Example:

```c
(void)Kronos_TaskResume("worker");
```

#### `Kronos_TaskResumeById`

```c
kronos_status_e Kronos_TaskResumeById(kronos_task_id_t taskId);
```

Visibility: Public application API.

What it does: Resumes a paused task by task ID.

Return values: Same resume status values as `Kronos_TaskResume()`, except lookup is by ID.

### Scheduler Control

#### `Kronos_SuspendScheduler`

```c
void Kronos_SuspendScheduler(void);
```

Visibility: Public application API.

Inputs: None.

Outputs: Increments scheduler suspend depth.

What it does: Prevents context switching while the current task runs a critical sequence.

How it works: Requests `KRONOS_SERVICE_SUSPEND_SCHEDULER`. The scheduler increments `g_schedulerSuspendDepth`.

Return values: None.

Notes:

- Calls are nestable.
- Blocking APIs return `KRONOS_STATUS_SCHEDULER_SUSPENDED` when they would need to block.
- Keep suspended sections short.

Example:

```c
Kronos_SuspendScheduler();
shared_state_update();
Kronos_ResumeScheduler();
```

#### `Kronos_ResumeScheduler`

```c
void Kronos_ResumeScheduler(void);
```

Visibility: Public application API.

Inputs: None.

Outputs: Decrements scheduler suspend depth.

What it does: Re-enables scheduler switching when the suspend depth reaches zero.

How it works: Requests `KRONOS_SERVICE_RESUME_SCHEDULER`. If a context switch was requested while suspended, the scheduler performs it after the final resume.

Return values: None.

Notes:

- Must match earlier `Kronos_SuspendScheduler()` calls.
- If called when not suspended, the internal service result becomes `KRONOS_STATUS_ERROR`, but the public function has no return value.

#### `Kronos_ForceSwitch`

```c
void Kronos_ForceSwitch(void);
```

Visibility: Public application API.

Inputs: None.

Outputs: Requests a context switch.

What it does: Yields the current task voluntarily.

How it works: Requests `KRONOS_SERVICE_FORCE_SWITCH`, which enters the scheduler and selects the next ready task.

Return values: None.

Example:

```c
while (work_queue_empty())
{
    Kronos_ForceSwitch();
}
```

#### `Kronos_DriverInit`

```c
kronos_status_e Kronos_DriverInit(kronos_driver_init_fn_t initFunction,
                                void *context);
```

Visibility: Public application API.

Inputs:

| Input | Description |
| --- | --- |
| `initFunction` | Driver initialization function to run in kernel context. |
| `context` | Caller-provided pointer passed to `initFunction`. May be null if the init function does not need context. |

Outputs: Executes the driver initialization function and returns its status.

What it does: Allows an unprivileged task to request privileged driver or peripheral initialization.

How it works: Before the scheduler starts, KronOS calls the function directly. After the scheduler starts, KronOS sends an SVC request. The STM32 SVC handler runs with interrupts disabled while `initFunction(context)` executes, so no task switch can happen during the initialization.

Return values:

| Return | Meaning |
| --- | --- |
| `KRONOS_STATUS_OK` | Driver init function completed successfully. |
| `KRONOS_STATUS_INVALID_ARGUMENT` | `initFunction` is null. |
| Other `kronos_status_e` | Status returned by `initFunction`. |

Notes:

- The init function must be short and deterministic.
- The init function must not call blocking RTOS APIs.
- This service is intended for peripheral setup that must run privileged and atomically.

Example:

```c
static kronos_status_e uart_driver_init(void *context)
{
    uart_config_t *config = (uart_config_t *)context;

    uart_register_init(config);
    return KRONOS_STATUS_OK;
}

void app_task(void)
{
    uart_config_t config = { 115200U };

    (void)Kronos_DriverInit(uart_driver_init, &config);
    for (;;)
    {
        Kronos_Delay(1000U);
    }
}
```

### Task Messaging

Task messaging uses per-task ingress queues and egress send APIs. A message contains a message ID, sender/receiver metadata, and a copied byte payload.

#### `Kronos_IngressResolve`

```c
kronos_status_e Kronos_IngressResolve(kronos_ingress_t *ingress,
                                    const char *taskName);
```

Visibility: Public application API.

Inputs:

| Input | Description |
| --- | --- |
| `ingress` | Output handle to fill. |
| `taskName` | Name of the target task. |

Outputs: Writes a stable ingress handle containing task ID and task name pointer.

What it does: Resolves a task name into an ingress handle that can be reused for faster sends.

How it works: Finds the task by name and verifies that it can accept ingress messages.

Return values:

| Return | Meaning |
| ---: | --- |
| `KRONOS_STATUS_OK` | Handle resolved. |
| `KRONOS_STATUS_INVALID_ARGUMENT` | `ingress` is null or `taskName` is null/empty. |
| `KRONOS_STATUS_NOT_FOUND` | No task with that name exists. |
| `KRONOS_STATUS_ERROR` | The task exists but cannot accept ingress messages. |

Example:

```c
kronos_ingress_t logger;

if (Kronos_IngressResolve(&logger, "logger") == KRONOS_STATUS_OK)
{
    (void)Kronos_EgressSend(&logger, 10U, "boot", 4U);
}
```

#### `Kronos_EgressSend`

```c
kronos_status_e Kronos_EgressSend(const kronos_ingress_t *ingress,
                                uint32_t messageId,
                                const void *payloadPtr,
                                uint32_t payloadSize);
```

Visibility: Public application API.

Inputs:

| Input | Description |
| --- | --- |
| `ingress` | Target ingress handle from `Kronos_IngressResolve()`. |
| `messageId` | Application-defined message ID. |
| `payloadPtr` | Pointer to payload bytes. May be null only when `payloadSize == 0`. |
| `payloadSize` | Number of bytes to copy into the message. |

Outputs: Queues one message into the target task ingress queue.

What it does: Sends a copied message to a previously resolved task ingress.

How it works: Requests the egress send service. The kernel validates that the handle still matches the task name, allocates a global mail slot, copies payload bytes, queues the slot, and wakes the receiver if it was waiting for ingress.

Return values:

| Return | Meaning |
| ---: | --- |
| `KRONOS_STATUS_OK` | Message queued. |
| `KRONOS_STATUS_INVALID_ARGUMENT` | Null ingress, null payload with non-zero size, or invalid service request. |
| `KRONOS_STATUS_ERROR` | Scheduler not started, invalid current task, or target cannot accept ingress. |
| `KRONOS_STATUS_NOT_FOUND` | Ingress handle no longer matches a valid task. |
| `KRONOS_STATUS_TOO_LARGE` | Payload is too large. |
| `KRONOS_STATUS_MAILBOX_FULL` | No mail slot is free. |

Example:

```c
typedef struct
{
    uint32_t value;
} sample_msg_t;

sample_msg_t msg = { .value = 42U };
(void)Kronos_EgressSend(&consumer_ingress, 1U, &msg, sizeof(msg));
```

#### `Kronos_EgressSendByName`

```c
kronos_status_e Kronos_EgressSendByName(const char *taskName,
                                      uint32_t messageId,
                                      const void *payloadPtr,
                                      uint32_t payloadSize);
```

Visibility: Public application API.

Inputs:

| Input | Description |
| --- | --- |
| `taskName` | Name of target task. |
| `messageId` | Application-defined message ID. |
| `payloadPtr` | Pointer to payload bytes. May be null only when `payloadSize == 0`. |
| `payloadSize` | Number of bytes to copy. |

Outputs: Queues one message into the named task ingress queue.

What it does: Sends a copied message by task name.

How it works: Looks up the task by name during the scheduler service, allocates a mail slot, copies the payload, and queues the message.

Return values: Same as `Kronos_EgressSend()`, except name lookup is used instead of an ingress handle.

Notes:

- This is simpler than `Kronos_EgressSend()`.
- Use `Kronos_IngressResolve()` plus `Kronos_EgressSend()` when sending repeatedly to the same task.

Example:

```c
uint32_t command = 7U;
(void)Kronos_EgressSendByName("motor", 100U, &command, sizeof(command));
```

#### `Kronos_EgressBroadcast`

```c
kronos_status_e Kronos_EgressBroadcast(uint32_t messageId,
                                     const void *payloadPtr,
                                     uint32_t payloadSize);
```

Visibility: Public application API.

Inputs:

| Input | Description |
| --- | --- |
| `messageId` | Application-defined message ID. |
| `payloadPtr` | Pointer to payload bytes. May be null only when `payloadSize == 0`. |
| `payloadSize` | Number of bytes to copy to each receiver. |

Outputs: Queues one copy of the message for every ingress-capable task.

What it does: Broadcasts a copied message to all tasks that can accept ingress.

How it works: Counts broadcast targets first. If enough global mail slots are free, it delivers one mail slot to each target.

Return values:

| Return | Meaning |
| ---: | --- |
| `KRONOS_STATUS_OK` | Broadcast queued to all targets. |
| `KRONOS_STATUS_ERROR` | Scheduler not started, invalid current task, or target delivery failed. |
| `KRONOS_STATUS_INVALID_ARGUMENT` | Null payload with non-zero size. |
| `KRONOS_STATUS_TOO_LARGE` | Payload is too large. |
| `KRONOS_STATUS_NOT_FOUND` | No ingress-capable target exists. |
| `KRONOS_STATUS_MAILBOX_FULL` | Not enough free mail slots for all targets. |

Notes:

- The current task can receive its own broadcast if it accepts ingress.
- The idle task does not accept ingress.
- Broadcast is all-or-nothing with respect to mail slot availability.

Example:

```c
uint32_t mode = 2U;
(void)Kronos_EgressBroadcast(200U, &mode, sizeof(mode));
```

#### `Kronos_IngressReceive`

```c
kronos_status_e Kronos_IngressReceive(kronos_mail_t *message);
```

Visibility: Public application API.

Inputs:

| Input | Description |
| --- | --- |
| `message` | Destination for received mail metadata and payload. |

Outputs: Fills `*message` and removes one pending message from the current task ingress queue.

What it does: Tries to receive one message without waiting.

How it works: Requests the ingress receive service. The kernel pops the head mail slot for the current task, copies it into `kronos_mail_t`, then releases the slot back to the global free list.

Return values:

| Return | Meaning |
| ---: | --- |
| `KRONOS_STATUS_OK` | Message received. |
| `KRONOS_STATUS_EMPTY` | No pending message. |
| `KRONOS_STATUS_INVALID_ARGUMENT` | `message` is null. |
| `KRONOS_STATUS_ERROR` | Scheduler not started or current task is invalid. |

Example:

```c
kronos_mail_t mail;

if (Kronos_IngressReceive(&mail) == KRONOS_STATUS_OK)
{
    handle_message(mail.message_id, mail.payload, mail.payload_size);
}
```

#### `Kronos_IngressWait`

```c
kronos_status_e Kronos_IngressWait(void);
```

Visibility: Public application API.

Inputs: None.

Outputs: Blocks the current task until at least one ingress message is pending.

What it does: Waits for mail.

How it works: If the current ingress queue already has mail, it returns immediately. Otherwise, the current task is moved to `TASK_STATE_WAITING` with reason `KRONOS_WAIT_INGRESS`.

Return values:

| Return | Meaning |
| ---: | --- |
| `KRONOS_STATUS_OK` | Mail is available or the task was successfully blocked and later resumed. |
| `KRONOS_STATUS_ERROR` | Scheduler not started or current task is invalid. |
| `KRONOS_STATUS_SCHEDULER_SUSPENDED` | The call would block while the scheduler is suspended. |

Notes:

- This call does not copy a message. Call `Kronos_IngressReceive()` after it.
- Most applications should use `Kronos_IngressReceiveWait()`.

Example:

```c
if (Kronos_IngressWait() == KRONOS_STATUS_OK)
{
    (void)Kronos_IngressReceive(&mail);
}
```

#### `Kronos_IngressReceiveWait`

```c
kronos_status_e Kronos_IngressReceiveWait(kronos_mail_t *message);
```

Visibility: Public application API.

Inputs:

| Input | Description |
| --- | --- |
| `message` | Destination for received mail metadata and payload. |

Outputs: Fills `*message` with the next received message.

What it does: Receives one message, waiting if the ingress queue is empty.

How it works: Calls `Kronos_IngressReceive()`. If the queue is empty, it calls `Kronos_IngressWait()` and retries.

Return values:

| Return | Meaning |
| ---: | --- |
| `KRONOS_STATUS_OK` | Message received. |
| `KRONOS_STATUS_INVALID_ARGUMENT` | `message` is null. |
| Other negative status | Error from receive or wait. |

Example:

```c
for (;;)
{
    kronos_mail_t mail;

    if (Kronos_IngressReceiveWait(&mail) == KRONOS_STATUS_OK)
    {
        dispatch_mail(&mail);
    }
}
```

#### `Kronos_IngressPendingCount`

```c
uint32_t Kronos_IngressPendingCount(void);
```

Visibility: Public application API.

Inputs: None.

Outputs: None.

What it does: Returns the number of pending messages for the current task.

How it works: Reads `g_ingressState[g_currentTask].pending_count`.

Return values:

| Return | Meaning |
| ---: | --- |
| `0..N` | Pending message count for the current task. |
| `0` | Also returned if the current task index is invalid. |

Example:

```c
while (Kronos_IngressPendingCount() > 0U)
{
    (void)Kronos_IngressReceive(&mail);
}
```

#### `Kronos_IngressPendingCountByName`

```c
kronos_status_e Kronos_IngressPendingCountByName(const char *taskName,
                                               uint32_t *pendingCount);
```

Visibility: Public application API.

Inputs:

| Input | Description |
| --- | --- |
| `taskName` | Name of the task to inspect. |
| `pendingCount` | Output pointer for the pending message count. |

Outputs: Writes the target task pending ingress count.

What it does: Reads another task's ingress pending count by name.

How it works: Finds the task by name and copies its ingress queue count.

Return values:

| Return | Meaning |
| ---: | --- |
| `KRONOS_STATUS_OK` | Count written. |
| `KRONOS_STATUS_INVALID_ARGUMENT` | Null/empty name or null output pointer. |
| `KRONOS_STATUS_NOT_FOUND` | No task with that name exists. |

Example:

```c
uint32_t pending = 0U;

if (Kronos_IngressPendingCountByName("logger", &pending) == KRONOS_STATUS_OK)
{
    monitor_logger_depth(pending);
}
```

### Synchronization

#### `Kronos_MutexInit`

```c
kronos_status_e Kronos_MutexInit(kronos_mutex_t *mutex);
```

Visibility: Public application API.

Inputs:

| Input | Description |
| --- | --- |
| `mutex` | Mutex object to initialize. |

Outputs: Initializes `owner_task_id`, `lock_count`, and the internal initialization marker.

What it does: Prepares a mutex for use.

How it works: If the mutex is already registered and active, KronOS rejects reinitialization. Otherwise it sets owner to `KRONOS_INVALID_TASK_ID`, sets lock count to zero, marks the mutex initialized, and registers it for task-deletion cleanup.

Return values:

| Return | Meaning |
| --- | --- |
| `KRONOS_STATUS_OK` | Mutex initialized. |
| `KRONOS_STATUS_INVALID_ARGUMENT` | `mutex` is null. |
| `KRONOS_STATUS_IN_USE` | Mutex is locked or has waiters. |
| `KRONOS_STATUS_NO_MEMORY` | The sync object registry is full. |

Example:

```c
static kronos_mutex_t i2c_mutex;

void app_init(void)
{
    (void)Kronos_MutexInit(&i2c_mutex);
}
```

#### `Kronos_MutexLock`

```c
kronos_status_e Kronos_MutexLock(kronos_mutex_t *mutex);
```

Visibility: Public application API.

Inputs:

| Input | Description |
| --- | --- |
| `mutex` | Mutex to lock. |

Outputs: The current task becomes mutex owner or waits for the mutex.

What it does: Locks a mutex. The same owner can lock it recursively.

How it works: Requests the mutex lock service. If unlocked, the current task becomes owner. If already owned by the current task, `lock_count` increments. If owned by another task, the current task blocks unless the scheduler is suspended.

Return values:

| Return | Meaning |
| ---: | --- |
| `KRONOS_STATUS_OK` | Lock acquired, recursive lock accepted, or task blocked and later resumed. |
| `KRONOS_STATUS_INVALID_ARGUMENT` | Mutex is null, uninitialized, or corrupted. |
| `KRONOS_STATUS_ERROR` | Scheduler not started or current task invalid. |
| `KRONOS_STATUS_OVERFLOW` | Recursive lock count would overflow. |
| `KRONOS_STATUS_SCHEDULER_SUSPENDED` | Mutex is owned by another task and blocking is not allowed. |

Example:

```c
if (Kronos_MutexLock(&i2c_mutex) == KRONOS_STATUS_OK)
{
    i2c_transfer();
    (void)Kronos_MutexUnlock(&i2c_mutex);
}
```

#### `Kronos_MutexUnlock`

```c
kronos_status_e Kronos_MutexUnlock(kronos_mutex_t *mutex);
```

Visibility: Public application API.

Inputs:

| Input | Description |
| --- | --- |
| `mutex` | Mutex to unlock. |

Outputs: Decrements lock count and may wake the next waiting task.

What it does: Unlocks a mutex owned by the current task.

How it works: Verifies ownership. Decrements the recursive lock count. When the count reaches zero, clears ownership and wakes one waiting mutex task if present. The woken task becomes the new owner.

Return values:

| Return | Meaning |
| ---: | --- |
| `KRONOS_STATUS_OK` | Mutex unlocked. |
| `KRONOS_STATUS_INVALID_ARGUMENT` | Mutex is null, uninitialized, or corrupted. |
| `KRONOS_STATUS_ERROR` | Scheduler not started or current task invalid. |
| `KRONOS_STATUS_NOT_OWNER` | Current task does not own the mutex. |

Example:

```c
(void)Kronos_MutexUnlock(&i2c_mutex);
```

#### `Kronos_SemaphoreInit`

```c
kronos_status_e Kronos_SemaphoreInit(kronos_semaphore_t *semaphore,
                                   uint32_t initialCount,
                                   uint32_t maxCount);
```

Visibility: Public application API.

Inputs:

| Input | Description |
| --- | --- |
| `semaphore` | Semaphore object to initialize. |
| `initialCount` | Initial available count. |
| `maxCount` | Maximum count. Must be non-zero. |

Outputs: Initializes the semaphore counters and the internal initialization marker.

What it does: Prepares a counting semaphore for use.

How it works: Validates the counts, rejects reinitialization while tasks wait on the semaphore, stores `initialCount` and `maxCount`, marks the semaphore initialized, and registers it for validation.

Return values:

| Return | Meaning |
| --- | --- |
| `KRONOS_STATUS_OK` | Semaphore initialized. |
| `KRONOS_STATUS_INVALID_ARGUMENT` | Null semaphore, zero max count, or initial count greater than max count. |
| `KRONOS_STATUS_IN_USE` | Semaphore has waiters. |
| `KRONOS_STATUS_NO_MEMORY` | The sync object registry is full. |

Example:

```c
static kronos_semaphore_t rx_sem;

(void)Kronos_SemaphoreInit(&rx_sem, 0U, 8U);
```

#### `Kronos_SemaphoreTake`

```c
kronos_status_e Kronos_SemaphoreTake(kronos_semaphore_t *semaphore);
```

Visibility: Public application API.

Inputs:

| Input | Description |
| --- | --- |
| `semaphore` | Semaphore to take. |

Outputs: Decrements semaphore count or blocks the current task.

What it does: Takes one semaphore token.

How it works: If `count > 0`, decrements it. If count is zero, blocks the current task unless the scheduler is suspended.

Return values:

| Return | Meaning |
| ---: | --- |
| `KRONOS_STATUS_OK` | Token taken, or task blocked and later resumed. |
| `KRONOS_STATUS_INVALID_ARGUMENT` | Semaphore is null or corrupted. |
| `KRONOS_STATUS_ERROR` | Scheduler not started or current task invalid. |
| `KRONOS_STATUS_SCHEDULER_SUSPENDED` | No token is available and blocking is not allowed. |

Example:

```c
if (Kronos_SemaphoreTake(&rx_sem) == KRONOS_STATUS_OK)
{
    read_rx_buffer();
}
```

#### `Kronos_SemaphoreGive`

```c
kronos_status_e Kronos_SemaphoreGive(kronos_semaphore_t *semaphore);
```

Visibility: Public application API.

Inputs:

| Input | Description |
| --- | --- |
| `semaphore` | Semaphore to give. |

Outputs: Wakes one waiting task or increments semaphore count.

What it does: Gives one semaphore token.

How it works: First wakes one task waiting on the semaphore. If none is waiting, increments count unless already at max count.

Return values:

| Return | Meaning |
| ---: | --- |
| `KRONOS_STATUS_OK` | Token given or waiter woken. |
| `KRONOS_STATUS_INVALID_ARGUMENT` | Semaphore is null or corrupted. |
| `KRONOS_STATUS_ERROR` | Scheduler not started or current task invalid. |
| `KRONOS_STATUS_OVERFLOW` | Count is already at `max_count`. |

Example:

```c
void rx_event_task(void)
{
    for (;;)
    {
        poll_or_wait_for_rx_event();
        (void)Kronos_SemaphoreGive(&rx_sem);
    }
}
```

### Task Introspection

#### `Kronos_GetTaskTable`

```c
const TCB_t *Kronos_GetTaskTable(void);
```

Visibility: Public application API.

Inputs: None.

Outputs: None.

What it does: Returns a read-only pointer to the global task table.

How it works: Returns `g_tasks`.

Return values:

| Return | Meaning |
| --- | --- |
| `const TCB_t *` | Pointer to the first task control block. |

Notes:

- Use with `Kronos_GetTaskCount()`.
- Treat the returned table as read-only.
- Stack metrics are updated by scheduler and task-stat paths.

Example:

```c
const TCB_t *tasks = Kronos_GetTaskTable();
uint32_t count = Kronos_GetTaskCount();

for (uint32_t i = 0U; i < count; ++i)
{
    report_task(tasks[i].task_name, tasks[i].stack_free_words);
}
```

#### `Kronos_GetTaskCount`

```c
uint32_t Kronos_GetTaskCount(void);
```

Visibility: Public application API.

Inputs: None.

Outputs: None.

What it does: Returns the number of task table slots currently in use, including the internal idle task and stopped reusable slots below the high-water mark.

How it works: Returns `g_numTasks`.

Return values:

| Return | Meaning |
| ---: | --- |
| `0..MAX_TASKS` | Current number of task table entries. |

### Public Convenience Macros

#### `Kronos_EgressSendStruct`

```c
#define Kronos_EgressSendStruct(targetTaskName, messageId, objectPtr) \
    Kronos_EgressSendByName((targetTaskName), (messageId), (objectPtr), (uint32_t)sizeof(*(objectPtr)))
```

Visibility: Public application macro.

Inputs:

| Input | Description |
| --- | --- |
| `targetTaskName` | Name of the target task. |
| `messageId` | Application-defined message ID. |
| `objectPtr` | Pointer to the object to copy as payload. |

Outputs: Sends a message by task name.

What it does: Sends a typed object as a payload without manually writing `sizeof`.

Return values: Same as `Kronos_EgressSendByName()`.

Example:

```c
sensor_sample_t sample;
(void)Kronos_EgressSendStruct("logger", 1U, &sample);
```

#### `Kronos_EgressSendStructToIngress`

```c
#define Kronos_EgressSendStructToIngress(ingressPtr, messageId, objectPtr) \
    Kronos_EgressSend((ingressPtr), (messageId), (objectPtr), (uint32_t)sizeof(*(objectPtr)))
```

Visibility: Public application macro.

What it does: Sends a typed object to a resolved ingress handle.

Return values: Same as `Kronos_EgressSend()`.

Example:

```c
control_msg_t command;
(void)Kronos_EgressSendStructToIngress(&control_ingress, 2U, &command);
```

## Port API

The port API is declared in `Port/kronos_port.h`. It is implemented by the STM32 port files.

Application code normally does not call these functions directly. They form the contract between the portable kernel and the CPU/MCU port.

#### `kronos_port_init_scheduler`

```c
void kronos_port_init_scheduler(uint32_t tickFrequencyHz);
```

Visibility: Port API called by the kernel.

Inputs:

| Input | Description |
| --- | --- |
| `tickFrequencyHz` | Desired SysTick frequency. KronOS passes `TICK_FREQ_HZ`. |

Outputs: Configures CPU exception priorities, SysTick, MPU, and optional FPU access.

What it does: Prepares the MCU for KronOS scheduling.

How it works: The STM32 ports configure SysTick reload from `SystemCoreClock`, set MemManage/PendSV/SysTick priorities, configure static MPU regions, enable MemManage faults, and start SysTick.

Return values: None.

Notes:

- `SystemCoreClock` should be valid before `Kronos_Start()`.
- If `SystemCoreClock == 0`, the port uses a family default clock.

#### `kronos_port_start_scheduler`

```c
void kronos_port_start_scheduler(void);
```

Visibility: Port API called by the kernel.

Inputs: None.

Outputs: Enters the first scheduler SVC path.

What it does: Starts task execution.

How it works: The STM32 ports execute `svc #0`. The SVC handler loads the first task process stack and returns to thread mode.

Return values: None.

#### `kronos_port_request_yield`

```c
void kronos_port_request_yield(void);
```

Visibility: Port API called by the kernel.

Inputs: None.

Outputs: Enters SVC.

What it does: Requests a supervisor service or voluntary yield.

How it works: The STM32 ports execute `svc #0`. The SVC handler saves task context and calls `kronos_core_handle_supervisor_call()`.

Return values: None.

#### `kronos_port_pend_context_switch`

```c
void kronos_port_pend_context_switch(void);
```

Visibility: Port API called by the kernel.

Inputs: None.

Outputs: Sets PendSV pending.

What it does: Requests an asynchronous context switch.

How it works: The STM32 ports set the `PENDSVSET` bit in `SCB_ICSR`.

Return values: None.

#### `kronos_port_configure_task_stack_region`

```c
void kronos_port_configure_task_stack_region(const uint32_t *stackBase,
                                         uint32_t stackWords);
```

Visibility: Port API called by the kernel.

Inputs:

| Input | Description |
| --- | --- |
| `stackBase` | Base address of the active task stack slot. |
| `stackWords` | Stack slot size in 32-bit words. |

Outputs: Programs or disables the MPU task stack region.

What it does: Gives the active task access to its own stack slot.

How it works: The STM32 ports use the last MPU region for the current task stack. Passing null or zero disables that region.

Return values: None.

#### `kronos_port_get_live_process_stack_pointer`

```c
uint32_t *kronos_port_get_live_process_stack_pointer(void);
```

Visibility: Port API called by the kernel.

Inputs: None.

Outputs: None.

What it does: Reads the current process stack pointer.

How it works: The STM32 ports use the `mrs` instruction to read `psp`.

Return values:

| Return | Meaning |
| --- | --- |
| `uint32_t *` | Current PSP value. |

#### `kronos_port_compute_task_slot_words`

```c
uint32_t kronos_port_compute_task_slot_words(uint32_t requestedStackWords);
```

Visibility: Port API called by the kernel.

Inputs:

| Input | Description |
| --- | --- |
| `requestedStackWords` | Application requested usable stack size in words. |

Outputs: None.

What it does: Calculates the actual stack slot size to allocate.

How it works: Adds `KRONOS_PORT_STACK_RECOVERY_WORDS`, then rounds up to the next power of two starting at 8 words.

Return values:

| Return | Meaning |
| ---: | --- |
| `slotWords` | Slot size in 32-bit words. |

Notes:

- The power-of-two slot size helps MPU region programming.
- The slot may be larger than the requested usable stack size.

#### `kronos_port_get_default_clock_hz`

```c
uint32_t kronos_port_get_default_clock_hz(void);
```

Visibility: Port API called by the kernel.

Inputs: None.

Outputs: None.

What it does: Returns the default core clock for the selected STM32 family.

Return values:

| Family | Return |
| --- | ---: |
| STM32L4 | `4000000UL` |
| STM32G4 | `16000000UL` |
| STM32H7 | `64000000UL` |
| STM32L5 | `4000000UL` |

#### `kronos_port_get_stack_warning_margin_words`

```c
uint32_t kronos_port_get_stack_warning_margin_words(void);
```

Visibility: Port API called by the kernel.

Inputs: None.

Outputs: None.

What it does: Returns the stack warning margin.

Return values:

| Return | Meaning |
| ---: | --- |
| `KRONOS_PORT_STACK_WARNING_MARGIN_WORDS` | Current port stack warning margin, default `16U`. |

## Internal API

Internal APIs are declared in `kronos_internal.h` and `kronos_kernel.h`.

### Task Internals

#### `kronos_tasks_reset_state`

```c
void kronos_tasks_reset_state(void);
```

Visibility: Internal task API.

Inputs: None.

Outputs: Clears task table, runtime state, current task, stack pool usage, and creates the idle task.

What it does: Resets the task subsystem.

How it works: Clears `g_tasks` and `g_taskRuntime`, resets counters, then calls `kronos_task_create_internal()` for `kronos_idle`.

Return values: None.

#### `kronos_task_create_internal`

```c
kronos_status_e kronos_task_create_internal(void (*taskFunction)(void),
                                          uint32_t stackWords,
                                          const char *taskName,
                                          task_kind_e taskKind,
                                          kronos_task_id_t *taskId);
```

Visibility: Internal task API.

Inputs:

| Input | Description |
| --- | --- |
| `taskFunction` | Task entry function. |
| `stackWords` | Requested usable stack words. |
| `taskName` | Unique task name. |
| `taskKind` | Application, system, or idle. |
| `taskId` | Output pointer for the created task ID. |

Outputs: Adds a task and stack slot, and writes the task ID to `*taskId`.

What it does: Creates any kind of KronOS task.

How it works: Validates arguments, computes effective stack size, asks the port for slot size, reuses a stopped task stack when possible, allocates from `g_taskStackPool` when needed, initializes TCB fields, builds the initial CPU stack frame, and updates stack metrics.

Return values:

| Return | Meaning |
| --- | --- |
| `KRONOS_STATUS_OK` | Task created. |
| `KRONOS_STATUS_INVALID_ARGUMENT` | Invalid function, name, stack size, or output pointer. |
| `KRONOS_STATUS_IN_USE` | Duplicate active task name. |
| `KRONOS_STATUS_NO_MEMORY` | No task slot or stack pool space is available. |

#### `kronos_task_delete_internal`

```c
kronos_status_e kronos_task_delete_internal(kronos_task_id_t taskId);
```

Visibility: Internal task API.

Inputs:

| Input | Description |
| --- | --- |
| `taskId` | Task ID to delete. |

Outputs: Stops the task and cleans runtime-owned resources.

What it does: Kernel implementation of task deletion.

How it works: Rejects the idle task, marks the target deleting, cleans mailbox queues and registered mutex ownership, clears runtime wait/service state, clears the task name/function, and marks the slot stopped.

Return values: `KRONOS_STATUS_OK`, `KRONOS_STATUS_NOT_FOUND`, or `KRONOS_STATUS_INVALID_ARGUMENT`.

#### `kronos_task_pause_internal`

```c
kronos_status_e kronos_task_pause_internal(kronos_task_id_t taskId);
```

Visibility: Internal task API.

What it does: Kernel implementation of task pause.

Return values: `KRONOS_STATUS_OK`, `KRONOS_STATUS_INVALID_ARGUMENT`, or `KRONOS_STATUS_INVALID_STATE`.

#### `kronos_task_resume_internal`

```c
kronos_status_e kronos_task_resume_internal(kronos_task_id_t taskId);
```

Visibility: Internal task API.

What it does: Kernel implementation of task resume.

Return values: `KRONOS_STATUS_OK`, `KRONOS_STATUS_NOT_FOUND`, or `KRONOS_STATUS_INVALID_STATE`.

#### `kronos_task_update_all_stats`

```c
void kronos_task_update_all_stats(void);
```

Visibility: Internal task API.

Inputs: None.

Outputs: Updates stack metrics and stack fault flags for all tasks.

What it does: Refreshes task stack usage.

How it works: Uses the live PSP for the running task and saved `stack_top_ptr` for other tasks, then calls `kronos_task_check_stack()`.

Return values: None.

#### `kronos_task_update_stack_metrics`

```c
void kronos_task_update_stack_metrics(TCB_t *tcb,
                                   const uint32_t *stackPtr);
```

Visibility: Internal task API.

Inputs:

| Input | Description |
| --- | --- |
| `tcb` | Task control block. |
| `stackPtr` | Current or saved stack pointer. |

Outputs: Updates `stack_used_words`, `stack_free_words`, and `stack_high_watermark_words`.

What it does: Calculates stack usage for one task.

How it works: Compares `stackPtr` against the stack slot start/end and usable stack size.

Return values: None.

#### `kronos_task_check_stack`

```c
kronos_stack_check_e kronos_task_check_stack(TCB_t *tcb,
                                           const uint32_t *stackPtr);
```

Visibility: Internal task API.

Inputs:

| Input | Description |
| --- | --- |
| `tcb` | Task control block. |
| `stackPtr` | Current or saved stack pointer. |

Outputs: Updates metrics and fault flags.

What it does: Checks whether a task stack is healthy, near limit, or overflowed.

How it works: Updates metrics, then compares `stackPtr` with `stack_slot_start_ptr`, `stack_limit_ptr`, and `stack_warning_ptr`.

Return values:

| Return | Meaning |
| --- | --- |
| `KRONOS_STACK_CHECK_OK` | Stack pointer is above the warning threshold. |
| `KRONOS_STACK_CHECK_WARNING` | Stack pointer is below warning threshold but not overflowed. |
| `KRONOS_STACK_CHECK_OVERFLOW` | Stack pointer is null or below stack limit/start. |

#### `kronos_task_quarantine`

```c
void kronos_task_quarantine(TCB_t *tcb,
                           uint32_t *faultStackPtr,
                           uint32_t faultFlags,
                           uint32_t faultAddress);
```

Visibility: Internal task API.

Inputs:

| Input | Description |
| --- | --- |
| `tcb` | Faulted task. |
| `faultStackPtr` | Stack pointer captured during fault handling. |
| `faultFlags` | Fault flags to record. |
| `faultAddress` | Faulting address, when known. |

Outputs: Marks the task as error state and clears wait/service runtime state.

What it does: Isolates a faulted task so the scheduler can continue with another task.

How it works: Normalizes stack-related fault flags, stores fault metadata, clears delay and wait state, and sets task state to `TASK_STATE_ERROR` or `TASK_STATE_ERROR_STACK_OVERFLOW`.

Return values: None.

#### `kronos_task_find_by_name`

```c
int32_t kronos_task_find_by_name(const char *taskName);
```

Visibility: Internal task API.

Inputs:

| Input | Description |
| --- | --- |
| `taskName` | Name to search for. |

Outputs: None.

What it does: Finds a task by exact name.

How it works: Linear search over `g_tasks[0..g_numTasks)`.

Return values:

| Return | Meaning |
| ---: | --- |
| `>= 0` | Task index. |
| `-1` | Null/empty name or not found. |

#### `kronos_task_select_next_ready`

```c
int32_t kronos_task_select_next_ready(void);
```

Visibility: Internal task API.

Inputs: None.

Outputs: Removes the selected task from its ready queue.

What it does: Selects the next ready task in bounded time.

How it works: Checks the application, system, and idle ready queue heads in that fixed order and pops the first available task.

Return values:

| Return | Meaning |
| ---: | --- |
| `>= 0` | Selected task index. |
| `-1` | No ready task is available. |

#### `kronos_task_find_waiting`

```c
int32_t kronos_task_find_waiting(kronos_wait_reason_e waitReason,
                               const void *waitObject,
                               uint32_t startTask);
```

Visibility: Internal task API.

Inputs:

| Input | Description |
| --- | --- |
| `waitReason` | Wait reason to match. |
| `waitObject` | Wait object pointer to match. |
| `startTask` | Index to start after. |

Outputs: None.

What it does: Finds a task waiting for a specific object.

How it works: Scans the task table for `TASK_STATE_WAITING`, matching runtime wait reason and object pointer.

Return values:

| Return | Meaning |
| ---: | --- |
| `>= 0` | Waiting task index. |
| `-1` | No matching waiter. |

#### `kronos_task_block`

```c
void kronos_task_block(TCB_t *tcb,
                      kronos_wait_reason_e waitReason,
                      void *waitObject);
```

Visibility: Internal task API.

Inputs:

| Input | Description |
| --- | --- |
| `tcb` | Task to block. |
| `waitReason` | Reason for blocking. |
| `waitObject` | Object the task waits on. |

Outputs: Sets runtime wait fields and task state.

What it does: Blocks a task on an internal wait object.

How it works: Computes the task index from the TCB pointer, removes the task from scheduler queues if needed, and updates `g_taskRuntime`.

Return values: None.

#### `kronos_task_unblock`

```c
void kronos_task_unblock(TCB_t *tcb);
```

Visibility: Internal task API.

Inputs:

| Input | Description |
| --- | --- |
| `tcb` | Task to unblock. |

Outputs: Clears wait fields and makes task ready.

What it does: Wakes a waiting task.

How it works: Clears `wait_reason` and `wait_object_ptr`, sets `task_state` to `TASK_STATE_READY`, and appends the task to its ready queue.

Return values: None.

#### `kronos_task_activate`

```c
void kronos_task_activate(uint32_t taskIndex);
```

Visibility: Internal task API.

Inputs:

| Input | Description |
| --- | --- |
| `taskIndex` | Task index to activate. |

Outputs: Updates `g_currentTask`, marks the task running, and configures its stack MPU region.

What it does: Makes a task the current running task.

How it works: Sets the task state to `TASK_STATE_RUNNING` and calls `kronos_port_configure_task_stack_region()`.

Return values: None.

### Scheduler and Core Internals

#### `kronos_scheduler_reset_state`

```c
void kronos_scheduler_reset_state(void);
```

Visibility: Internal scheduler API.

Inputs: None.

Outputs: Resets scheduler globals.

What it does: Clears tick count, scheduler-started flag, suspend depth, and pending switch flag.

Return values: None.

#### `kronos_scheduler_request_service`

```c
void kronos_scheduler_request_service(kronos_service_e serviceRequest,
                                    void *serviceObject,
                                    uint32_t serviceParameter);
```

Visibility: Internal scheduler API.

Inputs:

| Input | Description |
| --- | --- |
| `serviceRequest` | Service to execute in the scheduler context. |
| `serviceObject` | Service-specific pointer. |
| `serviceParameter` | Service-specific scalar parameter. |

Outputs: Stores service request in the current task runtime state and triggers SVC.

What it does: Common path for task services such as delay, lock, send, receive, and yield.

How it works: Writes the request fields into `g_taskRuntime[g_currentTask]`, initializes `service_result`, and calls `kronos_port_request_yield()`.

Return values: None.

Notes:

- If the scheduler is not started or the current task is invalid, the request is ignored.

#### `kronos_core_on_tick`

```c
void kronos_core_on_tick(void);
```

Visibility: Internal kernel API called from SysTick.

Inputs: None.

Outputs: Increments tick count, updates delayed tasks, and may pend a context switch.

What it does: Handles one system tick.

How it works: Increments `g_tickCount`, processes expired entries from the ordered delay-list head, moves expired tasks to ready queues, and pends PendSV unless scheduler switching is suspended.

Return values: None.

#### `kronos_core_check_current_task_stack`

```c
kronos_stack_check_e kronos_core_check_current_task_stack(uint32_t *exceptionStackPtr);
```

Visibility: Internal kernel API called from SVC/PendSV assembly.

Inputs:

| Input | Description |
| --- | --- |
| `exceptionStackPtr` | Stack pointer captured by exception handler. |

Outputs: Updates current task stack metrics and fault flags.

What it does: Checks the current task stack before context service/switch.

How it works: Calls `kronos_task_check_stack()` for `g_tasks[g_currentTask]`.

Return values: See `kronos_task_check_stack()`.

#### `kronos_core_prepare_first_task`

```c
uint32_t *kronos_core_prepare_first_task(void);
```

Visibility: Internal kernel API called from first SVC.

Inputs: None.

Outputs: Marks scheduler started and updates first task stack metrics.

What it does: Prepares the initial task context for the port handler.

How it works: Sets `g_schedulerStarted = 1U`, updates metrics for the current task, and returns its saved stack pointer.

Return values:

| Return | Meaning |
| --- | --- |
| `uint32_t *` | Initial task stack pointer to restore. |

#### `kronos_core_handle_supervisor_call`

```c
uint32_t *kronos_core_handle_supervisor_call(uint32_t *savedStackPtr);
```

Visibility: Internal kernel API called from SVC after scheduler start.

Inputs:

| Input | Description |
| --- | --- |
| `savedStackPtr` | Saved process stack pointer after callee-saved registers were pushed. |

Outputs: Executes the current task's pending service and returns the stack pointer to restore.

What it does: Main service dispatcher for SVC-based kernel calls.

How it works: Stores current context, reads `g_taskRuntime[g_currentTask].service_request`, executes the matching scheduler, sync, or channel service, stores `service_result`, and either continues the current task or switches.

Return values:

| Return | Meaning |
| --- | --- |
| `uint32_t *` | Stack pointer for the task that should resume. |

#### `kronos_core_switch_task`

```c
uint32_t *kronos_core_switch_task(uint32_t *savedStackPtr);
```

Visibility: Internal kernel API called from PendSV.

Inputs:

| Input | Description |
| --- | --- |
| `savedStackPtr` | Saved process stack pointer after callee-saved registers were pushed. |

Outputs: Stores current context and selects next task if allowed.

What it does: Runs a normal context switch.

How it works: Saves current stack pointer and metrics, then calls scheduler switch/continue logic.

Return values:

| Return | Meaning |
| --- | --- |
| `uint32_t *` | Stack pointer for the task that should resume. |

#### `kronos_core_quarantine_current_task`

```c
uint32_t *kronos_core_quarantine_current_task(uint32_t *faultStackPtr,
                                           uint32_t faultFlags,
                                           uint32_t faultAddress);
```

Visibility: Internal kernel API called from fault and stack-check paths.

Inputs:

| Input | Description |
| --- | --- |
| `faultStackPtr` | Fault-time task stack pointer. |
| `faultFlags` | Fault flags to record. |
| `faultAddress` | Fault address if known. |

Outputs: Quarantines current task and activates a ready replacement if available.

What it does: Removes a faulted task from scheduling and returns the next runnable stack pointer.

How it works: Calls `kronos_task_quarantine()`, selects the next ready task, activates it, and returns its stack pointer.

Return values:

| Return | Meaning |
| --- | --- |
| `uint32_t *` | Stack pointer to restore after fault handling. |

### Channel Internals

#### `kronos_channels_reset_state`

```c
void kronos_channels_reset_state(void);
```

Visibility: Internal channel API.

Inputs: None.

Outputs: Resets all ingress queues and mail slots.

What it does: Initializes channel state for a clean RTOS start.

How it works: Clears each task ingress state and rebuilds the global free list of mail slots.

Return values: None.

#### `kronos_channels_cleanup_task`

```c
void kronos_channels_cleanup_task(kronos_task_id_t taskId);
```

Visibility: Internal channel API.

Inputs:

| Input | Description |
| --- | --- |
| `taskId` | Task ID being deleted or quarantined. |

Outputs: Releases matching queued mail slots.

What it does: Removes pending messages where the task is either sender or receiver.

How it works: Walks each ingress queue, unlinks matching mail slots, decrements pending counts, and returns slots to the global free list.

Return values: None.

#### `kronos_channels_ingress_receive`

```c
void kronos_channels_ingress_receive(kronos_mail_t *message,
                                   kronos_service_outcome_t *outcome);
```

Visibility: Internal channel service.

Inputs:

| Input | Description |
| --- | --- |
| `message` | Destination for copied mail. |
| `outcome` | Service result output. |

Outputs: Copies one pending mail to `message`, releases its slot, and sets service outcome.

What it does: Kernel implementation of non-blocking ingress receive.

Return values: Written into `outcome->status`: `KRONOS_STATUS_OK`, `KRONOS_STATUS_EMPTY`, or `KRONOS_STATUS_INVALID_ARGUMENT`.

#### `kronos_channels_ingress_wait`

```c
void kronos_channels_ingress_wait(kronos_service_outcome_t *outcome);
```

Visibility: Internal channel service.

Inputs:

| Input | Description |
| --- | --- |
| `outcome` | Service result output. |

Outputs: May block current task on its ingress state.

What it does: Kernel implementation of mailbox wait.

How it works: Returns OK if mail is already pending. Otherwise blocks the current task with wait reason `KRONOS_WAIT_INGRESS` unless the scheduler is suspended.

Return values: Written into `outcome->status`.

#### `kronos_channels_egress_send`

```c
void kronos_channels_egress_send(const kronos_channel_request_t *request,
                               kronos_service_outcome_t *outcome);
```

Visibility: Internal channel service.

Inputs:

| Input | Description |
| --- | --- |
| `request` | Send request containing ingress or task name, payload, and message ID. |
| `outcome` | Service result output. |

Outputs: Queues one mail slot to a target task.

What it does: Kernel implementation of unicast message send.

How it works: Resolves the target, validates payload size and pointer, allocates a slot, copies payload, queues it, and wakes a waiting target task.

Return values: Written into `outcome->status`.

#### `kronos_channels_egress_broadcast`

```c
void kronos_channels_egress_broadcast(const kronos_channel_request_t *request,
                                    kronos_service_outcome_t *outcome);
```

Visibility: Internal channel service.

Inputs:

| Input | Description |
| --- | --- |
| `request` | Broadcast request containing payload and message ID. |
| `outcome` | Service result output. |

Outputs: Queues one mail slot to every ingress-capable task.

What it does: Kernel implementation of broadcast message send.

How it works: Validates payload, counts broadcast targets, checks that enough free slots exist, then delivers to each target.

Return values: Written into `outcome->status`.

### Synchronization Internals

#### `kronos_sync_reset_state`

```c
void kronos_sync_reset_state(void);
```

Visibility: Internal sync API.

Inputs: None.

Outputs: Clears the internal mutex and semaphore registries.

What it does: Resets synchronization bookkeeping during `Kronos_Init()`.

Return values: None.

#### `kronos_sync_cleanup_task`

```c
void kronos_sync_cleanup_task(kronos_task_id_t taskId);
```

Visibility: Internal sync API.

Inputs:

| Input | Description |
| --- | --- |
| `taskId` | Task ID being deleted or quarantined. |

Outputs: Releases registered mutexes owned by the task.

What it does: Prevents a deleted or faulted task from leaving a registered mutex permanently owned.

How it works: Scans registered mutexes, clears ownership for mutexes owned by `taskId`, and wakes one waiter if present.

Return values: None.

#### `kronos_sync_mutex_lock`

```c
void kronos_sync_mutex_lock(TCB_t *currentTcb,
                          kronos_mutex_t *mutex,
                          kronos_service_outcome_t *outcome);
```

Visibility: Internal sync service.

Inputs:

| Input | Description |
| --- | --- |
| `currentTcb` | Current task control block. |
| `mutex` | Mutex to lock. |
| `outcome` | Service result output. |

Outputs: Locks mutex or blocks current task.

What it does: Kernel implementation of mutex lock.

How it works: Supports recursive ownership by current task. Blocks on `KRONOS_WAIT_MUTEX` if another task owns the mutex and scheduler switching is allowed.

Return values: Written into `outcome->status`.

#### `kronos_sync_mutex_unlock`

```c
void kronos_sync_mutex_unlock(TCB_t *currentTcb,
                            kronos_mutex_t *mutex,
                            kronos_service_outcome_t *outcome);
```

Visibility: Internal sync service.

Inputs:

| Input | Description |
| --- | --- |
| `currentTcb` | Current task control block. |
| `mutex` | Mutex to unlock. |
| `outcome` | Service result output. |

Outputs: Unlocks mutex and may wake one waiter.

What it does: Kernel implementation of mutex unlock.

How it works: Verifies ownership, decrements recursive lock count, clears owner at zero, finds a mutex waiter, and transfers ownership to that waiter.

Return values: Written into `outcome->status`.

#### `kronos_sync_semaphore_take`

```c
void kronos_sync_semaphore_take(TCB_t *currentTcb,
                              kronos_semaphore_t *semaphore,
                              kronos_service_outcome_t *outcome);
```

Visibility: Internal sync service.

Inputs:

| Input | Description |
| --- | --- |
| `currentTcb` | Current task control block. |
| `semaphore` | Semaphore to take. |
| `outcome` | Service result output. |

Outputs: Decrements count or blocks current task.

What it does: Kernel implementation of semaphore take.

How it works: Decrements count when available. Otherwise blocks on `KRONOS_WAIT_SEMAPHORE` unless the scheduler is suspended.

Return values: Written into `outcome->status`.

#### `kronos_sync_semaphore_give`

```c
void kronos_sync_semaphore_give(kronos_semaphore_t *semaphore,
                              kronos_service_outcome_t *outcome);
```

Visibility: Internal sync service.

Inputs:

| Input | Description |
| --- | --- |
| `semaphore` | Semaphore to give. |
| `outcome` | Service result output. |

Outputs: Wakes one waiter or increments count.

What it does: Kernel implementation of semaphore give.

How it works: Tries to wake a waiting task first. If no task waits, increments count unless it is already at max.

Return values: Written into `outcome->status`.

## Interrupt Handlers

These handlers are exposed by the STM32 port source files and must not be replaced accidentally by startup code.

#### `SysTick_Handler`

```c
void SysTick_Handler(void);
```

Visibility: MCU interrupt handler.

What it does: Calls `kronos_core_on_tick()`.

Use case: Drives delays and time slicing.

#### `PendSV_Handler`

```c
void PendSV_Handler(void);
```

Visibility: MCU exception handler.

What it does: Performs context switching.

How it works: Saves callee-saved registers, checks current task stack, calls `kronos_core_switch_task()` or quarantine path, restores the selected task context, and returns.

#### `SVC_Handler`

```c
void SVC_Handler(void);
```

Visibility: MCU exception handler.

What it does: Starts the first task or dispatches kernel services.

How it works: On first entry, loads the first task PSP and enters thread mode. Later entries save current context and call `kronos_core_handle_supervisor_call()`.

#### `MemManage_Handler`

```c
void MemManage_Handler(void);
```

Visibility: MCU exception handler.

What it does: Handles MPU memory faults from task mode.

How it works: Reads fault status/address, clears the fault status bits, calls the quarantine path, and resumes another task. Faults not using PSP loop forever in the handler.

## Data Structures

### Public Data Structures

#### `kronos_task_id_t`

```c
typedef uint32_t kronos_task_id_t;
```

Purpose: Task ID type used by task control APIs.

Notes:

- `KRONOS_INVALID_TASK_ID` marks an invalid or unset task ID.
- Task IDs are task table indexes and are kept stable while the task slot exists.

#### `kronos_driver_init_fn_t`

```c
typedef kronos_status_e (*kronos_driver_init_fn_t)(void *context);
```

Purpose: Function pointer type for privileged driver initialization requests.

Notes:

- Used with `Kronos_DriverInit()`.
- The function should return a `kronos_status_e`.

#### `kronos_mutex_t`

```c
typedef struct
{
    uint32_t owner_task_id;
    uint32_t lock_count;
    uint32_t initialized;
} kronos_mutex_t;
```

Purpose: Recursive mutex state.

Fields:

| Field | Meaning |
| --- | --- |
| `owner_task_id` | Current owner task ID, or `KRONOS_INVALID_TASK_ID` when unlocked. |
| `lock_count` | Recursive lock depth. Zero means unlocked. |
| `initialized` | Internal marker set by `Kronos_MutexInit()`. |

Notes:

- Initialize with `Kronos_MutexInit()`.
- Do not edit fields directly from application code.

#### `kronos_semaphore_t`

```c
typedef struct
{
    uint32_t count;
    uint32_t max_count;
    uint32_t initialized;
} kronos_semaphore_t;
```

Purpose: Counting semaphore state.

Fields:

| Field | Meaning |
| --- | --- |
| `count` | Current available tokens. |
| `max_count` | Maximum allowed tokens. |
| `initialized` | Internal marker set by `Kronos_SemaphoreInit()`. |

Notes:

- Initialize with `Kronos_SemaphoreInit()`.
- `max_count` must be greater than zero.

#### `kronos_ingress_t`

```c
typedef struct
{
    uint32_t task_id;
    const char *task_name;
} kronos_ingress_t;
```

Purpose: Resolved mailbox target handle.

Fields:

| Field | Meaning |
| --- | --- |
| `task_id` | Task table index at resolve time. |
| `task_name` | Target task name pointer. Used to validate that the ID still matches the same task. |

Notes:

- Fill with `Kronos_IngressResolve()`.
- Use with `Kronos_EgressSend()`.

#### `kronos_mail_t`

```c
typedef struct
{
    uint32_t sequence;
    uint32_t message_id;
    uint32_t payload_size;
    uint32_t sender_task_id;
    uint32_t receiver_task_id;
    const char *sender_task_name;
    const char *receiver_task_name;
    uint8_t payload[KRONOS_MESSAGE_MAX_PAYLOAD_BYTES];
} kronos_mail_t;
```

Purpose: Received message container.

Fields:

| Field | Meaning |
| --- | --- |
| `sequence` | Global message sequence number assigned at delivery. |
| `message_id` | Application-defined message ID. |
| `payload_size` | Number of valid bytes in `payload`. |
| `sender_task_id` | Sending task ID. |
| `receiver_task_id` | Receiving task ID. |
| `sender_task_name` | Sending task name when available. |
| `receiver_task_name` | Receiving task name when available. |
| `payload` | Copied payload bytes. |

Notes:

- Payload capacity is `KRONOS_MESSAGE_MAX_PAYLOAD_BYTES`.
- The payload is byte storage. Cast only when size and alignment are safe for your target.

#### `TCB_t`

```c
typedef struct
{
    uint32_t *stack_top_ptr;
    uint32_t *stack_slot_start_ptr;
    uint32_t *stack_limit_ptr;
    uint32_t *stack_warning_ptr;
    uint32_t *stack_slot_end_ptr;
    task_state_e task_state;
    task_kind_e task_kind;
    uint32_t remaining_delay;
    void (*task_fn)(void);
    const char *task_name;
    uint32_t requested_stack_words;
    uint32_t stack_slot_words;
    uint32_t stack_warning_words;
    uint32_t stack_reserved_words;
    uint32_t stack_used_words;
    uint32_t stack_free_words;
    uint32_t stack_high_watermark_words;
    uint32_t fault_flags;
    uint32_t fault_address;
} TCB_t;
```

Purpose: Task control block.

Fields:

| Field | Meaning |
| --- | --- |
| `stack_top_ptr` | Saved stack pointer for the task. |
| `stack_slot_start_ptr` | Start of allocated stack slot. |
| `stack_limit_ptr` | Lowest valid usable stack address. |
| `stack_warning_ptr` | Warning threshold for low stack. |
| `stack_slot_end_ptr` | End of allocated stack slot. |
| `task_state` | Current task state. |
| `task_kind` | Application, system, or idle. |
| `remaining_delay` | Delay ticks stored for delayed or paused-delayed tasks. Active delay ordering is maintained by the internal wake-tick list. |
| `task_fn` | Original task entry function. |
| `task_name` | Task name string. |
| `requested_stack_words` | Usable stack words requested after defaulting/rounding. |
| `stack_slot_words` | Actual allocated slot words. |
| `stack_warning_words` | Warning margin in words. |
| `stack_reserved_words` | Reserved/recovery words outside requested usable stack. |
| `stack_used_words` | Current estimated used stack words. |
| `stack_free_words` | Current estimated free usable stack words. |
| `stack_high_watermark_words` | Maximum observed stack usage. |
| `fault_flags` | Recorded task fault flags. |
| `fault_address` | Recorded memory fault address. |

Notes:

- Application code may inspect this through `Kronos_GetTaskTable()`.
- Application code should not modify it.

### Internal Data Structures

#### `kronos_service_outcome_t`

```c
typedef struct
{
    kronos_status_e status;
    uint32_t switch_required;
} kronos_service_outcome_t;
```

Purpose: Result from an internal kernel service.

Fields:

| Field | Meaning |
| --- | --- |
| `status` | Service result. |
| `switch_required` | Non-zero when the scheduler should switch after the service. |

#### `kronos_task_create_request_t`

```c
typedef struct
{
    void (*task_function)(void);
    const char *task_name;
    kronos_task_id_t *task_id_ptr;
    uint32_t stack_words;
} kronos_task_create_request_t;
```

Purpose: Internal request object for runtime task creation.

Fields:

| Field | Meaning |
| --- | --- |
| `task_function` | Task entry function. |
| `task_name` | New task name. |
| `task_id_ptr` | Output pointer for the created task ID. |
| `stack_words` | Requested usable stack size in words. |

#### `kronos_driver_init_request_t`

```c
typedef struct
{
    kronos_driver_init_fn_t init_function;
    void *context;
} kronos_driver_init_request_t;
```

Purpose: Internal request object for privileged driver initialization.

Fields:

| Field | Meaning |
| --- | --- |
| `init_function` | Function to execute in kernel context. |
| `context` | Pointer passed to the init function. |

#### `kronos_channel_request_t`

```c
typedef struct
{
    const kronos_ingress_t *ingress_ptr;
    const char *task_name;
    const void *payload_ptr;
    kronos_mail_t *message_ptr;
    uint32_t message_id;
    uint32_t payload_size;
} kronos_channel_request_t;
```

Purpose: Shared request object for channel send/receive services.

Fields:

| Field | Meaning |
| --- | --- |
| `ingress_ptr` | Target ingress handle for handle-based send. |
| `task_name` | Target task name for name-based send. |
| `payload_ptr` | Payload source pointer. |
| `message_ptr` | Receive destination pointer. |
| `message_id` | Application message ID. |
| `payload_size` | Payload byte count. |

#### `kronos_task_runtime_t`

```c
typedef struct
{
    void *wait_object_ptr;
    void *service_object_ptr;
    uint32_t service_parameter;
    kronos_status_e service_result;
    kronos_wait_reason_e wait_reason;
    kronos_service_e service_request;
} kronos_task_runtime_t;
```

Purpose: Runtime-only task state that is separate from the public TCB.

Fields:

| Field | Meaning |
| --- | --- |
| `wait_object_ptr` | Object the task waits on, such as a mutex, semaphore, or ingress state. |
| `service_object_ptr` | Pointer passed to the current SVC service. |
| `service_parameter` | Scalar parameter passed to the current SVC service. |
| `service_result` | Result returned by the service to the public wrapper. |
| `wait_reason` | Current wait reason. |
| `service_request` | Current requested kernel service. |

### Port Data Structures

#### `kronos_port_armv7m_region_t`

```c
typedef struct
{
    uint32_t base_address;
    uint32_t size_bytes;
    uint32_t attributes;
} kronos_port_armv7m_region_t;
```

Purpose: Static MPU region descriptor for ARMv7-M STM32 families.

Fields:

| Field | Meaning |
| --- | --- |
| `base_address` | MPU region base address. |
| `size_bytes` | MPU region size in bytes. |
| `attributes` | ARMv7-M MPU attributes. |

#### `kronos_port_armv7m_family_config_t`

```c
typedef struct
{
    uint32_t default_core_clock_hz;
    uint32_t static_region_count;
    const kronos_port_armv7m_region_t *static_regions;
} kronos_port_armv7m_family_config_t;
```

Purpose: Family-specific ARMv7-M STM32 port configuration.

Fields:

| Field | Meaning |
| --- | --- |
| `default_core_clock_hz` | Fallback CPU clock when `SystemCoreClock` is zero. |
| `static_region_count` | Number of static MPU regions. |
| `static_regions` | Pointer to region descriptor array. |

## Enumerations

### `task_state_e`

| Value | Meaning |
| --- | --- |
| `TASK_STATE_UNKNOWN` | Uninitialized or unknown state. |
| `TASK_STATE_READY` | Task can be scheduled. |
| `TASK_STATE_RUNNING` | Task is currently running. |
| `TASK_STATE_PAUSED` | Task is paused by API. |
| `TASK_STATE_TIME_DELAY` | Task is delayed until its internal wake tick expires. |
| `TASK_STATE_WAITING` | Task is blocked on mutex, semaphore, or ingress. |
| `TASK_STATE_DELETING` | Reserved delete state. |
| `TASK_STATE_STOPPED` | Task function returned and task stopped. |
| `TASK_STATE_ERROR` | Task is quarantined due to fault. |
| `TASK_STATE_ERROR_STACK_OVERFLOW` | Task is quarantined due to stack overflow. |

### `task_kind_e`

| Value | Meaning |
| --- | --- |
| `TASK_KIND_APPLICATION` | User application task. |
| `TASK_KIND_SYSTEM` | Kernel/system task kind. |
| `TASK_KIND_IDLE` | Internal idle task. |

### `kronos_status_e`

See [Status Codes](#status-codes).

### `kronos_stack_check_e`

| Value | Meaning |
| --- | --- |
| `KRONOS_STACK_CHECK_OK` | Stack is within safe range. |
| `KRONOS_STACK_CHECK_WARNING` | Stack is below warning threshold. |
| `KRONOS_STACK_CHECK_OVERFLOW` | Stack pointer is outside usable stack range. |

### `kronos_wait_reason_e`

| Value | Meaning |
| --- | --- |
| `KRONOS_WAIT_NONE` | Task is not waiting. |
| `KRONOS_WAIT_MUTEX` | Task waits for a mutex. |
| `KRONOS_WAIT_SEMAPHORE` | Task waits for a semaphore. |
| `KRONOS_WAIT_INGRESS` | Task waits for an ingress message. |

### `kronos_service_e`

| Value | Meaning |
| --- | --- |
| `KRONOS_SERVICE_NONE` | No pending service. |
| `KRONOS_SERVICE_DELAY` | Delay current task. |
| `KRONOS_SERVICE_FORCE_SWITCH` | Yield/current context switch request. |
| `KRONOS_SERVICE_SUSPEND_SCHEDULER` | Increase scheduler suspend depth. |
| `KRONOS_SERVICE_RESUME_SCHEDULER` | Decrease scheduler suspend depth. |
| `KRONOS_SERVICE_MUTEX_LOCK` | Lock mutex. |
| `KRONOS_SERVICE_MUTEX_UNLOCK` | Unlock mutex. |
| `KRONOS_SERVICE_SEMAPHORE_TAKE` | Take semaphore. |
| `KRONOS_SERVICE_SEMAPHORE_GIVE` | Give semaphore. |
| `KRONOS_SERVICE_INGRESS_RECEIVE` | Receive ingress message. |
| `KRONOS_SERVICE_INGRESS_WAIT` | Wait for ingress message. |
| `KRONOS_SERVICE_EGRESS_SEND` | Send one message. |
| `KRONOS_SERVICE_EGRESS_BROADCAST` | Broadcast one message. |
| `KRONOS_SERVICE_TASK_CREATE` | Create an application task. |
| `KRONOS_SERVICE_TASK_DELETE` | Delete an application task. |
| `KRONOS_SERVICE_TASK_PAUSE` | Pause an application task. |
| `KRONOS_SERVICE_TASK_RESUME` | Resume a paused task. |
| `KRONOS_SERVICE_DRIVER_INIT` | Run privileged driver initialization. |

## Constants and Configuration Macros

### Task and Stack Limits

| Macro | Default | Meaning |
| --- | ---: | --- |
| `MAX_APPLICATION_TASKS` | `10U` | Maximum application tasks. |
| `SYSTEM_TASK_COUNT` | `1U` | Reserved internal system task count. Currently used for idle. |
| `MAX_TASKS` | `MAX_APPLICATION_TASKS + SYSTEM_TASK_COUNT` | Total TCB capacity. |
| `DEFAULT_STACK_SIZE_WORDS` | `48U` | Default usable stack words when task creation passes `0`. |
| `MIN_STACK_SIZE_WORDS` | `32U` | Minimum accepted usable stack words. |
| `IDLE_TASK_STACK_SIZE_WORDS` | `64U` | Idle task usable stack words. |
| `STACK_POOL_SIZE_WORDS` | `2048U` | Total task stack pool in 32-bit words. |
| `TICK_FREQ_HZ` | `1000UL` | Scheduler tick frequency. |

### Messaging Limits

| Macro | Default | Meaning |
| --- | ---: | --- |
| `KRONOS_INGRESS_SLOT_COUNT` | `16U` | Number of global mail slots. Can be overridden before including/building. |
| `KRONOS_MESSAGE_MAX_PAYLOAD_BYTES` | `32U` | Maximum payload bytes per message. Can be overridden before including/building. |

### Synchronization Limits

| Macro | Default | Meaning |
| --- | ---: | --- |
| `KRONOS_SYNC_REGISTRY_SIZE` | `16U` | Number of mutex and semaphore pointers tracked for validation and cleanup. |

### Fault Flags

| Macro | Meaning |
| --- | --- |
| `KRONOS_TASK_FAULT_NONE` | No fault recorded. |
| `KRONOS_TASK_FAULT_STACK_WARNING` | Stack crossed warning threshold. |
| `KRONOS_TASK_FAULT_STACK_OVERFLOW` | Stack overflow detected. |
| `KRONOS_TASK_FAULT_MEMORY_ACCESS` | MPU memory access fault detected. |
| `KRONOS_INVALID_TASK_ID` | Invalid task ID marker, defined as `UINT32_MAX`. |

### Port Macros

| Macro | Default | Meaning |
| --- | ---: | --- |
| `KRONOS_PORT_STACK_WARNING_MARGIN_WORDS` | `16U` | Stack warning margin. |
| `KRONOS_PORT_STACK_RECOVERY_WORDS` | `32U` | Extra stack slot words reserved for recovery/fault handling. |
| `KRONOS_PORT_STACK_POOL_ALIGNMENT_BYTES` | `8192U` | Alignment for the stack pool section. |
| `KRONOS_PORT_STACK_POOL_SECTION` | attribute | Places task stack pool in `.kronos_task_stacks`. |
| `KRONOS_PORT_MPU_REGION_COUNT` | `8U` | Number of MPU regions used by the STM32 port model. |
| `KRONOS_PORT_TASK_STACK_REGION_INDEX` | last region | MPU region index reserved for the active task stack. |

### STM32 Family Selection Macros

The port auto-detects one active STM32 family from CMSIS or startup defines.

| Macro | Meaning |
| --- | --- |
| `KRONOS_PORT_STM32_FAMILY_L4` | STM32L4 selected. |
| `KRONOS_PORT_STM32_FAMILY_G4` | STM32G4 selected. |
| `KRONOS_PORT_STM32_FAMILY_L5` | STM32L5 selected. |
| `KRONOS_PORT_STM32_FAMILY_H7` | STM32H7 selected. |
| `KRONOS_PORT_STM32_IS_ARMV7M` | True for L4, G4, and H7. |

Only one family can be selected. If none or more than one are selected, the port emits a compile-time error.

## Global Variables

### Publicly Declared Globals

These are declared in `kronos_core.h`.

| Variable | Type | Meaning |
| --- | --- | --- |
| `g_tasks` | `TCB_t[MAX_TASKS]` | Global task table. |
| `g_numTasks` | `uint32_t` | Number of task table slots currently in use, including stopped reusable slots below the high-water mark. |
| `g_currentTask` | `uint32_t` | Index of the current task. |

Application code should prefer `Kronos_GetTaskTable()` and `Kronos_GetTaskCount()` instead of direct access.

### Internal Globals

These are declared in `kronos_internal.h`.

| Variable | Type | Meaning |
| --- | --- | --- |
| `g_tickCount` | `volatile uint32_t` | Global tick counter incremented by SysTick. |
| `g_schedulerStarted` | `uint32_t` | Non-zero after first task preparation. |
| `g_stackPoolWordsUsed` | `uint32_t` | Words consumed in the task stack pool. |
| `g_schedulerSuspendDepth` | `volatile uint32_t` | Nested scheduler suspend counter. |
| `g_schedulerSwitchPending` | `volatile uint32_t` | Pending switch flag recorded while suspended. |
| `g_taskRuntime` | `kronos_task_runtime_t[MAX_TASKS]` | Per-task runtime wait and service state. |

### Port Globals

| Variable | Type | Meaning |
| --- | --- | --- |
| `g_kronosPortArmv7mFamilyConfig` | `const kronos_port_armv7m_family_config_t` | Selected ARMv7-M STM32 family port configuration. |
| `SystemCoreClock` | `uint32_t` | External CMSIS clock variable used to configure SysTick. |

## Examples

### Minimal Application

```c
#include "kronos_core.h"

static void blink_task(void)
{
    for (;;)
    {
        board_led_toggle();
        Kronos_Delay(250U);
    }
}

int main(void)
{
    board_init();

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

### Producer and Consumer With Mailbox

```c
#include "kronos_core.h"

typedef struct
{
    uint32_t counter;
} counter_msg_t;

static void producer_task(void)
{
    counter_msg_t msg = { 0U };

    for (;;)
    {
        msg.counter++;
        (void)Kronos_EgressSendStruct("consumer", 1U, &msg);
        Kronos_Delay(1000U);
    }
}

static void consumer_task(void)
{
    kronos_mail_t mail;

    for (;;)
    {
        if (Kronos_IngressReceiveWait(&mail) == KRONOS_STATUS_OK)
        {
            if ((mail.message_id == 1U) && (mail.payload_size == sizeof(counter_msg_t)))
            {
                const counter_msg_t *msg = (const counter_msg_t *)mail.payload;
                consume_counter(msg->counter);
            }
        }
    }
}
```

### Mutex-Protected Peripheral Access

```c
#include "kronos_core.h"

static kronos_mutex_t spi_mutex;

static void spi_user_task(void)
{
    for (;;)
    {
        if (Kronos_MutexLock(&spi_mutex) == KRONOS_STATUS_OK)
        {
            spi_transfer();
            (void)Kronos_MutexUnlock(&spi_mutex);
        }

        Kronos_Delay(10U);
    }
}

void app_init(void)
{
    (void)Kronos_MutexInit(&spi_mutex);
}
```

### Semaphore for Event Signaling

```c
#include "kronos_core.h"

static kronos_semaphore_t event_sem;

static void worker_task(void)
{
    for (;;)
    {
        if (Kronos_SemaphoreTake(&event_sem) == KRONOS_STATUS_OK)
        {
            process_event();
        }
    }
}

static void event_source_task(void)
{
    for (;;)
    {
        wait_for_hardware_event();
        (void)Kronos_SemaphoreGive(&event_sem);
    }
}

void app_init(void)
{
    (void)Kronos_SemaphoreInit(&event_sem, 0U, 4U);
}
```

### Runtime Task Creation and Deletion

```c
#include "kronos_core.h"

static uint8_t worker_running = 0U;

static void worker_task(void)
{
    for (;;)
    {
        do_worker_step();
        Kronos_Delay(20U);
    }
}

static void manager_task(void)
{
    for (;;)
    {
        if (should_start_worker() && (worker_running == 0U))
        {
            if (Kronos_TaskCreate(worker_task, 96U, "worker") != KRONOS_STATUS_OK)
            {
                worker_running = 0U;
            }
            else
            {
                worker_running = 1U;
            }
        }

        if (should_stop_worker() && (worker_running != 0U))
        {
            (void)Kronos_TaskDelete("worker");
            worker_running = 0U;
        }

        Kronos_Delay(100U);
    }
}
```

### Privileged Driver Initialization

```c
#include "kronos_core.h"

typedef struct
{
    uint32_t baudrate;
} uart_init_context_t;

static kronos_status_e uart_init_privileged(void *context)
{
    const uart_init_context_t *uart_context = (const uart_init_context_t *)context;

    if (uart_context == NULL)
    {
        return KRONOS_STATUS_INVALID_ARGUMENT;
    }

    uart_hw_init(uart_context->baudrate);
    return KRONOS_STATUS_OK;
}

static void comms_task(void)
{
    uart_init_context_t uart_context = { 115200U };

    (void)Kronos_DriverInit(uart_init_privileged, &uart_context);

    for (;;)
    {
        comms_step();
        Kronos_Delay(10U);
    }
}
```

### Task Stack Monitoring

```c
#include "kronos_core.h"

static void monitor_task(void)
{
    for (;;)
    {
        const TCB_t *tasks = Kronos_GetTaskTable();
        uint32_t count = Kronos_GetTaskCount();

        for (uint32_t i = 0U; i < count; ++i)
        {
            if (tasks[i].task_name != NULL)
            {
                report_stack(tasks[i].task_name,
                             tasks[i].stack_used_words,
                             tasks[i].stack_free_words,
                             tasks[i].fault_flags);
            }
        }

        Kronos_Delay(1000U);
    }
}
```
