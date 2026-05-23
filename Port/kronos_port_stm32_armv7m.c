/*!
@file   kronos_port_stm32_armv7m.c
@brief  Shared KronOS STM32 port for ARMv7-M STM32 families
@t.odo  -
*/

/******************************************************************************
* Includes
******************************************************************************/

#include "kronos_port.h"

#include "kronos_kernel.h"
#include "kronos_port_stm32_armv7m.h"

#include <stddef.h>

#if KRONOS_PORT_STM32_IS_ARMV7M

/******************************************************************************
* Declaration | Public Functions
******************************************************************************/

void SysTick_Handler(void);
void PendSV_Handler(void);
void SVC_Handler(void);
void MemManage_Handler(void);

/******************************************************************************
* Preprocessor Definitions & Macros
******************************************************************************/

#define REG32(address) (*(volatile uint32_t *)(address))

#define SYSTICK_CTRL REG32(0xE000E010UL)
#define SYSTICK_LOAD REG32(0xE000E014UL)
#define SYSTICK_VAL  REG32(0xE000E018UL)

#define SCB_CPACR REG32(0xE000ED88UL)
#define SCB_ICSR  REG32(0xE000ED04UL)
#define SCB_SHPR1 REG32(0xE000ED18UL)
#define SCB_SHPR3 REG32(0xE000ED20UL)
#define SCB_SHCSR REG32(0xE000ED24UL)
#define SCB_CFSR  REG32(0xE000ED28UL)
#define SCB_MMFAR REG32(0xE000ED34UL)

#define MPU_CTRL REG32(0xE000ED94UL)
#define MPU_RNR  REG32(0xE000ED98UL)
#define MPU_RBAR REG32(0xE000ED9CUL)
#define MPU_RASR REG32(0xE000EDA0UL)

#define SYSTICK_CTRL_ENABLE_Msk    (1UL << 0)
#define SYSTICK_CTRL_TICKINT_Msk   (1UL << 1)
#define SYSTICK_CTRL_CLKSOURCE_Msk (1UL << 2)

#define SCB_CPACR_CP10_CP11_FULL_ACCESS_Msk ((3UL << 20) | (3UL << 22))
#define SCB_ICSR_PENDSVSET_Msk               (1UL << 28)
#define SCB_SHCSR_MEMFAULTENA_Msk            (1UL << 16)
#define SCB_CFSR_MEMFAULT_MASK               0x000000FFUL
#define SCB_CFSR_MMARVALID_Msk               (1UL << 7)

#define MPU_CTRL_ENABLE_Msk     (1UL << 0)
#define MPU_CTRL_PRIVDEFENA_Msk (1UL << 2)

#define MPU_RASR_ENABLE_Msk (1UL << 0)

#define MEMFAULT_PRIORITY_BYTE 0x20UL
#define PENDSV_PRIORITY_BYTE   0xFFUL
#define SYSTICK_PRIORITY_BYTE  0xFEUL

/******************************************************************************
* Enumerations, Structures & Variables
******************************************************************************/

extern uint32_t SystemCoreClock;

static volatile uint32_t g_portSchedulerStarted = 0U;

/******************************************************************************
* Declaration | Static Functions
******************************************************************************/

static void kronos_port_enable_fpu_if_present(void);
static uint32_t kronos_port_encode_region_size(uint32_t sizeBytes);
static void kronos_port_program_region(uint32_t regionNumber, uint32_t baseAddress, uint32_t sizeBytes, uint32_t attributes);
static void kronos_port_disable_region(uint32_t regionNumber);
static void kronos_port_configure_static_regions(void);
static __attribute__((used)) uint32_t *kronos_port_handle_memmanage(uint32_t *faultStackPtr);

/******************************************************************************
* Definition | Static Functions
******************************************************************************/

static void kronos_port_enable_fpu_if_present(void)
{
#if defined(__ARM_FP) && (__ARM_FP != 0)
    SCB_CPACR |= SCB_CPACR_CP10_CP11_FULL_ACCESS_Msk;
    __asm volatile ("dsb");
    __asm volatile ("isb");
#endif
}

static uint32_t kronos_port_encode_region_size(uint32_t sizeBytes)
{
    uint32_t regionBytes;
    uint32_t sizeField;

    regionBytes = 32U;
    sizeField = 4U;

    while (regionBytes < sizeBytes)
    {
        regionBytes <<= 1U;
        sizeField++;
    }

    return sizeField << 1U;
}

static void kronos_port_program_region(uint32_t regionNumber, uint32_t baseAddress, uint32_t sizeBytes, uint32_t attributes)
{
    MPU_RNR = regionNumber;
    MPU_RBAR = baseAddress;
    MPU_RASR = attributes | kronos_port_encode_region_size(sizeBytes) | MPU_RASR_ENABLE_Msk;
}

static void kronos_port_disable_region(uint32_t regionNumber)
{
    MPU_RNR = regionNumber;
    MPU_RBAR = 0U;
    MPU_RASR = 0U;
}

static void kronos_port_configure_static_regions(void)
{
    uint32_t regionNumber;

    for (regionNumber = 0U; regionNumber < KRONOS_PORT_MPU_REGION_COUNT; ++regionNumber)
    {
        kronos_port_disable_region(regionNumber);
    }

    for (regionNumber = 0U; regionNumber < g_kronosPortArmv7mFamilyConfig.static_region_count; ++regionNumber)
    {
        const kronos_port_armv7m_region_t *region;

        region = &g_kronosPortArmv7mFamilyConfig.static_regions[regionNumber];
        kronos_port_program_region(regionNumber, region->base_address, region->size_bytes, region->attributes);
    }

    kronos_port_disable_region(KRONOS_PORT_TASK_STACK_REGION_INDEX);
}

static __attribute__((used)) uint32_t *kronos_port_handle_memmanage(uint32_t *faultStackPtr)
{
    uint32_t faultStatus;
    uint32_t faultAddress;

    faultStatus = SCB_CFSR & SCB_CFSR_MEMFAULT_MASK;
    faultAddress = ((faultStatus & SCB_CFSR_MMARVALID_Msk) != 0U) ? SCB_MMFAR : 0U;

    SCB_CFSR = faultStatus;

    return Kronos_CoreQuarantineCurrentTask(faultStackPtr, KRONOS_TASK_FAULT_MEMORY_ACCESS, faultAddress);
}

/******************************************************************************
* Definition | Public Functions
******************************************************************************/

void Kronos_PortInitScheduler(uint32_t tickFrequencyHz)
{
    uint32_t shpr1;
    uint32_t shpr3;
    uint32_t systickReload;

    g_portSchedulerStarted = 0U;

    kronos_port_enable_fpu_if_present();

    if (SystemCoreClock == 0U)
    {
        SystemCoreClock = Kronos_PortGetDefaultClockHz();
    }

    systickReload = (SystemCoreClock / tickFrequencyHz) - 1U;

    SYSTICK_CTRL = 0U;
    SYSTICK_LOAD = systickReload;
    SYSTICK_VAL = 0U;

    shpr1 = SCB_SHPR1;
    shpr1 &= 0xFFFFFF00UL;
    shpr1 |= MEMFAULT_PRIORITY_BYTE;
    SCB_SHPR1 = shpr1;

    shpr3 = SCB_SHPR3;
    shpr3 &= 0x0000FFFFUL;
    shpr3 |= (PENDSV_PRIORITY_BYTE << 16);
    shpr3 |= (SYSTICK_PRIORITY_BYTE << 24);
    SCB_SHPR3 = shpr3;

    kronos_port_configure_static_regions();
    SCB_SHCSR |= SCB_SHCSR_MEMFAULTENA_Msk;
    MPU_CTRL = MPU_CTRL_ENABLE_Msk | MPU_CTRL_PRIVDEFENA_Msk;

    SYSTICK_CTRL = SYSTICK_CTRL_CLKSOURCE_Msk |
                   SYSTICK_CTRL_TICKINT_Msk |
                   SYSTICK_CTRL_ENABLE_Msk;
}

void Kronos_PortStartScheduler(void)
{
    __asm volatile ("svc #0");
}

void Kronos_PortRequestYield(void)
{
    __asm volatile ("svc #0");
}

void Kronos_PortPendContextSwitch(void)
{
    SCB_ICSR = SCB_ICSR_PENDSVSET_Msk;
}

void Kronos_PortConfigureTaskStackRegion(const uint32_t *stackBase, uint32_t stackWords)
{
    if ((stackBase == NULL) || (stackWords == 0U))
    {
        kronos_port_disable_region(KRONOS_PORT_TASK_STACK_REGION_INDEX);
        return;
    }

    kronos_port_program_region(
        KRONOS_PORT_TASK_STACK_REGION_INDEX,
        (uint32_t)(uintptr_t)stackBase,
        stackWords * sizeof(uint32_t),
        KRONOS_PORT_ARMV7M_REGION_ATTR_READ_WRITE_XN);
}

uint32_t *Kronos_PortGetLiveProcessStackPointer(void)
{
    uint32_t *stackPtr;

    __asm volatile ("mrs %0, psp" : "=r" (stackPtr));

    return stackPtr;
}

uint32_t Kronos_PortComputeTaskSlotWords(uint32_t requestedStackWords)
{
    uint32_t minimumWords;
    uint32_t slotWords;

    minimumWords = requestedStackWords + KRONOS_PORT_STACK_RECOVERY_WORDS;
    slotWords = 8U;

    while (slotWords < minimumWords)
    {
        slotWords <<= 1U;
    }

    return slotWords;
}

uint32_t Kronos_PortGetDefaultClockHz(void)
{
    return g_kronosPortArmv7mFamilyConfig.default_core_clock_hz;
}

uint32_t Kronos_PortGetStackWarningMarginWords(void)
{
    return KRONOS_PORT_STACK_WARNING_MARGIN_WORDS;
}

void SysTick_Handler(void)
{
    Kronos_CoreOnTick();
}

__attribute__((naked)) void PendSV_Handler(void)
{
    __asm volatile (
        "cpsid  i                              \n"
        "mrs    r0, psp                        \n"
        "push   {lr}                           \n"
        "bl     Kronos_CoreCheckCurrentTaskStack \n"
        "cmp    r0, #2                         \n"
        "bne    1f                             \n"
        "mrs    r0, psp                        \n"
        "movs   r1, #2                         \n"
        "movs   r2, #0                         \n"
        "bl     Kronos_CoreQuarantineCurrentTask \n"
        "b      2f                             \n"
        "1:                                    \n"
        "mrs    r0, psp                        \n"
        "stmdb  r0!, {r4-r11}                  \n"
        "bl     Kronos_CoreSwitchTask          \n"
        "2:                                    \n"
        "pop    {lr}                           \n"
        "ldmia  r0!, {r4-r11}                  \n"
        "msr    psp, r0                        \n"
        "cpsie  i                              \n"
        "bx     lr                             \n"
    );
}

__attribute__((naked)) void SVC_Handler(void)
{
    __asm volatile (
        "ldr    r3, =g_portSchedulerStarted    \n"
        "ldr    r2, [r3]                       \n"
        "cbnz   r2, 1f                         \n"
        "movs   r2, #1                         \n"
        "str    r2, [r3]                       \n"
        "bl     Kronos_CorePrepareFirstTask    \n"
        "ldmia  r0!, {r4-r11}                  \n"
        "msr    psp, r0                        \n"
        "movs   r0, #3                         \n"
        "msr    CONTROL, r0                    \n"
        "isb                                   \n"
        "cpsie  i                              \n"
        "ldr    lr, =0xFFFFFFFD                \n"
        "bx     lr                             \n"
        "1:                                    \n"
        "cpsid  i                              \n"
        "mrs    r0, psp                        \n"
        "push   {lr}                           \n"
        "bl     Kronos_CoreCheckCurrentTaskStack \n"
        "cmp    r0, #2                         \n"
        "bne    2f                             \n"
        "mrs    r0, psp                        \n"
        "movs   r1, #2                         \n"
        "movs   r2, #0                         \n"
        "bl     Kronos_CoreQuarantineCurrentTask \n"
        "b      3f                             \n"
        "2:                                    \n"
        "mrs    r0, psp                        \n"
        "stmdb  r0!, {r4-r11}                  \n"
        "bl     Kronos_CoreHandleSupervisorCall \n"
        "3:                                    \n"
        "pop    {lr}                           \n"
        "ldmia  r0!, {r4-r11}                  \n"
        "msr    psp, r0                        \n"
        "cpsie  i                              \n"
        "bx     lr                             \n"
    );
}

__attribute__((naked)) void MemManage_Handler(void)
{
    __asm volatile (
        "tst    lr, #4                         \n"
        "beq    1f                             \n"
        "cpsid  i                              \n"
        "mrs    r0, psp                        \n"
        "push   {lr}                           \n"
        "bl     kronos_port_handle_memmanage   \n"
        "pop    {lr}                           \n"
        "ldmia  r0!, {r4-r11}                  \n"
        "msr    psp, r0                        \n"
        "cpsie  i                              \n"
        "ldr    lr, =0xFFFFFFFD                \n"
        "bx     lr                             \n"
        "1:                                    \n"
        "b      1b                             \n"
    );
}

/******************************************************************************
* EOF - NO CODE AFTER THIS LINE
******************************************************************************/
#endif