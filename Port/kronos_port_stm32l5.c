/*!
@file   kronos_port_stm32l5.c
@brief  KronOS STM32L5 port using the ARMv8-M MPU register model
@t.odo  -
*/

/******************************************************************************
* Includes
******************************************************************************/

#include "kronos_port.h"

#include "kronos_kernel.h"
#include "kronos_port_stm32_family.h"

#include <stddef.h>

#if KRONOS_PORT_STM32_FAMILY_L5

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

#define MPU_CTRL  REG32(0xE000ED94UL)
#define MPU_RNR   REG32(0xE000ED98UL)
#define MPU_RBAR  REG32(0xE000ED9CUL)
#define MPU_RLAR  REG32(0xE000EDA0UL)
#define MPU_MAIR0 REG32(0xE000EDC0UL)

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

#define MPU_RBAR_XN_Pos    0U
#define MPU_RBAR_AP_Pos    1U
#define MPU_RLAR_ENABLE_Msk (1UL << 0)

#define MPU_MAIR_ATTR_NORMAL_WBWA  0xFFUL
#define MPU_MAIR_ATTR_DEVICE_nGnRE 0x04UL

#define MEMFAULT_PRIORITY_BYTE 0x20UL
#define PENDSV_PRIORITY_BYTE   0xFFUL
#define SYSTICK_PRIORITY_BYTE  0xFEUL

#define KRONOS_PORT_L5_ATTR_INDEX_NORMAL 0U
#define KRONOS_PORT_L5_ATTR_INDEX_DEVICE 1U
#define KRONOS_PORT_L5_AP_READ_WRITE     0U
#define KRONOS_PORT_L5_AP_READ_ONLY      2U

/******************************************************************************
* Enumerations, Structures & Variables
******************************************************************************/

typedef struct
{
    uint32_t base_address;
    uint32_t size_bytes;
    uint32_t ap_bits;
    uint32_t execute_never;
    uint32_t attr_index;
} kronos_port_l5_region_t;

static const kronos_port_l5_region_t g_kronosPortStaticRegions[] = {
    { 0x08000000UL, 0x00100000UL, KRONOS_PORT_L5_AP_READ_ONLY, 0U, KRONOS_PORT_L5_ATTR_INDEX_NORMAL },
    { 0x20000000UL, 0x00080000UL, KRONOS_PORT_L5_AP_READ_WRITE, 1U, KRONOS_PORT_L5_ATTR_INDEX_NORMAL },
    { 0x40000000UL, 0x20000000UL, KRONOS_PORT_L5_AP_READ_WRITE, 1U, KRONOS_PORT_L5_ATTR_INDEX_DEVICE },
    { 0xE0000000UL, 0x00001000UL, KRONOS_PORT_L5_AP_READ_WRITE, 1U, KRONOS_PORT_L5_ATTR_INDEX_DEVICE }
};

extern uint32_t SystemCoreClock;

static volatile uint32_t g_portSchedulerStarted = 0U;

/******************************************************************************
* Declaration | Static Functions
******************************************************************************/

static void kronos_port_enable_fpu_if_present(void);
static void kronos_port_program_region(uint32_t regionNumber, uint32_t baseAddress, uint32_t sizeBytes, uint32_t apBits, uint32_t executeNever, uint32_t attrIndex);
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

static void kronos_port_program_region(uint32_t regionNumber, uint32_t baseAddress, uint32_t sizeBytes, uint32_t apBits, uint32_t executeNever, uint32_t attrIndex)
{
    uint32_t limitAddress;

    limitAddress = baseAddress + sizeBytes - 1U;

    MPU_RNR = regionNumber;
    MPU_RBAR = (baseAddress & 0xFFFFFFE0UL) |
               ((apBits & 0x3UL) << MPU_RBAR_AP_Pos) |
               ((executeNever & 0x1UL) << MPU_RBAR_XN_Pos);
    MPU_RLAR = (limitAddress & 0xFFFFFFE0UL) |
               ((attrIndex & 0x7UL) << 1U) |
               MPU_RLAR_ENABLE_Msk;
}

static void kronos_port_disable_region(uint32_t regionNumber)
{
    MPU_RNR = regionNumber;
    MPU_RBAR = 0U;
    MPU_RLAR = 0U;
}

static void kronos_port_configure_static_regions(void)
{
    uint32_t regionNumber;

    MPU_MAIR0 = MPU_MAIR_ATTR_NORMAL_WBWA |
                (MPU_MAIR_ATTR_DEVICE_nGnRE << 8);

    for (regionNumber = 0U; regionNumber < KRONOS_PORT_MPU_REGION_COUNT; ++regionNumber)
    {
        kronos_port_disable_region(regionNumber);
    }

    for (regionNumber = 0U; regionNumber < (sizeof(g_kronosPortStaticRegions) / sizeof(g_kronosPortStaticRegions[0])); ++regionNumber)
    {
        const kronos_port_l5_region_t *region;

        region = &g_kronosPortStaticRegions[regionNumber];
        kronos_port_program_region(
            regionNumber,
            region->base_address,
            region->size_bytes,
            region->ap_bits,
            region->execute_never,
            region->attr_index);
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
        KRONOS_PORT_L5_AP_READ_WRITE,
        1U,
        KRONOS_PORT_L5_ATTR_INDEX_NORMAL);
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
    return 4000000UL;
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