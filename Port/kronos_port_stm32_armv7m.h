/*!
@file   kronos_port_stm32_armv7m.h
@brief  Shared ARMv7-M STM32 port definitions for KronOS
*/

#ifndef KRONOS_PORT_STM32_ARMV7M_H
#define KRONOS_PORT_STM32_ARMV7M_H

#include <stdint.h>

#include "kronos_port_stm32_family.h"

#define KRONOS_PORT_ARMV7M_REGION_ATTR_READ_ONLY       (6UL << 24)
#define KRONOS_PORT_ARMV7M_REGION_ATTR_READ_WRITE_XN  ((3UL << 24) | (1UL << 28))

typedef struct
{
    uint32_t base_address;
    uint32_t size_bytes;
    uint32_t attributes;
} kronos_port_armv7m_region_t;

typedef struct
{
    uint32_t default_core_clock_hz;
    uint32_t static_region_count;
    const kronos_port_armv7m_region_t *static_regions;
} kronos_port_armv7m_family_config_t;

extern const kronos_port_armv7m_family_config_t g_kronosPortArmv7mFamilyConfig;

#endif /* KRONOS_PORT_STM32_ARMV7M_H */