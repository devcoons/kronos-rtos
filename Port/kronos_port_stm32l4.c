/*!
@file   kronos_port_stm32l4.c
@brief  STM32L4 family configuration for the KronOS ARMv7-M port
@t.odo  -
*/

/******************************************************************************
* Includes
******************************************************************************/

#include "kronos_port_stm32_armv7m.h"

#if KRONOS_PORT_STM32_FAMILY_L4

/******************************************************************************
* Enumerations, Structures & Variables
******************************************************************************/

static const kronos_port_armv7m_region_t g_kronosPortStaticRegions[] = {
    { 0x08000000UL, 0x00100000UL, KRONOS_PORT_ARMV7M_REGION_ATTR_READ_ONLY },
    { 0x20000000UL, 0x00040000UL, KRONOS_PORT_ARMV7M_REGION_ATTR_READ_WRITE_XN },
    { 0x40000000UL, 0x20000000UL, KRONOS_PORT_ARMV7M_REGION_ATTR_READ_WRITE_XN },
    { 0xE0000000UL, 0x00001000UL, KRONOS_PORT_ARMV7M_REGION_ATTR_READ_WRITE_XN }
};

const kronos_port_armv7m_family_config_t g_kronosPortArmv7mFamilyConfig = {
    4000000UL,
    sizeof(g_kronosPortStaticRegions) / sizeof(g_kronosPortStaticRegions[0]),
    g_kronosPortStaticRegions
};

/******************************************************************************
* EOF - NO CODE AFTER THIS LINE
******************************************************************************/
#endif