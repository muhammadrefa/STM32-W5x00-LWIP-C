/**
 * Copyright (c) 2022 WIZnet Co.,Ltd
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

/**
 * ----------------------------------------------------------------------------------------------------
 * Includes
 * ----------------------------------------------------------------------------------------------------
 */
#include <stdio.h>
#include <stdint.h>
#include "timer.h"

/**
 * ----------------------------------------------------------------------------------------------------
 * Variables
 * ----------------------------------------------------------------------------------------------------
 */
/* Timer */
void (*callback_ptr)(void);

/**
 * ----------------------------------------------------------------------------------------------------
 * Functions
 * ----------------------------------------------------------------------------------------------------
 */
/* Timer */
void wizchip_1ms_timer_initialize(void (*callback)(void))
{
    // 1 ms timer initialization done in main.c (via STM32CubeMX)
    // Call the callback inside stm32xx_it.c
    callback_ptr = callback;
}

uint8_t wizchip_1ms_timer_callback()
{
    if (callback_ptr != NULL)
    {
        callback_ptr();
    }
}

/* Delay */
void wizchip_delay_ms(uint32_t ms)
{
    HAL_Delay(ms);
}
