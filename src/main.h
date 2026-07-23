#pragma once
/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

#include <stddef.h>

#pragma once

#pragma pack(push, 1)

template<size_t NS, size_t TNS, typename VT>
struct ConfigBlock
{
    char start_marker[9];
    char name[NS];
    char type[TNS];
    uint8_t value_len;
    volatile VT def_value;
    volatile VT config_value;
    char end_marker[10];
};

#pragma pack(pop)

#define CONFIG_ITEM(var_name, cfg_name_unused, type, def_value_unused) extern volatile type& var_name;

#include "config_list.h"

#undef CONFIG_ITEM

// Shifted priority macro for STM32 (only top 4 bits used in NVIC)
#define IRQ_PRI(x) ((x) << 4)

// Priority levels (lower = higher priority)
#define IRQ_PRIORITY_CAN_RX     IRQ_PRI(14)
#define IRQ_PRIORITY_SCHED      IRQ_PRI(15) // dont preemt anything but CAN?

struct global_data
{
    int stopButtonCounter = 0;

    bool ccsKickoff = CONFIG_SX ? true : false;
    bool powerOffPending = false;
    bool powerOffPendingViaButton = false;
    bool ccsEnded = false;
    bool moreLogging = false;

    uint32_t auto_power_off_timer_count_up_ms = 0;

    bool ccsLifesign = false;
};

extern global_data _global;

extern volatile uint32_t system_millis;
