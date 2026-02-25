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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

#define ADAPTER_MAX_AMPS 200
#define ADAPTER_MAX_VOLTS 500 //Porsche Taycan requires 750V, but setting this value to 750 might break compatibility with many chargers. As default value 500V is good!

// just some value seen in can logs.. seen 10, 15. Some say car will allow 10A deviation from what you say? So if we always say we use 10A, we can use anything between 0-20A? 
// For any current above 20A then need real measured amps?
#define MAX_DISCHARGE_AMPS 20

//#define CHADEMO_STANDALONE_TESTING

// Shifted priority macro for STM32 (only top 4 bits used in NVIC)
#define IRQ_PRI(x) ((x) << 4)

// Priority levels (lower = higher priority)
#define IRQ_PRIORITY_CAN_RX     IRQ_PRI(14)
#define IRQ_PRIORITY_SCHED      IRQ_PRI(15) // dont preemt anything but CAN?

struct global_data
{
    int stopButtonCounter = 0;

    bool ccsKickoff = false;
    bool powerOffPending = false;
    bool ccsEnded = false;
    bool moreLogging = false;
    int alternative_voltage = GITHUB_AV;

    bool ccsPreChargeDoneButStalledTrigger = false;

    uint32_t auto_power_off_timer_count_up_ms = 0;
};

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
