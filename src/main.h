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


    // Shifted priority macro for STM32 (only top 4 bits used in NVIC)
#define IRQ_PRI(x) ((x) << 4)

// Priority levels (lower = higher priority)
//#define IRQ_PRIORITY_LOG     IRQ_PRI(13)  // dont interupt logging
#define IRQ_PRIORITY_CAN_TX     IRQ_PRI(14)
#define IRQ_PRIORITY_CAN_RX     IRQ_PRI(14)
#define IRQ_PRIORITY_SCHED      IRQ_PRI(15) // dont preemt anything but CAN?




#define CHA_CYCLE_MS 100 // was 10
#define CHA_CYCLES_PER_SEC (1000 / CHA_CYCLE_MS)

    struct global_data
    {
        int stopButtonCounter = 0;

        bool ccsKickoff = false;
        bool relayUnweldingAttempt = false;
        bool powerOffPending = false;
        bool ccsEnded = false;

        bool ccsPreChargeStartedTrigger = false;
        bool ccsCurrentDemandStartedTrigger = false;
        bool ccsPreChargeDoneButStalledTrigger = false;
        bool ccsConnectorLockingTrigger = false;

        uint32_t auto_power_off_timer_count_up_ms = 0;

        float adc_3_3_volt = 0.0f;
        float adc_4_volt = 0.0f;
        float adc_12_volt = 0.0f;
    };




#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
