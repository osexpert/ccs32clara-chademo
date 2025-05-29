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
#define IRQ_PRIORITY_CAN        IRQ_PRI(14) // can most important?
#define IRQ_PRIORITY_SCHED      IRQ_PRI(15) // dont preemt anything but can?




#define CHA_CYCLE_MS 10
#define CHA_CYCLES_PER_SEC (1000 / CHA_CYCLE_MS)

    struct global_data
    {
        int stopButtonCounter = 0;

        bool ccsKickoff = false;

        bool stopButtonEvent = false;

        // ccs delivered 1 amps or more
        bool ccsDeliveredAmpsEvent = false;

        bool ccsPowerRelayOnTrigger_prechargeDone = false;

        uint32_t auto_power_off_timer_count_up_ms = 0;

        int cha100 = 0;
        int cha101 = 0;
        int cha102 = 0;
        int chaOther = 0;

        int cha108 = 0;
        int cha109 = 0;

        uint32_t cha108last = 0;
        uint32_t cha108dur = 0;
        uint32_t cha109last = 0;
        uint32_t cha109dur = 0;
    };



/* Exported functions prototypes ---------------------------------------------*/
//void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
//#define OUT_STATE_C_CONTROL_Pin GPIO_PIN_4
//#define OUT_STATE_C_CONTROL_GPIO_Port GPIOB
//#define OUT_CONTACTOR_CONTROL1_Pin GPIO_PIN_5
//#define OUT_CONTACTOR_CONTROL1_GPIO_Port GPIOB
//#define OUT_CONTACTOR_CONTROL2_Pin GPIO_PIN_6
//#define OUT_CONTACTOR_CONTROL2_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
