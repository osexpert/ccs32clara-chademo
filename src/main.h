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

constexpr bool CONFIG_SX = GITHUB_SX;
constexpr bool CONFIG_V2X = GITHUB_V2X;
constexpr uint8_t CONFIG_ALTERNATIVE_VOLTAGE = GITHUB_AV;
constexpr bool CONFIG_ALWAYS_ON = GITHUB_AO;

// Jdemo: custom kit for Toyota RAV4, Tesla Rodster etc. Made by Quick Charge Power https://quickchargepower.com/ (QC Charge https://qccharge.com/).
// Jdemo is quirky in some ways:
// - after D2=true, charger must ACK (ChargerStatus::CHARGING=true / ChargerStatus::STOPPED=false) within 2 sec. It is more restrictive than the spec: 2.5sec.
// - If AvailableOutputCurrent is changed/lowered so that RequestCurrent reack AvailableOutputCurrent, the charging just stops (at least for lower values as 10).
// - It will/can increase RequestCurrent until it reaches AvailableOutputCurrent - 10 (so for a 200A charger, it will/can ask for 190A until it backs down). The kits were sold as 125A kits, but some were delivered with 200A cables.
// - It is possible that CHADEMO_MAX_UNDERSUPPLY_AMPS = 20 would work...since it has this weird 10A thing...but also never changing AvailableOutputCurrent seems to work.
constexpr bool CONFIG_JDEMO = GITHUB_JDEMO;

constexpr uint8_t ADAPTER_MAX_AMPS = 200;
constexpr uint16_t ADAPTER_MAX_VOLTS = 500; //Porsche Taycan requires 750V, but setting this value to 750 might break compatibility with many chargers. As default value 500V is good!

// Default SOC% at which the adapter will stop charging
constexpr uint8_t STOP_CHARGING_SOC = 100;

// Just some value seen in can logs.. seen 10, 15. Some say car will allow 10A deviation from what you say? So if we always say we use 10A, we can use anything between 0-20A? 
// For any current above 20A then need real measured amps? Try 40.
constexpr uint8_t MAX_DISCHARGE_AMPS_FALLBACK = 40;

constexpr bool CHADEMO_VOLTAGE_MODULATION = false;

constexpr uint16_t DX_CCS_WaitForPreChargeStart_MS = 2000;

// Based on logs, worst case is 1600ms before asking for amps, but use 2000 for now.
constexpr uint16_t CHADEMO_PRE1_WaitForCarContactorsClosed_MS = 2000;

constexpr uint8_t CHADEMO_MAX_UNDERSUPPLY_AMPS = 10;

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
