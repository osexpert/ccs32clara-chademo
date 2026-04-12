/*
 * This file is part of the stm32-template project.
 *
 * Copyright (C) 2020 Johannes Huebner <dev@johanneshuebner.com>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "myLogging.h"

/***** enums ******/


enum _stopreasons
{
   STOP_REASON_NONE,
   STOP_REASON_POWER_OFF_PENDING,
   STOP_REASON_CHARGER_SHUTDOWN,
   STOP_REASON_ACCU_FULL,
   STOP_REASON_CHARGER_EMERGENCY_SHUTDOWN,
   STOP_REASON_CHARGER_EVSE_MALFUNCTION,
   STOP_REASON_TIMEOUT
};


//Generated enum-string for possible errors
//extern const char* errorListString;
//Generated enum string for PEV states
//extern const char* pevSttString;


struct ccs_params
{
    int logging = DEFAULT_LOGGINGMASK;
    int opmode = 0;
    int checkpoint = 0;

    int MaxPower = 100; // kW
    int MaxVoltage = 410; // V
    int MaxCurrent = 125; // A

    int TargetVoltage = 0;
    int TargetCurrent = 0;

    int soc = 0;
    int BatteryVoltage = 0;

    int EvseMaxCurrent = 0;
    int EvseMaxVoltage = 0;

    int EvseVoltage = 0;
    int EvseCurrent = 0;
    int EvseMaxCurrentInCurrentDemandRes = 0;
    _stopreasons CurrentDemandStopReason = STOP_REASON_NONE;
};
