#pragma once
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
#include "main.h"

/***** enums ******/


enum _stopreasons
{
   STOP_REASON_NONE,
   STOP_REASON_POWER_OFF_PENDING,
   STOP_REASON_CHARGER_SHUTDOWN,
   STOP_REASON_BATTERY_FULL,
   STOP_REASON_CHARGER_EMERGENCY_SHUTDOWN,
   STOP_REASON_CHARGER_EVSE_MALFUNCTION,
   STOP_REASON_TIMEOUT
};


struct ccs_params
{
    ccs_params()
    {
        init();
    }

    int logging = DEFAULT_LOGGINGMASK;
    int opmode;
    int checkpoint;

    int MaxPower;
    int MaxVoltage;
    int MaxCurrent;

    int TargetVoltage;
    int TargetCurrent;

    int soc;
    int BatteryVoltage;

    int EvseMaxCurrent;
    int EvseMaxVoltage;
    int EvseMinimumVoltage;

    int EvseVoltage;
    int EvseCurrent;
    int EvseMaxCurrentInCurrentDemandRes;
    int CurrentDemandStopReason;

    int EvseDynCurrent() const
    {
        return EvseMaxCurrentInCurrentDemandRes == 0 ?
            EvseMaxCurrent :
            EvseMaxCurrentInCurrentDemandRes;
    }

    void init()
    {
        //logging = DEFAULT_LOGGINGMASK; keep
        opmode = 0;
        checkpoint = 0;

        MaxPower = 100;           // kW
        MaxVoltage = 410;         // V
        MaxCurrent = ADAPTER_MAX_AMPS;

        TargetVoltage = 0;
        TargetCurrent = 0;

        soc = 0;
        BatteryVoltage = 0;

        EvseMaxCurrent = 0;
        EvseMaxVoltage = 0;
        EvseMinimumVoltage = 0;

        EvseVoltage = 0;
        EvseCurrent = 0;
        EvseMaxCurrentInCurrentDemandRes = 0;
        CurrentDemandStopReason = STOP_REASON_NONE;
    }
};

extern ccs_params _ccs_params;
