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

/* This file contains all parameters used in your project
 * See main.cpp on how to access them.
 * If a parameters unit is of format "0=Choice, 1=AnotherChoice" etc.
 * It will be displayed as a dropdown in the web interface
 * If it is a spot value, the decimal is translated to the name, i.e. 0 becomes "Choice"
 * If the enum values are powers of two, they will be displayed as flags, example
 * "0=None, 1=Flag1, 2=Flag2, 4=Flag3, 8=Flag4" and the value is 5.
 * It means that Flag1 and Flag3 are active -> Display "Flag1 | Flag3"
 *
 * Every parameter/value has a unique ID that must never change. This is used when loading parameters
 * from flash, so even across firmware versions saved parameters in flash can always be mapped
 * back to our list here. If a new value is added, it will receive its default value
 * because it will not be found in flash.
 * The unique ID is also used in the CAN module, to be able to recover the CAN map
 * no matter which firmware version saved it to flash.
 * Make sure to keep track of your ids and avoid duplicates. Also don't re-assign
 * IDs from deleted parameters because you will end up loading some random value
 * into your new parameter!
 * IDs are 16 bit, so 65535 is the maximum
 */


#include "myLogging.h"

//Next param id (increase when adding new parameter!): 34
//Next value Id: 2036
/*              category     name                  unit       min     max     default id */
#define PARAM_LIST \
/*    PARAM_ENTRY(CAT_HARDWARE,UdcDivider,           "dig/V",   0,      100,    10,     1   ) \ */ \
/*    PARAM_ENTRY(CAT_HARDWARE,EconomizerDuty,       "%",       0,      100,    100,    7   ) \ */ \
/*    PARAM_ENTRY(CAT_HARDWARE,InletVtgSrc,          IVSRC,     0,      2,      0,      8   ) \ */ \
/*    PARAM_ENTRY(CAT_HARDWARE,LockDuty,             "%",      -100,    100,    30,     14  ) \ */ \
/*    PARAM_ENTRY(CAT_HARDWARE,LockRunTime,          "ms",      0,      10000,  1500,   13  ) \  */ \
/*    PARAM_ENTRY(CAT_HARDWARE,LockClosedThresh,     "dig",     0,      4095,   0,      11  ) \ */ \
/*    PARAM_ENTRY(CAT_HARDWARE,LockOpenThresh,       "dig",     0,      4095,   0,      12  ) \ */ \
/*    PARAM_ENTRY(CAT_HARDWARE,TempSensorNomRes,     "Ohm",     1,      1000000,10000,  26  ) \ */ \
/*    PARAM_ENTRY(CAT_HARDWARE,TempSensorBeta,       "",        1,      100000, 3900,   27  ) \ */ \
/*    PARAM_ENTRY(CAT_HARDWARE,ppvariant,            PPVARIANT, 0,      9,      2,      28  ) \ */ \
/*    PARAM_ENTRY(CAT_HARDWARE,WakeupPinFunc,        WAKEUP,    0,      4,      0,      30  ) \ */ \
 /*   PARAM_ENTRY(CAT_HARDWARE,AllowUnlock,          OFFON,     0,      1,      1,      32 ) \ */ \
/*    PARAM_ENTRY(CAT_COMM,    NodeId,               "",        1,      63,     22,     21  ) \ */ \
 /*   PARAM_ENTRY(CAT_COMM,    CanSpeed,             CANSPEEDS, 0,      4,      2,      22  ) \ */ \
    PARAM_ENTRY(CAT_CHARGE,  MaxPower,             "kW",      0,      1000,   100,    17  ) \
    PARAM_ENTRY(CAT_CHARGE,  MaxVoltage,           "V",       0,      1000,   410,    18  ) \
    PARAM_ENTRY(CAT_CHARGE,  MaxCurrent,           "A",       0,      500,    125,    19  ) \
/*    PARAM_ENTRY(CAT_CHARGE,  MaxPinTemperature,    "°C",      0,      120,    70,     31  ) \ */ \
/*    PARAM_ENTRY(CAT_CHARGE,  AcChargeControl,      ACMODES,   0,      1,      0,      33  ) \ */ \
    TESTP_ENTRY(CAT_CHARGE,  TargetVoltage,        "V",       0,      1000,   0,      3   ) \
    TESTP_ENTRY(CAT_CHARGE,  ChargeCurrent,        "A",       0,      500,    0,      4   ) \
    TESTP_ENTRY(CAT_CHARGE,  soc,                  "%",       0,      100,    0,      5   ) \
    TESTP_ENTRY(CAT_CHARGE,  BatteryVoltage,       "V",       0,      1000,   0,      6   ) \
 /*   TESTP_ENTRY(CAT_CHARGE,  enable,               OFFON,     0,      1,      1,      23  ) */ \
 /*   TESTP_ENTRY(CAT_CHARGE,  AcObcState,           ACOBCSTT,  0,      5,      0,      29  ) \ */ \
/*    PARAM_ENTRY(CAT_TEST,    DemoVoltage,          "V",       0,      500,    0,      20  ) */  \
/*    PARAM_ENTRY(CAT_TEST,    DemoControl,          DEMOCTRL,  0,      511,    0,      25  ) \ */ \
/*    TESTP_ENTRY(CAT_TEST,    ActuatorTest,         ACTEST,    0,      7,      0,      9   ) \ */ \
    PARAM_ENTRY(CAT_TEST,    logging,              MODULES,   0,      2047,   DEFAULT_LOGGINGMASK,    15  ) \
    VALUE_ENTRY(opmode,             pevSttString,    2000 ) \
/*    VALUE_ENTRY(version,            VERSTR,          2001 ) \ */ \
/*    VALUE_ENTRY(lasterr,            errorListString, 2002 ) \ */ \
/*    VALUE_ENTRY(PortState,          PORTSTAT,        2030 ) */ \
/*     VALUE_ENTRY(BasicAcCharging,    OFFON,           2031 ) \ */ \
    VALUE_ENTRY(EvseVoltage,        "V",             2006 ) \
    VALUE_ENTRY(EvseCurrent,        "A",             2010 ) \
/*    VALUE_ENTRY(TempLimitedCurrent, "A",             2027 ) \ */ \
/*    VALUE_ENTRY(EVTargetCurrent,    "A",             2029 ) \ */ \
/*    VALUE_ENTRY(LimitationReason,   LIMITREASONS,    2028 ) \ */ \
/*    VALUE_ENTRY(InletVoltage,       "V",             2007 ) \ */ \
    VALUE_ENTRY(EvseMaxCurrent,     "A",             2008 ) \
    VALUE_ENTRY(EvseMaxVoltage,     "V",             2009 ) \
/*    VALUE_ENTRY(ControlPilotDuty,   "%",             2012 ) \ */ \
/*    VALUE_ENTRY(temp1,              "°C",            2003 ) \ */ \
/*    VALUE_ENTRY(temp2,              "°C",            2004 ) \ */ \
/*    VALUE_ENTRY(temp3,              "°C",            2005 ) \ */ \
/*    VALUE_ENTRY(MaxTemp,            "°C",            2024 ) \ */ \
/*    VALUE_ENTRY(ContactorDuty,      "%",             2013 ) \ */ \
/*    VALUE_ENTRY(AdcLockFeedback,    "dig",           2011 ) \ */ \
/*    VALUE_ENTRY(AdcProximityPilot,  "dig",           2018 ) \ */ \
/*    VALUE_ENTRY(ResistanceProxPilot,"ohm",           2019 ) \ */ \
/*     VALUE_ENTRY(CableCurrentLimit,  "A",             2020 ) \ */ \
/*    VALUE_ENTRY(EvseAcCurrentLimit, "A",             2021 ) \ */ \
/*    VALUE_ENTRY(PlugPresent,        "",              2034 ) \ */ \
/*    VALUE_ENTRY(AdcHwVariant,       "",              2022 ) \ */ \
/*    VALUE_ENTRY(HardwareVariant,    "",              2025 ) \*/  \
/*    VALUE_ENTRY(AdcIpropi,          "",              2023 ) \ */  \
/*    VALUE_ENTRY(LockState,          LOCK,            2014 ) \ */ \
    VALUE_ENTRY(StopReason,         STOPREASONS,     2017 ) \
    VALUE_ENTRY(checkpoint,         "dig",           2015 ) \
/*    VALUE_ENTRY(CanWatchdog,        "dig",           2016 ) \ */ \
/*    VALUE_ENTRY(CanAwake,           OFFON,           2032 ) \ */ \
/*    VALUE_ENTRY(ButtonPushed,       OFFON,           2033 ) \ */ \
/*    VALUE_ENTRY(VehicleSideIsoMonAllowed, OFFON,     2035 ) \ */ \
 /*   VALUE_ENTRY(cpuload,            "%",             2094 ) */


/***** Enum String definitions *****/
//#define IVSRC        "0=ChargerOutput, 1=AnalogInput, 2=CAN"
//#define LOCK         "0=None, 1=Open, 2=Closed, 3=Opening, 4=Closing"
//#define ACTEST       "0=None, 1=OpenLock, 2=CloseLock, 3=Contactor, 4=LedRed, 5=LedGreen, 6=LedBlue, 7=StateC"
#define MODULES      "0=None, 1=ConnMgr, 2=HwInterface, 4=Homeplug, 8=StateMachine, 16=QCA, 32=Tcp, 64=TcpTraffic, 128=IPV6, 256=ModemFinder, 512=SDP, 1024=EthTraffic, 2047=All, 959=AllButTraffic"
//#define CANSPEEDS    "0=125k, 1=250k, 2=500k, 3=800k, 4=1M"
//#define OFFON        "0=Off, 1=On"
//#define DEMOCTRL     "0=CAN, 234=StandAlone"
//#define ACMODES      "0=CANRemoteControlled, 1=StandAlone"
#define STOPREASONS  "0=None, 1=Button, 2=MissingEnable, 3=CANTimeout, 4=ChargerShutdown, 5=AccuFull, 6=ChargerEmergency, 7=InletOverheat, 8=EvseMalfunction"
//#define WAKEUP       "0=Level, 1=Pulse, 2=LevelOnValidCp, 3=PulseOnValidCp, 4=LevelOnValidPP"
//#define CAT_HARDWARE "Hardware Config"
//#define CAT_CHARGE   "Charge parameters"
//#define CAT_COMM     "Communication"
//#define CAT_TEST     "Testing"
//#define LIMITREASONS "0=None, 1=Charger, 2=Battery, 3=InletHot"
//#define PPVARIANT    "0=Foccci4.1_3V3_1k, 1=Foccci4.2_5V_330up_3000down, 2=Foccci4.5_5V_330up_no_down, 3=Foccci5_330up_no_down, 4=Foccci5_330up_2700down, 5=Foccci5_330up_3300down"
//#define ACOBCSTT     "0=Idle, 1=Lock, 2=Charging, 3=Pause, 4=Complete, 5=Error"
//#define PORTSTAT     "0=Idle, 1=PluggedIn, 2=Ready, 3=ChargingAC, 4=ChargingDC, 5=Stopping, 6=Unlock, 7=PortError"

/***** enums ******/


enum _stopreasons
{
   STOP_REASON_NONE,
   STOP_REASON_POWER_OFF_PENDING,
//   STOP_REASON_MISSING_ENABLE,
//   STOP_REASON_CAN_TIMEOUT,
   STOP_REASON_CHARGER_SHUTDOWN,
   STOP_REASON_ACCU_FULL,
   STOP_REASON_CHARGER_EMERGENCY_SHUTDOWN,
//   STOP_REASON_INLET_OVERHEAT,
   STOP_REASON_CHARGER_EVSE_MALFUNCTION
};


//Generated enum-string for possible errors
extern const char* errorListString;
//Generated enum string for PEV states
extern const char* pevSttString;

