/* Hardware Interface module */

#include "ccs32_globals.h"

extern global_data _global;


int16_t hardwareInterface_getInletVoltage(void)
{
    // only called in precharge
    _global.auto_power_off_timer_count_up_ms = 0;

    // we have no inlet voltage sensor. 
    return Param::GetInt(Param::EvseVoltage);
}

int16_t hardwareInterface_getAccuVoltage(void)
{
   return Param::GetInt(Param::BatteryVoltage);
}

int16_t hardwareInterface_getChargingTargetVoltage(void)
{
   return Param::GetInt(Param::TargetVoltage);
}

int16_t hardwareInterface_getChargingTargetCurrent(void)
{
    // only called in CurrentDemand
    _global.auto_power_off_timer_count_up_ms = 0;

    return Param::GetInt(Param::ChargeCurrent);
}

uint8_t hardwareInterface_getSoc(void)
{
   /* SOC in percent */
   return Param::GetInt(Param::soc);
}

uint8_t hardwareInterface_getIsAccuFull(void)
{
   // Chademo: it make more sense that charger or the car should decide when to stop, and not the adapter?
   return Param::GetInt(Param::soc) == 100;
}

void hardwareInterface_setPowerRelayOn(void)
{
    println("hardwareInterface_setPowerRelayOn");
}

void hardwareInterface_setPowerRelayOff(void)
{
    println("hardwareInterface_setPowerRelayOff");
}

void hardwareInterface_setStateB(void)
{
   println("hardwareInterface_setStateB");
   DigIo::state_c_out_inverted.Set();
}

void hardwareInterface_setStateC(void)
{
   println("hardwareInterface_setStateC");
   DigIo::state_c_out_inverted.Clear();
}

void hardwareInterface_triggerConnectorLocking(void)
{
    Param::SetInt(Param::LockState, LOCK_CLOSED);

    _global.ccsConnectorLockingTrigger = true;
}

void hardwareInterface_triggerConnectorUnlocking(void)
{
    Param::SetInt(Param::LockState, LOCK_OPEN);
}

uint8_t hardwareInterface_isConnectorLocked(void)
{
    return Param::GetInt(Param::LockState) == LOCK_CLOSED;
}

uint8_t hardwareInterface_getPowerRelayConfirmation(void)
{
   /* todo */
   return 1;
}

bool hardwareInterface_stopChargeRequested()
{
    uint8_t stopReason = STOP_REASON_NONE;

    if (_global.powerOffPending)
    {
        stopReason = STOP_REASON_BUTTON;
        Param::SetInt(Param::StopReason, stopReason);
        addToTrace(MOD_HWIF, "Power off pending.");
    }

    if (!Param::GetBool(Param::enable))
    {
        stopReason = STOP_REASON_MISSING_ENABLE;
        Param::SetInt(Param::StopReason, stopReason);
        addToTrace(MOD_HWIF, "Got enable=false.");
    }

    return (stopReason != STOP_REASON_NONE);
}


