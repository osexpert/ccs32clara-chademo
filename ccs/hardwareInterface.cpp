/* Hardware Interface module */

#include "ccs32_globals.h"

extern global_data _global;
//extern bool stopButtonEvent;
//extern int auto_power_off_timer_count_up_sec;
//extern bool ccsPowerRelayOnTrigger;

void hardwareInterface_showOnDisplay(char*, char*, char*)
{
}

void hardwareInterface_initDisplay(void)
{
}

int hardwareInterface_sanityCheck()
{
   return 0; /* 0 is OK */
}

void hardwareInterface_simulatePreCharge(void)
{
}

void hardwareInterface_simulateCharging(void)
{
}

int16_t hardwareInterface_getInletVoltage(void)
{
    // only called in precharge, so we can abuse it as trigger
    _global.ccsPreChargeStartedEvent = true;

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
    // only called in CurrentDemand, so abuse it as trigger
    _global.ccsCurrentDemandStartedEvent = true;

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
   return Param::GetInt(Param::soc) > 95;
}

void hardwareInterface_setPowerRelayOn(void)
{
    printf("hardwareInterface_setPowerRelayOn\r\n");

    // abuse the fact that this is called at end of preCharge
    _global.ccsPreChargeDoneEvent = true;

    // D1 does not belong here??? or possibly...this will kick of the can...
    // even so....i would have used a different signaling
//    DigIo::switch_d1_out.Set();
}

void hardwareInterface_setPowerRelayOff(void)
{
    printf("hardwareInterface_setPowerRelayOff\r\n");

 //   printf("open adapter contactor\r\n");
    // adapter does nothing here...
    //DigIo::unknown_relay_out.Clear();

    // TODO: what to do here???
}

void hardwareInterface_setStateB(void)
{
   DigIo::state_c_out_inverted.Set();
}

void hardwareInterface_setStateC(void)
{
   DigIo::state_c_out_inverted.Clear();
}

void hardwareInterface_triggerConnectorLocking(void)
{
    Param::SetInt(Param::LockState, LOCK_CLOSED);
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

    return (stopReason!=STOP_REASON_NONE);
}

void hardwareInterface_resetSimulation(void)
{
}

/* send the measured CP duty cycle and PP resistance etc to the serial console for debugging. */
void hardwareInterface_LogTheCpPpPhysicalData(void) 
{
//      addToTrace(MOD_HWIF, "cpDuty [%] ", (int16_t)Param::GetInt(Param::ControlPilotDuty));
//      addToTrace(MOD_HWIF, "AdcProximityPilot ", (int16_t)Param::GetInt(Param::AdcProximityPilot));
//      addToTrace(MOD_HWIF, "ResistanceProxPilot [ohm] ", (int16_t)Param::GetInt(Param::ResistanceProxPilot));
//      addToTrace(MOD_HWIF, "HardwareVariant ", (int16_t)Param::GetInt(Param::HardwareVariant));
}

