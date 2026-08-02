/* Hardware Interface module */

#include "ccs32_globals.h"

int16_t hardwareInterface_getInletVoltage(void)
{
    // we have no inlet voltage sensor. 
    return _ccs_params.EvseVoltage;
}

int16_t hardwareInterface_getBatteryVoltage(void)
{
    return _ccs_params.BatteryVoltage;
}

int16_t hardwareInterface_getChargingTargetVoltage(void)
{
    return _ccs_params.TargetVoltage;
}

int16_t hardwareInterface_getChargingTargetCurrent(void)
{
    return _ccs_params.TargetCurrent;
}

uint8_t hardwareInterface_getSoc(void)
{
    /* SOC in percent */
    return _ccs_params.soc;
}

bool hardwareInterface_getIsBatteryFull(void)
{
    return _ccs_params.soc >= STOP_CHARGING_SOC;
}

void hardwareInterface_setPowerRelayOn(void)
{
    log(MOD_PEV, "hardwareInterface_setPowerRelayOn");
    _ccs_params.PowerRelayOn = true;
}

void hardwareInterface_setPowerRelayOff(void)
{
    log(MOD_PEV, "hardwareInterface_setPowerRelayOff");
    _ccs_params.PowerRelayOn = false;
}

void hardwareInterface_setStateB(void)
{
    log(MOD_PEV, "hardwareInterface_setStateB");
    DigIo::state_c_out_inverted.Set();
}

void hardwareInterface_setStateC(void)
{
    log(MOD_PEV, "hardwareInterface_setStateC");
    DigIo::state_c_out_inverted.Clear();
}

void hardwareInterface_lockConnector(void)
{
    log(MOD_PEV, "Lock charging plug");
    _ccs_params.ConnectorLocked = true;
}

void hardwareInterface_unlockConnector(void)
{
    log(MOD_PEV, "Unlock charging plug");
    _ccs_params.ConnectorLocked = false;
}

bool hardwareInterface_isConnectorLocked(void)
{
    return _ccs_params.ConnectorLocked;
}

bool hardwareInterface_stopChargeRequested()
{
    return _global.powerOffPending;
}


