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
    return _ccs_params.soc >= SOC_STOP_CHARGING;
}

void hardwareInterface_setPowerRelayOn(void)
{
    println("hardwareInterface_setPowerRelayOn");
    _ccs_params.PowerRelayOn = true;
}

void hardwareInterface_setPowerRelayOff(void)
{
    println("hardwareInterface_setPowerRelayOff");
    _ccs_params.PowerRelayOn = false;
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

void hardwareInterface_lockConnector(void)
{
    println("[ccs] Lock charging plug");
    _ccs_params.ConnectorLocked = true;
}

void hardwareInterface_unlockConnector(void)
{
    println("[ccs] Unlock charging plug");
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


