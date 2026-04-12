/* Hardware Interface module */

#include "ccs32_globals.h"

extern global_data _global;
extern ccs_params _ccs_params;

int16_t hardwareInterface_getInletVoltage(void)
{
    // we have no inlet voltage sensor. 
    return _ccs_params.EvseVoltage;
}

int16_t hardwareInterface_getAccuVoltage(void)
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

bool hardwareInterface_getIsAccuFull(void)
{
    // Chademo: it make more sense that charger or the car should decide when to stop, and not the adapter?
    return _ccs_params.soc == 100;
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

static bool _connectorLocked = false;

void hardwareInterface_lockConnector(void)
{
    println("[ccs] Lock charging plug");
    _connectorLocked = true;
}

void hardwareInterface_unlockConnector(void)
{
    println("[ccs] Unlock charging plug");
    _connectorLocked = false;
}

bool hardwareInterface_isConnectorLocked(void)
{
    return _connectorLocked;
}

bool hardwareInterface_stopChargeRequested()
{
    return _global.powerOffPending;
}


