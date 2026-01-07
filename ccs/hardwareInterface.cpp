/* Hardware Interface module */

#include "ccs32_globals.h"

extern global_data _global;


int16_t hardwareInterface_getInletVoltage(void)
{
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

uint8_t hardwareInterface_getPowerRelayConfirmation(void)
{
    /* todo */
    return 1;
}

bool hardwareInterface_stopChargeRequested()
{
    return _global.powerOffPending;
}


