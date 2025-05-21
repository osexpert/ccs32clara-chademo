#include <stdint.h>
#include "chademoCharger.h"
#include <libopencm3/stm32/can.h>
#include "params.h"
#include "digio.h"
#include "hwinit.h"

void ChademoCharger::CanSend(int id, bool ext, bool rtr, int len, uint8_t* data)
{
    can_transmit(CAN1, id, ext, rtr, len, data);
};

void ChademoCharger::SetSwitchD2(bool set)
{
    if (set)
        DigIo::switch_d2_out_inverted.Clear(); // 0: on
    else
        DigIo::switch_d2_out_inverted.Set(); // 1: off
};

void ChademoCharger::SetSwitchD1(bool set)
{
    if (set)
        DigIo::switch_d1_out.Set();
    else
        DigIo::switch_d1_out.Clear();
};

bool ChademoCharger::IsChargerLive()
{
    return prechargeCompleted;
};

bool ChademoCharger::GetSwitchK()
{
    return DigIo::switch_k_in_inverted.Get() == false; // inverted...0 = on
};

bool ChademoCharger::IsChargingStoppedByAdapter()
{
    return stopButtonPressed;
};

void ChademoCharger::NotifyCarContactorsClosed()
{
    // close our contactor too. it is not opened until power off
    DigIo::contactor_out.Set();
    // DigIo::external_led_out.Clear(); led on. pointless?
};

void ChademoCharger::StopPowerDelivery()
{
    Param::Set(Param::ChargeCurrent, 0);
    // TODO: should we disable at the end of the machine instead?
    Param::Set(Param::enable, false);
};

void ChademoCharger::StopVoltageDelivery()
{
    Param::Set(Param::TargetVoltage, 0);
    Param::Set(Param::BatteryVoltage, 0);
    // TODO: should we disable at the end of the machine instead?
    Param::Set(Param::enable, false);
};

bool ChademoCharger::IsChargingStoppedByCharger()
{
    return Param::GetInt(Param::StopReason) != _stopreasons::STOP_REASON_NONE;
}
