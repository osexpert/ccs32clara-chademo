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

bool ChademoCharger::GetSwitchK()
{
    return DigIo::switch_k_in_inverted.Get() == false; // inverted...0 = on
};

bool ChademoCharger::IsChargingStoppedByAdapter()
{
    return stopButtonPressed;
};

void ChademoCharger::NotifyAdapterGpioStuffAfterContactorClosed()
{
    // wonder what this is...
    DigIo::chademo_unknown_out.Set();
    // DigIo::internal_led_out.Clear(); pointless
};

void ChademoCharger::StopPowerDelivery()
{
    // TODO: also lower the voltage?
    Param::Set(Param::enable, false);
};

bool ChademoCharger::IsChargingStoppedByCharger()
{
    return Param::GetInt(Param::StopReason) != _stopreasons::STOP_REASON_NONE;
}
