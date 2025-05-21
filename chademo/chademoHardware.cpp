#include <stdint.h>
#include "chademoCharger.h"
#include <libopencm3/stm32/can.h>
#include "params.h"
#include "digio.h"
#include "hwinit.h"

void ChademoCharger::CanSend(int id, bool ext, bool rtr, int len, uint8_t* data)
{
    //if (can_available_mailbox() > 0) {
    //    send_can_message();
    //}
    //else {
    //    // mailbox busy — queue message in software buffer and try later
    //}

    int res = can_transmit(CAN1, id, ext, rtr, len, data);
    if (res == -1)
    {
        printf("can_transmit mailbox full, try one more time...\r\n");
        res = can_transmit(CAN1, id, ext, rtr, len, data);
        if (res == -1)
        {
            printf("full on second try as well:-)\r\n");
            // We send so often, a few dropped messages should not be a dealbreaker
        }
    }
};

void ChademoCharger::ReadPendingCanMessages()
{
    uint32_t id;
    bool is_extended;
    bool is_rtr;
    uint8_t filter_match_index;
    uint8_t dlc;
    uint8_t data[8];
    uint16_t ts;

    // Check if a message is received in FIFO 0
    while (can_fifo_pending(CAN1, 0) > 0)
    {
        // Read the message from FIFO 0
        // Call the receive function
        uint32_t result = can_receive(
            CAN1,           // CAN port (e.g., 0 for CAN1)
            0,           // FIFO (usually 0 or 1)
            true,        // release = true → frees the mailbox after reading
            &id,
            &is_extended,
            &is_rtr,
            &filter_match_index,
            &dlc,
            data,
            &ts
        );

        // result = messages left in queue?
        if (result)
        {
            HandleChademoMessage(id, data);
        }
    }
}

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
    return prechargeCompletedTrigger;
};

bool ChademoCharger::GetSwitchK()
{
    return DigIo::switch_k_in_inverted.Get() == false; // inverted...0 = on
};

bool ChademoCharger::IsChargingStoppedByAdapter()
{
    return stopButtonPressedTrigger;
};

void ChademoCharger::NotifyCarContactorsClosed()
{
    // close our contactor too. it is not opened until power off
    DigIo::contactor_out.Set();
    // DigIo::external_led_out.Clear(); led on. pointless?
};

void ChademoCharger::StopPowerDelivery()
{
    Param::SetInt(Param::ChargeCurrent, 0);
    // TODO: should we disable at the end of the machine instead?
    Param::SetInt(Param::enable, false);
};

void ChademoCharger::StopVoltageDelivery()
{
    Param::SetInt(Param::TargetVoltage, 0);
//    Param::Set(Param::BatteryVoltage, 0);
    // TODO: should we disable at the end of the machine instead?
    Param::SetInt(Param::enable, false);
};

bool ChademoCharger::IsChargingStoppedByCharger()
{
    return Param::GetInt(Param::StopReason) != _stopreasons::STOP_REASON_NONE;
}
