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
        _canTxDrop++;
    }
    else
    {
//        _lastCanSendTime = 
    }

    // try 2 times
    //for (int i = 0; i < 2; i++)
    //{
    //    int res = can_transmit(CAN1, id, ext, rtr, len, data);

    //    //@returns int 0, 1 or 2 on success and depending on which outgoing mailbox got
    //    //    selected. - 1 if no mailbox was available and no transmission got queued.

    //    if (res == -1)
    //    {
    //        printf("mailbox full, retry %d\r\n", i);
    //        // We send so often, a few dropped messages should not be a dealbreaker
    //    }
    //    else // queued
    //    {
    //        _lastCanSendTime = rtc_get_ms();
    //        return;
    //    }

    //}

    //int res = can_transmit(CAN1, id, ext, rtr, len, data);
    //if (res == -1)
    //{
    //    printf("can_transmit mailbox full, try one more time...\r\n");
    //    res = can_transmit(CAN1, id, ext, rtr, len, data);
    //    if (res == -1)
    //    {
    //        printf("full on second try as well:-)\r\n");
    //        // We send so often, a few dropped messages should not be a dealbreaker
    //    }
    //}
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

        // result = nt 0-3 depending on how many messages where pending before
        if (result)
        {
//            _lastCanRecieveTime = rtc_get_ms();
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
    return ccsPowerRelayOnTrigger;
};

bool ChademoCharger::GetSwitchK()
{
    return DigIo::switch_k_in_inverted.Get() == false; // inverted...0 = on
};

bool ChademoCharger::IsChargingStoppedByAdapter()
{
    return stopButtonTrigger;
};

void ChademoCharger::NotifyCarContactorsClosed()
{
    printf("cha: Adapter contactor closed\r\n");

    // close our contactor too. it is not opened until power off
    DigIo::contactor_out.Set();
    // DigIo::external_led_out.Clear(); led on. pointless?
};

