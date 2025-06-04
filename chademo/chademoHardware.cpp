#include <stdint.h>
#include "chademoCharger.h"
#include <libopencm3/stm32/can.h>
#include "params.h"
#include "digio.h"
#include "hwinit.h"
#include "main.h"

extern volatile uint32_t system_millis;

extern global_data _global;

void ChademoCharger::SetSwitchD2(bool set)
{
    printf("[cha] set switch (d2) %d\r\n", set);

    if (set)
        DigIo::switch_d2_out_inverted.Clear(); // 0: on
    else
        DigIo::switch_d2_out_inverted.Set(); // 1: off
};

void ChademoCharger::SetSwitchD1(bool set)
{
    printf("[cha] set switch (d1) %d\r\n", set);

    if (set)
        DigIo::switch_d1_out.Set();
    else
        DigIo::switch_d1_out.Clear();

    _switch_d1 = set;
};

//bool ChademoCharger::IsCurrentDemandStarted()
//{
//    return _global.ccsCurrentDemandStartedEvent;
//};

//bool ChademoCharger::IsPreChargeStarted()
//{
//    //return _global.ccsPowerRelayOnTrigger_prechargeDone;
//    return _global.ccsPreChargeStartedEvent;
//};

bool ChademoCharger::GetSwitchK()
{
    return DigIo::switch_k_in_inverted.Get() == false; // inverted...0 = on
};

/// <summary>
/// We are just mirroring the car contactors, so the adapter contactor pointless,
/// unless the software allows ccs to go live before adapter is connected to the car:-)
/// And yes, yhe manual says to connect ccs first and then power on, and after ccs is live, connect to car.
/// In this case, the adapter contactor has meaning. But this is not the sensible way to use the adapter,
/// but it is probably how they developed it, without plugged into a car. So it made sense to them,
/// not having a liv adapter in their lap. And the original software still works like this, it make ccs live
/// regardless of chademo state.
/// This software require car/chademo before ccs starts, so then adapter contactor is a NOOP.
/// 
/// OTOH: if any arching _should_ happen during close, it will all be on the adapters contactor, and this is good.
/// BUT to get the same protection when opening, the adapter contactor should open FIRST. And it currently does not, not in original FW nor here.
/// Thou...its hard to open them BEFORE the car, logically we would mess with the cars logic and the welding check.
/// </summary>
void ChademoCharger::CloseAdapterContactor()
{
    printf("[cha] Adapter contactor closing\r\n");

    // close our contactor too. it is not opened until power off
    DigIo::contactor_out.Set();
    // DigIo::external_led_out.Clear(); led on. pointless?
};

void ChademoCharger::OpenAdapterContactor()
{
    printf("[cha] Adapter contactor opening\r\n");

    // close our contactor too. it is not opened until power off (it is now)
    DigIo::contactor_out.Clear();
    // DigIo::external_led_out.Clear(); led on. pointless?
};
