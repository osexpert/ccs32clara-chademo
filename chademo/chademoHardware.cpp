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

bool ChademoCharger::IsCurrentDemandStarted()
{
    return _global.ccsCurrentDemandStartedEvent;
};

//bool ChademoCharger::IsPreChargeDone()
//{
//    return _global.ccsPowerRelayOnTrigger_prechargeDone;
//};


bool ChademoCharger::GetSwitchK()
{
    return DigIo::switch_k_in_inverted.Get() == false; // inverted...0 = on
};

bool ChademoCharger::IsChargingStoppedByAdapter()
{
    return _global.stopButtonEvent;
};

/// <summary>
/// We are just mirroring the car contactors, so the adapter contactor is pointless?
/// Yes...it may seem so...
/// BUT it seems it is needed, to adapt the speeds of the 2 protocols.
/// ccs2 is very slow, chademo very fast. So if we were not able to isolate us from the potentionally very slow rising voltage during
/// precharge, it could perhaps be tricky to fool the car to close the contactors? Not sure.
/// At least with this contactor, we have ability to simulate/present 0 voltage to the car while ccs is busy precharging and we are busy simulating insulation test.
/// When all done and car close contactors, thinking it is 0 volt, adapter contactor is closed and hey, here is battery voltage for you, both in can-msg and inlet.
/// </summary>
void ChademoCharger::CloseAdapterContactor()
{
    printf("[cha] Adapter contactor closing\r\n");

    DigIo::contactor_out.Set();
};

void ChademoCharger::OpenAdapterContactors()
{
    printf("[cha] Adapter contactor opening\r\n");

    DigIo::contactor_out.Clear();
};
