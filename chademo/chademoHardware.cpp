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
    println("[cha] set switch (d2) -> %d", set);

    if (set)
        DigIo::switch_d2_out_inverted.Clear(); // 0: on
    else
        DigIo::switch_d2_out_inverted.Set(); // 1: off
};

void ChademoCharger::SetSwitchD1(bool set)
{
    println("[cha] set switch (d1) -> %d", set);

    if (set)
        DigIo::switch_d1_out.Set();
    else
        DigIo::switch_d1_out.Clear();

    _switch_d1 = set;
};

bool ChademoCharger::GetSwitchK()
{
    return not DigIo::switch_k_in_inverted.Get(); // inverted...0 = on
};


/// <summary>
/// We are just mirroring the car contactors, so the adapter contactor is pointless? It may seem so...
/// But its seems hard or impossible to make the car close its contactors after D2=true otherwise.
/// Having 0 volt on the wire seems to always work (regardless of what volt you tell it over CAN).
/// Having a voltage on the wire, the car complains, go into turtle mode, and require clearing of DTC!
/// It may be possible it can work without ensuring 0 volt using a contactor, but not sure how.
/// 
/// Looking at can-logs it seem car often close contactor (CAR_STATUS_CONTACTOR_OPEN_OR_WELDING_DETECTION_DONE = false)
/// at high voltages, BUT chargers may use similar tricks...and actually presenting 0 volt on the wire when they set D2=true,
/// even thou CAN says otherwise... We can not know since we do not have D2 in the can-log, nor the real voltage.
/// </summary>
void ChademoCharger::CloseAdapterContactor()
{
    println("[cha] Adapter contactor closing");

    DigIo::contactor_out.Set();
};

void ChademoCharger::OpenAdapterContactor()
{
    println("[cha] Adapter contactor opening");

    DigIo::contactor_out.Clear();
};
