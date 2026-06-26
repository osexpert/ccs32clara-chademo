
#include <string.h>
#include <stdint.h>
#include "chademoCharger.h"
#include "ccs_params.h"
#include "main.h"
#include "pevStateMachine.h"

#include <libopencm3/stm32/can.h>

extern ChademoCharger* chademoCharger;

#define low_byte(x)  ((uint8_t)(x))
#define high_byte(x) ((uint8_t)((x) >> 8))
#define max_byte(x) ((x) > 0xFF ? 0xFF : (x))

#define SWAP16(x) (uint16_t)((((x) & 0x00FF) << 8) | (((x) & 0xFF00) >> 8))

#define COMPARE_SET(oldval, newval, fmt) \
    do { \
        if ((oldval) != (newval)) { \
            println(fmt, (oldval), (newval)); \
            (oldval) = (newval); \
        } \
    } while (0)

#define COMPARE_SET_X2(oldval, newval, fmt) \
    do { \
        if ((oldval) != (newval)) { \
            println(fmt, (oldval), (oldval), (newval), (newval)); \
            (oldval) = (newval); \
        } \
    } while (0)

#define LAST_REQUEST_CURRENT_TIMEOUT_CYCLES (CHA_CYCLES_PER_SEC * 1) // 1 second

/// <summary>
/// get estimated battery volt from target and soc.
/// make a lot of assumtions:-)
/// maxVolt = target - 10
/// Based on Leaf data (soc from dash, volts from leafSpy).
/// Previous logic used soc and volts from leafSpy, but soc's in dash are completely different from those in leafSpy, and obviosly dash soc are sent in chademo.
/// </summary>
static float GetEstimatedBatteryVoltage(float target, float soc, float nomVolt = 0, float maxVolt = 0, float adjustBelowSoc = 0, float adjustBelowFactor = 0.0f)
{
    if (maxVolt == 0)
    {
        maxVolt = target - 10;
    }

    if (nomVolt == 0)
    {
        // Based on 2 extreme data points: iMiev 370 -> 330 and BMW i5 M60: 450 -> 400
        // For 410 -> 365 (leaf 20-30kwh is 380, leaf 40+ is 355, so 365 is not a bad estimate)
        // PS: For known targets, we set nomVoltOverride, so this will only be used for unknown targets
        nomVolt = 0.875f * target + 6.25f;
    }

    float minVolt = nomVolt - (maxVolt - nomVolt); // symetric

    float deltaLow = 0.35f * (nomVolt - minVolt); // delta 20-50
    float volt20 = nomVolt - deltaLow;

    float deltaHigh = 0.55f * (maxVolt - nomVolt); // delta 50-80
    float volt80 = nomVolt + deltaHigh;

    if (adjustBelowSoc != 0 && soc < adjustBelowSoc)
    {
        float volt0 = minVolt - adjustBelowFactor * (volt20 - minVolt);
        return volt0 + (soc / adjustBelowSoc) * (volt20 - volt0);
    }
    else if (soc < 50.0f)
    {
        return volt20 + ((soc - 20.0f) / 30.0f) * (nomVolt - volt20);
    }
    else if (soc < 80.0f)
    {
        return nomVolt + ((soc - 50.0f) / 30.0f) * (volt80 - nomVolt);
    }
    else
    {
        return volt80 + ((soc - 80.0f) / 20.0f) * (maxVolt - volt80);
    }
}

void ChademoCharger::HandlePendingCarMessages()
{
    static msg100 _msg100 = {};
    static msg101 _msg101 = {};
    static msg102 _msg102 = {};
    static msg110 _msg110 = {};
    static msg200 _msg200 = {};
    static msg201 _msg201 = {};

    if (_msg100_pending)
    {
        _msg100_pending = false;

        COMPARE_SET(_msg100.m.MinCurrent, _msg100_isr.m.MinCurrent, "100.MinCurrent %d -> %d");
        COMPARE_SET(_msg100.m.MaxCurrent, _msg100_isr.m.MaxCurrent, "100.MaxCurrent %d -> %d");

        COMPARE_SET(_msg100.m.MinVoltage, _msg100_isr.m.MinVoltage, "100.MinVoltage %d -> %d");
        COMPARE_SET(_msg100.m.MaxVoltage, _msg100_isr.m.MaxVoltage, "100.MaxVoltage %d -> %d");

        COMPARE_SET(_msg100.m.SocPercentConstant, _msg100_isr.m.SocPercentConstant, "100.SocPercentConstant %d -> %d");
        
        COMPARE_SET(_msg100.m.Unused7, _msg100_isr.m.Unused7, "100.Unused7 %d -> %d");

        _carData.MinVoltage = _msg100.m.MinVoltage;
        _carData.MaxVoltage = _msg100.m.MaxVoltage;

        _carData.MinCurrent = _msg100.m.MinCurrent;
        _carData.MaxCurrent = _msg100.m.MaxCurrent;
    }
    if (_msg101_pending)
    {
        _msg101_pending = false;

        COMPARE_SET(_msg101.m.MaxChargingTime10s, _msg101_isr.m.MaxChargingTime10s, "101.MaxChargingTime10s %d -> %d");
        COMPARE_SET(_msg101.m.MaxChargingTimeMinutes, _msg101_isr.m.MaxChargingTimeMinutes, "101.MaxChargingTimeMinutes %d -> %d");
        COMPARE_SET(_msg101.m.EstimatedChargingTimeMinutes, _msg101_isr.m.EstimatedChargingTimeMinutes, "101.EstimatedChargingTimeMins %d -> %d");
        COMPARE_SET(_msg101.m.BatteryCapacity, _msg101_isr.m.BatteryCapacity, "101.BatteryCapacity %d -> %d");

        COMPARE_SET(_msg101.m.Unused0, _msg101_isr.m.Unused0, "101.Unused0 %d -> %d");
        COMPARE_SET(_msg101.m.Unused4, _msg101_isr.m.Unused4, "101.Unused4 %d -> %d");
        COMPARE_SET(_msg101.m.Unused7, _msg101_isr.m.Unused7, "101.Unused7 %d -> %d");

        _carData.EstimatedChargingTimeMins = _msg101.m.EstimatedChargingTimeMinutes;

        if (_msg101.m.MaxChargingTime10s == 0xff)
            _carData.MaxChargingTimeSec = _msg101.m.MaxChargingTimeMinutes * 60;
        else
            _carData.MaxChargingTimeSec = _msg101.m.MaxChargingTime10s * 10;

        // Unstable before switch(k).
        _carData.BatteryCapacityKwh = _msg101.m.BatteryCapacity * 0.1f;
    }
    if (_msg102_pending)
    {
        _msg102_pending = false;

        COMPARE_SET(_msg102.m.ProtocolNumber, _msg102_isr.m.ProtocolNumber, "102.ProtocolNumber %d -> %d");
        COMPARE_SET(_msg102.m.TargetVoltage, _msg102_isr.m.TargetVoltage, "102.TargetVoltage %d -> %d");
        COMPARE_SET(_msg102.m.RequestCurrent, _msg102_isr.m.RequestCurrent, "102.RequestCurrent %d -> %d");
        COMPARE_SET(_msg102.m.Faults, _msg102_isr.m.Faults, "102.Faults 0x%02x -> 0x%02x");
        COMPARE_SET_X2(_msg102.m.Status, _msg102_isr.m.Status, "102.Status %08b (0x%02x) -> %08b (0x%02x)");
        COMPARE_SET(_msg102.m.SocPercent, _msg102_isr.m.SocPercent, "102.SocPercent %d -> %d");
        COMPARE_SET(_msg102.m.Unused7, _msg102_isr.m.Unused7, "102.Unused7 %d -> %d");

        _carData.CyclesSinceCarLastRequestCurrent = 0; // for timeout
        _carData.Faults = (CarFaults)_msg102.m.Faults;
        _carData.Status = (CarStatus)_msg102.m.Status;
        _carData.ProtocolNumber = _msg102.m.ProtocolNumber;

        if (_state == ChargerState::ChargingLoop && _msg102.m.TargetVoltage > _chargerData.AvailableOutputVoltage)
        {
            println("[cha] Car asking (%d) for more than max (%d) volts. Stopping.", _msg102.m.TargetVoltage, _chargerData.AvailableOutputVoltage);
            set_flag(&_chargerData.Status, ChargerStatus::CHARGING_SYSTEM_ERROR); // let error handler deal with it
        }

        // XPeng update TargetVoltage after closing its contactors, (it seems) from real target (battery max at 4.2v) to the same as MaxVolt.
        // Why? I guess...they think it will give them faster charging? In any case, ignore changes to TargetVoltage after d2 is set
		// TargetVoltage should normally never change, so alternative could be to snapshot it as soon as switch(k) is set. But this seemed easier.
        if (not _d2)
        {
            _carData.TargetVoltage = _msg102.m.TargetVoltage;
        }

        // We will limit this later anyways. But it can happen in SX mode, where the ccs data is suddenly available, and they are lower than adapter max.
        _carData.RequestCurrent = _msg102.m.RequestCurrent;

        // soc and the constant both unstable before switch(k)
        if (_carData.Switch_k && has_flag(_carData.Status, CarStatus::READY_TO_CHARGE))
        {
            if (_msg100.m.SocPercentConstant > 0 && _msg100.m.SocPercentConstant != 100)
                _carData.SocPercent = ((float)_msg102.m.SocPercent / _msg100.m.SocPercentConstant) * 100;
            else
                _carData.SocPercent = _msg102.m.SocPercent;

            if (_carData.SocPercent > 100)
            {
                // TODO: if this happens...maybe failing would be a better way to handle it...
                println("[cha] Car report soc %d > 100. Failover to 100.", _carData.SocPercent);
                _carData.SocPercent = 100;
            }

            // now that TargetVoltage is stable, set overrides, if any
            SetBatteryVoltOverridesOnce();

            _carData.EstimatedBatteryVoltage = GetEstimatedBatteryVoltage(_carData.TargetVoltage, 
                _carData.SocPercent, 
                _carData.NomVoltOverride, 
                _carData.MaxVoltOverride,
                _carData.AdjustBelowSoc, 
                _carData.AdjustBelowFactor);

            _carData.EstimatedBatteryVoltageSet = true;
        }

        _msg102_recieved = true;
    }
    if (_msg110_pending)
    {
        _msg110_pending = false;

        COMPARE_SET(_msg110.m.ExtendedFunction1, _msg110_isr.m.ExtendedFunction1, "110.ExtendedFunction1 0x%02x -> 0x%02x");

        _carData.ExtendedFunction1 = (ExtendedFunction1Flags)_msg110.m.ExtendedFunction1;
    }
    if (_msg200_pending)
    {
        _msg200_pending = false;

        COMPARE_SET(_msg200.m.MaxDischargeCurrentInverted, _msg200_isr.m.MaxDischargeCurrentInverted, "200.MaxDischargeCurrentInverted %d -> %d");
        COMPARE_SET(_msg200.m.Unused1, _msg200_isr.m.Unused1, "200.Unused1 %d -> %d");
        COMPARE_SET(_msg200.m.Unused2, _msg200_isr.m.Unused2, "200.Unused2 %d -> %d");
        COMPARE_SET(_msg200.m.Unused3, _msg200_isr.m.Unused3, "200.Unused3 %d -> %d");
        COMPARE_SET(_msg200.m.MinDischargeVoltage, _msg200_isr.m.MinDischargeVoltage, "200.MinDischargeVoltage %d -> %d");
        COMPARE_SET(_msg200.m.MinBatteryDischargeLevel, _msg200_isr.m.MinBatteryDischargeLevel, "200.MinBatteryDischargeLevel %d -> %d");
        COMPARE_SET(_msg200.m.MaxRemainingCapacityForCharging, _msg200_isr.m.MaxRemainingCapacityForCharging, "200.MaxRemainingCapacityForCharging %d -> %d");

        _carData.MaxDischargeCurrent = 0xff - _msg200.m.MaxDischargeCurrentInverted;
        _carData.MaxDischargeCurrentSet = true;
    }
    if (_msg201_pending)
    {
        _msg201_pending = false;

        COMPARE_SET(_msg201.m.ProtocolNumber, _msg201_isr.m.ProtocolNumber, "201.ProtocolNumber %d -> %d");
        COMPARE_SET(_msg201.m.ApproxDischargeCompletionTime, _msg201_isr.m.ApproxDischargeCompletionTime, "201.ApproxDischargeCompletionTime %d -> %d");
        COMPARE_SET(_msg201.m.AvailableVehicleEnergy, _msg201_isr.m.AvailableVehicleEnergy, "201.AvailableVehicleEnergy %d -> %d");
        COMPARE_SET(_msg201.m.Unused5, _msg201_isr.m.Unused5, "201.Unused5 %d -> %d");
        COMPARE_SET(_msg201.m.Unused6, _msg201_isr.m.Unused6, "201.Unused6 %d -> %d");
        COMPARE_SET(_msg201.m.Unused7, _msg201_isr.m.Unused7, "201.Unused7 %d -> %d");
    }
}

void ChademoCharger::Run()
{
    // HandlePendingMessages uses _switch_k
    COMPARE_SET(_carData.Switch_k, GetSwitchK(), "[cha] switch(k) %d -> %d");

    SetChargerDataFromCcsParams();
    HandlePendingCarMessages();
    RunStateMachine();
    SetCcsParamsFromCarData();
    SendChargerMessages();

    Log();
}

void ChademoCharger::SetCcsParamsFromCarData()
{
    // target +1 to silence warning in pev_sendCurrentDemandReq
    // TODO: use _carData.MaxVoltage? But what is the point? We always just ask for target voltage anyways...
    _ccs_params.MaxVoltage = _carData.TargetVoltage + 1;
    _ccs_params.soc = _carData.SocPercent;
    _ccs_params.BatteryVoltage = _carData.EstimatedBatteryVoltage;

    // Only ask ccs for amps in the charging loop, regardless of what the car says (hide that eg. iMiev is always asking for min 1A regardless)
    _ccs_params.TargetCurrent = (_state == ChargerState::ChargingLoop ? _carData.RequestCurrent : 0);

    if (_ccs_params.TargetCurrent > _ccs_params.EvseDynCurrent())
    {
        // If car does not support DynamicControl,
        // car may ask for more (MaxAvailableOutputCurrent) than is currently available (DynAvailableOutputCurrent),
        // and the charger may not like this. Asking for more than available is rude in any case, so cap it if is happens.
        _ccs_params.TargetCurrent = _ccs_params.EvseDynCurrent();
    }

    _ccs_params.TargetVoltage = _carData.TargetVoltage;
}

bool ChademoCharger::IsTimeoutSec(uint16_t sec)
{
    if (_cyclesInState > (sec * CHA_CYCLES_PER_SEC))
    {
        println("[cha] Timeout in %s (max:%dsec)", GetStateName(), sec);
        return true;
    }
    return false;
}

// same as IsTimeoutSec, but without the logging
bool ChademoCharger::HasElapsedSec(uint16_t sec)
{
    return (_cyclesInState > (sec * CHA_CYCLES_PER_SEC));
}

void ChademoCharger::SetBatteryVoltOverridesOnce()
{
    if (_carData.OverridesJudged)
        return;

    bool override = false;

    if (_carData.TargetVoltage == 410)
    {
        if (_global.alternative_voltage == 1) // Leaf 20-30
        {
            _carData.NomVoltOverride = 380;
            _carData.AdjustBelowSoc = 29;
            _carData.AdjustBelowFactor = 0.7f;
            override = true;
        }
        else // Leaf 40+ (default)
        {
            _carData.NomVoltOverride = 355; 
            override = true;
        }
    }
    else if (_carData.TargetVoltage == 336) // Peugeot iOn
    {
        _carData.NomVoltOverride = 310;
        _carData.MaxVoltOverride = 336;
    }

    if (override)
        println("[cha] AV%d: override target %dv => nomVolt:%dv maxVolt:%dv adjustBelowSoc:%d adjustBelowFactor:%f (0=default)", 
            _global.alternative_voltage, 
            _carData.TargetVoltage, 
            _carData.NomVoltOverride,
            _carData.MaxVoltOverride,
            _carData.AdjustBelowSoc,
            &_carData.AdjustBelowFactor // bypass float to double promotion by passing as reference
        );

    _carData.OverridesJudged = true;
}

// car seems to allows 20V deviation. Adding +- 20V in addition should allow 40V deviation. +-30V also worked, but if +-20V works, lets keep +-30 as backup:-)
static const int offsets[] = { 0, 20, 0, -20 };

int ChademoCharger::GetCyclicOffset()
{
    int result = offsets[_offset_index];
    _offset_index = (_offset_index + 1) % (sizeof(offsets) / sizeof(offsets[0]));
    return result;
}

void ChademoCharger::RunStateMachine()
{
    _cyclesInState++;

    if (_state < ChargerState::ChargingLoop)
    {
        StopReason stopReason = StopReason::NONE;
        // global reason
        if (_global.powerOffPending) set_flag(&stopReason, StopReason::POWER_OFF_PENDING);
        // car reason
        // STOP_BEFORE_CHARGING seems unreliable. At least some E-NV200 24kWh 2015 set this flag for no apperant reason, and still seem to work...
        //if (has_flag(_carData.Status, CarStatus::STOP_BEFORE_CHARGING)) set_flag(&stopReason, StopReason::CAR_STOP_BEFORE_CHARGING);
        if (has_flag(_carData.Status, CarStatus::ERROR)) set_flag(&stopReason, StopReason::CAR_ERROR);
        // charger reason
        if (has_flag(_chargerData.Status, ChargerStatus::CHARGER_ERROR)) set_flag(&stopReason, StopReason::CHARGER_ERROR);
        if (has_flag(_chargerData.Status, ChargerStatus::CHARGING_SYSTEM_ERROR)) set_flag(&stopReason, StopReason::CHARGING_SYSTEM_ERROR);
        if (has_flag(_chargerData.Status, ChargerStatus::BATTERY_INCOMPATIBLE)) set_flag(&stopReason, StopReason::BATTERY_INCOMPATIBLE);

        if (stopReason != StopReason::NONE)
        {
            println("[cha] Stopping before starting");
            SetState(ChargerState::Stopping_Start, stopReason);
        }

        _chargerData.ThresholdVoltage = min(_chargerData.AvailableOutputVoltage, _carData.MaxVoltage);

        _chargerData.RemainingChargeTimeSec = _carData.MaxChargingTimeSec;
        _chargerData.RemainingChargeTimeCycles = _carData.MaxChargingTimeSec * CHA_CYCLES_PER_SEC;
    }

    if (_state == ChargerState::WaitForChademoKickoff)
    {
        if (_global.CHADEMO_SX)
        {
            if (_global.ccsLifesign)
            {
                println("[cha] ccsLifesign -> start Chademo");
                SetState(ChargerState::Start);
            }
        }
        else // DX
        {
            if (chademoInterface_ccsInStateWaitForPreChargeStart())
            {
                println("[cha] ccs CableCheck done -> start Chademo");
                SetState(ChargerState::Start);
            }
        }
    }
    else if (_state == ChargerState::Start)
    {
        // reset (set during discovery)
        _msg102_recieved = false;

        SetSwitchD1(true);
        _send_can = true;  // will trigger car sending CAN

        SetState(ChargerState::WaitForCarReadyToCharge);
    }
    else if (_state == ChargerState::WaitForCarReadyToCharge)
    {
        /* may be updated until switch(k) is on:
        100.4+5 max volt
        100.6 soc divider (so effectively soc is also unstable before switch(k))
        101.1 max charge time 10s
        101.2 max charge time min
        101.5+6 battery capacity
        102.1+2 target volt
        */

        // will take a few seconds (approx 3) from D1 until car READY_TO_CHARGE
        // will take approx/max one second after _switch_k until car READY_TO_CHARGE
        if (_carData.Switch_k && has_flag(_carData.Status, CarStatus::READY_TO_CHARGE))
        {
            if (_discovery)
            {
                SetSwitchD1(false);

                _send_can = false;  // PS: even if we stop sending, car may continue to send 102 for a short time
                _discovery = false;
                println("[cha] Discovery completed => ccs kickoff");
                _global.ccsKickoff = true;

                SetState(ChargerState::WaitForChademoKickoff);
            }
            else if (_carData.TargetVoltage > _chargerData.AvailableOutputVoltage)
            {
                // Spec is confusing, but MinBatteryVoltage is ment to be used to judge incompatible battery (but only using target here),
                // and MinBatteryVoltage is unstable before switch(k), so indirectly, incompatible battery can not be judged before switch(k)
                println("[cha] car TargetVoltage %d > charger AvailableOutputVoltage %d (incompatible).", _carData.TargetVoltage, _chargerData.AvailableOutputVoltage);
                set_flag(&_chargerData.Status, ChargerStatus::BATTERY_INCOMPATIBLE); // let error handler deal with it
            }
            else
            {
                LockChargingPlug();
                set_flag(&_chargerData.Status, ChargerStatus::ENERGIZING_OR_PLUG_LOCKED);

                SetState(ChargerState::WaitForPreChargeDone);
            }
        }
        else if (_discovery)
        {
            // for initial discovery, just let the inactivity timer handle it (3 minutes)
        }
        // Not discovery, so we are "live", and ccs is waiting for us.
        // Spec: T-time from D1 set, to switch(k) and READY_TO_CHARGE set is 8 seconds.
        else if (IsTimeoutSec(8))
        {
            SetState(ChargerState::Stopping_Start, StopReason::TIMEOUT);
        }
    }
    else if (_state == ChargerState::WaitForPreChargeDone)
    {
        if (_global.CHADEMO_SX || _preChargeDoneButStalled)
        {
            // d2 = true is telling the car, you can close contactors now, so precharge voltage must be (close to) battery voltage at this point. 
            SetSwitchD2(true);
            println("[cha] Car progressing to ChargingLoop in its own time");

            SetState(ChargerState::WaitForCarContactorsClosed);
        }
        // Spec: C-time 20 seconds or less from Switch(k) && CarStatus::READY_TO_CHARGE to D2 is set. T-time not defined, but 20 seconds should be plenty!
        else if (IsTimeoutSec(20))
        {
            SetState(ChargerState::Stopping_Start, StopReason::TIMEOUT);
        }
    }
    else if (_state == ChargerState::WaitForCarContactorsClosed)
    {
        if ((_carData.ProtocolNumber >= ProtocolNumber::Chademo_1_0 && not has_flag(_carData.Status, CarStatus::CONTACTOR_OPEN_OR_WELDING_DETECTION_DONE)) // Typ: 1-2 seconds after D2, Spec: max 4 sec.
            // chademo 0.9 (and earlier) did not have the flag, wait 2 seconds (spec: compliance time 2 seconds). TODO: AI suggest maybe to use 3 seconds, for slower cars.
            || (_carData.ProtocolNumber < ProtocolNumber::Chademo_1_0 && HasElapsedSec(2))
            )
        {
            // Car seems to demand 0 volt at the inlet when D2=true, else it wont close contactors.
            // After car closes contactors, and it senses high voltage at the inlet (its own battery voltage), it will start to ask for amps.
            // Eg. i-Miev will never ask for amps, so guessing contactors are never closed (12V supply insuficient?) so it never senses its own high voltage. Doing CloseAdapterContactor anyways, so car will sense high voltage (thinking it is its own?) and ask for amps, does not help, and it make the situation look better than it is.

            println("[cha] Car contactors closed");
            _carData.CarContactorsClosed = true;
            _reportOutputVoltage = true; // let car "see"  the charger voltage

            if (_global.CHADEMO_SX)
            {
                // not yet
            }
            else
            {
                CloseAdapterContactor();
            }

            SetState(ChargerState::WaitForCarRequestCurrent);
        }
        // Spec: max 4 seconds from set D2 to CONTACTOR_OPEN_OR_WELDING_DETECTION_DONE cleared
        else if (IsTimeoutSec(4))
        {
            SetState(ChargerState::Stopping_Start, StopReason::TIMEOUT);
        }
    }
    else if (_state == ChargerState::WaitForCarRequestCurrent)
    {
        if (_carData.RequestCurrent > 0) // typ: 1-2 sec after 102.5.3
        {
            // Even thou charger not delivering amps yet, we set these flags (seen in canlogs)
            // Spec: set these flags <= 0.5sec after RequestCurrent > 0
            set_flag(&_chargerData.Status, ChargerStatus::CHARGING);
            clear_flag(&_chargerData.Status, ChargerStatus::STOPPED);

            SetState(ChargerState::ChargingLoop);
        }
        // Spec: max 4 seconds from CONTACTOR_OPEN_OR_WELDING_DETECTION_DONE cleared to RequestCurrent > 0
        else if (IsTimeoutSec(4))
        {
            SetState(ChargerState::Stopping_Start, StopReason::TIMEOUT);
        }
    }
    else if (_state == ChargerState::ChargingLoop)
    {
        _global.auto_power_off_timer_count_up_ms = 0;

        if (_chargerData.OutputCurrent > 0)
        {
            if (_chargerData.RemainingChargeTimeCycles > 0)
                _chargerData.RemainingChargeTimeCycles--;

            // If discharge enabled, make sure there is 5 sec. left, so we can go on forever.
            // This time can be seen inside the car? So count it down as long as it does not interfere with our plans (the plan is to stop countdown at 5 seconds).
            if (_dischargeEnabled && _chargerData.RemainingChargeTimeCycles < (CHA_CYCLES_PER_SEC * 5))
                _chargerData.RemainingChargeTimeCycles = (CHA_CYCLES_PER_SEC * 5);

            _chargerData.RemainingChargeTimeSec = _chargerData.RemainingChargeTimeCycles / CHA_CYCLES_PER_SEC;
        }

        StopReason stopReason = StopReason::NONE;
        // global reason
        if (_global.powerOffPending) set_flag(&stopReason, StopReason::POWER_OFF_PENDING);
        // car reasons
        if (_carData.CyclesSinceCarLastRequestCurrent++ > LAST_REQUEST_CURRENT_TIMEOUT_CYCLES) set_flag(&stopReason, StopReason::CAR_REQUEST_CURRENT_TIMEOUT);
        if (not _carData.Switch_k) set_flag(&stopReason, StopReason::CAR_SWITCH_K_OFF);
        if (not has_flag(_carData.Status, CarStatus::READY_TO_CHARGE)) set_flag(&stopReason, StopReason::CAR_NOT_READY_TO_CHARGE);
        if (has_flag(_carData.Status, CarStatus::NOT_IN_PARK)) set_flag(&stopReason, StopReason::CAR_NOT_IN_PARK);
        if (has_flag(_carData.Status, CarStatus::ERROR)) set_flag(&stopReason, StopReason::CAR_ERROR);
        // charger reasons
        if (has_flag(_chargerData.Status, ChargerStatus::CHARGING_SYSTEM_ERROR)) set_flag(&stopReason, StopReason::CHARGING_SYSTEM_ERROR);
        if (has_flag(_chargerData.Status, ChargerStatus::CHARGER_ERROR)) set_flag(&stopReason, StopReason::CHARGER_ERROR);
        if (_chargerData.RemainingChargeTimeSec == 0) set_flag(&stopReason, StopReason::CHARGING_TIME);

        if (stopReason != StopReason::NONE)
        {
            SetState(ChargerState::Stopping_Start, stopReason);
        }
        else
        {
            const int RAMP_AMPS_PER_STEP = 5; // Increase by 5A every 100ms (50A per second)

            if (chademoInterface_ccsChargingVoltageMirrorsTarget())
            {
                // All(?) portable dischargers mirror TargetVoltage->OutputVoltage. Chademo does not like this and will give deviating volts error.
                _chargerData.OutputVoltage = _carData.EstimatedBatteryVoltage; // else OutputVoltage would be Target, but this would only work on max soc.
                _chargerData.OutputVoltageIsEstimated = true;
            }

            if (_chargerData.OutputVoltageIsEstimated && _estimatedOutputVoltageModulation) {
                _chargerData.OutputVoltage += GetCyclicOffset();
            }

            // We do not have any timeout here currently. But possibly it is not needed since we have all the stop reasons?
            if (_global.CHADEMO_SX && _sxState != SX_DONE)
            {
                _chargerData.ChaAvailableOutputCurrent = 10; // temporarely limit to 10
                int carRequested = _carData.RequestCurrent;
                _carData.RequestCurrent = 1; // fake 1A request for ccs

                if (_sxState == SX_INITIAL)
                {
                    _sxState = SX_WAIT_FOR_preChargeDoneButStalled;
                    println("[cha] set sxState:%d", _sxState);
                }

                if (_sxState == SX_WAIT_FOR_preChargeDoneButStalled)
                {
                    if (_preChargeDoneButStalled)
                    {
                        CloseAdapterContactor(); // will trigger complete of ccs precharge
                        _sxState = SX_WAIT_FOR_ccsCurrentDemand;
                        println("[cha] set sxState:%d", _sxState);
                    }
                }
                else if (_sxState == SX_WAIT_FOR_ccsCurrentDemand)
                {
                    if (chademoInterface_ccsCurrentDemandPos() == 0)
                    {
                        _rampedRequestCurrent = _carData.RequestCurrent;
                        _sxState = SX_RAMP_UP_carDataRequestCurrent;
                        println("[cha] set sxState:%d", _sxState);
                    }
                }
                else if (_sxState == SX_RAMP_UP_carDataRequestCurrent)
                {
                    // Ramp up 5A per cycle toward target
                    _rampedRequestCurrent = min(_rampedRequestCurrent + RAMP_AMPS_PER_STEP, carRequested);
                    _carData.RequestCurrent = _rampedRequestCurrent;

                    if (_rampedRequestCurrent >= carRequested)
                    {
                        _rampedRequestCurrent = 0; // done with it -> tidy
                        _sxState = SX_DONE;
                        println("[cha] set sxState:%d", _sxState);
                    }
                }
            }
            else if (_dischargeEnabled)
            {
                bool isDischarging = false;
                if (_chargerData.OutputCurrent == 0)
                {
                    // zero amps for more than 3seconds -> assume discharging (the waiting may be pointless)
                    if (_zeroOutputAmpsCycles < (CHA_CYCLES_PER_SEC * 3)) {
                        _zeroOutputAmpsCycles++;
                    }
                    if (_zeroOutputAmpsCycles >= (CHA_CYCLES_PER_SEC * 3)) {
                        isDischarging = true;
                    }
                }
                else
                {
                    _zeroOutputAmpsCycles = 0;
                }

                if (isDischarging && not _isDischarging){
                    _rampedRequestCurrent = _carData.RequestCurrent; // we are discharging now, but was not before -> init rampedRequestCurrent 
                }
                COMPARE_SET(_isDischarging, isDischarging, "[cha] IsDischarging %d -> %d");

                if (isDischarging)
                {
                    _chargerData.RemainingDischargeTime = 5; // seconds?

                    // My car don't seem to care much about DischargeCurrent. I faked it to the max, but car did not care. So I wonder, what is it good for?
                    _chargerData.DischargeCurrent = min((uint8_t)10, _carData.MaxDischargeCurrent);

                    // V2X unit may use our request amps as a "sign" of how much we (the car) can discharge?
                    // Since we are using Chademo dynamic current control, we will limit asked amps to 10 or so.
                    // So fake a RequestCurrent to max allowed, and see if it makes a difference.
                    // NOTE: it may be scary to ask for too much, if the charger suddenly decide to give us what we ask for?
                    int maxDischargeAmps = _carData.MaxDischargeCurrentSet ?
                        _carData.MaxDischargeCurrent :
                        MAX_DISCHARGE_AMPS_FALLBACK; // no discharge message from car, use fallback

                    _rampedRequestCurrent = min(_rampedRequestCurrent + RAMP_AMPS_PER_STEP, maxDischargeAmps);

                    _carData.RequestCurrent = _rampedRequestCurrent;

                    // HACK: my car seems to time out after 6 minutes, if no current flows?
                    // Try to fake something every minute. Seems to work. Thou I wonder, if high discharge is currently in progress,
                    // maybe the car does not like it (in this case the hack is not needed, but its impossible for us to know!).
                    // I am starting to wonder, if the Chademo discharge messages has any purpose at all...because RemainingDischargeTime does not help nor do DischargeCurrent. It times out regardless.
                    // Dynamic control seems to be what keeps it alive more than 3 seconds in ChargingLoop,
                    // and flow of current (measured by car) or declaring OutputCurrent > 0 seems to be what keeps it alive more than 6 minutes.
                    // So is it possible...that V2X can be implemented, completely without using the V2X messages?
                    _fakeOutputCurrentCycles++;
                    if (not(_fakeOutputCurrentOnce) || _fakeOutputCurrentCycles >= (CHA_CYCLES_PER_SEC * 60))
                    {
                        _chargerData.DischargeCurrent = 0;
                        _chargerData.OutputCurrent = 1;

                        _fakeOutputCurrentCycles = 0;
                        _fakeOutputCurrentOnce = true;
                    }
                }
                else
                {
                    // keep RemainingDischargeTime at 5, discharge indefinitely
                    _chargerData.DischargeCurrent = 0;
                    _fakeOutputCurrentCycles = 0;
                }
            }

            // Is it possible that all the complicated logic above (for discharge support), could be replaced with this simple hack?
            // Because it seems the discharge messages really does not do much. DischargeCurrent do not seem to be validated by the car, RemainingDischargeTime do not keep the car alive etc.
            // It seems it is mainly OutputCurrent > 0 is what keeps the car alive.
            // For DEV_VOLTS, the car does measure voltage itself and too big difference triggers it.
            // But how does DEV_AMPS fit into this? Is this simply a check for RequestCurrent vs OutputCurrent? Or is real current measurement in the car involved?
            if (not _isDischarging && _chargerData.OutputCurrent == 0)
            {
                _chargerData.OutputCurrent = 1;
            }
        }
    }
    else if (_state == ChargerState::Stopping_Start)
    {
        set_flag(&_chargerData.Status, ChargerStatus::STOPPED);

        SetState(ChargerState::Stopping_WaitForLowAmps);
    }
    else if (_state == ChargerState::Stopping_WaitForLowAmps)
    {
        // For charger: C-time: 0.5 seconds from Switch_k cleared to OutputCurrent <= 5. But what is the timeout time?
        // For car: timeout after switch(k) is cleared and waiting for ChargerStatus::CHARGING cleared: 2.5 seconds.
        // For charger, I guess failure to drop amps is not an option, so maybe thats why I can't see any, but lets use 10 sec...
        if (_chargerData.OutputCurrent <= 5 || IsTimeoutSec(10))
        {
            _chargerData.RemainingChargeTimeCycles = 0;
            _chargerData.RemainingChargeTimeSec = 0;

            // TODO: what about discharge and chargerData.OutputVoltage/chargerData.DischargeCurrent?
            // DischargeCurrent will be left at what we set them to last, while OutputVoltage will suddenly rise to Target voltage... Not sure if it matters thou, at this stage.
            // We have no way to know if any discharge is still happening...so just hope it is not...
            _chargerData.DischargeCurrent = 0;
            _chargerData.RemainingDischargeTime = 0;

            // When car sees this flag cleared and OutputCurrent <= 5, car will start welding detection
            clear_flag(&_chargerData.Status, ChargerStatus::CHARGING);

            SetState(ChargerState::Stopping_WaitForSwitchKOff);
        }
    }
    else if (_state == ChargerState::Stopping_WaitForSwitchKOff)
    {
        // Chademo 1.0: car should clear switch_k within 2 seconds after 109.5.5 is set. Timeout: 4 seconds
        // Chademo 2.0 -> no need to wait for switch(k) to be off, before checking OutputCurrent <= 5 and clearing ChargerStatus::CHARGING.
        // Chademo 0.9 does not have CarStatus::CONTACTOR_OPEN_OR_WELDING_DETECTION_DONE, it can make sense to use switch(k) as synchronization point for the 4 second wait, until clearing D2.
        if (not(_carData.Switch_k) || IsTimeoutSec(4))
        {
            SetState(ChargerState::Stopping_WaitForCcsPowerRelayOff);
        }
    }
    else if (_state == ChargerState::Stopping_WaitForCcsPowerRelayOff)
    {
        // Wait for hardwareInterface_setPowerRelayOff() being called, we mirror its state, but we can't wait for too long...
        if (not _ccs_params.PowerRelayOn || IsTimeoutSec(4))
        {
            // Charger should drop volts during the car's welding detection.
            OpenAdapterContactor();
            _reportOutputVoltage = false; // show 0 volt on CAN as well, regardless of chargers real output voltage

            SetState(ChargerState::Stopping_WaitForCarContactorsOpen);
        }
    }
    else if (_state == ChargerState::Stopping_WaitForCarContactorsOpen)
    {
        // C-time <= 4.0s / T-time 10.0s, after Switch(k) cleared.
        // For simplicity, I ignore the "after Switck(k)" part.
        if ((_carData.ProtocolNumber >= ProtocolNumber::Chademo_1_0 && has_flag(_carData.Status, CarStatus::CONTACTOR_OPEN_OR_WELDING_DETECTION_DONE))
            || (_carData.ProtocolNumber < ProtocolNumber::Chademo_1_0 && HasElapsedSec(4))
            || IsTimeoutSec(10))
        {
            // welding detection done & car contactors open
            _carData.CarContactorsClosed = false;
            println("[cha] Car contactors opened");

            SetSwitchD2(false);

            SetState(ChargerState::Stopping_SetSwitchD1Off);
        }
    }
    else if (_state == ChargerState::Stopping_SetSwitchD1Off)
    {
        // spec says, after setting D2:false wait 500ms before setting D1:false
        if (_cyclesInState >= 5)
        {
            SetSwitchD1(false);

            // stop CAN in later state to make sure we send this message to car before we kill CAN
            clear_flag(&_chargerData.Status, ChargerStatus::ENERGIZING_OR_PLUG_LOCKED);

            SetState(ChargerState::Stopping_UnlockChargingPlug);
        }
    }
    else if (_state == ChargerState::Stopping_UnlockChargingPlug)
    {
        // Unlock charging plug in own state, to make sure the car get the CAN message before power off (plug unlocked = power off ok)
    	UnlockChargingPlug();

        SetState(ChargerState::End);
    }
    else if (_state == ChargerState::End)
    {
        // terminal state. We can never leave.

        _send_can = false;
    }

}

bool ChademoCharger::PreChargeCompleted()
{
    _preChargeDoneButStalled = true;

    if (_global.CHADEMO_SX)
    {
        if (not _adapterContactorClosed)
            println("[cha] PreCharge stalled until adapter contactor closed");
        return _adapterContactorClosed;
    }
    else // DX
    {
        // keep it hanging until car contactors closed. The voltage may drop fast after precharge is done, if the charger is "floating", so don't complete precharge to soon.
        if (not _carData.CarContactorsClosed)
            println("[cha] PreCharge stalled until car contactors closed");
        return _carData.CarContactorsClosed;
    }
}

bool chademoInterface_preChargeCompleted()
{
    return chademoCharger->PreChargeCompleted();
}

bool ChademoCharger::AdapterContactorOpened()
{
    return not _adapterContactorClosed;
}

bool chademoInterface_adapterContactorOpened()
{
    return chademoCharger->AdapterContactorOpened();
}

int ChademoCharger::GetChargingLoopPos()
{
    if (chademoCharger->_state < ChargerState::ChargingLoop)
        return -1;
    else if (chademoCharger->_state == ChargerState::ChargingLoop)
        return 0;
    else
        return 1;
}

int chademoInterface_chargingLoopPos()
{
    return chademoCharger->GetChargingLoopPos();
}

void ChademoCharger::SetState(ChargerState newState, StopReason stopReason)
{
    println("[cha] ====>>>> set state %s", _stateNames[newState]);
    _state = newState;
    _cyclesInState = 0;

    set_flag(&_stopReason, stopReason);
    if (stopReason != StopReason::NONE)
        println("[cha] Stopping: 0x%02x", stopReason);
};

const char* ChademoCharger::GetStateName()
{
    return _stateNames[_state];
};

#define MAX_UNDERSUPPLY_AMPS 10

void ChademoCharger::SetChargerData(uint16_t maxV, uint16_t maxA, uint16_t dynA, uint16_t outV, bool outV_is_estimated, uint16_t outA)
{
    _chargerData.AvailableOutputVoltage = maxV;
    if (_chargerData.AvailableOutputVoltage > ADAPTER_MAX_VOLTS)
        _chargerData.AvailableOutputVoltage = ADAPTER_MAX_VOLTS;

    _chargerData.MaxAvailableOutputCurrent = clampToUint8(maxA);
    if (_chargerData.MaxAvailableOutputCurrent > ADAPTER_MAX_AMPS)
        _chargerData.MaxAvailableOutputCurrent = ADAPTER_MAX_AMPS;

    _chargerData.DynAvailableOutputCurrent = clampToUint8(dynA);
    if (_chargerData.DynAvailableOutputCurrent > ADAPTER_MAX_AMPS)
        _chargerData.DynAvailableOutputCurrent = ADAPTER_MAX_AMPS;

    _chargerData.OutputVoltage = outV;
    _chargerData.OutputVoltageIsEstimated = outV_is_estimated;

    _chargerData.OutputCurrent = clampToUint8(outA);

    // If difference between RequestCurrent and OutputCurrent is too large, the car fails. If car support dynamic AvailableOutputCurrent,
    // we adjust ChaAvailableOutputCurrent down, forcing the car to ask for less amps, reducing the difference.
    // Its kind of silly...why did they not provide a flag to turn off the car failing part instead? :-)
    // I don't know exactly what difference is allowed (spec. says 10% or 20A). At least 10A difference seems to work fine. 40A certainly does not:-)
    if (_carData.DynamicControl())
    {
        if (_carData.RequestCurrent > _chargerData.OutputCurrent + MAX_UNDERSUPPLY_AMPS)
            _chargerData.ChaAvailableOutputCurrent = _chargerData.OutputCurrent + MAX_UNDERSUPPLY_AMPS;
        else
            _chargerData.ChaAvailableOutputCurrent = _chargerData.DynAvailableOutputCurrent;
    }
    else
    {
        _chargerData.ChaAvailableOutputCurrent = _chargerData.MaxAvailableOutputCurrent;
    }

    // Assume charger max supply is same as chargers max demand
    _chargerData.MaxDischargeCurrent = _chargerData.MaxAvailableOutputCurrent;
};

void ChademoCharger::SetChargerDataFromCcsParams()
{
    if (_discovery || (_global.CHADEMO_SX && chademoInterface_ccsCurrentDemandPos() < 0))
    {
        SetChargerData(ADAPTER_MAX_VOLTS,
            ADAPTER_MAX_AMPS,
            ADAPTER_MAX_AMPS, // dyn
            _carData.EstimatedBatteryVoltage,
            true, // voltage is estimated
            0 // _carData.RequestCurrent
        );
    }
    else
    {
        // mirror these values
        SetChargerData(
            _ccs_params.EvseMaxVoltage,
            _ccs_params.EvseMaxCurrent,
            _ccs_params.EvseDynCurrent(),
            _ccs_params.EvseVoltage,
            false, // voltage not estimated
            _ccs_params.EvseCurrent
        );
    }
}

void ChademoCharger::Log()
{
    if (_logCycleCounter++ > (CHA_CYCLES_PER_SEC * 1))
    {
        // every second
        println("[cha] state:%s cycles:%d out:%dV/%dA max:%dV/%dA/%dA rem_t:%ds st=0x%02x car: req:%dA est_t:%dm max_t:%ds st:0x%02x err:0x%02x target:%dV max:%dV soc:%d%% batt:%dV cap=%fkWh",
            GetStateName(),
            _cyclesInState,
            _chargerData.OutputVoltage,
            _chargerData.OutputCurrent,
            _chargerData.AvailableOutputVoltage,
            _chargerData.MaxAvailableOutputCurrent,
            _chargerData.DynAvailableOutputCurrent,
            _chargerData.RemainingChargeTimeSec,
            _chargerData.Status,

            _carData.RequestCurrent,
            _carData.EstimatedChargingTimeMins,
            _carData.MaxChargingTimeSec,
            _carData.Status,
            _carData.Faults,
            _carData.TargetVoltage,
            _carData.MaxVoltage,
            _carData.SocPercent,
            _carData.EstimatedBatteryVoltage,
            &_carData.BatteryCapacityKwh // bypass float to double promotion by passing as reference
        );

        _logCycleCounter = 0;
    }
}

void ChademoCharger::HandleCanMessageIsr(uint32_t id, uint32_t data[2])
{
    if (id == 0x100)
    {
        _msg100_isr.pair[0] = data[0];
        _msg100_isr.pair[1] = data[1];
        _msg100_pending = true;
    }
    else if (id == 0x101)
    {
        _msg101_isr.pair[0] = data[0];
        _msg101_isr.pair[1] = data[1];
        _msg101_pending = true;
    }
    else if (id == 0x102)
    {
        _msg102_isr.pair[0] = data[0];
        _msg102_isr.pair[1] = data[1];
        _msg102_pending = true;
    }
    else if (id == 0x110)
    {
        _msg110_isr.pair[0] = data[0];
        _msg110_isr.pair[1] = data[1];
        _msg110_pending = true;
    }
    else if (id == 0x200)
    {
        _msg200_isr.pair[0] = data[0];
        _msg200_isr.pair[1] = data[1];
        _msg200_pending = true;
    }
    else if (id == 0x201)
    {
        _msg201_isr.pair[0] = data[0];
        _msg201_isr.pair[1] = data[1];
        _msg201_pending = true;
    }
}


/* Interrupt service routines */
extern "C" void can1_rx0_isr(void)
{
    uint32_t id;
    bool ext, rtr;
    uint8_t length, fmi;
    uint32_t data[2];

    while (can_receive(CAN1,
        0, // fifo
        true,  // release
        &id,
        &ext,
        &rtr,
        &fmi, // ID of the matched filter
        &length,
        (uint8_t*)data,
        0 // timestamp
    ) > 0 && length == 8) // log if len is <> 8?
    {
        chademoCharger->HandleCanMessageIsr(id, data);
    }
}

// CAN: 100ms +-10ms
#define CAN_TRANSMIT_TIMEOUT_MS 10

/// <summary>
/// can_transmit is not fifo! It can choose avail. mailbox at will and send them in any order.
/// But chademo says 108 must come before 109, order of messages are defined.
/// So...wait until the message is send before q-ing next (can_transmit = add to mailbox q)
/// </summary>
void can_transmit_blocking_fifo(uint32_t canport, uint32_t id, bool ext, bool rtr, uint8_t len, uint8_t* data)
{
    // Check if CAN is initialized and not in bus-off state
    //if (CAN_MSR(canport) & CAN_MSR_INAK) {
    //    println("[can] transmit: peripheral not initialized");
    //    return;// CAN_TX_ERROR;
    //}

    int mailbox = can_transmit(canport, id, ext, rtr, len, data);
    if (mailbox < 0) {
        println("[cha] can transmit: msg:0x%x no mailbox available", id);
        return;// CAN_TX_NO_MAILBOX;
    }

    uint32_t txok_mask = CAN_TSR_TXOK0 << mailbox;
    uint32_t rqcp_mask = CAN_TSR_RQCP0 << mailbox;
//    uint32_t terr_mask = CAN_TSR_TERR0 << mailbox;

    // wait until done (TX done or error) or timeout
    uint32_t start = system_millis;
    while ((CAN_TSR(canport) & rqcp_mask) == 0) {
        if ((system_millis - start) > CAN_TRANSMIT_TIMEOUT_MS) {
            println("[cha] can transmit: msg:0x%x timeout", id);
            break;
        }
    }

    // Now safe to check success or error
    if ((CAN_TSR(canport) & txok_mask) != 0) {
        // success
    }
    else {
        // failure or abort
    }

    // Always clear request complete flag RQCPx
    CAN_TSR(canport) |= rqcp_mask;
}

void ChademoCharger::SendChargerMessages()
{
    if (_send_can && _msg102_recieved)
    {
        UpdateChargerMessages();

        can_transmit_blocking_fifo(CAN1, 0x108, 0x108 > 0x7FF, false, 8, _msg108.bytes);
        can_transmit_blocking_fifo(CAN1, 0x109, 0x109 > 0x7FF, false, 8, _msg109.bytes);

        can_transmit_blocking_fifo(CAN1, 0x118, 0x118 > 0x7FF, false, 8, _msg118.bytes);

        if (_dischargeEnabled && not _discovery)
        {
            // need to send DC messages if charger declare DC, else car fails
            can_transmit_blocking_fifo(CAN1, 0x208, 0x208 > 0x7FF, false, 8, _msg208.bytes);
            can_transmit_blocking_fifo(CAN1, 0x209, 0x209 > 0x7FF, false, 8, _msg209.bytes);
        }
    }
}


void ChademoCharger::UpdateChargerMessages()
{
    COMPARE_SET(_msg108.m.WeldingDetection, _chargerData.SupportWeldingDetection, "108.WeldingDetection %d -> %d");
    COMPARE_SET(_msg108.m.AvailableOutputCurrent, _chargerData.ChaAvailableOutputCurrent, "108.AvailableOutputCurrent %d -> %d");
    COMPARE_SET(_msg108.m.AvailableOutputVoltage, _chargerData.AvailableOutputVoltage, "108.AvailableOutputVoltage %d -> %d");
    COMPARE_SET(_msg108.m.ThresholdVoltage, _chargerData.ThresholdVoltage, "108.ThresholdVoltage %d -> %d");

    COMPARE_SET(_msg109.m.ProtocolNumber, _chargerData.ProtocolNumber, "109.ProtocolNumber %d -> %d");
    COMPARE_SET(_msg109.m.PresentChargingCurrent, _chargerData.OutputCurrent, "109.OutputCurrent %d -> %d");

    // ZE0 seems to hangs the second time, if discharge is enabled during discovery
    COMPARE_SET(_msg109.m.DischargeCompatible, _dischargeEnabled && not _discovery, "109.DischargeCompatible %d -> %d");

    uint16_t outputVolt = _reportOutputVoltage ? _chargerData.OutputVoltage : 0;
    COMPARE_SET(_msg109.m.PresentVoltage, outputVolt, "109.OutputVoltage %d -> %d");

    uint8_t remainingChargingTime10s = 0;
    uint8_t remainingChargingTimeMins = 0;
    if (_chargerData.RemainingChargeTimeSec < 60)
    {
        remainingChargingTime10s = _chargerData.RemainingChargeTimeSec / 10;
        remainingChargingTimeMins = 0;
    }
    else
    {
        remainingChargingTime10s = 0xff;
        remainingChargingTimeMins = _chargerData.RemainingChargeTimeSec / 60;
    }

    COMPARE_SET(_msg109.m.RemainingChargingTime10s, remainingChargingTime10s, "109.RemainingChargingTime10s %d -> %d");
    COMPARE_SET(_msg109.m.RemainingChargingTimeMinutes, remainingChargingTimeMins, "109.RemainingChargingTimeMinutes %d -> %d");
    COMPARE_SET(_msg109.m.Status, _chargerData.Status, "109.Status 0x%02x -> 0x%02x");
    COMPARE_SET(_msg118.m.ExtendedFunction1, _chargerData.ExtendedFunction1, "118.ExtendedFunction1 0x%02x -> 0x%02x");

    if (_dischargeEnabled && not _discovery)
    {
        COMPARE_SET(_msg208.m.MaxDischargeCurrentInverted, 0xff - _chargerData.MaxDischargeCurrent, "208.MaxDischargeCurrentInverted %d -> %d"); // 15
        
        COMPARE_SET(_msg208.m.PresentDischargeCurrentInverted, 0xff - _chargerData.DischargeCurrent, "208.PresentDischargeCurrentInverted %d -> %d"); // 0

        // todo: use _chargerData.AvailableOutputVoltage??
        COMPARE_SET(_msg208.m.MaxDischargeVoltage, 500, "208.MaxDischargeVoltage %d -> %d"); // same
        // 250...seems random...I think this is something inverted, eg. 255 - 250 = 5. And maybe amps instead of volts...
        COMPARE_SET(_msg208.m.MinDischargeVoltage, 250, "208.MinDischargeVoltage %d -> %d");

        COMPARE_SET(_msg209.m.RemainingDischargeTime, _chargerData.RemainingDischargeTime, "209.RemainingDischargeTime %d -> %d");

        COMPARE_SET(_msg209.m.ProtocolNumber, _chargerData.DischargeProtocolNumber, "209.ProtocolNumber %d -> %d");
    }
}

bool ChademoCharger::IsPowerOffOk()
{
    return not _chargingPlugLocked;
}
