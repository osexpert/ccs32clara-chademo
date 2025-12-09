
#include <string.h>
#include <stdint.h>
#include "chademoCharger.h"
#include "params.h"
#include "main.h"
#include "pevStateMachine.h"

#include <libopencm3/stm32/can.h>

extern ChademoCharger* chademoCharger;

#define low_byte(x)  ((uint8_t)(x))
#define high_byte(x) ((uint8_t)((x) >> 8))
#define max_byte(x) ((x) > 0xFF ? 0xFF : (x))

#define SWAP16(x) (uint16_t)((((x) & 0x00FF) << 8) | (((x) & 0xFF00) >> 8))

// only for bytes
//#define set_flag(x, flag)    ((x) = static_cast<decltype(x)>((x) | (flag)))
//#define clear_flag(x, flag)  ((x) = static_cast<decltype(x)>((x) & ~(flag)))
//#define has_flag(x, flag)    (((x) & (flag)) != static_cast<decltype(x)>(0))
//#define has_flags(x, flags) (((x) & (flags)) == static_cast<decltype(x)>(flags))

#define COMPARE_SET(oldval, newval, fmt) \
    do { \
        if ((oldval) != (newval)) { \
            println(fmt, (oldval), (newval)); \
            (oldval) = (newval); \
        } \
    } while (0)

extern volatile uint32_t system_millis;
extern global_data _global;


#define LAST_REQUEST_CURRENT_TIMEOUT_CYCLES (CHA_CYCLES_PER_SEC * 1) // 1 second

/// <summary>
/// get estimated battery volt from target and soc.
/// make a lot of assumtions:-)
/// maxVolt = target - 10
/// Based on Leaf data (soc from dash, volts from leafSpy).
/// Previous logic used soc and volts from leafSpy, but soc's in dash are completely different from those in leafSpy, and obviosly dash soc are sent in chademo.
/// </summary>
static float GetEstimatedBatteryVoltage(float target, float soc, float nomVolt = 0, float adjustBelowSoc = 0, float adjustBelowFactor = 0.0f)
{
    float maxVolt = target - 10;

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
        COMPARE_SET(_msg102.m.Status, _msg102_isr.m.Status, "102.Status 0x%02x -> 0x%02x");
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
        else
        {
            _carData.TargetVoltage = _msg102.m.TargetVoltage;
        }

        if (_state == ChargerState::ChargingLoop && _msg102.m.RequestCurrent > _chargerData.MaxAvailableOutputCurrent)
        {
            println("[cha] Car asking (%d) for more than max (%d) amps. Stopping.", _msg102.m.RequestCurrent, _chargerData.MaxAvailableOutputCurrent);
            set_flag(&_chargerData.Status, ChargerStatus::CHARGING_SYSTEM_ERROR); // let error handler deal with it
        }
        else
        {
            _carData.RequestCurrent = _msg102.m.RequestCurrent;
        }

        // soc and the constant both unstable before switch(k)
        if (_switch_k)
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

            _carData.EstimatedBatteryVoltage = GetEstimatedBatteryVoltage(_carData.TargetVoltage, _carData.SocPercent, _nomVoltOverride, _adjustBelowSoc, _adjustBelowFactor);
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

bool ChademoCharger::IsDiscoveryCompleted()
{
    // if discovery, we go here when done
    return _state == ChargerState::PreStart_DiscoveryCompleted_WaitForCableCheckDone;
}

void ChademoCharger::Run()
{
    // HandlePendingMessages uses _switch_k
    COMPARE_SET(_switch_k, GetSwitchK(), "[cha] switch(k) %d -> %d");

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
    Param::SetInt(Param::MaxVoltage, _carData.TargetVoltage + 1);
    Param::SetInt(Param::soc, _carData.SocPercent);
    Param::SetInt(Param::BatteryVoltage, _carData.EstimatedBatteryVoltage);

    // Only ask ccs for amps in the charging loop, regardless of what the car says (hide that eg. iMiev is always asking for min 1A regardless)
    Param::SetInt(Param::ChargeCurrent, _state == ChargerState::ChargingLoop ? _carData.RequestCurrent : 0);

    Param::SetInt(Param::TargetVoltage, _carData.TargetVoltage);
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

// car seems to allows 20V deviation. Adding +- 20V in addition should allow 40V deviation. +-30V also worked, but if +-20V works, lets keep +-30 as backup:-)
static const int offsets[] = { 20, 0, -20, 0 };
static int offset_index = 0;

static int GetCyclicOffset()
{
    int result = offsets[offset_index];
    offset_index = (offset_index + 1) % (sizeof(offsets) / sizeof(offsets[0]));
    return result;
}

void ChademoCharger::SetBatteryVoltOverrides()
{
    bool known = false;

    // Set for some known targets
    if (_carData.TargetVoltage == 370)
    {
        _nomVoltOverride = 330; // iMiev
        known = true;
    }
    else if (_carData.TargetVoltage == 450)
    {
        _nomVoltOverride = 400; // BMW i5 M60 
        known = true;
    }
    else if (_carData.TargetVoltage == 410)
    {
        if (_global.alternative_function == 1)
        {
            // Leaf 20-30
            _nomVoltOverride = 380;
            _adjustBelowSoc = 29;
            _adjustBelowFactor = 0.7f;
            known = true;
        }
        else // default
        {
            _nomVoltOverride = 355; // Leaf 40+
            known = true;
        }
    }

    if (known)
        println("[cha] AF%d: known target %dv => nomVolt:%dv adjustBelowSoc:%d adjustBelowFactor:%f (0=default)", 
            _global.alternative_function, 
            _carData.TargetVoltage, 
            _nomVoltOverride, 
            _adjustBelowSoc,
            &_adjustBelowFactor // bypass float to double promotion by passing as reference
        );
}

void ChademoCharger::RunStateMachine()
{
    _chargerData.DischargeCurrent = 0; // reset every iteration
    _chargerData.RemainingDischargeTime = 0; // reset every iteration

    _cyclesInState++;

    if (_state < ChargerState::ChargingLoop)
    {
        StopReason stopReason = StopReason::NONE;
        // global reason
        if (_global.powerOffPending) set_flag(&stopReason, StopReason::POWER_OFF_PENDING);
        // car reason
        if (has_flag(_carData.Status, CarStatus::STOP_BEFORE_CHARGING)) set_flag(&stopReason, StopReason::CAR_STOP_BEFORE_CHARGING);
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
        _chargerData.RemainingChargeTimeCycles = _chargerData.RemainingChargeTimeSec * CHA_CYCLES_PER_SEC;
    }

    if (_state == ChargerState::PreStart_DiscoveryCompleted_WaitForCableCheckDone)
    {
#ifdef CHADEMO_STANDALONE_TESTING
        if (true)
#else
        if (chademoInterface_ccsCableCheckDone())
#endif
        {
            SetState(ChargerState::Start);
        }
    }
    else if (_state == ChargerState::Start)
    {
        // reset in case set during discovery
        _msg102_recieved = false;
        _carData.Faults = {};
        _carData.Status = {};
        _chargerData.Status = ChargerStatus::STOPPED;
        _stopReason = StopReason::NONE;

        SetSwitchD1(true); // will trigger car sending CAN

        SetState(ChargerState::WaitForCarSwitchK);
    }
    else if (_state == ChargerState::WaitForCarSwitchK)
    {
        if (_switch_k)
        {
            SetBatteryVoltOverrides();
            // refresh EstimatedBatteryVoltage after (potentionally) setting overrides
            _carData.EstimatedBatteryVoltage = GetEstimatedBatteryVoltage(_carData.TargetVoltage, _carData.SocPercent, _nomVoltOverride, _adjustBelowSoc, _adjustBelowFactor);

            // Spec is confusing, but MinBatteryVoltage is ment to be used to judge incompatible battery (but only using target here),
            // and MinBatteryVoltage is unstable before switch(k), so indirectly, incompatible battery can not be judged before switch(k)
            if (_carData.TargetVoltage > _chargerData.AvailableOutputVoltage)
            {
                println("[cha] car TargetVoltage %d > charger AvailableOutputVoltage %d (incompatible).", _carData.TargetVoltage, _chargerData.AvailableOutputVoltage);
                set_flag(&_chargerData.Status, ChargerStatus::BATTERY_INCOMPATIBLE); // let error handler deal with it
            }
            else if (_discovery)
            {
                SetSwitchD1(false); // PS: even if we set to false, car can continue to send 102 for a short time
                _discovery = false;
                println("[cha] Discovery completed");

                SetState(ChargerState::PreStart_DiscoveryCompleted_WaitForCableCheckDone);
            }
            else
            {
                SetState(ChargerState::WaitForCarReadyToCharge);
            }
        }
        else if (IsTimeoutSec(20))
        {
            SetState(ChargerState::Stopping_Start, StopReason::TIMEOUT);
        }
    }
    else if (_state == ChargerState::WaitForCarReadyToCharge)
    {
        if (has_flag(_carData.Status, CarStatus::READY_TO_CHARGE)) // will take a few seconds (ca. 3) until car is ready
        {
            _idleRequestCurrent = _carData.RequestCurrent; // iMiev ask for 1A from the start

            LockChargingPlug();
            set_flag(&_chargerData.Status, ChargerStatus::ENERGIZING_OR_PLUG_LOCKED);

            //PerformInsulationTest();

            SetState(ChargerState::WaitForPreChargeDone);
        }
        else if (IsTimeoutSec(20))
        {
            SetState(ChargerState::Stopping_Start, StopReason::TIMEOUT);
        }
    }
    else if (_state == ChargerState::WaitForPreChargeDone)
    {
        // TODO: if we start ccs precharge when we enter WaitForPreChargeDone, how long time can we use, after car is ready to charge and before we set d2, without chademo timeout? I think the spec says 20-22sec.
		// If chademo allows for enough time, entering state WaitForPreChargeDone could trigger ccs precharge start (allthou we would loose some seconds where things could be run in paralell).
		// Alternative: add 2 seconds delay between cableCheck done and ccs preCharge start.

        if (_preChargeDoneButStalled)
        {
            // d2 = true is telling the car, you can close contactors now, so precharge voltage must be (close to) battery voltage at this point. 
            SetSwitchD2(true);
            println("[cha] Car progressing to ChargingLoop in its own time");

            SetState(ChargerState::WaitForCarContactorsClosed);
        }
        else if (IsTimeoutSec(30))
        {
            SetState(ChargerState::Stopping_Start, StopReason::TIMEOUT);
        }
    }
    else if (_state == ChargerState::WaitForCarContactorsClosed)
    {
        if ((_carData.ProtocolNumber >= ProtocolNumber::Chademo_1_0 && not has_flag(_carData.Status, CarStatus::CONTACTOR_OPEN_OR_WELDING_DETECTION_DONE)) // Typ: 1-2 seconds after D2, Spec: max 4 sec.
            // chademo 0.9 (and earlier) did not have the flag, use RequestCurrent as trigger
            || (_carData.ProtocolNumber < ProtocolNumber::Chademo_1_0 && _carData.RequestCurrent > _idleRequestCurrent)
            )
        {
            // Car seems to demand 0 volt at the inlet when D2=true, else it wont close contactors.
            // After car closes contactors, and it senses high voltage at the inlet (its own battery voltage), it will start to ask for amps.
            // Eg. i-Miev will never ask for amps, so guessing contactors are never closed (12V supply insuficient?) so it never senses its own high voltage. Doing CloseAdapterContactor anyways, so car will sense high voltage (thinking it is its own?) and ask for amps, does not help, and it make the situation look better than it is.
            CloseAdapterContactor();

            SetState(ChargerState::WaitForCarRequestCurrent);
        }
        else if (IsTimeoutSec(20))
        {
            SetState(ChargerState::Stopping_Start, StopReason::TIMEOUT);
        }
    }
    else if (_state == ChargerState::WaitForCarRequestCurrent)
    {
        if (_carData.RequestCurrent > _idleRequestCurrent) // 1-2 sec after 102.5.3
        {
            // Even thou charger not delivering amps yet, we set these flags (seen in canlogs)
            // Spec: set these flags <= 0.5sec after RequestCurrent > 0
            set_flag(&_chargerData.Status, ChargerStatus::CHARGING);
            clear_flag(&_chargerData.Status, ChargerStatus::STOPPED);

            SetState(ChargerState::ChargingLoop);
        }
        else if (IsTimeoutSec(20))
        {
            SetState(ChargerState::Stopping_Start, StopReason::TIMEOUT);
        }
    }
    else if (_state == ChargerState::ChargingLoop)
    {
        if (_chargerData.RemainingChargeTimeCycles > 0)
            _chargerData.RemainingChargeTimeCycles--;
        _chargerData.RemainingChargeTimeSec = _chargerData.RemainingChargeTimeCycles / CHA_CYCLES_PER_SEC;

        StopReason stopReason = StopReason::NONE;
        // global reason
        if (_global.powerOffPending) set_flag(&stopReason, StopReason::POWER_OFF_PENDING);
        // car reasons
        if (_carData.CyclesSinceCarLastRequestCurrent++ > LAST_REQUEST_CURRENT_TIMEOUT_CYCLES) set_flag(&stopReason, StopReason::CAR_REQUEST_CURRENT_TIMEOUT);
        if (not _switch_k) set_flag(&stopReason, StopReason::CAR_SWITCH_K_OFF);
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
            // Seen in a log that set RemainingDischargeTime=5 right after car ask for amps, and keep it like this for the rest of the session.
            // Not sure if this is good for anything...if it triggers anything...
            // Possibly this is also a way to get more time, without asking for DischargeCurrent. Not tried it.

            bool dischargeUnit = false;
            bool dischargeSimulation = false;
            if (_dischargeEnabled && has_flag(_carData.Status, CarStatus::DISCHARGE_COMPATIBLE))
            {
                // one discharger is observed to mirror target voltage as output voltage. may not apply to all dischargers...
                bool chargerIsDischarger = chademoInterface_ccsChargingVoltageMirrorsTarget();
                if (chargerIsDischarger)
                {
                    // one discharger is observed to mirror asked amps as delivered amps.
                    // Chademo does not like this and will give deviation amps error. Set to 0, to match reality (the discharger is not delivering any amps...)
                    _chargerData.OutputCurrent = 0;
                    _chargerData.DischargeCurrent = min((uint8_t)10, _carData.MaxDischargeCurrent); // use 10 (or less). Supposedly the allowed deviation is 10, so this should allow for discharging 0-20 amps.
                    _chargerData.OutputVoltage += GetCyclicOffset(); // not sure if it has meaning. it is a hack that works for higly deviating batt volt estimate, but it seems we can't charge either...
                    _chargerData.RemainingDischargeTime = 5; // seen in canlog. not sure what it does...possibly MaxDischargeCurrent can sometimes be 0 but we still want to defeat the 4 second timeouit?
                    dischargeUnit = true;
                }
                // 3 seconds passed without amps delivered? We are living dangerously. Try to buy us more time!
                // Consider: what if output current drops to 0 _during_ charging? I guess...we should buy time as usual?
                else if (_cyclesInState > (CHA_CYCLES_PER_SEC * 3) && _chargerData.OutputCurrent == 0)
                {
                    // this will/should put the car into discharge mode, where it no longer care about if amps are delivered
                    // (allthou the car will still ask for plenty, it will be happy with getting none)
                    _chargerData.DischargeCurrent = min((uint8_t)1, _carData.MaxDischargeCurrent);
                    //dont think cycliv voltage make any sense when buying time. now the contactors are already closed and voltages met anyways
//                    _chargerData.OutputVoltage += GetCyclicOffset(); // not sure if it has meaning. it is a hack that works for higly deviating batt volt estimate, but it seems we can't charge either...
                    _chargerData.RemainingDischargeTime = 5; // seen in canlog. not sure what it does...possibly MaxDischargeCurrent can sometimes be 0 but we still want to defeat the 4 second timeouit?
                    dischargeSimulation = true;
                }
            }
            COMPARE_SET(dischargeUnit, _dischargeUnit, "[cha] DischargeUnit %d -> %d");
            COMPARE_SET(dischargeSimulation, _dischargeSimulation, "[cha] DischargeSimulation %d -> %d");
        }
    }
    else if (_state == ChargerState::Stopping_Start)
    {
        set_flag(&_chargerData.Status, ChargerStatus::STOPPED);

        SetState(ChargerState::Stopping_WaitForLowAmps);
    }
    else if (_state == ChargerState::Stopping_WaitForLowAmps)
    {
        if (_chargerData.OutputCurrent <= 5 || IsTimeoutSec(10))
        {
            clear_flag(&_chargerData.Status, ChargerStatus::CHARGING);

            SetState(ChargerState::Stopping_WaitForCarContactorsOpen);
        }
    }
    else if (_state == ChargerState::Stopping_WaitForCarContactorsOpen)
    {
        if (has_flag(_carData.Status, CarStatus::CONTACTOR_OPEN_OR_WELDING_DETECTION_DONE)
            || (_carData.ProtocolNumber < ProtocolNumber::Chademo_1_0 && HasElapsedSec(4)) // Spec: car should perform WD within 4 seconds after amps drops <= 5
            || IsTimeoutSec(10))
        {
            // welding detection done & car contactors open
            println("[cha] Car contactors opened");
            
            SetSwitchD2(false);

            SetState(ChargerState::Stopping_WaitForLowVolts);
        }
    }
    else if (_state == ChargerState::Stopping_WaitForLowVolts)
    {
        // cha spec says <= 10 but we need to play by ccs rules here. Seen some chargers never drop below 28v. Clara uses 40.
        if (_chargerData.OutputVoltage <= 40 || IsTimeoutSec(10))
        {
            OpenAdapterContactor();

            UnlockChargingPlug();
            clear_flag(&_chargerData.Status, ChargerStatus::ENERGIZING_OR_PLUG_LOCKED);

            // do stop CAN in own state to make sure we send this message to car before we kill CAN
            SetState(ChargerState::Stopping_End);
        }
    }
    else if (_state == ChargerState::Stopping_End)
    {
        // spec says, after setting D2:false wait 500ms before setting D1:false. We have passed 2 state changes already, so we have 300ms left.
        if (_cyclesInState > 3)
        {
            // this stops CAN
            SetSwitchD1(false);

            SetState(ChargerState::Stopped);
        }
    }
    else if (_state == ChargerState::Stopped)
    {
        if (_stopReason == StopReason::NONE)
        {
            println("[cha] BUG: StopReason not set. Failover to UNKNOWN");
            _stopReason = StopReason::UNKNOWN;
        }
    }
}

bool ChademoCharger::PreChargeCompleted()
{
    _preChargeDoneButStalled = true;
    _global.ccsPreChargeDoneButStalledTrigger = true;

    if (_precharge_Longer_So_We_Can_Measure_Battery_Voltage)
    {
        bool carAskingAmps = _state > ChargerState::WaitForCarRequestCurrent;
        if (carAskingAmps)
        {
            SetChargerDataFromCcsParams(); // update _chargerData.OutputVoltage
            println("[cha] Estimated battery voltage deviation:%d", _chargerData.OutputVoltage - _carData.EstimatedBatteryVoltage);
        }
        else
        {
            println("[cha] PreCharge stalled until car asking for amps");
        }
            
        return carAskingAmps;
    }
    else
    {
        // keep it hanging until car contactors closed. The voltage may drop fast after precharge is done, if the charger is "floating", so don't complete precharge to soon.
        bool carContactorsClosed = _state > ChargerState::WaitForCarContactorsClosed;
        if (not carContactorsClosed)
            println("[cha] PreCharge stalled until car contactors closed");
        return carContactorsClosed;
    }
}

extern "C" bool chademoInterface_preChargeCompleted()
{
    return chademoCharger->PreChargeCompleted();
}

bool ChademoCharger::CarContactorsOpened()
{
    bool contactorsOpened = _state > ChargerState::Stopping_WaitForCarContactorsOpen;
    return contactorsOpened;
}

extern "C" bool chademoInterface_carContactorsOpened()
{
    return chademoCharger->CarContactorsOpened();
}

bool ChademoCharger::PreChargeCanStart()
{
#ifdef CHADEMO_SINGLE_SESSION
    return _state > ChargerState::WaitForCarSwitchK;
#else
    return true;
#endif
}

extern "C" bool chademoInterface_preChargeCanStart()
{
    return chademoCharger->PreChargeCanStart();
}

void ChademoCharger::SetState(ChargerState newState, StopReason stopReason)
{
    println("[cha] ====>>>> set state %s", _stateNames[newState]);
    _state = newState;
    _cyclesInState = 0;

    set_flag(&_stopReason, stopReason);
    if (_stopReason != StopReason::NONE)
        println("[cha] Stopping: 0x%02x", _stopReason);
};

const char* ChademoCharger::GetStateName()
{
    return _stateNames[_state];
};

#define MAX_UNDERSUPPLY_AMPS 10

void ChademoCharger::SetChargerData(uint16_t maxV, uint16_t maxA, uint16_t outV, uint16_t outA)
{
    _chargerData.AvailableOutputVoltage = maxV;
    if (_chargerData.AvailableOutputVoltage > ADAPTER_MAX_VOLTS)
        _chargerData.AvailableOutputVoltage = ADAPTER_MAX_VOLTS;

    _chargerData.MaxAvailableOutputCurrent = clampToUint8(maxA);
    if (_chargerData.MaxAvailableOutputCurrent > ADAPTER_MAX_AMPS)
        _chargerData.MaxAvailableOutputCurrent = ADAPTER_MAX_AMPS;

    _chargerData.OutputVoltage = outV;

    _chargerData.OutputCurrent = clampToUint8(outA);

    // If difference between RequestCurrent and OutputCurrent is too large, the car fails. If car support dynamic AvailableOutputCurrent,
    // we adjust AvailableOutputCurrent down, forcing the car to ask for less amps, reducing the difference.
    // Its kind of silly...why did they not provide a flag to turn off the car failing part instead? :-)
    // I don't know exactly what difference is allowed (spec. says 10% or 20A). At least 10A difference seems to work fine. 40A certainly does not:-)
    if (_carData.RequestCurrent > _chargerData.OutputCurrent + MAX_UNDERSUPPLY_AMPS
        && (has_flag(_carData.ExtendedFunction1, ExtendedFunction1Flags::DYNAMIC_CONTROL) || has_flag(_carData.Status, CarStatus::LEGACY_DYNAMIC_CONTROL))
        )
    {
        _chargerData.AvailableOutputCurrent = _chargerData.OutputCurrent + MAX_UNDERSUPPLY_AMPS;
    }
    else
    {
        _chargerData.AvailableOutputCurrent = _chargerData.MaxAvailableOutputCurrent;
    }
};

void ChademoCharger::SetChargerDataFromCcsParams()
{
    if (_discovery)
    {
        // fake for discovery
        SetChargerData(ADAPTER_MAX_VOLTS, ADAPTER_MAX_AMPS, 0, 0);
    }
#ifdef CHADEMO_STANDALONE_TESTING
    else if (true)
    {
        SetChargerData(ADAPTER_MAX_VOLTS, 
            ADAPTER_MAX_AMPS, 
            _carData.EstimatedBatteryVoltage,
            0 //_carData.RequestCurrent
        );
    }
#endif
    else
    {
        // mirror these values
       SetChargerData(
            Param::GetInt(Param::EvseMaxVoltage),
            Param::GetInt(Param::EvseMaxCurrent),
            Param::GetInt(Param::EvseVoltage),
            Param::GetInt(Param::EvseCurrent)
        );
    }
}

void ChademoCharger::Log()
{
    if (_logCycleCounter++ > (CHA_CYCLES_PER_SEC * 1))
    {
        // every second
        println("[cha] state:%s cycles:%d out:%dV/%dA avail:%dV/%dA max:%dA rem_t:%ds st=0x%02x car: req:%dA est_t:%dm max_t:%ds st:0x%02x err:0x%02x target:%dV max:%dV soc:%d%% batt:%dV cap=%fkWh",
            GetStateName(),
            _cyclesInState,
            _chargerData.OutputVoltage,
            _chargerData.OutputCurrent,
            _chargerData.AvailableOutputVoltage,
            _chargerData.AvailableOutputCurrent,
            _chargerData.MaxAvailableOutputCurrent,
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
            &_carData.BatteryCapacityKwh  // bypass float to double promotion by passing as reference
        );

        _logCycleCounter = 0;
    }
}

void ChademoCharger::HandleCanMessageIsr(uint32_t id, uint32_t data[2])
{
    if (id == 0x100)
    {
        _msg100_pending = true;
        _msg100_isr.pair[0] = data[0];
        _msg100_isr.pair[1] = data[1];
    }
    else if (id == 0x101)
    {
        _msg101_pending = true;
        _msg101_isr.pair[0] = data[0];
        _msg101_isr.pair[1] = data[1];
    }
    else if (id == 0x102)
    {
        _msg102_pending = true;
        _msg102_isr.pair[0] = data[0];
        _msg102_isr.pair[1] = data[1];
    }
    else if (id == 0x110)
    {
        _msg110_pending = true;
        _msg110_isr.pair[0] = data[0];
        _msg110_isr.pair[1] = data[1];
    }
    else if (id == 0x200)
    {
        _msg200_pending = true;
        _msg200_isr.pair[0] = data[0];
        _msg200_isr.pair[1] = data[1];
    }
    else if (id == 0x201)
    {
        _msg201_pending = true;
        _msg201_isr.pair[0] = data[0];
        _msg201_isr.pair[1] = data[1];
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
    if (_switch_d1 && _msg102_recieved)
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
    COMPARE_SET(_msg108.m.AvailableOutputCurrent, _chargerData.AvailableOutputCurrent, "108.AvailableOutputCurrent %d -> %d");
    COMPARE_SET(_msg108.m.AvailableOutputVoltage, _chargerData.AvailableOutputVoltage, "108.AvailableOutputVoltage %d -> %d");
    COMPARE_SET(_msg108.m.ThresholdVoltage, _chargerData.ThresholdVoltage, "108.ThresholdVoltage %d -> %d");

    COMPARE_SET(_msg109.m.ProtocolNumber, _chargerData.ProtocolNumber, "109.ProtocolNumber %d -> %d");
    COMPARE_SET(_msg109.m.PresentChargingCurrent, _chargerData.OutputCurrent, "109.OutputCurrent %d -> %d");

    // ZE0 seems to hangs the second time, if discharge is enabled during discovery
    COMPARE_SET(_msg109.m.DischargeCompatible, _dischargeEnabled && not _discovery, "109.DischargeCompatible %d -> %d");

    // real outVolt after car contactors close. Before this, use the simulated volt (currently always 0).
    uint16_t outputVolt = _state > ChargerState::WaitForCarContactorsClosed ? _chargerData.OutputVoltage : 0;
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
    // Must have time to tell the car via CAN that plug is unlocked, so auto off when UnlockChargingPlug() is a bit too soon.

    if (_state == ChargerState::Stopped) {
        // power off ok if stopped
        return true;
    }
    else {
        // not stopped (maybe not even started), then power off ok if plug was never locked
        return not _chargingPlugLockedTrigger;
    }
}

