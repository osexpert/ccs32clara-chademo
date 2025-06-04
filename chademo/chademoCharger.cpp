
#include <string.h>
#include <stdint.h>
#include "chademoCharger.h"
#include "params.h"
#include "my_fp.h"
#include "main.h"

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
            printf(fmt, (oldval), (newval)); \
            (oldval) = (newval); \
        } \
    } while (0)

extern volatile uint32_t system_millis;
extern global_data _global;


#define LAST_ASKING_FOR_AMPS_TIMEOUT_CYCLES (CHA_CYCLES_PER_SEC * 1)

/// <summary>
/// get estimated battery volt from target and soc.
/// make a lot of assumtions:-)
/// maxVolt = target - 10
/// Based on Leaf/i-MiEV data
/// </summary>
float GetEstimatedBatteryVoltage(float target, float soc)
{
    float maxVolt = target - 10;
    float nomVolt = 0.58f * target + 117.2f; // Linear interpolation/extrapolation
    float minVolt = nomVolt - (maxVolt - nomVolt);

    float deltaLow = 0.14f * (nomVolt - minVolt);   // Steeper drop below 20%
    float deltaHigh = 0.10f * (maxVolt - nomVolt);  // Shallower rise above 80%

    float volt20 = nomVolt - deltaLow;
    float volt80 = nomVolt + deltaHigh;

    if (soc < 20.0f)
    {
        return minVolt + (soc / 20.0f) * (volt20 - minVolt);
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

    if (_msg100_pending)
    {
        _msg100_pending = false;

        COMPARE_SET(_msg100.m.MinimumChargeCurrent, _msg100_isr.m.MinimumChargeCurrent, "[cha] 100.MinimumChargeCurrent changed %d -> %d\r\n");
        // cha 2.0?
        COMPARE_SET(_msg100.m.MinimumBatteryVoltage, _msg100_isr.m.MinimumBatteryVoltage, "[cha] 100.MinimumBatteryVoltage changed %d -> %d\r\n");
        COMPARE_SET(_msg100.m.MaximumBatteryVoltage, _msg100_isr.m.MaximumBatteryVoltage, "[cha] 100.MaximumBatteryVoltage changed %d -> %d\r\n");

        // Allways 100%? no...240 is seen
        COMPARE_SET(_msg100.m.SocPercentConstant, _msg100_isr.m.SocPercentConstant, "[cha] 100.SocPercentConstant changed %d -> %d\r\n");

        COMPARE_SET(_msg100.m.Unused1, _msg100_isr.m.Unused1, "[cha] 100.Unused1 changed %d -> %d\r\n");
        COMPARE_SET(_msg100.m.Unused7, _msg100_isr.m.Unused7, "[cha] 100.Unused7 changed %d -> %d\r\n");

        _carData.MinimumChargeCurrent = _msg100.m.MinimumChargeCurrent;
        _carData.MaxBatteryVoltage = _msg100.m.MaximumBatteryVoltage;
    }
    if (_msg101_pending)
    {
        _msg101_pending = false;

        COMPARE_SET(_msg101.m.MaximumChargingTime10s, _msg101_isr.m.MaximumChargingTime10s, "[cha] 101.MaximumChargingTime10s changed %d -> %d\r\n");
        COMPARE_SET(_msg101.m.MaximumChargingTimeMinutes, _msg101_isr.m.MaximumChargingTimeMinutes, "[cha] 101.MaximumChargingTimeMinutes changed %d -> %d\r\n");
        COMPARE_SET(_msg101.m.EstimatedChargingTimeMinutes, _msg101_isr.m.EstimatedChargingTimeMinutes, "[cha] 101.EstimatedChargingTimeMins changed %d -> %d\r\n");
        COMPARE_SET(_msg101.m.BatteryCapacity, _msg101_isr.m.BatteryCapacity, "[cha] 101.BatteryCapacity changed %d -> %d\r\n");

        COMPARE_SET(_msg101.m.Unused0, _msg101_isr.m.Unused0, "[cha] 101.Unused0 changed %d -> %d\r\n");
        COMPARE_SET(_msg101.m.Unused4, _msg101_isr.m.Unused4, "[cha] 101.Unused4 changed %d -> %d\r\n");
        COMPARE_SET(_msg101.m.Unused7, _msg101_isr.m.Unused7, "[cha] 101.Unused7 changed %d -> %d\r\n");

        _carData.EstimatedChargingTimeMins = _msg101.m.EstimatedChargingTimeMinutes;

        if (_msg101.m.MaximumChargingTime10s == 0xff)
            _carData.MaxChargingTimeSec = _msg101.m.MaximumChargingTimeMinutes * 60;
        else
            _carData.MaxChargingTimeSec = _msg101.m.MaximumChargingTime10s * 10;

        // Even thou spec says 0.1, 0.11 give correct value on Leaf 40kwh: 38.94kwt (with 0.1: 35.4kwt)
        _carData.BatteryCapacityKwh = _msg101.m.BatteryCapacity * 0.11f;
    }
    if (_msg102_pending)
    {
        _msg102_pending = false;

        COMPARE_SET(_msg102.m.ProtocolNumber, _msg102_isr.m.ProtocolNumber, "[cha] 102.ProtocolNumber changed %d -> %d\r\n");
        COMPARE_SET(_msg102.m.TargetBatteryVoltage, _msg102_isr.m.TargetBatteryVoltage, "[cha] 102.TargetBatteryVoltage changed %d -> %d\r\n");
        COMPARE_SET(_msg102.m.ChargingCurrentRequest, _msg102_isr.m.ChargingCurrentRequest, "[cha] 102.ChargingCurrentRequest changed %d -> %d\r\n");
        COMPARE_SET(_msg102.m.Faults, _msg102_isr.m.Faults, "[cha] 102.Faults changed 0x%x -> 0x%x\r\n");
        COMPARE_SET(_msg102.m.Status, _msg102_isr.m.Status, "[cha] 102.Status changed 0x%x -> 0x%x\r\n");
        COMPARE_SET(_msg102.m.SocPercent, _msg102_isr.m.SocPercent, "[cha] 102.SocPercent changed %d -> %d\r\n");
        COMPARE_SET(_msg102.m.Unused7, _msg102_isr.m.Unused7, "[cha] 102.Unused7 changed %d -> %d\r\n");

        _carData.CyclesSinceCarLastAskingAmps = 0; // for timeout
        _carData.TargetBatteryVoltage = _msg102.m.TargetBatteryVoltage;
        _carData.Faults = (CarFaults)_msg102.m.Faults;
        _carData.Status = (CarStatus)_msg102.m.Status;
        _carData.ProtocolNumber = _msg102.m.ProtocolNumber;

        // limit to adapter max
        if (_msg102.m.ChargingCurrentRequest > ADAPTER_MAX_AMPS)
            _carData.AskingAmps = ADAPTER_MAX_AMPS;
        else
            _carData.AskingAmps = _msg102.m.ChargingCurrentRequest;

        // soc and the constant both unstable before switch(k) (and also capacity, but don't care about that)
        if (_switch_k)
        {
            if (_msg100.m.SocPercentConstant > 0 && _msg100.m.SocPercentConstant != 100)
                _carData.SocPercent = (uint8_t)((float)_msg102.m.SocPercent / _msg100.m.SocPercentConstant * 100.0f);
            else
                _carData.SocPercent = _msg102.m.SocPercent;

            _carData.EstimatedBatteryVoltage = GetEstimatedBatteryVoltage(_carData.TargetBatteryVoltage, _carData.SocPercent);
        }

        _msg102_recieved = true;
    }
}

void ChademoCharger::Run()
{
    // HandlePendingMessages uses _switch_k
    COMPARE_SET(_switch_k, GetSwitchK(), "[cha] switch(k) changed %d -> %d\r\n");

    SetChargerDataFromCcsParams();
    HandlePendingCarMessages();
    RunStateMachine();
    SetCcsParamsFromCarData();
    SendChargerMessages();

    Log();
}

void ChademoCharger::SetCcsParamsFromCarData()
{
    Param::SetInt(Param::soc, _carData.SocPercent);

    // from can logs it seems...chademo do not give a damn about the battery voltage, so just use target
    // I guess it make sense, because battery voltage is not part of chademo, so how could it possibly use it for any logic, at least before contactors are closed :-D
    // After contactors close, the car battery will initially drive the voltage down anyways (I assume).
    Param::SetInt(Param::BatteryVoltage, _carData.TargetBatteryVoltage);// _carData.EstimatedBatteryVoltage);
    
    if (_stop_delivering_volts)
    {
        Param::SetInt(Param::TargetVoltage, 0);
        Param::SetInt(Param::enable, false); // stop charger completely
    }
    else
    {
        Param::SetInt(Param::TargetVoltage, _carData.TargetBatteryVoltage);
    }

    if (_stop_delivering_amps)
    {
        Param::SetInt(Param::ChargeCurrent, 0);
    }
    else
    {
        Param::SetInt(Param::ChargeCurrent, _carData.AskingAmps);
    }

    // target +1 to silence warning in pev_sendCurrentDemandReq
    Param::SetInt(Param::MaxVoltage, _carData.TargetBatteryVoltage + 1);
}

bool ChademoCharger::IsTimeoutSec(uint16_t max_sec)
{
    if (_cyclesInState > (max_sec * CHA_CYCLES_PER_SEC))
    {
        printf("[cha] Timeout in %d/%s (max:%dsec)\r\n", _state, GetStateName(), max_sec);
        return true;
    }
    return false;
}

/// <summary>
/// PS: Do not care about timeout before the charging loop (power off is allowed).
/// But after we have been inside the charging loop, we must get to the end somehow...
/// </summary>
void ChademoCharger::RunStateMachine()
{
    _cyclesInState++;

    if (_delayCycles > 0)
    {
        _delayCycles--;
        return;
    }

    if (_state < ChargerState::ChargingLoop)
    {
        if (has_flag(_carData.Status, CarStatus::CAR_STATUS_STOP_BEFORE_CHARGING))
        {
            printf("[cha] Car stopped before starting\r\n");
            SetState(ChargerState::Stopping_WaitForCarContactorsOpen);
        }
        if (has_flag(_carData.Status, CarStatus::CAR_STATUS_ERROR))
        {
            printf("[cha] Car status error before starting\r\n");
            SetState(ChargerState::Stopping_WaitForCarContactorsOpen);
        }
        if (_global.powerOffPending)
        {
            printf("[cha] Power off pending before starting\r\n");
            SetState(ChargerState::Stopping_WaitForCarContactorsOpen);
        }

        if (_state < ChargingLoop)
        {
            _chargerData.ThresholdVoltage = min(_chargerData.AvailableOutputVoltage, _carData.MaxBatteryVoltage);
            _chargerData.RemainingChargeTimeSec = _carData.MaxChargingTimeSec;
            _chargerData.RemainingChargeTimeCycles = _chargerData.RemainingChargeTimeSec * CHA_CYCLES_PER_SEC;
        }
    }

    //if (_state >= ChargerState::WaitForCarReadyToCharge && _state < ChargerState::ChargingLoop)
    //{
    //    IsTimeoutSec(20)
    //}

    if (_state == ChargerState::WaitForPreChargeStart)
    {
        if (_global.ccsPreChargeStartedEvent)
        {
            SetSwitchD1(true); // will trigger car sending can

            SetState(ChargerState::WaitForCarReadyToCharge);
        }
    }
    else if (_state == ChargerState::WaitForCarReadyToCharge)
    {
        if (_switch_k && has_flag(_carData.Status, CarStatus::CAR_STATUS_READY_TO_CHARGE))
        {
            _powerOffOk = false;

            // car probably want to compare that output voltage we claim matches what it can measure (it can probably measure volts before its contactors)
            // FIXME: maybe it should be a different place...
//            CloseAdapterContactor();

            SetState(ChargerState::WaitForPreChangeDone);
        }
        else if (IsTimeoutSec(20))
        {
            SetState(ChargerState::Stopping_Start);
        }
    }
    else if (_state == ChargerState::WaitForPreChangeDone)
    {
        // New idea: since ccs closes relays at end of precharge, this matches well with chademo car closing them soon(?) after we set D2=true.
        // Sometimes the car does not close contactors after d2=true but instead fails...don't know why....

        // prevent precharge from completing before we get here.
        _global.ccsPreChargeDoneKickoff = true;

        if (_global.ccsPreChargeDoneEvent)
        {
            // the car need live volts so it can measure and compare what we say we have is what it sees, specially when we say volts are not 0.
            // not sure if this is the right place, maybe we could close it even earlier, right after LockChargingPlug
            CloseAdapterContactor();

            set_flag(&_chargerData.Status, ChargerStatus::CHARGER_STATUS_ENERGIZING);

            // give car power for its contactors (it need both d1 and d2)
            SetSwitchD2(true);

            SetState(ChargerState::WaitForCarContactorsClosed);
        }
        else if (IsTimeoutSec(20)) // spec: Max 20sec from CarReadyToCharge to D2=true (should be plenty?)
        {
            SetState(ChargerState::Stopping_Start);
        }
    }
    else if (_state == ChargerState::WaitForCarContactorsClosed)
    {
        // for cha 0.9 this is always 0?
        if (has_flag(_carData.Status, CarStatus::CAR_STATUS_CONTACTOR_OPEN_WELDING_DETECTION_DONE) == false)
        {
            // hard to tell where it make sense to do this....it depends totally on if the car uses voltage measurements _before_ the contactors, in its logic
//            CloseAdapterContactor();

            // AFTER CAR OPEN CONTACTOR, it will start asking for amps pretty fast. and if it dont get it, it will fail pretty fast too.....
            SetState(ChargerState::WaitForCarAskingAmps);
        }
        else if (IsTimeoutSec(20))
        {
            SetState(ChargerState::Stopping_Start);
        }
    }
    else if (_state == ChargerState::WaitForCarAskingAmps)
    {
        // normally happens 3sec after we set D2=true
        if (_carData.AskingAmps > 0)
        {
            // At this point (car asked for amps), CAR_STATUS_STOP_BEFORE_CHARGING is no longer valid (State >= ChargingLoop)
            // this is the trigger for the charger to turn off CHARGER_STATUS_STOP and instead turn on CHARGER_STATUS_CHARGING

            // Even thou charger not delivering amps yet, we set these flags.
            set_flag(&_chargerData.Status, ChargerStatus::CHARGER_STATUS_CHARGING);
            clear_flag(&_chargerData.Status, ChargerStatus::CHARGER_STATUS_STOPPED);

            SetState(ChargerState::ChargingLoop);
        }
        else if (IsTimeoutSec(20))
        {
            SetState(ChargerState::Stopping_Start);
        }
    }
    else if (_state == ChargerState::ChargingLoop)
    {
        if (_chargerData.RemainingChargeTimeCycles > 0)
            _chargerData.RemainingChargeTimeCycles--;
        _chargerData.RemainingChargeTimeSec = _chargerData.RemainingChargeTimeCycles / CHA_CYCLES_PER_SEC;

        _stopReason = StopReason::NONE;
        if (has_flag(_carData.Status, CarStatus::CAR_STATUS_READY_TO_CHARGE) == false) set_flag(&_stopReason, StopReason::CAR_NOT_READY_TO_CHARGE);
        if (has_flag(_carData.Status, CarStatus::CAR_STATUS_NOT_IN_PARK)) set_flag(&_stopReason, StopReason::CAR_NOT_IN_PARK);
        if (has_flag(_carData.Status, CarStatus::CAR_STATUS_ERROR)) set_flag(&_stopReason, StopReason::CAR_ERROR);
        if (_switch_k == false) set_flag(&_stopReason, StopReason::CAR_SWITCH_K_OFF);
        if (_global.powerOffPending) set_flag(&_stopReason, StopReason::POWER_OFF_PENDING);
        if (_carData.CyclesSinceCarLastAskingAmps++ > LAST_ASKING_FOR_AMPS_TIMEOUT_CYCLES) set_flag(&_stopReason, StopReason::CAR_CAN_AMPS_TIMEOUT);
        if (_chargerData.RemainingChargeTimeSec == 0) set_flag(&_stopReason, StopReason::CHARGING_TIME);

        if (_stopReason != StopReason::NONE)
        {
            printf("[cha] Stopping: 0x%x\r\n", _stopReason);

            SetState(ChargerState::Stopping_Start);
        }
    }
    else if (_state == ChargerState::Stopping_Start)
    {
        set_flag(&_chargerData.Status, ChargerStatus::CHARGER_STATUS_STOPPED);

        // stop amps, but continue delivering volts so car can do welding detection
        _stop_delivering_amps = true;

        SetState(ChargerState::Stopping_WaitForLowAmps);
    }
    else if (_state == ChargerState::Stopping_WaitForLowAmps)
    {
        if (_chargerData.OutputCurrent <= 5 || IsTimeoutSec(10))
        {
            clear_flag(&_chargerData.Status, ChargerStatus::CHARGER_STATUS_CHARGING);

            SetState(ChargerState::Stopping_WaitForCarContactorsOpen);
        }
    }
    else if (_state == ChargerState::Stopping_WaitForCarContactorsOpen)
    {
        // Got stuck here once... with ProtocolNumber = 1, i got hanging here. It seems it did then never get to set CAR_STATUS_CONTACTOR_OPEN in the first place....
        if (has_flag(_carData.Status, CarStatus::CAR_STATUS_CONTACTOR_OPEN_WELDING_DETECTION_DONE) || IsTimeoutSec(10))
        {
            // welding detection done, car no longer need live volts

            //OpenAdapterContactor();

            // we now revoke the cars contactor power (it need both d1+d2), but it alreadyt said they were opened, so fine
            SetSwitchD2(false);
            // now also stop volts and charger completely.
            _stop_delivering_volts = true;

            // Not sure if this fits best before or after D2 = false. I guess its more transparent to the car to have it here.
            OpenAdapterContactor();

            // spec says, after setting D2:false wait 0.5sec (500ms) before setting D1:false. We have 2 states after, so we already have 2 * 200ms and 300ms left.
            SetState(ChargerState::Stopping_WaitForLowVolts, 3);
        }
    }
    else if (_state == ChargerState::Stopping_WaitForLowVolts)
    {
        if (_chargerData.OutputVoltage <= 10 || IsTimeoutSec(10))
        {
            clear_flag(&_chargerData.Status, ChargerStatus::CHARGER_STATUS_ENERGIZING);

            // do stop can in own state to make sure we send this message to car before we kill can
            SetState(ChargerState::Stopping_End);
        }
    }
    else if (_state == ChargerState::Stopping_End)
    {
        // this stops can (allthou not necesarely immediately, I have recieved messages after setting d1=false)
        SetSwitchD1(false);

        SetState(ChargerState::Stopped);
    }
    else if (_state == ChargerState::Stopped)
    {
        // NOTE: must have time to tell the car via can that plug is unlocked....so auto off when LockChargingPlug(false) is a bit too soon.
        _powerOffOk = true;
    }
}

void ChademoCharger::SetState(ChargerState newState, int delayCycles)
{
    printf("[cha] enter state %d/%s\r\n", newState, _stateNames[newState]);
    _state = newState;
    _cyclesInState = 0;
    _delayCycles = delayCycles;

    // force log on state change
    Log(true);
};

void ChademoCharger::Log(bool force)
{
    if (force || _logCycleCounter++ > (CHA_CYCLES_PER_SEC * 1))
    {
        // every second or when forced
        printf("[cha] state:%d/%s cycles:%d charger: out:%dV/%dA avail:%dV/%dA rem_t:%ds thres=%dV st=0x%x car: ask:%dA cap=%fkWh est_t:%dm err:0x%x max:%dV max_t:%ds min:%dA soc:%d%% st:0x%x pn:%d target:%dV batt:%dV\r\n",
            _state,
            GetStateName(),
            _cyclesInState,
            _chargerData.OutputVoltage,
            _chargerData.OutputCurrent,
            _chargerData.AvailableOutputVoltage,
            _chargerData.AvailableOutputCurrent,
            _chargerData.RemainingChargeTimeSec,
            _chargerData.ThresholdVoltage,
            _chargerData.Status,

            _carData.AskingAmps,
            FP_FROMFLT(_carData.BatteryCapacityKwh),
            _carData.EstimatedChargingTimeMins,
            _carData.Faults,
            _carData.MaxBatteryVoltage,
            _carData.MaxChargingTimeSec,
            _carData.MinimumChargeCurrent,
            _carData.SocPercent,
            _carData.Status,
            _carData.ProtocolNumber,
            _carData.TargetBatteryVoltage,
            _carData.EstimatedBatteryVoltage
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

// can: 100ms +-10ms
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
    //    printf("[can] transmit: peripheral not initialized\r\n");
    //    return;// CAN_TX_ERROR;
    //}

    int mailbox = can_transmit(canport, id, ext, rtr, len, data);
    if (mailbox < 0) {
        printf("[can] transmit: no mailbox available\r\n");
        return;// CAN_TX_NO_MAILBOX;
    }

    uint32_t txok_mask = CAN_TSR_TXOK0 << mailbox;
    uint32_t rqcp_mask = CAN_TSR_RQCP0 << mailbox;
//    uint32_t terr_mask = CAN_TSR_TERR0 << mailbox;

    // wait until done (TX done or error) or timeout
    uint32_t start = system_millis;
    while ((CAN_TSR(canport) & rqcp_mask) == 0) {
        if ((system_millis - start) > CAN_TRANSMIT_TIMEOUT_MS) {
            printf("[can] transmit timeout for ID 0x%x\r\n", id);
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
    }
}

void ChademoCharger::UpdateChargerMessages()
{
    COMPARE_SET(_msg108.m.WeldingDetection, _chargerData.SupportWeldingDetection, "[cha] 108.WeldingDetection changed %d -> %d\r\n");
    COMPARE_SET(_msg108.m.AvailableOutputCurrent, _chargerData.AvailableOutputCurrent, "[cha] 108.AvailableOutputCurrent changed %d -> %d\r\n");
    COMPARE_SET(_msg108.m.AvailableOutputVoltage, _chargerData.AvailableOutputVoltage, "[cha] 108.AvailableOutputVoltage changed %d -> %d\r\n");
    COMPARE_SET(_msg108.m.ThresholdVoltage, _chargerData.ThresholdVoltage, "[cha] 108.ThresholdVoltage changed %d -> %d\r\n");

    COMPARE_SET(_msg109.m.ProtocolNumber, _chargerData.ProtocolNumber, "[cha] 109.ProtocolNumber changed %d -> %d\r\n");
    COMPARE_SET(_msg109.m.PresentChargingCurrent, _chargerData.OutputCurrent, "[cha] 109.OutputCurrent changed %d -> %d\r\n");
    COMPARE_SET(_msg109.m.PresentVoltage, _chargerData.OutputVoltage, "[cha] 109.OutputVoltage changed %d -> %d\r\n");

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

    COMPARE_SET(_msg109.m.RemainingChargingTime10s, remainingChargingTime10s, "[cha] 109.RemainingChargingTime10s changed %d -> %d\r\n");
    COMPARE_SET(_msg109.m.RemainingChargingTimeMinutes, remainingChargingTimeMins, "[cha] 109.RemainingChargingTimeMinutes changed %d -> %d\r\n");
    COMPARE_SET(_msg109.m.Status, _chargerData.Status, "[cha] 109.Status changed 0x%x -> 0x%x\r\n");
}

const char* ChademoCharger::GetStateName()
{
    return _stateNames[_state];
};

void ChademoCharger::SetChargerData(uint16_t maxV, uint16_t maxA, uint16_t outV, uint16_t outA)
{
    _chargerData.AvailableOutputVoltage = maxV;

    _chargerData.AvailableOutputCurrent = clampToUint8(maxA);
    if (_chargerData.AvailableOutputCurrent > ADAPTER_MAX_AMPS)
        _chargerData.AvailableOutputCurrent = ADAPTER_MAX_AMPS;

    _chargerData.OutputVoltage = outV;
    _chargerData.OutputCurrent = clampToUint8(outA);
};

void ChademoCharger::SetChargerDataFromCcsParams()
{
    SetChargerData(
        Param::GetInt(Param::EvseMaxVoltage),
        Param::GetInt(Param::EvseMaxCurrent),
        Param::GetInt(Param::EvseVoltage),
        Param::GetInt(Param::EvseCurrent)
    );
}

