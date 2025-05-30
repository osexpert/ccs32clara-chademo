
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


// Approximate ln(x) for x > 0 using float and similar idea as your fp_ln
float fast_ln(float x)
{
    if (x <= 0.0f)
        return -1e30f;  // or some error indicator for ln(0) or negative input

    // Decompose x = m * 2^n, where m in [1, 2)
    int n = 0;
    float m = x;

    while (m >= 2.0f)
    {
        m *= 0.5f;
        n++;
    }
    while (m < 1.0f)
    {
        m *= 2.0f;
        n--;
    }

    // ln(x) = ln(m) + n * ln(2)
    const float LN2 = 0.6931471806f;

    // Approximate ln(m) on [1,2) using a series or polynomial
    // Here, use a simple polynomial approximation of ln(m) around 1:
    // ln(m) ˜ (m-1) - (m-1)^2/2 + (m-1)^3/3 - (m-1)^4/4

    float y = m - 1.0f;
    float y2 = y * y;
    float y3 = y2 * y;
    float y4 = y3 * y;

    float ln_m = y - y2 * 0.5f + y3 / 3.0f - y4 * 0.25f;

    return ln_m + n * LN2;
}


float EstimateVoltage(float targetVoltage, float socPercentage)
{
    float maxVoltage = targetVoltage - 10.0f;

    if (socPercentage < 0.0f) socPercentage = 0.0f;
    if (socPercentage > 100.0f) socPercentage = 100.0f;

    float nominalVoltage = maxVoltage * 0.9f;
    float minVoltage = maxVoltage * 0.75f;

    float voltage;
    if (socPercentage < 50.0f)
    {
        float scale = 1.5f;
        float x = 1.0f + (socPercentage / 50.0f) * scale;
        float logValue = fast_ln(x);
        float maxLog = fast_ln(1.0f + scale); // ln(1 + scale)
        float t = logValue / maxLog;
        voltage = minVoltage + (nominalVoltage - minVoltage) * t;
    }
    else
    {
        float t = socPercentage / 100.0f;
        voltage = minVoltage + (maxVoltage - minVoltage) * t;
    }

    return voltage;
}

void ChademoCharger::HandlePendingMessages()
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
        COMPARE_SET(_msg102.m.Fault, _msg102_isr.m.Fault, "[cha] 102.Fault changed 0x%x -> 0x%x\r\n");
        COMPARE_SET(_msg102.m.Status, _msg102_isr.m.Status, "[cha] 102.Status changed 0x%x -> 0x%x\r\n");
        COMPARE_SET(_msg102.m.SocPercent, _msg102_isr.m.SocPercent, "[cha] 102.SocPercent changed %d -> %d\r\n");
        COMPARE_SET(_msg102.m.Unused7, _msg102_isr.m.Unused7, "[cha] 102.Unused7 changed %d -> %d\r\n");

        // for timeout
        _carData.CyclesSinceLastAskingAmps = 0;
        _carData.TargetBatteryVoltage = _msg102.m.TargetBatteryVoltage;
        _carData.Fault = (CarFaults)_msg102.m.Fault;
        _carData.Status = (CarStatus)_msg102.m.Status;
        _carData.ProtocolNumber = _msg102.m.ProtocolNumber;

        // limit to adapter max
        if (_msg102.m.ChargingCurrentRequest > ADAPTER_MAX_AMPS)
            _carData.AskingAmps = ADAPTER_MAX_AMPS;
        else
            _carData.AskingAmps = _msg102.m.ChargingCurrentRequest;

        // soc and the constant both unstable before switch(k)
        if (_switch_k)
        {
            if (_msg100.m.SocPercentConstant > 0 && _msg100.m.SocPercentConstant != 100)
                _carData.SocPercent = (uint8_t)((float)_msg102.m.SocPercent / _msg100.m.SocPercentConstant * 100.0f);
            else
                _carData.SocPercent = _msg102.m.SocPercent;

            Param::SetInt(Param::soc, _carData.SocPercent);
        }

        //suspect a crash here...EstimateVoltage
//        Param::SetInt(Param::BatteryVoltage, (int)EstimateVoltage(_carData.TargetBatteryVoltage, _carData.SocPercent));
        //}
        //else
        //{
        //    // can't trust soc yet. fake it. no..then how can we detect we are done with autodetect? i guess
        //    Param::SetInt(Param::soc, 10);

        // not really correct but...target is often 10+ over max battery voltage. With a reliable SOC we could estimate it better, but soc is unreliable, at least initially
        Param::SetInt(Param::BatteryVoltage, _carData.TargetBatteryVoltage - 10);

        Param::SetInt(Param::TargetVoltage, _carData.TargetBatteryVoltage);
        Param::SetInt(Param::MaxVoltage, _carData.TargetBatteryVoltage); // set max to target as well.....

        Param::SetInt(Param::ChargeCurrent, _carData.AskingAmps);

        _sendCanMessages = true;
    }
}

bool ChademoCharger::IsAutodetectCompleted()
{
    // if autodetect, we go here when done
    return _state == PreStart_WaitForChargerLive;
}

void ChademoCharger::Run()
{
    // HandlePendingMessages uses _switch_k
    COMPARE_SET(_switch_k, GetSwitchK(), "[cha] switch (k) changed %d -> %d\r\n");

    ExtractAndSetCcsData();
    HandlePendingMessages();
    RunStateMachine();
    UpdateChargerMessages();
    SendCanMessages();

    Log();
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
            SetState(ChargerState::Stopping_WaitForLowAmpsDelivered);
        }
        if (has_flag(_carData.Status, CarStatus::CAR_STATUS_ERROR))
        {
            printf("[cha] Car status error before starting\r\n");
            SetState(ChargerState::Stopping_WaitForLowAmpsDelivered);
        }
        if (IsChargingStoppedByCharger())
        {
            printf("[cha] Charger stopped before starting\r\n");
            SetState(ChargerState::Stopping_WaitForLowAmpsDelivered);
        }
        if (IsChargingStoppedByAdapter())
        {
            printf("[cha] Adapter stopped before starting\r\n");
            SetState(ChargerState::Stopping_WaitForLowAmpsDelivered);
        }

        if (_state < ChargingLoop)
        {
            _chargerData.ThresholdVoltage = min(_chargerData.AvailableOutputVoltage, _carData.MaxBatteryVoltage);
            _chargerData.RemainingChargeTimeSec = _carData.MaxChargingTimeSec;
            _chargerData.RemainingChargeTimeCycles = _chargerData.RemainingChargeTimeSec * CHA_CYCLES_PER_SEC;
        }
    }

    // Idle-state ignored on purpose. First state must be set actively.
    if (_state == ChargerState::PreStart_WaitForChargerLive)
    {
        if (IsChargerLive())
        {
            SetState(ChargerState::Start);
        }
    }
    else if (_state == ChargerState::Start)
    {
        SetSwitchD1(true); // will trigger car sending can

        SetState(ChargerState::WaitForCarReadyToCharge);
    }
    else if (_state == ChargerState::WaitForCarReadyToCharge)
    {
        if (_switch_k && has_flag(_carData.Status, CarStatus::CAR_STATUS_READY_TO_CHARGE))
        {
            SetState(ChargerState::CarReadyToCharge);
        }
    }
    else if (_state == ChargerState::CarReadyToCharge)
    {
        if (_autoDetect)
        {
            // autodetect done. Clean up for fresh restart.
            // what did we gain?
            // - _carData.SocPercent
            // - _carData.TargetBatteryVoltage
            // - _carData.MaxBatteryVoltage (maybe not so usefull...)
            // Mainly we had to go this far (switch (k)) to be sure SOC could be trusted

            SetSwitchD1(false);
            _sendCanMessages = false;

            _autoDetect = false;

            _carData.Fault = CAR_FAULT_NONE;
            _carData.Status = CAR_STATUS_NONE;
            _chargerData.Status = ChargerStatus::CHARGER_STATUS_STOPPED;

            //SetCarDataSoc(); // set manually since we won't get any more can after we disabled switch(d1) so any logic in there is too late
            Param::SetInt(Param::soc, _carData.SocPercent);

            SetState(ChargerState::PreStart_WaitForChargerLive);
        }
        else
        {
            LockChargingPlug(true);

            set_flag(&_chargerData.Status, ChargerStatus::CHARGER_STATUS_PLUG_LOCKED);

            PerformInsulationTest();

            SetState(ChargerState::WaitForChargerLive);
        }
    }
    else if (_state == ChargerState::WaitForChargerLive)
    {
        if (IsChargerLive())
        {
            SetSwitchD2(true);

            SetState(ChargerState::WaitForCarContactorsClosed);
        }
    }
    else if (_state == ChargerState::WaitForCarContactorsClosed)
    {
        // for cha 0.9 this is always 0?
        if (has_flag(_carData.Status, CarStatus::CAR_STATUS_CONTACTOR_OPEN_WELDING_DETECTION_DONE) == false)
        {
            NotifyCarContactorsClosed();

            SetState(ChargerState::WaitForCarAskingAmps);
        }
    }
    else if (_state == ChargerState::WaitForCarAskingAmps)
    {
        if (_carData.AskingAmps > 0)
        {
            // At this point (car asked for amps), CAR_STATUS_STOP_BEFORE_CHARGING is no longer valid (State >= ChargingLoop)
            // this is the trigger for the charger to turn off CHARGER_STATUS_STOP and instead turn on CHARGER_STATUS_CHARGING

            // Even thou charger not delivering amps yet, we set these flags.
            set_flag(&_chargerData.Status, ChargerStatus::CHARGER_STATUS_CHARGING);
            clear_flag(&_chargerData.Status, ChargerStatus::CHARGER_STATUS_STOPPED);

            NotifyCarAskingForAmps();

            SetState(ChargerState::ChargingLoop);
        }
    }
    else if (_state == ChargerState::ChargingLoop)
    {
        if (_chargerData.RemainingChargeTimeCycles > 0)
            _chargerData.RemainingChargeTimeCycles--;
        _chargerData.RemainingChargeTimeSec = _chargerData.RemainingChargeTimeCycles / CHA_CYCLES_PER_SEC;

        StopReason stopReason = StopReason::NONE;
        if (has_flag(_carData.Status, CarStatus::CAR_STATUS_READY_TO_CHARGE) == false) set_flag(&stopReason, StopReason::CAR_NOT_READY_TO_CHARGE);
        if (has_flag(_carData.Status, CarStatus::CAR_STATUS_NOT_IN_PARK)) set_flag(&stopReason, StopReason::CAR_NOT_IN_PARK);
        if (has_flag(_carData.Status, CarStatus::CAR_STATUS_ERROR)) set_flag(&stopReason, StopReason::CAR_ERROR);
        if (_switch_k == false) set_flag(&stopReason, StopReason::CAR_SWITCH_K_OFF);
        if (IsChargingStoppedByCharger()) set_flag(&stopReason, StopReason::CHARGER);
        if (IsChargingStoppedByAdapter()) set_flag(&stopReason, StopReason::ADAPTER_STOP_BUTTON);
        if (_carData.CyclesSinceLastAskingAmps++ > LAST_ASKING_FOR_AMPS_TIMEOUT_CYCLES) set_flag(&stopReason, StopReason::CAR_CAN_AMPS_TIMEOUT);
        if (_chargerData.RemainingChargeTimeSec == 0) set_flag(&stopReason, StopReason::CHARGING_TIME);

        if (stopReason != StopReason::NONE)
        {
            printf("[cha] Stopping: 0x%x\r\n", stopReason);

            set_flag(&_chargerData.Status, ChargerStatus::CHARGER_STATUS_STOPPED);

            // make sure ccs stop delivering amps and turn down volts (if stop initiated by adapter or car)
            StopPowerDelivery();

            SetState(ChargerState::Stopping_WaitForLowAmpsDelivered);
        }
    }
    else if (_state == ChargerState::Stopping_WaitForLowAmpsDelivered)
    {
        if (_chargerData.OutputCurrent <= 5 || IsTimeoutSec(10))
        {
            _chargerData.RemainingChargeTimeSec = 0;

            // this just mirror the OutputCurrent during stop
            clear_flag(&_chargerData.Status, ChargerStatus::CHARGER_STATUS_CHARGING);

            StopVoltageDelivery();

            SetState(ChargerState::Stopping_WaitForCarContactorsOpen);
        }
    }
    else if (_state == ChargerState::Stopping_WaitForCarContactorsOpen)
    {
        // Got stuck here once... with ProtocolNumber = 1, i got hanging here. It seems it did then never get to set CAR_STATUS_CONTACTOR_OPEN in the first place....
        if (has_flag(_carData.Status, CarStatus::CAR_STATUS_CONTACTOR_OPEN_WELDING_DETECTION_DONE) 
            || IsTimeoutSec(10))
        {
            NotifyCarContactorsOpen();

            SetSwitchD2(false);

            // spec says, after setting D2:false wait 0.5sec (500ms) before setting D1:false
            SetState(ChargerState::Stopping_WaitForLowVoltsDelivered, 500);
        }
    }
    else if (_state == ChargerState::Stopping_WaitForLowVoltsDelivered)
    {
        if (_chargerData.OutputVoltage <= 10 || IsTimeoutSec(10))
        {
            SetState(ChargerState::Stopping_UnlockPlug);
        }
    }
    else if (_state == ChargerState::Stopping_UnlockPlug)
    {
        // safe to unlock plug
        LockChargingPlug(false);
        clear_flag(&_chargerData.Status, ChargerStatus::CHARGER_STATUS_PLUG_LOCKED);

        SetState(ChargerState::Stopping_StopCan);
    }
    else if (_state == ChargerState::Stopping_StopCan)
    {
        // this stops can
        SetSwitchD1(false);
        _sendCanMessages = false;

        SetState(ChargerState::End);
    }
    else if (_state == ChargerState::End)
    {
        // NOTE: must have time to tell the car via can that plug is unlocked....so auto off when LockChargingPlug(false) is a bit too soon.
        _powerOffOk = true;
    }
}

void ChademoCharger::EnableAutodetect()
{
    _autoDetect = true;
}

void ChademoCharger::SetState(ChargerState newState, int delay_ms)
{
    printf("[cha] enter state %d/%s (autodetect:%d)\r\n", newState, _stateNames[newState], _autoDetect);
    _state = newState;
    _cyclesInState = 0;
    _delayCycles = delay_ms / CHA_CYCLE_MS;

    // force log on state change
    Log(true);
};

void ChademoCharger::Log(bool force)
{
    if (force || _logCycleCounter++ > (CHA_CYCLES_PER_SEC * 1))
    {
        // every second or when forced
        printf("[cha] state:%d/%s cycles:%d charger: out:%dV/%dA avail:%dV/%dA rem_t:%ds thres=%dV st=0x%x car: ask:%dA cap=%fkWh est_t:%dm err:0x%x max:%dV max_t:%ds min:%dA soc:%d%% st:0x%x pn:%d target:%dV\r\n",
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
            _carData.Fault,
            _carData.MaxBatteryVoltage,
            _carData.MaxChargingTimeSec,
            _carData.MinimumChargeCurrent,
            _carData.SocPercent,
            _carData.Status,
            _carData.ProtocolNumber,
            _carData.TargetBatteryVoltage
        );

        _logCycleCounter = 0;
    }
}


void ChademoCharger::HandleCanMessageIsr(uint32_t id, uint32_t data[2])
{
    if (id == 0x100)
    {
        _global.cha100++;
        _msg100_pending = true;
        _msg100_isr.pair[0] = data[0];
        _msg100_isr.pair[1] = data[1];
    }
    else if (id == 0x101)
    {
        _global.cha101++;
        _msg101_pending = true;
        _msg101_isr.pair[0] = data[0];
        _msg101_isr.pair[1] = data[1];
    }
    else if (id == 0x102)
    {
        _global.cha102++;
        _msg102_pending = true;
        _msg102_isr.pair[0] = data[0];
        _msg102_isr.pair[1] = data[1];
    }
    else
    {
        _global.chaOther++;
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
        //lastRxTimestamp = time_value;
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

    // Always clear  request complete flag RQCPx
    CAN_TSR(canport) |= rqcp_mask;
}

void ChademoCharger::SendCanMessages()
{
    if (_sendCanMessages)
    {
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

void ChademoCharger::SetCcsData(uint16_t maxV, uint16_t maxA, uint16_t outV, uint16_t outA)
{
    _chargerData.AvailableOutputVoltage = maxV;

    _chargerData.AvailableOutputCurrent = clampToUint8(maxA);
    if (_chargerData.AvailableOutputCurrent > ADAPTER_MAX_AMPS)
        _chargerData.AvailableOutputCurrent = ADAPTER_MAX_AMPS;

    _chargerData.OutputVoltage = outV;
    _chargerData.OutputCurrent = clampToUint8(outA);
};

void ChademoCharger::ExtractAndSetCcsData()
{
    if (_autoDetect)
    {
        // fake it for autodetect
        SetCcsData(450, 100, 0, 0);
    }
    else
    {
        // mirror these values (Change method is only called for some params...)
        SetCcsData(
            Param::GetInt(Param::EvseMaxVoltage),
            Param::GetInt(Param::EvseMaxCurrent),
            Param::GetInt(Param::EvseVoltage),
            Param::GetInt(Param::EvseCurrent)
        );
    }
}

void ChademoCharger::StopPowerDelivery()
{
    printf("[cha] StopPowerDelivery\r\n");

    Param::SetInt(Param::ChargeCurrent, 0);
    // TODO: should we disable at the end of the machine instead?
    Param::SetInt(Param::enable, false);
};

void ChademoCharger::StopVoltageDelivery()
{
    printf("[cha] StopVoltageDelivery\r\n");

    Param::SetInt(Param::TargetVoltage, 0);
    //    Param::Set(Param::BatteryVoltage, 0);
        // TODO: should we disable at the end of the machine instead?
    Param::SetInt(Param::enable, false);
};

bool ChademoCharger::IsChargingStoppedByCharger()
{
    return Param::GetInt(Param::StopReason) != _stopreasons::STOP_REASON_NONE;
}
