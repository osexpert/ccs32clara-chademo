
#include <string.h>
#include <stdint.h>
#include "chademoCharger.h"
#include "params.h"
#include "my_fp.h"
#include "main.h"

#include <libopencm3/stm32/can.h>

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



void ChademoCharger::HandlePendingIsrMessages()
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

        _carData.MinimumBatteryVoltage = _msg100.m.MinimumBatteryVoltage;
        _carData.MaxBatteryVoltage = _msg100.m.MaximumBatteryVoltage;

        Param::SetInt(Param::MaxVoltage, _carData.MaxBatteryVoltage);
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

        // limit to adapter max
        if (_msg102.m.ChargingCurrentRequest > ADAPTER_MAX_AMPS)
            _carData.AskingAmps = ADAPTER_MAX_AMPS;
        else
            _carData.AskingAmps = _msg102.m.ChargingCurrentRequest;

        if (_msg100.m.SocPercentConstant > 0 && _msg100.m.SocPercentConstant != 100)
            _carData.SocPercent = (uint8_t)((float)_msg102.m.SocPercent / _msg100.m.SocPercentConstant * 100.0f);
        else
            _carData.SocPercent = _msg102.m.SocPercent;

        Param::SetInt(Param::TargetVoltage, _carData.TargetBatteryVoltage);
        Param::SetInt(Param::BatteryVoltage, _carData.TargetBatteryVoltage - 10); // not really correct but...
        Param::SetInt(Param::soc, _carData.SocPercent);
        Param::SetInt(Param::ChargeCurrent, _carData.AskingAmps);

//        _sendCanKickoff = true;
    }

}


void ChademoCharger::Run()
{
    ExtractAndSetCcsData();

    HandlePendingIsrMessages();

    RunStateMachine();

    UpdateChargerMessages();

    //if (_sendCanKickoff)
    //    AddCanSendTask();

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
/// TODO: should we not send can immediately?
/// TODO: should we stop sending can at some point?
/// </summary>
void ChademoCharger::RunStateMachine()
{
    _cyclesInState++;

    bool k = GetSwitchK();
    if (k != _k_switch)
    {
        printf("[cha] k-switch changed %d -> %d\r\n", _k_switch, k);
        _k_switch = k;
    }

    if (_delayCycles > 0)
    {
        _delayCycles--;
        return;
    }

    if (_state < ChargerState::ChargingLoop)
    {
        // not in cha0.9?
        // everyone seems to ignor it??? lets try................
        // hack removed
        //if (has_flag(_carData.Status, CarStatus::CAR_STATUS_STOP_BEFORE_CHARGING))
        //{
        //    printf("[CHA] Car stopped before starting\r\n");
        //    SetState(ChargerState::Stopping_WaitForLowAmpsDelivered);
        //}
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

        

        // Take car as initial value and countdown the minutes
//        if (_carData.MaxChargingTimeMins > 0 && )
  //          _chargerData.RemainingChargeTimeMins = _carData.MaxChargingTimeMins;

    }

    // Car seem to immediately go into stopped before starting, i am guessing if available volts and amps are 0 from the start. So do it in two steps...
    if (_state == ChargerState::Start)
    {
        printf("[cha] start\r\n");

        SetSwitchD1(true); // will trigger car sending can 102?

        SetState(ChargerState::WaitForCarMaxAndTargetVolts);
    }
    else if (_state == ChargerState::WaitForCarMaxAndTargetVolts)
    {
        // if we have these 2 set, we have data from at least 100 and 102.
        if (_carData.MaxBatteryVoltage > 0 && _carData.TargetBatteryVoltage > 0)
        {
            // we got volts! done with can for now. turn off to make car happy, reset its states, not timeout, stop before starting etc.
            // TODO: was not sending CAN earier........................it may also be the prioblem........................anyways....this seems like a better solution...
            // we can't wait here for more tha 3-4 sec before er get error.
//            SetSwitchD1(false);

            // wait 500ms before we ack. MaxBatteryVoltage -> threashold voltage
            SetState(ChargerState::WaitForChargerAvailableVoltsAndAmps, 400); //or 500?

            // clear misc stuff, ready for next restart (or not)
           // _carData.Faults = CAR_FAULT_NONE;
         //   _carData.Status = CAR_STATUS_NONE;
        }
    }
    else if (_state == ChargerState::WaitForChargerAvailableVoltsAndAmps)
    {
        // FIXME: this check is pointless now??????
        if (_chargerData.AvailableOutputVoltage > 0 && _chargerData.CcsAvailableOutputCurrent > 0)
        {
            // we waited 500ms and now we can calc it. If we calc it too soon, the CAR will punish us with an error...
            // WE canø 3 things: charger max amps, thresshodl voltage, remaining chanrging time
            _chargerData.ThresholdVoltage = min(_chargerData.AvailableOutputVoltage, _carData.MaxBatteryVoltage);
            _chargerData.RemainingChargeTimeSec = _carData.MaxChargingTimeSec;
            // its ment to not be set before this point??
            _chargerData.AvailableOutputCurrent = _chargerData.CcsAvailableOutputCurrent;

            // wrong place....according to spec......
//            set_flag(&_chargerData.Status, ChargerStatus::CHARGER_STATUS_PLUG_LOCKED);

            // TODO: fake a current calc from watts?????????????????????????????????
//            _chargerData.AvailableOutputCurrent =

            // we have gotten so far we got availables back from ccs. We can then enable can again (or is it too soon??? it may be. seems like i take 16 sec. until we reach CurrentDemandReq,
            // it may be too much, but from Using-OCPP-with-CHAdeMO.pdf it seems like T-time 22+6 seconds, so should be ok...)
//            SetSwitchD1(true); // will trigger car sending can

            SetState(ChargerState::WaitForCarReadyToCharge);
        }
    }
    else if (_state == ChargerState::WaitForCarReadyToCharge)
    {
        // why never here????

        
       

        /*if ((canEna || _k_switch) && (canEna != _k_switch))
            printf("[cha] only one of can %d and SwitchK %d enabled", canEna, _k_switch);*/

        if (_k_switch)
        {
            SetState(ChargerState::CarReadyToCharge);
        }
    }
    else if (_state == ChargerState::CarReadyToCharge)
    {
        LockChargingPlug(true);

        set_flag(&_chargerData.Status, ChargerStatus::CHARGER_STATUS_PLUG_LOCKED);

        PerformInsulationTest();

        bool canEna = has_flag(_carData.Status, CarStatus::CAR_STATUS_READY_TO_CHARGE);
        if (canEna)
        {
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

            // Take car as initial value and countdown the minutes
            //_chargerData.RemainingChargeTimeMins = _carData.MaxChargingTimeMins;

            NotifyCarAskingForAmps();

            SetState(ChargerState::ChargingLoop);
        }
    }
    else if (_state == ChargerState::ChargingLoop)
    {
        StopReason stopReason = StopReason::NONE;
        if (has_flag(_carData.Status, CarStatus::CAR_STATUS_READY_TO_CHARGE) == false) set_flag(&stopReason, StopReason::CAR_NOT_READY_TO_CHARGE);
        if (has_flag(_carData.Status, CarStatus::CAR_STATUS_NOT_IN_PARK)) set_flag(&stopReason, StopReason::CAR_NOT_IN_PARK);
        if (has_flag(_carData.Status, CarStatus::CAR_STATUS_ERROR)) set_flag(&stopReason, StopReason::CAR_ERROR);
        if (GetSwitchK() == false) set_flag(&stopReason, StopReason::CAR_K_OFF);
        if (IsChargingStoppedByCharger()) set_flag(&stopReason, StopReason::CHARGER);
        if (IsChargingStoppedByAdapter()) set_flag(&stopReason, StopReason::ADAPTER_STOP_BUTTON);
        if (_carData.CyclesSinceLastAskingAmps++ > LAST_ASKING_FOR_AMPS_TIMEOUT_CYCLES) set_flag(&stopReason, StopReason::CAR_CAN_AMPS_TIMEOUT);

        if (stopReason != StopReason::NONE)
        {
            printf("[cha] Stopping: 0x%x\r\n", stopReason);

            // Checking for State >= ChargerState.Stopping_WaitForLowAmpsDelivered is probably better if we need to know we are in this state, instead of mutating the car data?
            _carData.AskingAmps = 0;

            set_flag(&_chargerData.Status, ChargerStatus::CHARGER_STATUS_STOPPED);

            // make sure ccs stop delivering amps and turn down volts (if stop initiated by adapter or car)
            StopPowerDelivery();

            SetState(ChargerState::Stopping_WaitForLowAmpsDelivered);
        }
    }
    else if (_state == ChargerState::Stopping_WaitForLowAmpsDelivered)
    {
        if (_chargerData.OutputCurrent <= 5)
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
        if (has_flag(_carData.Status, CarStatus::CAR_STATUS_CONTACTOR_OPEN_WELDING_DETECTION_DONE)
            // With RawVer = 1, i got hanging here. It seems it did then never get to set CAR_STATUS_CONTACTOR_OPEN in the first place....
            || IsTimeoutSec(10) // if can get stuck, make sure we can progress stopping TODO: ADD timeout only if needed?? Got stuck here once...
            )
        {
            SetSwitchD2(false);

            // spec says, after setting D2:false wait 0.5sec (500ms) before setting D1:false
            SetState(ChargerState::Stopping_WaitForLowVoltsDelivered, 500);
        }
    }
    else if (_state == ChargerState::Stopping_WaitForLowVoltsDelivered)
    {
        // TODO: is it possible the volt is never dropped? need timeout here too perhaps????
        if (_chargerData.OutputVoltage <= 10)
        {
            SetState(ChargerState::Stopping_UnlockPlug);
        }
    }
    // TODO: if can is lost, the machine will halt and we never get to unlock the plug
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
        SetState(ChargerState::End);
    }
    else if (_state == ChargerState::End)
    {
        // NOTE: must have time to tell the car via can that plug is unlocked....so auto off when LockChargingPlug(false) is a bit too soon.
        _powerOffOk = true;
    }

    // TODO: we need some place to do all cleanup in case of timeout.

}

void ChademoCharger::SetState(ChargerState newState, int delay_ms)
{
    printf("[cha] enter state %d/%s\r\n", newState, _stateNames[newState]);
    _state = newState;
    _cyclesInState = 0;
    _delayCycles = delay_ms / CHA_CYCLE_MS;

    // force log on state change
    Log(true);
};

void ChademoCharger::Log(bool force)
{
    if (force || _logCycleCounter++ > (CHA_CYCLES_PER_SEC * 1))//|| _carData != _lastLoggedCarData)
    {
        // every second or when forced
        printf("[cha] state:%d/%s cycles:%d charger: out:%dV/%dA avail:%dV/ccs:%d/cha:%dA rem_t:%ds weld:%d rv:%d thres=%dV st=0x%x car: ask:%dA cap=%fkWh est_t:%dm err:0x%x max:%dV max_t:%ds min:%dA soc:%d%% st:0x%x target:%dV ksw:%d min:%dV\r\n",
            _state,
            GetStateName(),
            _cyclesInState,
            _chargerData.OutputVoltage,
            _chargerData.OutputCurrent,
            _chargerData.AvailableOutputVoltage,
            _chargerData.CcsAvailableOutputCurrent,
            _chargerData.AvailableOutputCurrent,
            _chargerData.RemainingChargeTimeSec,
            _chargerData.SupportWeldingDetection,
            _chargerData.ProtocolNumber,
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
            _carData.TargetBatteryVoltage,
            _k_switch,
            _carData.MinimumBatteryVoltage
        );

        _logCycleCounter = 0;
        //_lastLoggedCarData = _carData;
    }
}

// recalc every time _carData.MaxBatteryVoltage or _chargerData.AvailableOutputVoltage is set
//void ChademoCharger::CalcChargerThreasholdVoltage()
//{
//    //if (_state < ChargerState::ChargingLoop)
//    {
//        // set to the minimum of AvailableOutputVoltage and MaxChargeVoltage, until charging start.
//        if (_carData.MaxBatteryVoltage > 0 && _carData.MaxBatteryVoltage < _chargerData.AvailableOutputVoltage)
//            _chargerData.ThresholdVoltage = _carData.MaxBatteryVoltage;
//        else
//            _chargerData.ThresholdVoltage = _chargerData.AvailableOutputVoltage;
//    }
//}



extern void AddCanSendTask();

void ChademoCharger::HandleCanMessage(uint32_t id, uint32_t data[2])
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

        AddCanSendTask();
    }
    else
    {
        _global.chaOther++;
    }
}




#define CAN_TX_TIMEOUT_MS 5 // 5ms timeout

//#ifdef CAN_DEBUG
//#include <stdio.h>
//#define CAN_LOG(...) printf(__VA_ARGS__)
//#else
//#define CAN_LOG(...)
//#endif

//typedef enum {
//    CAN_TX_OK,
//    CAN_TX_NO_MAILBOX,
//    CAN_TX_TIMEOUT,
//    CAN_TX_ERROR
//} can_tx_result_t;

uint8_t check_can_tx_ok(uint32_t can_peripheral) {
    uint32_t tsr = CAN_TSR(can_peripheral);
    uint8_t result = 0;

    if ((tsr & CAN_TSR_TXOK0) != 0) {
        result++;
    }
    if ((tsr & CAN_TSR_TXOK1) != 0) {
        result++;
    }
    if ((tsr & CAN_TSR_TXOK2) != 0) {
        result++;
    }

    return result;  // 0–3 mailboxes with successful transmission
}

void can_transmit_blocking(uint32_t canport, uint32_t id, bool ext, bool rtr, uint8_t len, uint8_t* data)
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

    //uint32_t tsr = CAN_TSR(canport);

    //uint32_t rqcp_mask = CAN_TSR_RQCP0 << mailbox;
    while (CAN_TSR(canport) && (CAN_TSR_TXOK0 << mailbox) == 0)
    {
        // loop
    }


    //uint32_t terr_mask = CAN_TSR_TERR0 << mailbox;

    //// Use SysTick for precise timeout
    //uint32_t start = system_millis;
    //while ((CAN_TSR(canport) & (rqcp_mask | txok_mask)) != (rqcp_mask | txok_mask)) {
    //    if (system_millis - start > CAN_TX_TIMEOUT_MS) {
    //        printf("[can] transmit: timeout on mailbox %d\r\n", mailbox);
    //        CAN_TSR(canport) |= rqcp_mask; // Clear request complete flag
    //        return CAN_TX_TIMEOUT;
    //    }
    //    if (CAN_TSR(canport) & terr_mask) {
    //        printf("[can] transmit: error on mailbox %d\r\n", mailbox);
    //        CAN_TSR(canport) |= terr_mask | rqcp_mask; // Clear error and request flags
    //        return CAN_TX_ERROR;
    //    }
    //}

    //// Clear request complete flag
    //CAN_TSR(canport) |= rqcp_mask;
    //return CAN_TX_OK;
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
    

    //if (_chargerData.RemainingChargeTimeMins > 0)
    //{
    //    data[6] = 0xFF; //RemainingChargeTime10Sec
    //    data[7] = _chargerData.RemainingChargeTimeMins;
    //}
    //else
    //{
    //    data[6] = 0; // adapter uses 0xff from start
    //    data[7] = 0; // adapter uses 0xff from start
    //}
}

void ChademoCharger::SendCanMessages()
{
//    if (_canSend)
    {
        can_transmit_blocking(CAN1, 0x108, false, false, 8, _msg108.bytes);
        _global.cha108++;
        _global.cha108dur = system_millis - _global.cha108last;
        _global.cha108last = system_millis;

        can_transmit_blocking(CAN1, 0x109, false, false, 8, _msg109.bytes);
        _global.cha109++;
        _global.cha109dur = system_millis - _global.cha109last;
        _global.cha109last = system_millis;
    }
}

const char* ChademoCharger::GetStateName()
{
    return _stateNames[_state];
};

void ChademoCharger::SetCcsData(uint16_t maxV, uint16_t maxA, uint16_t outV, uint16_t outA)
{
    _chargerData.AvailableOutputVoltage = maxV;

    if (_chargerData.ThresholdVoltage == 0)
        _chargerData.ThresholdVoltage = _chargerData.AvailableOutputVoltage;

    _chargerData.CcsAvailableOutputCurrent = clampToUint8(maxA);
    if (_chargerData.CcsAvailableOutputCurrent > ADAPTER_MAX_AMPS)
        _chargerData.CcsAvailableOutputCurrent = ADAPTER_MAX_AMPS;

    _chargerData.OutputVoltage = outV;
    _chargerData.OutputCurrent = clampToUint8(outA);

//    _chargerData.AvailableWatts = _chargerData.AvailableOutputVoltage * _chargerData.AvailableOutputCurrent;

//    CalcChargerThreasholdVoltage();
};

void ChademoCharger::ExtractAndSetCcsData()
{
    // mirror these values (Change method is only called for some params...)
    SetCcsData(
        Param::GetInt(Param::EvseMaxVoltage),
        Param::GetInt(Param::EvseMaxCurrent),
        Param::GetInt(Param::EvseVoltage),
        Param::GetInt(Param::EvseCurrent)
    );
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
