
#include <string.h>
#include <stdint.h>
#include "chademoCharger.h"
#include "params.h"
#include "my_fp.h"
#include "main.h"

#define LOW_BYTE(x)  ((uint8_t)((x) & 0xFF))
#define HIGH_BYTE(x) ((uint8_t)(((x) >> 8) & 0xFF))

extern global_data _global;

#define LAST_ASKING_FOR_AMPS_TIMEOUT_CYCLES 10 // 10 x 100ms = 1sec.
#define CYCLES_PER_SEC 10 // 100ms ticks


void ChademoCharger::Run()
{
    SetChargerData();

   // ReadPendingCanMessages();

    //uint16_t timeout_s = _stateTimeoutsSec[_state];
//if (timeout_s > 0 && (timeout_s * 10) > _cyclesInState)
//{
//    printf("cha: timeout in state %d/%s. What todo....\r\n", _state, GetStateName());
// 
//     TODO: timeout may be needed to make sure we make it to the plug unlocked state, because this is important for safety and auto off.
//     TODO: so if we get stuck on some state, we can not rely on plug locked/unlocker for chademo
//     TODO: also unlocked is a nice check that chademo is rundown completely
//}

    RunStateMachine();

    Log();
}

void ChademoCharger::RunSend()
{
    // this was not being called at all before...but lets try the new logic:-)
    // only send when d1 is set
    if (_state >= WaitForCarReadyToCharge && _state < End)
        SendCanMessages();
}

bool ChademoCharger::IsTimeoutSec(uint16_t max_sec)
{
    if (_cyclesInState > (max_sec * CYCLES_PER_SEC))
    {
        printf("cha: Timeout in %d/%s (max:%dsec)\r\n", _state, GetStateName(), max_sec);
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

    if (_delayCycles > 0)
    {
        _delayCycles--;
        return;
    }

    if (_state < ChargerState::ChargingLoop)
    {
        if (has_flag(_carData.Status, CarStatus::CAR_STATUS_STOP_BEFORE_CHARGING))
        {
            printf("cha: Car stopped before starting\r\n");
            // cancel before start. go straight to rundown.
            SetState(ChargerState::Stopping_WaitForLowAmpsDelivered);
        }
        if (has_flag(_carData.Status, CarStatus::CAR_STATUS_ERROR))
        {
            printf("cha: Car status error\r\n");
            // cancel before start. go straight to rundown.
            SetState(ChargerState::Stopping_WaitForLowAmpsDelivered);
        }
        if (IsChargingStoppedByCharger())
        {
            printf("cha: Charger stopped before starting\r\n");
            // cancel before start. go straight to rundown.
            SetState(ChargerState::Stopping_WaitForLowAmpsDelivered);
        }
        if (IsChargingStoppedByAdapter())
        {
            printf("cha: Adapter stopped before starting\r\n");
            // cancel before start. go straight to rundown.
            SetState(ChargerState::Stopping_WaitForLowAmpsDelivered);
        }

        // set to the minimum of AvailableOutputVoltage and MaxChargeVoltage, until charging start.
        // special case for MaxChargeVoltage == 0 if no message recieved from car yet
        if (/*_carData.MaxChargeVoltage == 0 ||*/ _chargerData.AvailableOutputVoltage < _carData.MaxChargeVoltage)
            _chargerData.ThresholdVoltage = _chargerData.AvailableOutputVoltage;
        else
            _chargerData.ThresholdVoltage = _carData.MaxChargeVoltage;

        // Take car as initial value and countdown the minutes
        _chargerData.RemainingChargeTimeMins = _carData.MaxChargingTimeMins;
    }

    // Car seem to immediately go into stopped before starting, i am guessing if available volts and amps are 0 from the start. So do it in two steps...
    if (_state == ChargerState::Start)
    {
        printf("cha: start\r\n");

        SetSwitchD1(true); // will trigger car sending can

        SetState(ChargerState::WaitForCarMaxAndTargetVolts);
    }
    else if (_state == ChargerState::WaitForCarMaxAndTargetVolts)
    {
        if (_carData.MaxChargeVoltage > 0 && _carData.TargetVoltage > 0)
        {
            // we got volts! done with can for now. turn off to make car happy, reset its states, not timeout, stop before starting etc.
            // TODO: was not sending CAN earier........................it may also be the prioblem........................anyways....this seems like a better solution...
            // we can't wait here for more tha 3-4 sec before er get error.
//            SetSwitchD1(false);

            SetState(ChargerState::WaitForChargerAvailableVoltsAndAmps);

            // clear misc stuff, ready for next restart (or not)
           // _carData.Faults = CAR_FAULT_NONE;
         //   _carData.Status = CAR_STATUS_NONE;
        }
    }
    else if (_state == ChargerState::WaitForChargerAvailableVoltsAndAmps)
    {
        if (_chargerData.AvailableOutputVoltage > 0 && _chargerData.AvailableOutputCurrent > 0)
        {
            // we have gotten so far we got availables back from ccs. We can then enable can again (or is it too soon??? it may be. seems like i take 16 sec. until we reach CurrentDemandReq,
            // it may be too much, but from Using-OCPP-with-CHAdeMO.pdf it seems like T-time 22+6 seconds, so should be ok...)
//            SetSwitchD1(true); // will trigger car sending can

            SetState(ChargerState::WaitForCarReadyToCharge);
        }
    }
    else if (_state == ChargerState::WaitForCarReadyToCharge)
    {
        // simulate what I saw in some canlog...alternating
        /*if (_chargerData.OutputVoltage == 0)
            _chargerData.OutputVoltage = 1;
        else if (_chargerData.OutputVoltage == 1)
            _chargerData.OutputVoltage = 0;*/

        bool canEna = has_flag(_carData.Status, CarStatus::CAR_STATUS_READY_TO_CHARGE);
        bool switchEna = GetSwitchK() == true;

        if ((canEna || switchEna) && (canEna != switchEna))
            printf("cha: only one of can %d and SwitchK %d enabled", canEna, switchEna);

        if (canEna && switchEna)
        {
            SetState(ChargerState::CarReadyToCharge);
        }
    }
    else if (_state == ChargerState::CarReadyToCharge)
    {
        // Mismatch between spec and log:
        // Spec: Lock charging connector -> Insulation test
        // Log: charger uses ca 4 seconds to gradually increase the voltage. Then it locks the plug and continue the gradually increase in voltage in 4 more seconds?

        LockChargingPlug(true);
        // Add artificial delay here?
        set_flag(&_chargerData.Status, ChargerStatus::CHARGER_STATUS_PLUG_LOCKED);

        // Ramp-up/down: 0volt -> max volt -> 0volt (allthou logs show it often stays high / lowers to batt/target)
        // End of insulation test: measured volt <= 20v. BUT logs tell a different story...volt is kept after insultayion test, only lowerd to eg. 380 (from eg. 480)
        PerformInsulationTest();

        SetState(ChargerState::WaitForChargerLive);
    }
    else if (_state == ChargerState::WaitForChargerLive)
    {
        if (IsChargerLive())
        {
            // This means the charger has its voltage at "battery" voltage we gave it and is ready to charge

            // this will trigger car to acticate contactors
            SetSwitchD2(true);

            SetState(ChargerState::WaitForCarContactorsClosed);
        }
    }
    else if (_state == ChargerState::WaitForCarContactorsClosed)
    {
        if (has_flag(_carData.Status, CarStatus::CAR_STATUS_CONTACTOR_OPEN_WELDING_DETECTION_DONE) == false)
        {
            // Contactors closed

            // Adapter set 2 GPIO's at this point. They do not fit into the spec/flow chart in any way...so its not easy to tell what they are.
            NotifyCarContactorsClosed();

            // At this point, deliveredVolts should match battery + 10v?
            // No...I think it should be 0 and gradually increased.

            // Next, AskingAmps is going to increase. It take approx 2 seconds between the transition from 0 to 2 amps in the log.
            // Only after this does the charger remove its CHARGER_STATUS_STOP

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
        // Spec: k-signal and CAR_NOT_READY_TO_CHARGE both exist to make sure at least one of them reach the charger in case of cable error.

        StopReason stopReason = StopReason::NONE;
        //if (_carData.AskingAmps == 0) stopReason |= StopReason.CAR_ASK_FOR_ZERO_AMPS; no....this is not a valid reason!
        if (has_flag(_carData.Status, CarStatus::CAR_STATUS_READY_TO_CHARGE) == false) set_flag(&stopReason, StopReason::CAR_NOT_READY_TO_CHARGE);
        if (has_flag(_carData.Status, CarStatus::CAR_STATUS_NOT_IN_PARK)) set_flag(&stopReason, StopReason::CAR_NOT_IN_PARK);
        if (has_flag(_carData.Status, CarStatus::CAR_STATUS_ERROR)) set_flag(&stopReason, StopReason::CAR_ERROR);
        if (GetSwitchK() == false) set_flag(&stopReason, StopReason::CAR_K_OFF);
        if (IsChargingStoppedByCharger()) set_flag(&stopReason, StopReason::CHARGER);
        if (IsChargingStoppedByAdapter()) set_flag(&stopReason, StopReason::ADAPTER_STOP_BUTTON);
        if (_carData.CyclesSinceLastAskingAmps++ > LAST_ASKING_FOR_AMPS_TIMEOUT_CYCLES) set_flag(&stopReason, StopReason::CAR_CAN_AMPS_TIMEOUT);

        if (stopReason != StopReason::NONE)
        {
            printf("cha: Stopping: 0x%x\r\n", stopReason);

            // Checking for State >= ChargerState.Stopping_WaitForLowAmpsDelivered is probably better if we need to know we are in this state, instead of mutating the car data?
            _carData.AskingAmps = 0;

            // TODO: reason the charger stopped?
            set_flag(&_chargerData.Status, ChargerStatus::CHARGER_STATUS_STOPPED);

            // make sure ccs stop delivering amps and turn down volts (if stop initiated by adapter or car)
            StopPowerDelivery();

            SetState(ChargerState::Stopping_WaitForLowAmpsDelivered);
        }
    }
    else if (_state == ChargerState::Stopping_WaitForLowAmpsDelivered)
    {
        // Spec says <= 5 amps. Weird but true... Why not 0 or 1?
        // TODO: IS this only something the car need to do? According to spec...yes. But the log show that CHARGER_STATUS_CHARGING flag is only driven by OutputCurrent...
        if (_chargerData.OutputCurrent <= 5)
        {
            _chargerData.RemainingChargeTimeMins = 0;

            // this just mirror the OutputCurrent during stop
            clear_flag(&_chargerData.Status, ChargerStatus::CHARGER_STATUS_CHARGING);

            // spec: after this, car has 4 seconds max to perform welding detection (turn contactors off, on, off)
            // If charger tell car it support welding detection, charger should help the car: (its the car that perform the welding detection)
            // The circuit voltage shall drop below 25 % of circuit voltage, which is monitored before EV
            // contactors are opened, within 1 s after the charger terminates charging output and EV contactors
            // are opened.
             
            // FIXME: what is the charger really supposed to do to help the car with WD?????
            // Charger should drop its voltage below 25 % of circuit voltage within 1 second. I guess this is the help.
            // I a charger is not helping WD, it can not drop its voltage this quickly, in case the car will probably skip WD (after 4 sec).
            // It may be hard to know up front if ccs can drop volt this quickly.
            StopVoltageDelivery();

            // The log take 2 seconds from remove of CHARGER_STATUS_CHARGING to car log showing CAR_STATUS_CONTACTOR_OPEN (yes, it used this tme to do welding detection)

            SetState(ChargerState::Stopping_WaitForCarContactorsOpen);
        }
    }
    else if (_state == ChargerState::Stopping_WaitForCarContactorsOpen)
    {
        // The car will open the contactor by itself, when amps drop <= 5 and welding detection done (based on what we told it in M108 CarWeldingDetection)
        // TODO: if can-bus broke down, we will never get past this...so only wait max 4 sec. (40 * 100ms)
        if (has_flag(_carData.Status, CarStatus::CAR_STATUS_CONTACTOR_OPEN_WELDING_DETECTION_DONE) 
            //|| IsTimeoutSec(4) // if can get stuck, make sure we can progress stopping TODO: ADD timeout only if needed??
            )
        {
            SetSwitchD2(false);

            // spec says, after setting D2:false wait 0.5sec before setting D1:false (so 5 cycles)
            SetState(ChargerState::Stopping_WaitForLowVoltsDelivered, 5);
        }
    }
    else if (_state == ChargerState::Stopping_WaitForLowVoltsDelivered)
    {
        // charger output voltage should drop <= 10 before plug can be unlocked.
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

        // remove plug locked flag
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

void ChademoCharger::SetState(ChargerState newState, int delay_cycles)
{
    printf("cha: enter state %d/%s\r\n", newState, _stateNames[newState]);
    _state = newState;
    _cyclesInState = 0;
    _delayCycles = delay_cycles;

    // force log on state change
    Log(true);
};

void ChademoCharger::Log(bool force)
{
    if (force || _logCycleCounter++ > 10 )//|| _carData != _lastLoggedCarData)
    {
        //if (_carData.ChargingRateReferenceConstant != 100)
        {
          //  printf("Alt. soc %d%%\r\n", (int)((float)_carData.SocPercent / _carData.ChargingRateReferenceConstant * 100.0f));
        }

        int alt_soc = (int)((float)_carData.SocPercent / _carData.ChargingRateReferenceConstant * 100.0f);

        // every second or when forced
        printf("cha: state:%d/%s cycles:%d charger: out:%dV/%dA avail:%dV/%dA rem_t:%dm weld:%d thres=%dV st=0x%x car: ask:%dA cap=%fkWh rv=%d rate:%d est_t:%dm err:0x%x max:%dV max_t:%dm min:%dA soc:%d%% st:0x%x target:%dV alt_soc:%d%%\r\n",
            _state,
            GetStateName(),
            _cyclesInState,
            _chargerData.OutputVoltage,
            _chargerData.OutputCurrent,
            _chargerData.AvailableOutputVoltage,
            _chargerData.AvailableOutputCurrent,
            _chargerData.RemainingChargeTimeMins,
            _chargerData.SupportWeldingDetection,
            _chargerData.ThresholdVoltage,
            _chargerData.Status,

            _carData.AskingAmps,
            FP_FROMFLT(_carData.BatteryCapacityKwh),
            _carData.ChademoRawVersion,
            _carData.ChargingRateReferenceConstant,
            _carData.EstimatedChargingTimeMins,
            _carData.Faults,
            _carData.MaxChargeVoltage,
            _carData.MaxChargingTimeMins,
            _carData.MinimumChargeCurrent,
            _carData.SocPercent,
            _carData.Status,
            _carData.TargetVoltage,
            alt_soc
        );

        _logCycleCounter = 0;
        //_lastLoggedCarData = _carData;
    }
}


void ChademoCharger::SetChargerData(uint16_t maxV, uint16_t maxA, uint16_t outV, uint16_t outA)
{
    _chargerData.AvailableOutputVoltage = maxV;
    // convert from 16 to 8 bit int...default is to trunc...
    _chargerData.AvailableOutputCurrent = maxA > 0xFF ? 0xFF : maxA;
    if (_chargerData.AvailableOutputCurrent > ADAPTER_MAX_AMPS)
        _chargerData.AvailableOutputCurrent = ADAPTER_MAX_AMPS;

    _chargerData.OutputVoltage = outV;
    // convert from 16 to 8 bit int...default is to trunc... (impossible that this can be over 0xff!!!)
    _chargerData.OutputCurrent = outA > 0xFF ? 0xFF : outA;
};

//void ChademoCharger::SetChargerSetMaxVoltage(uint16_t maxV)
//{
//
//};

void ChademoCharger::SendCanMessages()
{
    uint8_t length = 8;                // Data length (0–8)
    uint8_t data[8];

    data[0] = _chargerData.SupportWeldingDetection;

    data[1] = LOW_BYTE(_chargerData.AvailableOutputVoltage);
    data[2] = HIGH_BYTE(_chargerData.AvailableOutputVoltage); // upper byte

    data[3] = _chargerData.AvailableOutputCurrent;
    // Threshold voltage for terminating the charging process to protect the car battery
    // BUT WHY IS THE CHARGER SENDING THIS TO THE CAR?????
    data[4] = LOW_BYTE(_chargerData.ThresholdVoltage);
    data[5] = HIGH_BYTE(_chargerData.ThresholdVoltage);

    data[6] = 0;
    data[7] = 0;

    CanSend(0x108, length, data);

    // Adapter uses 1...
    data[0] = _chargerData.ChademoRawVersion;// ChademoRawVersion 1:0.9 2:1.0 etc.

    data[1] = LOW_BYTE(_chargerData.OutputVoltage);
    data[2] = HIGH_BYTE(_chargerData.OutputVoltage);

    data[3] = _chargerData.OutputCurrent;

    data[4] = 0;

    //DischargeCompatitible = data[4] == 1;
    data[5] = _chargerData.Status;

    if (_chargerData.RemainingChargeTimeMins > 0)
    {
        data[6] = 0xFF; //RemainingChargeTime10Sec
        data[7] = _chargerData.RemainingChargeTimeMins;
    }
    else
    {
        data[6] = 0;
        data[7] = 0;
    }

    CanSend(0x109, length, data);
}



void ChademoCharger::HandleChademoMessage(uint32_t id, uint8_t* data, uint8_t len)
{
    if (len != 8)
    {
        printf("wrong len %d %d\r\n", len, id);
        return;
    }

    if (id == 0x100)
    {
        _global.cha100++;

        _carData.MinimumChargeCurrent = data[0];
        _carData.MaxChargeVoltage = (uint16_t)(data[4] | data[5] << 8);
        // Allways 100%? no...240 is seen
        _carData.ChargingRateReferenceConstant = data[6]; // Charged rate reference constant, 100% fixed

        Param::SetInt(Param::MaxVoltage, _carData.MaxChargeVoltage);
    }
    else if (id == 0x101)
    {
        _global.cha101++;

        uint8_t maxChargingTime10Sec = data[1];
        uint8_t maxChargingTimeMins = data[2]; // = 90; //ask for how long of a charge? Charging will be forceably stopped if we hit this time (from any side?)

        if (maxChargingTime10Sec == 0xff)
            _carData.MaxChargingTimeMins = maxChargingTimeMins;
        else
        {
            printf("seconds %d\r\n", maxChargingTime10Sec);
            _carData.MaxChargingTimeMins = (uint8_t)(maxChargingTime10Sec / 6);
        }

        // estimated charging time mins
        // Added in Chademo 1.0.1 so the (old) leaf does not exmit this? Seems to always be 0?
        _carData.EstimatedChargingTimeMins = data[3];// = 60; //how long we think the charge will actually take

        // 0 first, then 55, then correct :-) Correct after car ready?
        _carData.BatteryCapacityKwh = (float)(data[5] | data[6] << 8) * 0.11f;
    }
    else if (id == 0x102)
    {
        _global.cha102++;

        _carData.ChademoRawVersion = data[0];// 1: v0.9, 2: v1.0

        // This is the charging voltage? I think so...
        _carData.TargetVoltage = (uint16_t)(data[1] | data[2] << 8);

        // Leaf 410 typical
        Param::SetInt(Param::TargetVoltage, _carData.TargetVoltage);
        Param::SetInt(Param::BatteryVoltage, _carData.TargetVoltage - 10); // not really correct but...
 
        _carData.AskingAmps = data[3];
        // limit to adapter max
        if (_carData.AskingAmps > ADAPTER_MAX_AMPS)
            _carData.AskingAmps = ADAPTER_MAX_AMPS;

        Param::SetInt(Param::ChargeCurrent, _carData.AskingAmps);
        // for timeout
        _carData.CyclesSinceLastAskingAmps = 0;

        _carData.Faults = (CarFaults)data[4];
        _carData.Status = (CarStatus)data[5];
        //uint8_t kiloWattHours = msg.data[6];

        // Start as 3, then 1, then to the real value:-)
        _carData.SocPercent = data[6];

        // changes from very low to higher :-) i guess this is ok...
        // TODO: set only after X has happened (k-switch)
        // strange values seen sometimes...
        //if (_state >= CarReadyToCharge) //_carData.SocPercent <= 100 &&
        {
            /*Charged rate(for display) =
                Charged rate(#102.6) / Charged rate
                reference constant(#100.6) × 100*/
            
            //if (_carData.ChargingRateReferenceConstant != 100)
            //{
            //    printf("Alt. soc %d%%\r\n", (int)((float)_carData.SocPercent / _carData.ChargingRateReferenceConstant * 100.0f));
            //}

            Param::SetInt(Param::soc, _carData.SocPercent);
            // TODO: estimate battery volts?
        }
    }
}


const char* ChademoCharger::GetStateName()
{
    return _stateNames[_state];
};

void ChademoCharger::SetChargerData()
{
    // mirror these values (Change method is only called for some params...)
    SetChargerData(
        Param::GetInt(Param::EvseMaxVoltage),
        Param::GetInt(Param::EvseMaxCurrent),
        Param::GetInt(Param::EvseVoltage),
        Param::GetInt(Param::EvseCurrent)
    );
}

void ChademoCharger::StopPowerDelivery()
{
    printf("cha: StopPowerDelivery\r\n");

    Param::SetInt(Param::ChargeCurrent, 0);
    // TODO: should we disable at the end of the machine instead?
    Param::SetInt(Param::enable, false);
};

void ChademoCharger::StopVoltageDelivery()
{
    printf("cha: StopVoltageDelivery\r\n");

    Param::SetInt(Param::TargetVoltage, 0);
    //    Param::Set(Param::BatteryVoltage, 0);
        // TODO: should we disable at the end of the machine instead?
    Param::SetInt(Param::enable, false);
};

bool ChademoCharger::IsChargingStoppedByCharger()
{
    return Param::GetInt(Param::StopReason) != _stopreasons::STOP_REASON_NONE;
}
