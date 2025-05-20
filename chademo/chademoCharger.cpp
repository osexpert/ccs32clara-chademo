
#include <string.h>
#include <stdint.h>
#include "chademoCharger.h"
#include "params.h"

#define LAST_ASKING_FOR_AMPS_TIMEOUT 10 // 10 x 100ms = 1sec.

void ChademoCharger::RunStateMachine()
{
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
        if (_carData.MaxChargeVoltage == 0 || _chargerData.AvailableOutputVoltage < _carData.MaxChargeVoltage)
            _chargerData.ThresholdVoltage = _chargerData.AvailableOutputVoltage;
        else
            _chargerData.ThresholdVoltage = _carData.MaxChargeVoltage;
    }


    if (_state == ChargerState::Start)
    {
        printf("cha: start\r\n");

        SetSwitchD1(true);

        SetState(ChargerState::WaitForChargerReady);
    }
    else if (_state == ChargerState::WaitForChargerReady)
    {
        if (IsChargerReady())
        {
            SetState(ChargerState::WaitForCarSwitchK);
        }
    }
    else if (_state == ChargerState::WaitForCarSwitchK)
    {
        if (GetSwitchK() == true)
        {
            // spec: car should not update MaxChargingTimeMins after SwitchK, so take it here
            // Take car as initial value and countdown the minutes
            _chargerData.RemainingChargeTimeMins = _carData.MaxChargingTimeMins;

            SetState(ChargerState::WaitForCarReadyToCharge);
        }
    }
    else if (_state == ChargerState::WaitForCarReadyToCharge)
    {
        if (has_flag(_carData.Status, CarStatus::CAR_STATUS_READY_TO_CHARGE))
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

        SetState(ChargerState::WaitForChargerHot);
    }
    else if (_state == ChargerState::WaitForChargerHot)
    {
        if (IsPreChargeDone_PowerDeliveryOk_AdapterContactorClosed_Hot())
        {
            // This means the charger has its voltage at nominal voltage we gave it and is ready to charge

            // this will give the car the 12v it needs to acticate contactors
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
            NotifyAdapterGpioStuffAfterContactorClosed();

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

            NotifyCarAskingForAmps_ChargingStarted_ChargerShouldStartDeliveringAmps();

            SetState(ChargerState::ChargingLoop);
        }
    }
    else if (_state == ChargerState::ChargingLoop)
    {
        printf("cha: charging\r\n");

        // Spec: k-signal and CAR_NOT_READY_TO_CHARGE both exist to make sure at least one of them reach the charger in case of cable error.

        StopReason stopReason = StopReason::NONE;
        //if (_carData.AskingAmps == 0) stopReason |= StopReason.CAR_ASK_FOR_ZERO_AMPS; no....this is not a valid reason!
        if (has_flag(_carData.Status, CarStatus::CAR_STATUS_READY_TO_CHARGE) == false) set_flag(&stopReason, StopReason::CAR_NOT_READY_TO_CHARGE);
        if (has_flag(_carData.Status, CarStatus::CAR_STATUS_NOT_IN_PARK)) set_flag(&stopReason, StopReason::CAR_NOT_IN_PARK);
        if (GetSwitchK() == false) set_flag(&stopReason, StopReason::CAR_K_OFF);
        if (IsChargingStoppedByCharger()) set_flag(&stopReason, StopReason::CHARGER);
        if (IsChargingStoppedByAdapter()) set_flag(&stopReason, StopReason::ADAPTER_STOP_BUTTON);
        if (_carData.SinceLastAskingAmpsCounter++ > LAST_ASKING_FOR_AMPS_TIMEOUT) set_flag(&stopReason, StopReason::CAR_CAN_AMPS_TIMEOUT);

        if (stopReason != StopReason::NONE)
        {
            printf("cha: Stopping: 0x%x\r\n", stopReason);

            // Checking for State >= ChargerState.Stopping_WaitForLowAmpsDelivered is probably better if we need to know we are in this state, instead of mutating the car data?
            //_carData.AskingAmps = 0;

            // TODO: reason the charger stopped?
            set_flag(&_chargerData.Status, ChargerStatus::CHARGER_STATUS_STOPPED);

            // reset countdown?
            _chargerData.RemainingChargeTimeMins = 0;

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
            //StopVoltageDelivery();

            // The log take 2 seconds from remove of CHARGER_STATUS_CHARGING to car log showing CAR_STATUS_CONTACTOR_OPEN (yes, it used this tme to do welding detection)

            SetState(ChargerState::Stopping_WaitForCarContactorsOpen);
        }
    }
    else if (_state == ChargerState::Stopping_WaitForCarContactorsOpen)
    {
        // The car will open the contactor by itself, when amps drop <= 5 and welding detection done (based on what we told it in M108 CarWeldingDetection)
        if (has_flag(_carData.Status, CarStatus::CAR_STATUS_CONTACTOR_OPEN_WELDING_DETECTION_DONE))
        {
            SetSwitchD2(false);

            // Since adapter D1 does nothing, its no need to wait...
            //_delayCycles = 5; // spec says, after setting D2:false wait 0.5sec before setting D1:false

            SetState(ChargerState::Stopping_WaitForLowVoltsDelivered);
        }
    }
    else if (_state == ChargerState::Stopping_WaitForLowVoltsDelivered)
    {
        // charger output voltage should drop <= 10 before plug can be unlocked.
        if (_chargerData.OutputVoltage <= 10)
        {
            SetState(ChargerState::Stopping_UnlockPlug);
        }
     }
    else if (_state == ChargerState::Stopping_UnlockPlug)
    {
        SetSwitchD1(false);

        // safe to unlock plug
        LockChargingPlug(false);
        // remove plug locked flag
        clear_flag(&_chargerData.Status, ChargerStatus::CHARGER_STATUS_PLUG_LOCKED);

        SetState(ChargerState::End);
    }
    else if (_state == ChargerState::End)
    {
        // nop
        printf("cha: the end\r\n");
    }

    if (_logCounter++ > 10)
    {
        float f = _carData.BatteryCapacityKwh + 0.5f; // round up
        if (f > 255.0f)
            f = 255.0f;
        uint8_t cap = static_cast<uint8_t>(f);

        // every second
        printf("cha: state:%d, charger: avail:%dV,%dA out:%dV,%dA rem_t:%dm weld:%d thres=%dV st=0x%x  car: ask:%dA cap=%dkWh rv=%d rate:%d est_t:%dm err:0x%x max:%dV max_t:%dm min:%dA soc:%d% st:0x%x target:%dV \r\n",
            _state,
            _chargerData.AvailableOutputVoltage,
            _chargerData.AvailableOutputCurrent,
            _chargerData.OutputVoltage,
            _chargerData.OutputCurrent,
            _chargerData.RemainingChargeTimeMins,
            _chargerData.SupportWeldingDetection,
            _chargerData.ThresholdVoltage,
            _chargerData.Status,

            _carData.AskingAmps,
            cap,
            _carData.ChademoRawVersion,
            _carData.ChargingRateReferenceConstant,
            _carData.EstimatedChargingTimeMins,
            _carData.Faults,
            _carData.MaxChargeVoltage, 
            _carData.MaxChargingTimeMins,
            _carData.MinimumChargeCurrent,
            _carData.SocPercent,
            _carData.Status,
            _carData.TargetVoltage
            );

        _logCounter = 0;
    }
}


void ChademoCharger::SendCanMessages()
{
    bool ext = false;                  // Standard frame (not extended)
    bool rtr = false;                  // Data frame (not remote request)
    uint8_t length = 8;                // Data length (0–8)
    uint8_t data[8];

    data[0] = _chargerData.SupportWeldingDetection;

    data[1] = _chargerData.AvailableOutputVoltage & 0xFF;
    data[2] = (_chargerData.AvailableOutputVoltage >> 8) & 0xFF; // upper byte

    data[3] = _chargerData.AvailableOutputCurrent;
    // Threshold voltage for terminating the charging process to protect the car battery
    // BUT WHY IS THE CHARGER SENDING THIS TO THE CAR?????
    data[4] = _chargerData.ThresholdVoltage & 0xFF;
    data[5] = (_chargerData.ThresholdVoltage >> 8) & 0xFF;

    CanSend(0x108, ext, rtr, length, data);



    data[0] = _chargerData.ChademoRawVersion;// ChademoRawVersion 1:0.9 2:1.0 etc.

    data[1] = _chargerData.OutputVoltage & 0xFF;
    data[2] = (_chargerData.OutputVoltage >> 8) & 0xFF;

    data[3] = _chargerData.OutputCurrent;

    //DischargeCompatitible = data[4] == 1;
    data[5] = _chargerData.Status;

    data[6] = 0xFF; //RemainingChargeTime10Sec
    data[7] = _chargerData.RemainingChargeTimeMins;

    CanSend(0x109, ext, rtr, length, data);
}

void ChademoCharger::HandleChademoMessage(uint32_t id, uint8_t* data)
{
    // fixme: we may need to use the params before k-switch...
    //bool paramsValid = CarDataCanBeUsedSafely();

    if (id == 0x100)
    {
        _carData.MinimumChargeCurrent = data[0];
        _carData.MaxChargeVoltage = (uint16_t)(data[4] | data[5] << 8);
        // Allways 100%?
        _carData.ChargingRateReferenceConstant = data[6]; // Charged rate reference constant, 100% fixed

        Param::Set(Param::MaxVoltage, _carData.MaxChargeVoltage);
    }
    else if (id == 0x101)
    {
        uint8_t maxChargingTime10Sec = data[1];
        uint8_t maxChargingTimeMins = data[2]; // = 90; //ask for how long of a charge? Charging will be forceably stopped if we hit this time (from any side?)

        if (maxChargingTime10Sec == 0xff)
            _carData.MaxChargingTimeMins = maxChargingTimeMins;
        else
            _carData.MaxChargingTimeMins = (uint8_t)(maxChargingTime10Sec / 6);

        // estimated charging time mins
        // Added in Chademo 1.0.1 so the (old) leaf does not exmit this? Seems to always be 0?
        _carData.EstimatedChargingTimeMins = data[3];// = 60; //how long we think the charge will actually take

        // 0 first, then 55, then correct :-)
        _carData.BatteryCapacityKwh = (float)(data[5] | data[6] << 8) * 0.11f;
    }
    else if (id == 0x102)
    {
        _carData.ChademoRawVersion = data[0];// 1: v0.9, 2: v1.0

        // This is the charging voltage? I think so...
        _carData.TargetVoltage = (uint16_t)(data[1] | data[2] << 8);

        // Leaf 410 typical
        Param::Set(Param::TargetVoltage, _carData.TargetVoltage);
        Param::Set(Param::BatteryVoltage, _carData.TargetVoltage); // not really correct but...
 
        _carData.AskingAmps = data[3];
        // limit to adapter max
        if (_carData.AskingAmps > ADAPTER_MAX_AMPS)
            _carData.AskingAmps = ADAPTER_MAX_AMPS;

        // for timeout
        _carData.SinceLastAskingAmpsCounter = 0;

        Param::Set(Param::EVTargetCurrent, _carData.AskingAmps);


        _carData.Faults = (CarFaults)data[4];
        _carData.Status = (CarStatus)data[5];
        //uint8_t kiloWattHours = msg.data[6];

        // Start as 3, then 1, then to the real value:-)
        _carData.SocPercent = data[6];

        Param::Set(Param::soc, _carData.SocPercent);
    }
}
