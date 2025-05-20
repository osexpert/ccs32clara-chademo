

/* Global Functions */
//#ifdef __cplusplus
//
//extern "C" {
//#endif
//	/* pev state machine */
//	extern void HandleChademoMessage(uint8_t* msg);
//
//
//
//
//#ifdef __cplusplus
//}
//
//#endif

#include <type_traits>
#include "printf.h"

#define ADAPTER_MAX_AMPS 200

template <typename E>
typename std::enable_if<std::is_enum<E>::value>::type
set_flag(E* value, E flag) {
    using U = typename std::underlying_type<E>::type;
    *value = static_cast<E>(static_cast<U>(*value) | static_cast<U>(flag));
}

template <typename E>
typename std::enable_if<std::is_enum<E>::value>::type
clear_flag(E* value, E flag) {
    using U = typename std::underlying_type<E>::type;
    *value = static_cast<E>(static_cast<U>(*value) & ~static_cast<U>(flag));
}

template <typename E>
constexpr typename std::enable_if<std::is_enum<E>::value, bool>::type
has_flag(E value, E flag) {
    using U = typename std::underlying_type<E>::type;
    return (static_cast<U>(value) & static_cast<U>(flag)) != 0;
}


enum CarFaults
{
    CAR_FAULT_OVER_VOLT = 1,
    CAR_FAULT_UNDER_VOLT = 2,
    CAR_FAULT_DEV_AMPS = 4,
    CAR_FAULT_OVER_TEMP = 8,
    CAR_FAULT_DEV_VOLT = 16
};


enum CarStatus
{
    CAR_STATUS_READY_TO_CHARGE = 1,// charging enabled/allowed
    CAR_STATUS_NOT_IN_PARK = 2, // shifter not in safe state (0: park 1: other)
    CAR_STATUS_ERROR = 4, // car did something dumb (fault caused by the car or the charger and detected by the car)

    /// <summary>
    /// Set the flag to 0 when the vehicle relay is closed, and set as 1 after the termination of welding detection.
    /// </summary>
    CAR_STATUS_CONTACTOR_OPEN_WELDING_DETECTION_DONE = 8, // main contactor open (Special: 0: During contact sticking detection, 1: Contact sticking detection completed). Called StatusVehicle in docs!!!
   

    CAR_STATUS_STOP_BEFORE_CHARGING = 16, // charger stop before charging (changed my mind:-)

    /// <summary>
    /// Vehicle reports a fault specifically in its charging system. (Bit 6)?
    /// Possibly battery cooling in progress??? Or hot battery in general?
    /// </summary>
    CAR_STATUS_UNK_64 = 64,

    CAR_STATUS_DISCHARGE_COMPATIBLE = 128, // car is V2X compatible (can deliver power to grid)
};

enum ChargerStatus
{
    /// <summary>
    /// during rundown: This is tied 1:1 with OutputCurrent > 0. Meaning we can be stopped, but still charging since amps > 0.
    /// During startup, CHARGER_STATUS_CHARGING is set and then amps are still 0.
    /// </summary>
    CHARGER_STATUS_CHARGING = 1, // 0: standby 1: charging (power transfer from charger)

    CHARGER_STATUS_ERROR = 2, // something went wrong (fault caused by (or inside) the charger)
    CHARGER_STATUS_PLUG_LOCKED = 4, // connector is currently locked (electromagnetic lock, plug locked into the car)
    CHARGER_STATUS_INCOMPAT = 8,// parameters between vehicle and charger not compatible (battery incompatible?)
    CHARGER_STATUS_CAR_ERROR = 16, // problem with the car, such as improper connection (or something wrong with the battery?)
    CHARGER_STATUS_STOPPED = 32, //charger is stopped (charger shutdown or end of charging). this is also initially set to stop, before charging.
};

enum StopReason
{
    NONE = 0,
    CAR_CAN_AMPS_TIMEOUT = 1,
    //            CAR_ASK_FOR_ZERO_AMPS = 2, // asking for 0 amps
    CAR_NOT_READY_TO_CHARGE = 4,
    CAR_NOT_IN_PARK = 8,
    CAR_K_OFF = 16,
    /// <summary>
    /// Typically the ccs charger want us to stop
    /// </summary>
    CHARGER = 32,
    ADAPTER_STOP_BUTTON = 64,
};

enum ChargerState
{
    /// <summary>
    /// First time Charger Available volta and current is set
    /// </summary>
    Start,
//    WaitForChargerAvailableVoltAndCurrent,
    //SendCarStartSignal, // D1
    WaitForChargerReady,
    WaitForCarSwitchK,
    WaitForCarReadyToCharge,
    CarReadyToCharge, // LockPlug
    WaitForChargerHot, // D2
    WaitForCarContactorsClosed,
    WaitForCarAskingAmps,
    ChargingLoop,
    Stopping_WaitForLowAmpsDelivered,
    Stopping_WaitForCarContactorsOpen, // D2 OFF
    Stopping_WaitForLowVoltsDelivered,
    Stopping_UnlockPlug, // wait 500ms, D1 OFF, UnlockPlug
    End
};


struct CarData
{
    // valid after kswitch
    uint16_t MaxChargeVoltage;// = 435;

    uint8_t MaxChargingTimeMins;

    // Valid after kSwitch
    uint16_t TargetVoltage;

    uint16_t SinceLastAskingAmpsCounter;

    //ushort BattNominalVoltage = > (ushort)Program.get_estimated_nominal_voltage(TargetVoltage);
    uint8_t AskingAmps;

    // valid after k-switch
    // soc = SocPercent / ChargingRateReferenceConstant * 100 ... .weird stuff... I guess if ChargingRateReferenceConstant
    uint8_t SocPercent;

    /// <summary>
    /// To be safe, only use SOC,MaxChargingTimeMins,BatteryCapacityKwh after car is ready to charge (before they can have invalid values).
    /// </summary>
    CarStatus Status;
    CarFaults Faults;
    float BatteryCapacityKwh;

    // ChargingRate reference constant
    // valid after k-switch
    uint8_t ChargingRateReferenceConstant;

    uint8_t MinimumChargeCurrent
        ;
    uint8_t ChademoRawVersion;
    uint8_t EstimatedChargingTimeMins;
    //uint8_t MaxChargingTime10Sec;
};

struct ChargerData
{
    uint8_t ChademoRawVersion = 2; // 2: chademo 1.0

    /// <summary>
    /// Initial status is stopped
    /// </summary>
    ChargerStatus Status = ChargerStatus::CHARGER_STATUS_STOPPED;

    uint16_t AvailableOutputVoltage; //??

    /// <summary>
    /// If true, the charger support helping the car to do welding detection (by lowering the voltage)
    /// Not sure if 1 is right here...
    /// But I think 1 if we assume charger can drop its voltage fast, after charging was stopped
    /// </summary>
    bool SupportWeldingDetection = 1;

    uint8_t AvailableOutputCurrent;// ??

    uint16_t OutputCurrent;
    uint16_t OutputVoltage;

    // initial value from car, charger count it down
    uint8_t RemainingChargeTimeMins;


    uint16_t ThresholdVoltage;

};


class ChademoCharger
{
public:
    ChargerState _state = ChargerState::Start;

    CarData _carData;
    ChargerData _chargerData;

    /// <summary>
    /// Iyitially some values are wrong, such as soc, capacity etc. Spec also say they car is allowed to change cenrtain stuff until k-switch,
    /// but not after, so assuming this is the point where all params will be valid.
    /// </summary>
    /// <returns></returns>
    /*bool CarDataCanBeUsedSafely()
    {
        return _state > ChargerState::CarReadyToCharge;
    }*/

    void SendCanMessages();

    void RunStateMachine(void);

	void HandleChademoMessage(uint32_t id, uint8_t* data);

    void SetState(ChargerState newState)
    {
        printf("cha: enter state %d (%s)\r\n", newState, GetStateName());

        _state = newState;
    };

    //bool ChargerStopBeforeCharging(void) 
    //{
    //    // don't know how to detect.
    //    return false;
    //};

    void SetSwitchD1(bool set) 
    {
        (void)set; // unused warn
        // adapter seem to do nothing here...
    };
    void SetSwitchD2(bool set);

    //void StopVoltageDelivery() 
    //{
    //};

    bool IsChargingStoppedByAdapter();
    
    void SetChargerSetMaxCurrent(uint16_t maxA)
    {
        _chargerData.AvailableOutputCurrent = maxA;
        if (_chargerData.AvailableOutputCurrent > ADAPTER_MAX_AMPS)
            _chargerData.AvailableOutputCurrent = ADAPTER_MAX_AMPS;
    };
    void SetChargerSetMaxVoltage(uint16_t maxV)
    {
        _chargerData.AvailableOutputVoltage = maxV;
    };

   // bool AdapterStopBeforeCharging();
    void StopPowerDelivery();
    bool GetSwitchK();

    int _delayCycles;
    int _logCounter;
    bool _locked;

    bool IsChargingPlugLocked()
    {
        return _locked;
    };

    void LockChargingPlug(bool lock) 
    {
        _locked = lock;
    };

    bool IsChargerReady()
    {
        // kickoff: Start of precharge??
        return true;
    };

    void NotifyAdapterGpioStuffAfterContactorClosed();

    void PerformInsulationTest() 
    {
        // NOP
    };

    bool IsPreChargeDone_PowerDeliveryOk_AdapterContactorClosed_Hot()
    {
        // TODO: can we wait for precharge done?
        //kickoff: end of precharge or start of powerDelivery
        return true;
    };
    void NotifyCarAskingForAmps_ChargingStarted_ChargerShouldStartDeliveringAmps()
    {
    };
    bool IsChargingStoppedByCharger();
    

    void CanSend(int id, bool ext, bool rem, int len, uint8_t* data);

    const char* GetStateName()
    {
        switch (_state)
        {
        case ChargerState::Start: return "Start";
        case ChargerState::WaitForChargerReady: return "WaitForChargerReady";
        case ChargerState::WaitForCarSwitchK: return "WaitForCarSwitchK";
        case ChargerState::WaitForCarReadyToCharge: return "WaitForCarReadyToCharge";
        case ChargerState::CarReadyToCharge: return "CarReadyToCharge";
        case ChargerState::WaitForChargerHot: return "WaitForChargerHot";
        case ChargerState::WaitForCarContactorsClosed: return "WaitForCarContactorsClosed";
        case ChargerState::WaitForCarAskingAmps: return "WaitForCarAskingAmps";
        case ChargerState::ChargingLoop: return "ChargingLoop";
        case ChargerState::Stopping_WaitForLowAmpsDelivered: return "Stopping_WaitForLowAmpsDelivered";
        case ChargerState::Stopping_WaitForCarContactorsOpen: return "Stopping_WaitForCarContactorsOpen";
        case ChargerState::Stopping_WaitForLowVoltsDelivered: return "Stopping_WaitForLowVoltsDelivered";
        case ChargerState::Stopping_UnlockPlug: return "Stopping_UnlockPlug";
        case ChargerState::End: return "End";
        }

        return "Unknown";
    };
};
