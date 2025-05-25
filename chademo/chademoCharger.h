

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

/// <summary>
/// errors during charging, related to the battery.
/// </summary>
enum CarFaults
{
    //CAR_FAULT_NONE = 0,

    CAR_FAULT_OVER_VOLT = 1,
    CAR_FAULT_UNDER_VOLT = 2,
    CAR_FAULT_DEV_AMPS = 4,
    CAR_FAULT_OVER_TEMP = 8,
    CAR_FAULT_DEV_VOLT = 16
};


enum CarStatus
{
    //CAR_STATUS_NONE = 0,

    /// <summary>
    /// 102.5.0 Vehicle charging enabled
    /// </summary>
    CAR_STATUS_READY_TO_CHARGE = 0x1,

    /// <summary>
    /// 102.5.1 Vehicle shift position
    /// </summary>
    CAR_STATUS_NOT_IN_PARK = 0x2,

    /// <summary>
    /// 102.5.2 Charging system fault
    /// Can timeout? Other timeout? Too long/short in state?
    /// This is analog to ChargerStatus::CHARGER_STATUS_ERROR, only opposite direction
    /// </summary>
    CAR_STATUS_ERROR = 0x4,

    /// <summary>
    /// 102.5.3 Vehicle status
    /// Car initially send 0 here, kind of wrong...then it changes to 1 (OPEN).
    /// Set the flag to 0 when the vehicle relay is closed (start)
    /// and set as 1 after the termination of welding detection (end)
    /// </summary>
    CAR_STATUS_CONTACTOR_OPEN_WELDING_DETECTION_DONE = 0x8, // main contactor open (Special: 0: During contact sticking detection, 1: Contact sticking detection completed). Called StatusVehicle in docs!!!
   
    /// <summary>
    /// 102.5.4 Normal stop request before charging
    /// </summary>
    CAR_STATUS_STOP_BEFORE_CHARGING = 0x10,

    /// <summary>
    /// Vehicle reports a fault specifically in its charging system. (Bit 6)?
    /// Possibly battery cooling in progress??? Or hot battery in general?
    /// </summary>
    CAR_STATUS_UNK_64 = 0x40,

    CAR_STATUS_DISCHARGE_COMPATIBLE = 0x80, // car is V2X compatible (can deliver power to grid)
};

enum ChargerStatus
{
    /// <summary>
    /// during rundown: This is tied 1:1 with OutputCurrent > 0. Meaning we can be stopped, but still charging since amps > 0.
    /// During startup, CHARGER_STATUS_CHARGING is set and then amps are still 0.
    /// </summary>
    CHARGER_STATUS_CHARGING = 0x1, // 0: standby 1: charging (power transfer from charger)

    /// <summary>
    /// 
    /// </summary>
    CHARGER_STATUS_ERROR = 0x2, // something went wrong (fault caused by (or inside) the charger)

    CHARGER_STATUS_PLUG_LOCKED = 0x4, // connector is currently locked (electromagnetic lock, plug locked into the car)
    CHARGER_STATUS_INCOMPAT = 0x8,// parameters between vehicle and charger not compatible (battery incompatible?)

    CHARGER_STATUS_CAR_ERROR = 0x10, // can be CAN timeout, or other timeout (too short/long in state). problem with the car, such as improper connection (or something wrong with the battery?)

    CHARGER_STATUS_STOPPED = 0x20, //charger is stopped (charger shutdown or end of charging). this is also initially set to stop, before charging.
};

enum StopReason
{
    NONE = 0x0,
    CAR_CAN_AMPS_TIMEOUT = 0x1,
    //            CAR_ASK_FOR_ZERO_AMPS = 2, // asking for 0 amps
    CAR_NOT_READY_TO_CHARGE = 0x4,
    CAR_NOT_IN_PARK = 0x8,
    CAR_K_OFF = 0x10,
    /// <summary>
    /// Typically the ccs charger want us to stop
    /// </summary>
    CHARGER = 0x20,
    ADAPTER_STOP_BUTTON = 0x40,
    CAR_ERROR = 0x80,
};
//
//enum ChargerState
//{
//    Start,
//    WaitForCarMaxAndTargetVolts,
//    WaitForChargerAvailableVoltsAndAmps,
//    WaitForCarReadyToCharge,
//    CarReadyToCharge, // LockPlug
//    WaitForChargerLive, // D2
//    WaitForCarContactorsClosed,
//    WaitForCarAskingAmps,
//    ChargingLoop,
//    Stopping_WaitForLowAmpsDelivered,
//    Stopping_WaitForCarContactorsOpen, // D2 OFF
//    Stopping_WaitForLowVoltsDelivered,
//    Stopping_UnlockPlug, // wait 500ms, D1 OFF, UnlockPlug
//    Stopping_StopCan,
//    End
//};

#define CHARGER_STATE_LIST \
    X(Start, 0) \
    X(WaitForCarMaxAndTargetVolts, 0) \
    X(WaitForChargerAvailableVoltsAndAmps, 0) \
    X(WaitForCarReadyToCharge, 0) \
    X(CarReadyToCharge, 0) \
    X(WaitForChargerLive, 0) \
    X(WaitForCarContactorsClosed, 8) \
    X(WaitForCarAskingAmps, 0) \
    X(ChargingLoop, 0) \
    X(Stopping_WaitForLowAmpsDelivered, 0) \
    X(Stopping_WaitForCarContactorsOpen, 0) \
    X(Stopping_WaitForLowVoltsDelivered, 0) \
    X(Stopping_UnlockPlug, 0) \
    X(Stopping_StopCan, 0) \
    X(End, 0)

enum ChargerState {
#define X(name, timeout) name,
    CHARGER_STATE_LIST
#undef X
};

const char* const _stateNames[] = {
#define X(name, timeout) #name,
    CHARGER_STATE_LIST
#undef X
};

const uint16_t _stateTimeoutsSec[] = {
#define X(name, timeout) timeout,
    CHARGER_STATE_LIST
#undef X
};

struct CarData
{
    // valid after kswitch
    uint16_t MaxChargeVoltage;// = 435;

    uint8_t MaxChargingTimeMins;

    // Valid after kSwitch
    uint16_t TargetVoltage;

    uint16_t CyclesSinceLastAskingAmps;

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

    uint8_t MinimumChargeCurrent;
    uint8_t ChademoRawVersion;
    uint8_t EstimatedChargingTimeMins;
    //uint8_t MaxChargingTime10Sec;

    //bool operator==(const CarData& other) const {
    //    return MaxChargeVoltage == other.MaxChargeVoltage &&
    //        MaxChargingTimeMins == other.MaxChargingTimeMins &&
    //        TargetVoltage == other.TargetVoltage &&
    //        AskingAmps
   // }
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

    uint8_t OutputCurrent;
    uint16_t OutputVoltage;

    // initial value from car, charger count it down
    uint8_t RemainingChargeTimeMins;
    uint16_t ThresholdVoltage;
};


class ChademoCharger
{
public:

    void SetChargerData();
    void SendCanMessages();
    void RunStateMachine(void);
    void Run();
    void RunSend();
	void HandleChademoMessage(uint32_t id, uint8_t* data, uint8_t len);

    void SetState(ChargerState newState, int delay_cycled = 0);

    void SetSwitchD1(bool set);
    void SetSwitchD2(bool set);

    void StopVoltageDelivery();
    bool IsChargingStoppedByAdapter();
    
    void SetChargerData(uint16_t maxV, uint16_t maxA, uint16_t outV, uint16_t outA);

    void StopPowerDelivery();
    bool GetSwitchK();

    bool IsPowerOffOk()
    {
        return _powerOffOk;
    }
  /*  bool IsChargingPlugLocked()
    {
        return _locked;
    };*/

    void LockChargingPlug(bool lock)
    {
//        _locked = lock;

        if (lock)
            _powerOffOk = false;
    };

    //bool IsChargerReady()
    //{
    //    // kickoff: Start of precharge??
    //    // or end...maybe? hardwareInterface_setPowerRelayOn??
    //    return true;
    //};

    void NotifyCarContactorsClosed();
    void PerformInsulationTest() { /* NOP */ }
    
    bool IsChargerLive();

    void NotifyCarAskingForAmps()
    {
        // NOP
    };

    bool IsChargingStoppedByCharger();
    
    void CanSend(int id, int len, uint8_t* data);

    //void ReadPendingCanMessages();
    void Log(bool force = false);

    const char* GetStateName();
    bool IsTimeoutSec(uint16_t max_sec);

    private:

        //int _canTxDrop = 0;
        int _delayCycles = 0;
        int _logCycleCounter = 0;
//        bool _locked = false;
        bool _powerOffOk = true;
        int _cyclesInState = 0;
        
//        char _last[400];
  //      char _now[400];
  //      int _lastCanRecieveTime = 0;
//        int _lastCanSendTime = 0;



        //bool _gotMaxes = false;

        ChargerState _state = ChargerState::Start;

        CarData _carData = {};
        //CarData _lastLoggedCarData = {};
        ChargerData _chargerData = {};
};
