

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

template<typename T>
constexpr T min(T a, T b) {
    return (a < b) ? a : b;
}

inline uint8_t clampToUint8(uint16_t value) {
    return static_cast<uint8_t>(value > 0xFF ? 0xFF : value);
}

template<typename T>
constexpr T max(T a, T b) {
    return (a > b) ? a : b;
}

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
    CAR_STATUS_READY_TO_CHARGE = 0x1, // binary: 0000 0001

    /// <summary>
    /// 102.5.1 Vehicle shift position
    /// </summary>
    CAR_STATUS_NOT_IN_PARK = 0x2, // binary: 0000 0010

    /// <summary>
    /// 102.5.2 Charging system fault
    /// Can timeout? Other timeout? Too long/short in state?
    /// This is analog to ChargerStatus::CHARGER_STATUS_ERROR, only opposite direction
    /// </summary>
    CAR_STATUS_ERROR = 0x4, // binary: 0000 0100

    /// <summary>
    /// 102.5.3 Vehicle status
    /// Car initially send 0 here, kind of wrong...then it changes to 1 (OPEN).
    /// Set the flag to 0 when the vehicle relay is closed (start)
    /// and set as 1 after the termination of welding detection (end)
    /// main contactor open (Special: 0: During contact sticking detection, 1: Contact sticking detection completed). Called StatusVehicle in docs!!!
    /// </summary>
    CAR_STATUS_CONTACTOR_OPEN_WELDING_DETECTION_DONE = 0x8, // binary: 0000 1000
   
    /// <summary>
    /// 102.5.4 Normal stop request before charging
    /// </summary>
    CAR_STATUS_STOP_BEFORE_CHARGING = 0x10, // binary: 0001 0000

    CAR_STATUS_UNKNOWN_0x20 = 0x20, // binary: 0010 0000

    /// <summary>
    /// Vehicle reports a fault specifically in its charging system. (Bit 6)?
    /// Possibly battery cooling in progress??? Or hot battery in general?
    /// </summary>
    CAR_STATUS_UNKNOWN_0x40 = 0x40, // binary: 0100 0000

    // car is V2X compatible (can deliver power to grid)
    CAR_STATUS_DISCHARGE_COMPATIBLE = 0x80, // binary: 1000 0000
};

enum ChargerStatus
{
    /// <summary>
    /// during rundown: This is tied 1:1 with OutputCurrent > 0. Meaning we can be stopped, but still charging since amps > 0.
    /// During startup, CHARGER_STATUS_CHARGING is set and then amps are still 0.
    /// 0: standby 1: charging (power transfer from charger)
    /// </summary>
    CHARGER_STATUS_CHARGING = 0x1, // binary: 0000 0001

    /// <summary>
    /// something went wrong (fault caused by (or inside) the charger)
    /// </summary>
    CHARGER_STATUS_ERROR = 0x2, // binary: 0000 0010

    // connector is currently locked (electromagnetic lock, plug locked into the car)
    CHARGER_STATUS_PLUG_LOCKED = 0x4, // binary: 0000 0100

    // parameters between vehicle and charger not compatible (battery incompatible?)
    CHARGER_STATUS_INCOMPAT = 0x8, // binary: 0000 1000

    // can be CAN timeout, or other timeout (too short/long in state). problem with the car, such as improper connection (or something wrong with the battery?)
    CHARGER_STATUS_CAR_ERROR = 0x10, // binary: 0001 0000

    //charger is stopped (charger shutdown or end of charging). this is also initially set to stop, before charging.
    CHARGER_STATUS_STOPPED = 0x20, // binary: 0010 0000
};

enum StopReason
{
    NONE = 0x0, // binary: 0000 0000
    CAR_CAN_AMPS_TIMEOUT = 0x1, // binary: 0000 0001
    //            CAR_ASK_FOR_ZERO_AMPS = 2, // asking for 0 amps
    CAR_NOT_READY_TO_CHARGE = 0x4, // binary: 0000 0100
    CAR_NOT_IN_PARK = 0x8, // binary: 0000 1000
    CAR_K_OFF = 0x10, // binary: 0001 0000
    /// <summary>
    /// Typically the ccs charger want us to stop
    /// </summary>
    CHARGER = 0x20, // binary: 0010 0000
    ADAPTER_STOP_BUTTON = 0x40, // binary: 0100 0000
    CAR_ERROR = 0x80, // binary: 1000 0000
};


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

#pragma pack(push, 1)

// Car limits
struct msg100
{
    union {
        struct {
            uint8_t MinimumChargeCurrent;
            uint8_t Unused1;
            uint16_t MinimumBatteryVoltage;
            uint16_t MaximumBatteryVoltage;
            uint8_t SocPercentConstant;
            uint8_t Unused7;
        } m;
        uint8_t bytes[8];
        uint32_t pair[2];
    };
};

// Car calculated values based on charger and car limits (after car know how many amps etc. charger can deliver, and car can consume, it can calc how much time it need)
struct msg101
{
    union {
        struct {
            uint8_t Unused0;
            uint8_t MaximumChargingTime10s;
            uint8_t MaximumChargingTimeMinutes;
            uint8_t EstimatedChargingTimeMinutes;
            uint8_t Unused4;
            uint16_t BatteryCapacity;
            uint8_t Unused7;
        } m;
        uint8_t bytes[8];
        uint32_t pair[2];
    };
};

// Car (charging) status
struct msg102
{
    union {
        struct {
            uint8_t ProtocolNumber;
            uint16_t TargetBatteryVoltage;
            uint8_t ChargingCurrentRequest;
            uint8_t Fault;
            uint8_t Status;
            uint8_t SocPercent;
            uint8_t Unused7;
        } m;
        uint8_t bytes[8];
        uint32_t pair[2];
    };
};

// Charger limits/availability
struct msg108
{
    union {
        struct {
            uint8_t WeldingDetection;
            uint16_t AvailableOutputVoltage;
            uint8_t AvailableOutputCurrent;
            uint16_t ThresholdVoltage;
            uint8_t Unused6;
            uint8_t Unused7;
        } m;
        uint8_t bytes[8];
        uint32_t pair[2];
    };
};

// Charger status
struct msg109
{
    union {
        struct {
            uint8_t ProtocolNumber;
            uint16_t PresentVoltage;
            uint8_t PresentChargingCurrent;
            uint8_t Unused4; // discharge compatible
            uint8_t Status = ChargerStatus::CHARGER_STATUS_STOPPED;
            uint8_t RemainingChargingTime10s;
            uint8_t RemainingChargingTimeMinutes;
        } m;
        uint8_t bytes[8];
        uint32_t pair[2];
    };
};

#pragma pack(pop)

struct CarData
{
    // valid after kswitch
    uint16_t MinimumBatteryVoltage;
    uint16_t MaxBatteryVoltage;

    uint16_t MaxChargingTimeSec;

    // Valid after kSwitch
    uint16_t TargetBatteryVoltage;

    uint16_t CyclesSinceLastAskingAmps;

    //ushort BattNominalVoltage = > (ushort)Program.get_estimated_nominal_voltage(TargetVoltage);
    uint8_t MinimumChargeCurrent;
    uint8_t AskingAmps;

    // valid after k-switch
    // soc = SocPercent / ChargingRateReferenceConstant * 100 ... .weird stuff... I guess if ChargingRateReferenceConstant

    uint8_t SocPercent;

    /// <summary>
    /// To be safe, only use SOC,MaxChargingTimeMins,BatteryCapacityKwh after car is ready to charge (before they can have invalid values).
    /// </summary>
    CarStatus Status;
    CarFaults Fault;

    float BatteryCapacityKwh;

    // ChargingRate reference constant
    // valid after k-switch
 
    uint8_t EstimatedChargingTimeMins;

};

struct ChargerData
{
    /// <summary>
    /// For v1: car send 100-102
    /// I think for v2 it also send 200-202. It may also be checking that we read those messages...
    /// So...it may be best to use raw v 1??
    /// Trying with v2 and no filter! The car may require we read them all....
    /// </summary>
    uint8_t ProtocolNumber = 2; // 1: chademo 0.9 2: chademo 1.0

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
    uint8_t CcsAvailableOutputCurrent;// ?? hack...

    uint8_t OutputCurrent;
    uint16_t OutputVoltage;

    // initial value from car, charger count it down
    uint8_t RemainingChargeTimeSec;// = 60; //HACK......
    uint16_t ThresholdVoltage;

   // uint32_t AvailableWatts; hack
};


class ChademoCharger
{
public:
    void UpdateChargerMessages();
   // void CalcChargerThreasholdVoltage();
    void HandlePendingIsrMessages();
    void ExtractAndSetCcsData();
    void SendCanMessages();
    void RunStateMachine(void);
    void Run();
    //void RunSend();
	void HandleCanMessage(uint32_t id, uint32_t data[2]);

    void SetState(ChargerState newState, int delay_ms = 0);

    void SetSwitchD1(bool set);
    void SetSwitchD2(bool set);

    void StopVoltageDelivery();
    bool IsChargingStoppedByAdapter();
    
    void SetCcsData(uint16_t maxV, uint16_t maxA, uint16_t outV, uint16_t outA);

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
    
   
    void Log(bool force = false);

    const char* GetStateName();
    bool IsTimeoutSec(uint16_t max_sec);

    int _delayCycles = 0;

    private:
        int _logCycleCounter = 0;
        bool _powerOffOk = true;
        int _cyclesInState = 0;
        bool _k_switch = false;

        // only allowed to use in: HandlePendingIsrMessages, HandleCanMessage
        bool _msg100_pending = false;
        msg100 _msg100_isr = {};
        bool _msg101_pending = false;
        msg101 _msg101_isr = {};
        bool _msg102_pending = false;
        msg102 _msg102_isr = {};

       // bool _sendCanKickoff = false;

        // only allowed to use in: SendCanMessages, UpdateChargerMessages
        msg108 _msg108 = {};
        msg109 _msg109 = {};

        ChargerState _state = ChargerState::Start;
        CarData _carData = {};
        ChargerData _chargerData = {};
};
