

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
    /// <summary>
    /// 102.4.0
    /// </summary>
    CAR_FAULT_OVER_VOLT = 0x1,
    /// <summary>
    /// 102.4.1
    /// </summary>
    CAR_FAULT_UNDER_VOLT = 0x2,
    /// <summary>
    /// 102.4.2
    /// </summary>
    CAR_FAULT_DEV_AMPS = 0x4,
    /// <summary>
    /// 102.4.3
    /// </summary>
    CAR_FAULT_OVER_TEMP = 0x8,
    /// <summary>
    /// 102.4.4
    /// </summary>
    CAR_FAULT_DEV_VOLT = 0x10
};


enum CarStatus
{
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
    /// main contactor open (Special: 0: During contact sticking detection, 1: Contact sticking detection completed). Called StatusVehicle in docs!!!
    /// </summary>
    CAR_STATUS_CONTACTOR_OPEN_WELDING_DETECTION_DONE = 0x8,
   
    /// <summary>
    /// 102.5.4 Normal stop request before charging
    /// </summary>
    CAR_STATUS_STOP_BEFORE_CHARGING = 0x10,

    /// <summary>
    /// 102.5.7
    /// car is V2X compatible (can deliver power to grid)
    /// </summary>
    CAR_STATUS_DISCHARGE_COMPATIBLE = 0x80,
};

enum ChargerStatus
{
    /// <summary>
    /// 109.5.0
    /// during rundown: This is tied 1:1 with OutputCurrent > 0. Meaning we can be stopped, but still charging since amps > 0.
    /// During startup, CHARGER_STATUS_CHARGING is set and then amps are still 0.
    /// 0: standby 1: charging (power transfer from charger)
    /// </summary>
    CHARGER_STATUS_CHARGING = 0x1,

    /// <summary>
    /// 109.5.1
    /// something went wrong (fault caused by (or inside) the charger)
    /// </summary>
    CHARGER_STATUS_ERROR = 0x2,

    /// <summary>
    /// 109.5.2
    /// connector is currently locked (electromagnetic lock, plug locked into the car)??? Or is it related to locking at all????
    /// </summary>
    CHARGER_STATUS_ENERGIZING_CONNECTOR_LOCKED = 0x4,

    /// <summary>
    /// 109.5.3
    /// parameters between vehicle and charger not compatible (battery incompatible?)
    /// </summary>
    CHARGER_STATUS_INCOMPAT = 0x8,

    /// <summary>
    /// 109.5.4
    /// can be CAN timeout, or other timeout (too short/long in state). problem with the car, such as improper connection (or something wrong with the battery?)
    /// </summary>
    CHARGER_STATUS_CAR_ERROR = 0x10,

    /// <summary>
    /// 109.5.5
    /// charger is stopped (charger shutdown or end of charging). this is also initially set to stop, before charging.
    /// </summary>
    CHARGER_STATUS_STOPPED = 0x20,
};

enum StopReason
{
    NONE = 0x0,
    CAR_CAN_AMPS_TIMEOUT = 0x1,
    CHARGING_TIME = 0x2, // out of time
    CAR_NOT_READY_TO_CHARGE = 0x4,
    CAR_NOT_IN_PARK = 0x8,
    CAR_SWITCH_K_OFF = 0x10,
    POWER_OFF_PENDING = 0x20,
    CAR_ERROR = 0x40
};


#define CHARGER_STATE_LIST \
    X(WaitForPreChargeStart) \
    X(WaitForCarReadyToCharge) \
    X(SetSwitchD2) \
    X(WaitForCarContactorsClosed) \
    X(WaitForCarAskingAmps) \
    X(ChargingLoop) \
    X(Stopping_Start) \
    X(Stopping_WaitForLowAmps) \
    X(Stopping_WaitForCarContactorsOpen) \
    X(Stopping_WaitForLowVolts) \
    X(Stopping_End) \
    X(Stopped)

enum ChargerState {
#define X(name) name,
    CHARGER_STATE_LIST
#undef X
};

const char* const _stateNames[] = {
#define X(name) #name,
    CHARGER_STATE_LIST
#undef X
};

//const uint16_t _stateTimeoutsSec[] = {
//#define X(name, timeout) timeout,
//    CHARGER_STATE_LIST
//#undef X
//};

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
            uint8_t Faults;
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
    uint16_t MaxBatteryVoltage;

    uint16_t MaxChargingTimeSec;

    // Valid after kSwitch
    uint16_t TargetBatteryVoltage = 410;
    uint16_t EstimatedBatteryVoltage;

    uint16_t CyclesSinceCarLastAskingAmps;

    uint16_t EstimatedNominalVoltage;

    uint8_t MinimumChargeCurrent;
    uint8_t AskingAmps;

    // PS: unstable before switch (k)
    // fake initial 10 perc.
    uint8_t SocPercent = 10;

    CarStatus Status;
    CarFaults Faults;

    // PS: unstable before switch (k)
    float BatteryCapacityKwh;

    uint8_t ProtocolNumber;

    uint8_t EstimatedChargingTimeMins;
};

struct ChargerData
{
    uint8_t ProtocolNumber = 2; // 1: chademo 0.9 2: chademo 1.0

    /// <summary>
    /// Initial status is stopped
    /// </summary>
    ChargerStatus Status = ChargerStatus::CHARGER_STATUS_STOPPED;

    uint16_t AvailableOutputVoltage;

    /// <summary>
    /// If true, the charger support helping the car to do welding detection (by lowering the voltage)
    /// Not sure if 1 is right here...
    /// But I think 1 if we assume charger can drop its voltage fast, after charging was stopped
    /// </summary>
    bool SupportWeldingDetection = 1;

    uint8_t AvailableOutputCurrent;

    uint8_t OutputCurrent;
    uint16_t OutputVoltage;

    // initial value from car, charger count it down
    uint16_t RemainingChargeTimeSec;
    uint32_t RemainingChargeTimeCycles;

    uint16_t ThresholdVoltage;
};


class ChademoCharger
{
public:
    void UpdateChargerMessages();
  //  bool IsCurrentDemandStarted();
//    bool IsPreChargeStarted();
    void HandlePendingCarMessages();
    void SetChargerDataFromCcsParams();
    void SendChargerMessages();
    void RunStateMachine(void);
    void Run();
    bool PreChargeCompleted(uint16_t batt);
  	void HandleCanMessageIsr(uint32_t id, uint32_t data[2]);
    void SetState(ChargerState newState, int delayCycles = 0);
    void OpenAdapterContactor();
    void SetSwitchD1(bool set);
    void SetSwitchD2(bool set);
    void SetCcsParamsFromCarData();
    void SetChargerData(uint16_t maxV, uint16_t maxA, uint16_t outV, uint16_t outA);
    bool GetSwitchK();

    bool IsPowerOffOk()
    {
        return _powerOffOk;
    }

    void CloseAdapterContactor();
    
    void Log(bool force = false);
    const char* GetStateName();
    bool IsTimeoutSec(uint16_t max_sec);
    
    StopReason GetStopReason()
    {
        return _stopReason;
    }

    private:
        int _delayCycles = 0;
        int _logCycleCounter = 0;
        bool _powerOffOk = true;
        int _cyclesInState = 0;

        // only allowed to use in: HandlePendingIsrMessages, HandleCanMessage
        bool _msg100_pending = false;
        msg100 _msg100_isr = {};
        bool _msg101_pending = false;
        msg101 _msg101_isr = {};
        bool _msg102_pending = false;
        msg102 _msg102_isr = {};

        bool _switch_k = false;
        bool _switch_d1 = false;

        bool _msg102_recieved = false;

        bool _stop_delivering_amps = false;
        bool _stop_delivering_volts = false;

        bool _nominalVoltsInPreChargeEvent = false;
        

        // only allowed to use in: SendCanMessages, UpdateChargerMessages
        msg108 _msg108 = {};
        msg109 _msg109 = {};
        
        ChargerState _state = ChargerState::WaitForPreChargeStart;
        CarData _carData = {};
        ChargerData _chargerData = {};

        StopReason _stopReason = StopReason::NONE;
};
