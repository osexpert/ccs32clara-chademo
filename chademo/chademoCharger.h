#pragma once

#include <type_traits>
#include "printf.h"

#define ADAPTER_MAX_AMPS 200
#define ADAPTER_MAX_VOLTS 500 //Porsche Taycan requires 750V, but setting this value to 750 might break compatibility with many chargers. As default value 500V is good!

#define CHA_CYCLE_MS 100
#define CHA_CYCLES_PER_SEC (1000 / CHA_CYCLE_MS)

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

enum ExtendedFunction1
{
    /// <summary>
    /// 110.0.0
    /// 118.0.0
    /// </summary>
    DYNAMIC_CONTROL = 0x1,
    /// <summary>
    /// 110.0.1
    /// 118.0.1
    /// </summary>
    HIGH_CURRENT_CONTROL = 0x2,
    /// <summary>
    /// 110.0.2
    /// 118.0.2
    /// </summary>
    HIGH_VOLTAGE_CONTROL = 0x4
};

/// <summary>
/// errors during charging, related to the battery.
/// </summary>
enum CarFaults
{
    /// <summary>
    /// 102.4.0
    /// </summary>
    OVER_VOLT = 0x1,
    /// <summary>
    /// 102.4.1
    /// </summary>
    UNDER_VOLT = 0x2,
    /// <summary>
    /// 102.4.2
    /// </summary>
    DEV_AMPS = 0x4,
    /// <summary>
    /// 102.4.3
    /// </summary>
    OVER_TEMP = 0x8,
    /// <summary>
    /// 102.4.4
    /// </summary>
    DEV_VOLT = 0x10
};


enum CarStatus
{
    /// <summary>
    /// 102.5.0 Vehicle charging enabled
    /// </summary>
    READY_TO_CHARGE = 0x1,

    /// <summary>
    /// 102.5.1 Vehicle shift position
    /// </summary>
    NOT_IN_PARK = 0x2,

    /// <summary>
    /// 102.5.2 Charging system fault
    /// CAN timeout? Other timeout? Too long/short in state?
    /// This is analog to ChargerStatus::ERROR, only opposite direction
    /// </summary>
    ERROR = 0x4,

    /// <summary>
    /// 102.5.3 Vehicle status
    /// Car initially send 0 here. If charger >= chademo 1.0, car changes to 1 (open) as soon as it discovers.
    /// If charger say it is < chademo 1.0, car keep this flag as 0 (AFAICS).
    /// 
    /// Set to 0 when the vehicle relay is closed (start) and set to 1 after the termination of welding detection (end)
    /// </summary>
    CONTACTOR_OPEN_OR_WELDING_DETECTION_DONE = 0x8,
   
    /// <summary>
    /// 102.5.4 Normal stop request before charging
    /// Only valid for use before car is asking for amps. After asking for amps, only the other/normal stop reasons apply.
    /// </summary>
    STOP_BEFORE_CHARGING = 0x10,

    /// <summary>
    /// 102.5.6
    /// Unknown. Possibly related to V2X?
    /// </summary>
    UNKNOWN_102_5_6 = 0x40,

    /// <summary>
    /// 102.5.7
    /// car is V2X compatible (can deliver power to grid)
    /// If you set this flag without also sending messages 208/209, car will fail/ERROR.
    /// </summary>
    DISCHARGE_COMPATIBLE = 0x80,
};

enum ChargerStatus
{
    /// <summary>
    /// 109.5.0
    /// during rundown: This is tied 1:1 with OutputCurrent > 0. Meaning we can be stopped, but still charging since amps > 0.
    /// During startup, CHARGER_STATUS_CHARGING is set and then amps are still 0.
    /// 0: standby 1: charging (power transfer from charger)
    /// </summary>
    CHARGING = 0x1,

    /// <summary>
    /// 109.5.1
    /// Something is wrong with the charger
    /// </summary>
    CHARGER_ERROR = 0x2,

    /// <summary>
    /// 109.5.2
    /// connector is currently locked. In later spec, it is only refered to as Energizing state (OutputVoltage > 10 volt)
    /// </summary>
    ENERGIZING_OR_PLUG_LOCKED = 0x4,

    /// <summary>
    /// 109.5.3
    /// parameters between vehicle and charger not compatible
    /// </summary>
    BATTERY_INCOMPATIBLE = 0x8,

    /// <summary>
    /// 109.5.4
    /// Something is wrong with the car and/or changer.
    /// CAN timeout, car asking for too much volts, car asking for too much amps. So mainly car stuff. But spec say this is for errors both in charger or car.
    /// </summary>
    CHARGING_SYSTEM_ERROR = 0x10,

    /// <summary>
    /// 109.5.5
    /// charger is stopped (charger shutdown or end of charging). this is also initially set to stop, before charging.
    /// </summary>
    STOPPED = 0x20,
};

// class make CHARGING_SYSTEM_ERROR not clash with ChargerStatus (insanity)
enum class StopReason
{
    NONE = 0x0,
    CAR_CAN_AMPS_TIMEOUT = 0x1,
    CHARGING_TIME = 0x2, // out of time
    CAR_NOT_READY_TO_CHARGE = 0x4,
    CAR_NOT_IN_PARK = 0x8,
    CAR_SWITCH_K_OFF = 0x10,
    POWER_OFF_PENDING = 0x20,
    CAR_ERROR = 0x40,
    CHARGING_SYSTEM_ERROR = 0x80,
    UNKNOWN = 0x100,
    CHARGER_ERROR = 0x200,
    CAR_STOP_BEFORE_CHARGING = 0x400,
    BATTERY_INCOMPATIBLE = 0x800,
    TIMEOUT = 0x1000
};


#define CHARGER_STATE_LIST \
    CHARGER_STATE(PreStart_DiscoveryCompleted_WaitForPreChargeStart) \
    CHARGER_STATE(Start) \
    CHARGER_STATE(WaitForCarReadyToCharge) \
    CHARGER_STATE(WaitForPreChargeDone) \
    CHARGER_STATE(WaitForCarContactorsClosed) \
    CHARGER_STATE(WaitForCarAskingAmps) \
    CHARGER_STATE(ChargingLoop) \
    CHARGER_STATE(Stopping_Start) \
    CHARGER_STATE(Stopping_WaitForLowAmps) \
    CHARGER_STATE(Stopping_WaitForCarContactorsOpen) \
    CHARGER_STATE(Stopping_WaitForLowVolts) \
    CHARGER_STATE(Stopping_End) \
    CHARGER_STATE(Stopped)

enum ChargerState {
#define CHARGER_STATE(name) name,
    CHARGER_STATE_LIST
#undef CHARGER_STATE
};

const char* const _stateNames[] = {
#define CHARGER_STATE(name) #name,
    CHARGER_STATE_LIST
#undef CHARGER_STATE
};

#pragma pack(push, 1)

// Car limits
struct msg100
{
    union {
        struct {
            uint8_t MinimumChargeCurrent; // added in cha 1.0
            uint8_t MaximumChargeCurrent; // added in cha 1.0? but only seen on some cars. 0->240->correctValue. unstable before switch(k).
            uint16_t MinimumBatteryVoltage; // added in cha 1.0
            uint16_t MaximumBatteryVoltage;
            /// <summary>
            /// Leaf 40kwh: always start out as 240. When car discover changer chademo version, it changes to 100 if >= chademo 1.0, or 255 if chademo 0.9. Weird stuff.
            /// </summary>
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
            uint8_t EstimatedChargingTimeMinutes; // added in cha 1.0
            uint8_t Unused4;
            uint16_t BatteryCapacity; // added in cha 1.0?
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


// Car extension
struct msg110
{
    union {
        struct {
            uint8_t ExtendedFunction1;
            uint8_t Unused1;
            uint8_t Unused2;
            uint8_t Unused3;
            uint8_t Unused4;
            uint8_t Unused5;
            uint8_t Unused6;
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
            uint8_t DischargeCompatible;
            uint8_t Status;
            uint8_t RemainingChargingTime10s;
            uint8_t RemainingChargingTimeMinutes;
        } m;
        uint8_t bytes[8];
        uint32_t pair[2];
    };
};

// Charger extension
struct msg118
{
    union {
        struct {
            uint8_t ExtendedFunction1;
            uint8_t Unused1;
            uint8_t Unused2;
            uint8_t Unused3;
            uint8_t Unused4;
            uint8_t Unused5;
            uint8_t Unused6;
            uint8_t Unused7;
        } m;
        uint8_t bytes[8];
        uint32_t pair[2];
    };
};

// V2X
// Trace: https://raw.githubusercontent.com/dalathegreat/EV-CANlogs/refs/heads/main/Nissan%20LEAF/V2X/v2h_startup__charge_ffrom_solar_excess_for_a_while_then_turn_on_oven_to_create_load_then_switch_off_load__then_end_session.csv

/* Car discharge limits
Vehicle 0x200, peer to EVSE 0x208?
 
Trace: shortly after chargingloop, settle to MaximumDischargeCurrentInverted:240 (15A), MinimumDischargeVoltage:250, MinimumBatteryDischargeLevel:69, MaxRemainingCapacityForCharging:185
Don't change more in the trace.
*/
struct msg200
{
    union {
        struct {
            uint8_t MaximumDischargeCurrentInverted; // FF
            uint8_t Unused1; // 00
            uint8_t Unused2; // 00
            uint8_t Unused3; // 00
            /// <summary>
            /// trace: 250v make no sense...310v is absolute minimum, so what does this really mean?
            /// I think it is inverted and means 5 amps, but 5 amps what?
            /// </summary>
            uint16_t MinimumDischargeVoltage; // FA 00
            /// <summary>
            /// trace: SOC%? But 69% seems very high? Can it be 100-69=31%?
            /// Or kwh? 31kwh?
            /// </summary>
            uint8_t MinimumBatteryDischargeLevel; // 1A
            /// <summary>
            /// kwh?
            /// </summary>
            uint8_t MaxRemainingCapacityForCharging; // FF
        } m;
        uint8_t bytes[8];
        uint32_t pair[2];
    };
};

/* Car discharge estimates
Vehicle 0x201, peer to EVSE 0x209
HOWEVER, 201 isn't even emitted in any of the v2x canlogs available
*/
struct msg201
{
    union {
        struct {
            uint8_t ProtocolNumber; // 2
            uint16_t ApproxDischargeCompletionTime;
            uint16_t AvailableVehicleEnergy;
            uint8_t Unused5;
            uint8_t Unused6;
            uint8_t Unused7;
        } m;
        uint8_t bytes[8];
        uint32_t pair[2];
    };
};

// 0x202 01 01 01 00 00 00 00 00

struct msg208
{
    union {
        struct {
            /// <summary>
            /// I am guessing...that if charger declare to support discharge and the car too, then changer will send this message, and if the value here is <> 0 (or FF??)
            /// then car WONT ask for amps, but instead is willing to give?
            /// 
            /// present discharge current is a measured value 0xFF - get_measured_current();
            /// </summary>
            /* Present discharge current is a measured value. In the absence of
            a shunt, the evse here is quite literally lying to the vehicle.The spec
                seems to suggest this is tolerated unless the current measured on the EV
                side continualy exceeds the maximum discharge current by 10amps
                x208_evse_dischg_cap.present_discharge_current = 0xFF - 6;

                So....I guess can just use 1 amp here for fun? Like a signal?
            */

            uint8_t PresentDischargeCurrentInverted; // FF
            uint16_t MaxDischargeVoltage; // AA 00
            uint8_t MaxDischargeCurrentInverted; // FF
            uint8_t Unused4; // 00
            uint8_t Unused5; // 00
            uint16_t LowerThresholdVoltage; // AA 00
        } m;
        uint8_t bytes[8];
        uint32_t pair[2];
    };
};

/* Discharge estimates: x209 EVSE = peer to x201 Vehicle
NOTE: x209 is emitted in CAN logs when x201 isn't even present
it may not be understood by leaf (or ignored unless >= a certain protocol version or v2h sequence number
In trace start: ProtocolNumber:2, RemainingDischargeTime:0.
After charging loop start, set to ProtocolNumber=2, RemainingDischargeTime=5
Not changed after this.
2 = CHADEMO_BIDIRECTIONAL?
 */
struct msg209
{
    union {
        struct {
            uint8_t ProtocolNumber; // 1 or 2 (seen both)
            /// <summary>
            /// Possibly setting this is > 0 make the car "alive" and never times out? No...seems to have no function.
            /// </summary>
            uint16_t RemainingDischargeTime;
            uint8_t Unused3;
            uint8_t Unused4;
            uint8_t Unused5;
            uint8_t Unused6;
            uint8_t Unused7;
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
    uint16_t TargetBatteryVoltage;
    uint16_t EstimatedBatteryVoltage;

    uint16_t CyclesSinceCarLastAskingAmps;

    uint8_t MinimumChargeCurrent;
    uint8_t AskingAmps;

    // PS: unstable before switch (k)
    uint8_t SocPercent;

    CarStatus Status;
    CarFaults Faults;

    // PS: unstable before switch (k)
    float BatteryCapacityKwh;

    /// <summary>
    /// 0: before 0.9
    /// 1: 0.9, 0.9.1
    /// 2: 1.0.0, 1.0.1
    /// 3: 2.0
    /// </summary>
    uint8_t ProtocolNumber;

    uint8_t EstimatedChargingTimeMins;

    bool SupportDynamicAvailableOutputCurrent;

    uint8_t MaxDischargeCurrent;
};

enum ProtocolNumber
{
    Chademo_0 = 0,
    Chademo_0_9 = 1,
    Chademo_1_0 = 2,
    Chademo_2_0 = 3,
};

struct ChargerData
{
    uint8_t ProtocolNumber = ProtocolNumber::Chademo_1_0;

    uint8_t DischargeProtocolNumber = 2;

    /// <summary>
    /// Initial status is stopped
    /// </summary>
    ChargerStatus Status = ChargerStatus::STOPPED;

    uint16_t AvailableOutputVoltage;

    /// <summary>
    /// If true, the charger support helping the car to do welding detection. But how can the charger help?
    /// I think...by lowering the voltage after charging is done (become "floating").
    /// This helps the car to use its inlet voltage detector to check it if measure the battery voltage when it closes the contactors,
    /// and that it measure no voltage when it opens the contactors.
    /// If the charger did not drop its voltage, the car inlet voltage detector would always measure voltage, and welding detection would not be possible.
    /// </summary>
    bool SupportWeldingDetection = true;

    uint8_t MaxAvailableOutputCurrent;
    uint8_t AvailableOutputCurrent;

    uint8_t OutputCurrent;
    uint16_t OutputVoltage;

    uint8_t DischargeCurrent;

    // just some value seen in can logs.. seen 10, 15. Some say car will allow 10A deviation from what you say? So if we always say we use 10A, we can use anything between 0-20A? 
    // For any current above 20A then need real measured amps.
    uint8_t MaxDischargeCurrent = 20; 
    uint16_t RemainingDischargeTime;

    bool DischargeEnabled = false;// safe to have this on for all?

    // initial value from car, charger count it down
    uint16_t RemainingChargeTimeSec;
    uint32_t RemainingChargeTimeCycles;

    uint16_t ThresholdVoltage;

    bool SupportDynamicAvailableOutputCurrent = true;
};


class ChademoCharger
{
public:
    bool IsPowerOffOk();
    bool PreChargeCompleted();
    bool CarContactorsOpened();
    void UpdateChargerMessages();
    void HandlePendingCarMessages();
    void SetChargerDataFromCcsParams();
    void SendChargerMessages();
    void RunStateMachine(void);
    void Run();
    bool IsDiscoveryCompleted();
	void HandleCanMessageIsr(uint32_t id, uint32_t data[2]);
    void SetState(ChargerState newState, StopReason stopReason = StopReason::NONE);
    void OpenAdapterContactor();
    void SetSwitchD1(bool set);
    void SetSwitchD2(bool set);
    void SetCcsParamsFromCarData();
    void SetChargerData(uint16_t maxV, uint16_t maxA, uint16_t outV, uint16_t outA);
    bool GetSwitchK();

    void CloseAdapterContactor();
   
    void Log(bool force = false);

    const char* GetStateName();
    bool IsTimeoutSec(uint16_t sec);
    bool HasElapsedSec(uint16_t sec);

    int _delayCycles = 0;

    StopReason GetStopReason()
    {
        return _stopReason;
    }

    void LockChargingPlug() {
        _chargingPlugLockedTrigger = true;
    }

    void UnlockChargingPlug() {
    }

    private:
        int _logCycleCounter = 0;
        int _cyclesInState = 0;
        bool _chargingPlugLockedTrigger = false;
        bool _switch_k = false;
        bool _switch_d1 = false;
        bool _msg102_recieved = false;
        bool _discovery = true;
        bool _preChargeDoneButStalled = false;

        // only allowed to use in: HandlePendingIsrMessages, HandleCanMessage
        bool _msg100_pending = false;
        msg100 _msg100_isr = {};
        bool _msg101_pending = false;
        msg101 _msg101_isr = {};
        bool _msg102_pending = false;
        msg102 _msg102_isr = {};
        bool _msg110_pending = false;
        msg110 _msg110_isr = {};
        bool _msg200_pending = false;
        msg200 _msg200_isr = {};
        bool _msg201_pending = false;
        msg201 _msg201_isr = {};

        // only allowed to use in: SendCanMessages, UpdateChargerMessages
        msg108 _msg108 = {};
        msg109 _msg109 = {};
        msg118 _msg118 = {};
        msg208 _msg208 = {};
        msg209 _msg209 = {};

        StopReason _stopReason = StopReason::NONE;
        ChargerState _state = ChargerState::Start;

        CarData _carData = {};
        ChargerData _chargerData = {};

        bool _dischargeActivated = false;
};
