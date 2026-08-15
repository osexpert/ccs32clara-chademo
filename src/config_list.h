// values that can be patched directly in the firmware

CONFIG_ITEM(ADAPTER_MAX_AMPS, "max-amps", uint8_t, 200);

// Default SOC% at which the adapter will stop charging
CONFIG_ITEM(STOP_CHARGING_SOC, "stop-charging-soc", uint8_t, 100);

//Porsche Taycan requires 750V, but setting this value to 750 might break compatibility with many chargers. As default value 500V is good!
CONFIG_ITEM(ADAPTER_MAX_VOLTS, "max-volts", uint16_t, 500); 

// Just some value seen in can logs.. seen 10, 15. Some say car will allow 10A deviation from what you say? So if we always say we use 10A, we can use anything between 0-20A? 
// For any current above 20A then need real measured amps? Try 40.
CONFIG_ITEM(MAX_DISCHARGE_AMPS_FALLBACK, "max-discharge-amps-fallback", uint8_t, 40);

// Jdemo: custom kit for Toyota RAV4, Tesla Rodster etc. Made by Quick Charge Power https://quickchargepower.com/ (QC Charge https://qccharge.com/).
// Jdemo is quirky in some ways:
// - after D2=true, charger must ACK (ChargerStatus::CHARGING=true / ChargerStatus::STOPPED=false) within 2 sec. It is more restrictive than the spec: 2.5sec.
// - If AvailableOutputCurrent is changed/lowered so that RequestCurrent reack AvailableOutputCurrent, the charging just stops (at least for lower values as 10).
// - It will/can increase RequestCurrent until it reaches AvailableOutputCurrent - 10 (so for a 200A charger, it will/can ask for 190A until it backs down). The kits were sold as 125A kits, but some were delivered with 200A cables.
// - It is possible that CHADEMO_MAX_UNDERSUPPLY_AMPS = 20 would work...since it has this weird 10A thing...but also never changing AvailableOutputCurrent seems to work (dynamic-control-fallback = false).
CONFIG_ITEM(DYNAMIC_CONTROL_FALLBACK, "dynamic-control-fallback", bool, true);

CONFIG_ITEM(CONFIG_SX, "sx", bool, false);

CONFIG_ITEM(CONFIG_V2X, "v2x", bool, false);

CONFIG_ITEM(CONFIG_ALWAYS_ON, "always-on", bool, false);

CONFIG_ITEM(CONFIG_ALT_ESTIMATED_VOLTAGE, "alt-estimated-voltage", uint8_t, 0);

// Based on logs, worst case is 1600ms before asking for amps, but use 2000 for now.
CONFIG_ITEM(CHADEMO_09_AssumeCarContactorsClosed_MS, "assume-car-contactors-closed-after-ms", uint16_t, 2000);

// Fake output current towards the car. May need to use 2 or 5 to make it work on all cars?
CONFIG_ITEM(CHADEMO_FAKE_IDLE_AMPS, "fake-idle-amps", uint8_t, 1);

// car seems to allows 20V deviation. Adding +- 20V in addition should allow 40V deviation. +-30V also worked, but if +-20V works, lets keep +-30 as backup:-) But 0 is default (no modulation).
CONFIG_ITEM(CHADEMO_VOLTAGE_MODULATION, "voltage-modulation", uint8_t, 0);

CONFIG_ITEM(DX_CCS_WaitForPreChargeStart_MS, "precharge-start-delay-ms", uint16_t, 2000);

CONFIG_ITEM(CHADEMO_MAX_UNDERSUPPLY_AMPS, "max-undersupply-amps", uint8_t, 10);

// chademo 1.* require welding detection, while in 0.9 it is optional, but maybe some cars still allow it?
// Tested with false on Leaf 2018, it does not complain, but I don't see any difference. Maybe it is just ignore and has no effect?
// Hard to say how other cars will behave.
CONFIG_ITEM(CONFIG_WELDING_DETECTION, "chademo-welding-detection", bool, true);

// Simulate chademo 0.9 instead of 1.0. In Chademo 0.9, setting welding detection was optional.
CONFIG_ITEM(CONFIG_CHADEMO_09, "chademo-0.9", bool, false);

// Pro: voltage will drop consistently and hopefully faster? Won't rely on every chargers bleed-down speed?
// Con: inlet wont bleed-down at all? But spec demand less than 1uF, but still...charge could be "trapped" inside the inlet and voltage never drop?
CONFIG_ITEM(CONFIG_OPEN_ADAPTER_CONTACTOR_BEFORE_WELDING_DETECTION, "open-adapter-contactor-before-welding-detection", bool, false);
