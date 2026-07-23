
// values that can be patched directly in the firmware

CONFIG_ITEM(ADAPTER_MAX_AMPS, "max-amps", uint8_t, 200);

// Default SOC% at which the adapter will stop charging
CONFIG_ITEM(STOP_CHARGING_SOC, "stop-charging-soc", uint8_t, 100);

//Porsche Taycan requires 750V, but setting this value to 750 might break compatibility with many chargers. As default value 500V is good!
CONFIG_ITEM(ADAPTER_MAX_VOLTS, "max-volts", uint16_t, 500); 

// Just some value seen in can logs.. seen 10, 15. Some say car will allow 10A deviation from what you say? So if we always say we use 10A, we can use anything between 0-20A? 
// For any current above 20A then need real measured amps? Try 40.
CONFIG_ITEM(MAX_DISCHARGE_AMPS_FALLBACK, "max-discharge-amps-fallback", uint8_t, 40);

CONFIG_ITEM(DYNAMIC_CONTROL_FALLBACK, "dynamic-control-fallback", bool, true);

CONFIG_ITEM(CONFIG_SX, "sx", bool, false);

CONFIG_ITEM(CONFIG_V2X, "v2x", bool, false);

CONFIG_ITEM(CONFIG_ALWAYS_ON, "always-on", bool, false);

CONFIG_ITEM(CONFIG_ALT_ESTIMATED_VOLTAGE, "alt-estimated-voltage", uint8_t, 0);

// Based on logs, worst case is 1600ms before asking for amps, but use 2000 for now.
CONFIG_ITEM(CHADEMO_PRE1_WaitForCarContactorsClosed_MS, "wait-for-contactors-closed-ms", uint16_t, 2000);

// Fake output current towards the car. May need to use 2 or 5 to make it work on all cars?
CONFIG_ITEM(CHADEMO_FAKE_IDLE_AMPS, "fake-idle-amps", uint8_t, 1);

CONFIG_ITEM(CHADEMO_VOLTAGE_MODULATION, "voltage-modulation", bool, false);

CONFIG_ITEM(DX_CCS_WaitForPreChargeStart_MS, "wait-for-precharge-start-ms", uint16_t, 2000);

CONFIG_ITEM(CHADEMO_MAX_UNDERSUPPLY_AMPS, "max-undersupply-amps", uint8_t, 10);
