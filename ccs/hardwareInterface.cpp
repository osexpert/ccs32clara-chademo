/* Hardware Interface module */

#include "ccs32_globals.h"

//#define CP_DUTY_VALID_TIMER_MAX 3 /* after 3 cycles with 30ms, we consider the CP connection lost, if
                               //      we do not see PWM interrupts anymore */
//#define CONTACTOR_CYCLES_FOR_FULL_PWM (33/5) /* 33 cycles per second. ~200ms should be more than enough, see https://github.com/uhi22/ccs32clara/issues/22  */
//#define CONTACTOR_CYCLES_SEQUENTIAL (33/3) /* ~300ms delay from one contactor to the other, to avoid high peak current consumption. https://github.com/uhi22/ccs32clara/issues/22  */

//static float cpDuty_Percent;
//static uint8_t cpDutyValidTimer;
//static uint8_t ContactorRequest;
//static int8_t ContactorOnTimer1, ContactorOnTimer2;
//static uint16_t dutyContactor1, dutyContactor2;

//static uint8_t LedBlinkDivider;

//static uint16_t lockTimer;
//static bool actuatorTestRunning = false;

//#define ACTUTEST_STATUS_IDLE 0
//#define ACTUTEST_STATUS_LOCKING_TRIGGERED 1
//#define ACTUTEST_STATUS_UNLOCKING_TRIGGERED 2

void hardwareInterface_showOnDisplay(char*, char*, char*)
{

}


void hardwareInterface_initDisplay(void)
{

}

int hardwareInterface_sanityCheck()
{
   return 0; /* 0 is OK */
}

//uint16_t hwIf_simulatedSoc_0p01;

void hardwareInterface_simulatePreCharge(void)
{
}

void hardwareInterface_simulateCharging(void)
{
}

int16_t hardwareInterface_getInletVoltage(void)
{
    // we have no inlet voltage sensor. 
    return Param::Get(Param::EvseVoltage);
}

int16_t hardwareInterface_getAccuVoltage(void)
{
   return Param::GetInt(Param::BatteryVoltage);
}

int16_t hardwareInterface_getChargingTargetVoltage(void)
{
   return Param::GetInt(Param::TargetVoltage);
}

int16_t hardwareInterface_getChargingTargetCurrent(void)
{
    return Param::GetInt(Param::EVTargetCurrent);
}

uint8_t hardwareInterface_getSoc(void)
{
   /* SOC in percent */
   return Param::GetInt(Param::soc);
}

uint8_t hardwareInterface_getIsAccuFull(void)
{
   return Param::GetInt(Param::soc) > 95;
}

void hardwareInterface_setPowerRelayOn(void)
{
    printf("close adapter contactor\r\n");
    DigIo::contactor_out.Set();
}

void hardwareInterface_setPowerRelayOff(void)
{
    printf("open adapter contactor\r\n");
    DigIo::contactor_out.Clear();
}

void hardwareInterface_setStateB(void)
{
   DigIo::state_c_out_inverted.Set();
}

void hardwareInterface_setStateC(void)
{
   DigIo::state_c_out_inverted.Clear();
}

void hardwareInterface_triggerConnectorLocking(void)
{
    Param::Set(Param::LockState, LOCK_CLOSED);
}

void hardwareInterface_triggerConnectorUnlocking(void)
{
    Param::Set(Param::LockState, LOCK_OPEN);
}

uint8_t hardwareInterface_isConnectorLocked(void)
{
    // no locking here anymore.
    // we use this to make the state machine not progress until we have battery/target voltage

    bool canCont = 
        Param::Get(Param::LockState) == LOCK_CLOSED
        // FIXME: can we get this reliably from chademo before this point?`???????????????????????????????????????????????????????????????????????????
        && Param::Get(Param::BatteryVoltage) > 0
        && Param::Get(Param::MaxVoltage) > 0
        && Param::Get(Param::TargetVoltage) > 0;

    ; /* no matter whether we have a real lock or just a simulated one, always give the state. */

    if (canCont)
        printf("LOCK_CLOSED, BatteryVoltage > 0 && MaxVoltage > 0 && TargetVoltage > 0 satisfied\r\n");

    return canCont;
}

uint8_t hardwareInterface_getPowerRelayConfirmation(void)
{
   /* todo */
   return 1;
}

bool hardwareInterface_stopChargeRequested()
{
    uint8_t stopReason = STOP_REASON_NONE;

    if (DigIo::stop_button_in_inverted.Get() == false)//  pushbutton_isPressed500ms()) 
    {
        stopReason = STOP_REASON_BUTTON;
        Param::SetInt(Param::StopReason, stopReason);
        addToTrace(MOD_HWIF, "User pressed the stop button.");
    }

    if (!Param::GetBool(Param::enable)) 
    {
        stopReason = STOP_REASON_MISSING_ENABLE;
        Param::SetInt(Param::StopReason, stopReason);
        addToTrace(MOD_HWIF, "Got enable=false.");
    }


    //if (Param::GetInt(Param::CanWatchdog) >= CAN_TIMEOUT)// && (Param::GetInt(Param::DemoControl) != DEMOCONTROL_STANDALONE)) {
    //{
    //    stopReason = STOP_REASON_CAN_TIMEOUT;
    //    Param::SetInt(Param::StopReason, stopReason);
    //    addToTrace(MOD_HWIF, "Timeout of CanWatchdog.");
    //}

    return (stopReason!=STOP_REASON_NONE);
}

void hardwareInterface_resetSimulation(void)
{
}

/* send the measured CP duty cycle and PP resistance etc to the serial console for debugging. */
void hardwareInterface_LogTheCpPpPhysicalData(void) 
{
//      addToTrace(MOD_HWIF, "cpDuty [%] ", (int16_t)Param::GetInt(Param::ControlPilotDuty));
//      addToTrace(MOD_HWIF, "AdcProximityPilot ", (int16_t)Param::GetInt(Param::AdcProximityPilot));
//      addToTrace(MOD_HWIF, "ResistanceProxPilot [ohm] ", (int16_t)Param::GetInt(Param::ResistanceProxPilot));
//      addToTrace(MOD_HWIF, "HardwareVariant ", (int16_t)Param::GetInt(Param::HardwareVariant));
}
