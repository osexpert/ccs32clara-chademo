/* wakecontrol: controlling the keep_power_on and similar things */

#include "ccs32_globals.h"

//static uint8_t wakecontrol_timer;
//static uint8_t allowSleep;

#define WAKECONTROL_TIMER_MAX 20 /* 20*100ms = 2s cycle time */
#define WAKECONTROL_TIMER_NEARLY_EXPIRED 5 /* 5*100ms = 500ms keep_power_on activation time */
#define WAKECONTROL_TIMER_END_OF_CYCLE__MEASUREMENT_ALLOWED 3 /* after turning the keep_power_on, 200ms time for
                                                                in-rush and adc sampling until measurement is considered as valid. */

//uint8_t wakecontrol_isPpMeasurementInvalid(void) {
//    /* The PP measurement is not valid, if the voltage on the PP is pulled up by the wakeup path.
//       Discussion was here: https://openinverter.org/forum/viewtopic.php?p=75629#p75629 */
//    if (allowSleep==0) return 0; /* as long as we are not ready to sleep, the PP is valid. */
//    if (wakecontrol_timer<=WAKECONTROL_TIMER_END_OF_CYCLE__MEASUREMENT_ALLOWED) return 0; /* valid because cyclic pulsing and sufficient propagation delay */
//    return 1; /* no PP measurement possible, because corrupted by KEEP_POWER_ON. */
//}

void wakecontrol_mainfunction(void) /* runs in 100ms cycle */
{
    //static uint32_t lastValidCp = 0;
    //if (Param::GetInt(Param::ControlPilotDuty) > 3)
    //    lastValidCp = rtc_get_counter();// rtc_get_counter_val();

    ////If no frequency on CP we allow shut down after 10s
    //if ((rtc_get_counter_val() - lastValidCp) > 1000)
    //{
    //    bool ppValid = Param::GetInt(Param::ResistanceProxPilot) < 2000;

    //    bool CanActive = Param::GetInt(Param::CanAwake);

    //    //WAKEUP_ONVALIDPP implies that we use PP for wakeup. So as long as PP is valid
    //    //Do not clear the supply pin as that will skew the PP measurement and we can't turn off anyway
    //    if ((Param::GetInt(Param::WakeupPinFunc) & WAKEUP_ONVALIDPP) == 0 || !ppValid)
    //    {
    //        allowSleep = !CanActive;
    //    }
    //}

    //if (!allowSleep) {
    //    DigIo::keep_power_on.Set(); /* Keep the power on */
    //    wakecontrol_timer=WAKECONTROL_TIMER_MAX;
    //} else {
    //    /* we could go to sleep. But there may be hardware situations, when we keep running, even if we turned-off the keep_power_on.
    //       In this case, we need to set the keep_power_on to active, to allow correct PP resistance measurement. */
    //    if (wakecontrol_timer==WAKECONTROL_TIMER_MAX) {
    //        /* at the beginning of the "sleep allowed" phase, we try to shutdown */
    //        DigIo::keep_power_on.Clear();
    //        wakecontrol_timer--;
    //    } else if (wakecontrol_timer==WAKECONTROL_TIMER_NEARLY_EXPIRED) {
    //        /* we tried to shut down, but something keeps us running. So turn the keep_power_on active for a moment. */
    //        DigIo::keep_power_on.Set();
    //        wakecontrol_timer--;
    //    } else if (wakecontrol_timer==0) {
    //        /* timer is expired. Start a new cycle. */
    //        wakecontrol_timer = WAKECONTROL_TIMER_MAX;
    //    } else {
    //        /* just in the middle of the counting */
    //        wakecontrol_timer--;
    //    }

    //}
}

void wakecontrol_init(void) {
    //DigIo::keep_power_on.Set(); /* Make sure board stays awake. */
    //allowSleep=0;
}


