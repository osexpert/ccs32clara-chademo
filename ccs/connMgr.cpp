/* Connection Manager */

/* This module is informed by the several state machines in case of good connection.
   It calculates an overall ConnectionLevel.
   This ConnectionLevel is provided to the state machines, so that each state machine
   has the possiblity to decide whether it needs to do something or just stays silent.

   The basic rule is, that a good connection on higher layer (e.g. TCP) implicitely
   confirms the good connection on lower layer (e.g. Modem presence). This means,
   the lower-layer state machine can stay silent as long as the upper layers are working
   fine.
*/

#include "ccs32_globals.h"

static uint16_t connMgr_level = CONNLEVEL_5_SDP_RECOVERY;

static uint16_t connMgr_cycles;

static bool sdpDoneTrigger;

//#define CONNMGR_CYCLES_PER_SECOND 33 /* 33 cycles per second, because 30ms cycle time */
//#define CONNMGR_TIMER_5s (5*33) /* 5 seconds until an OkReport is forgotten. */
//#define CONNMGR_TIMER_10s (10*33) /* 10 seconds until an OkReport is forgotten. */
//#define CONNMGR_TIMER_15s (15*33) /* 15 seconds until an OkReport is forgotten. */
//#define CONNMGR_TIMER_20s (20*33) /* 20 seconds until an OkReport is forgotten. */


uint8_t connMgr_getLevel(void)
{
    return connMgr_level;
}

void connMgr_printDebugInfos(void)
{
    addToTrace(MOD_CONNMGR, "[CONNMGR] %d", connMgr_level
    );
}

void connMgr_setLevel(uint16_t level)
{
    if (level != connMgr_level)
    {
        addToTrace(MOD_CONNMGR, "[CONNMGR] Level changed from %d to %d.", connMgr_level, level);
        connMgr_level = level;
    }

    if (level == CONNLEVEL_50_SDP_DONE_TCP_NEXT)
        sdpDoneTrigger = true; // done at least once
}

bool connMgr_sdpDoneTrigger()
{
    return sdpDoneTrigger;
}

void connMgr_restart(void)
{
    connMgr_setLevel(CONNLEVEL_5_SDP_RECOVERY);
}

void connMgr_Mainfunction(void)
{
    if ((connMgr_cycles % 33) == 0)
    {
        /* once per second */
        //connMgr_printDebugInfos();
    }
    connMgr_cycles++;
}


