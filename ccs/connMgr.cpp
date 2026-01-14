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

static uint16_t connMgr_state = CONNLEVEL_5_ETH_LINK_PRESENT; /* we have SPI, so just say ETH is up */
static uint16_t connMgr_timer = 0;

static uint16_t connMgr_cycles;

#define CONNMGR_CYCLES_PER_SECOND 33 /* 33 cycles per second, because 30ms cycle time */
#define CONNMGR_TIMER_5s (5*33) /* 5 seconds until an OkReport is forgotten. */
#define CONNMGR_TIMER_10s (10*33) /* 10 seconds until an OkReport is forgotten. */
#define CONNMGR_TIMER_15s (15*33) /* 15 seconds until an OkReport is forgotten. */
#define CONNMGR_TIMER_20s (20*33) /* 20 seconds until an OkReport is forgotten. */


uint8_t connMgr_getConnectionLevel(void)
{
    return connMgr_state;
}

void connMgr_printDebugInfos(void)
{
    addToTrace(MOD_CONNMGR, "[CONNMGR] %d --> %d",
        connMgr_timer, /* the timeout counter for application communication */
        connMgr_state /* the connection level, calculated based on the timeout counters */
    );
}

void setState(uint16_t newState, uint16_t timeout)
{
    if (newState != connMgr_state)
    {
        addToTrace(MOD_CONNMGR, "[CONNMGR] ConnectionLevel changed from %d to %d.", connMgr_state, newState);
        connMgr_state = newState;
    }
    connMgr_timer = timeout;
}

void connMgr_Mainfunction(void)
{
    if (connMgr_timer > 0)
    {
        connMgr_timer--;
        if (connMgr_timer == 0)
        {
            println("[CONNMGR] Timeout in %d", connMgr_state);
            setState(CONNLEVEL_5_ETH_LINK_PRESENT, 0 /* no timeout */);
        }
    }

    if ((connMgr_cycles % 33) == 0)
    {
        /* once per second */
        connMgr_printDebugInfos();
    }
    connMgr_cycles++;
}

//void connMgr_ModemLocalOk()
//{
//    // slac control own timeout, so use 0
//    setState(CONNLEVEL_10_ONE_MODEM_FOUND, 0);// CONNMGR_TIMER_5s);
//}

void connMgr_SlacOk(void)
{
    /* The SetKey was sent to the local modem. This leads to restart of the
    local modem, and potenially also for the remote modem. If both modems are up,
    they need additional time to pair. We need to be patient during this process. */

    // allow 20s between SLAC (Set-Key-Cnf) and SDP
    setState(CONNLEVEL_15_SLAC_DONE, CONNMGR_TIMER_20s);
}

void connMgr_SdpOk(void)
{
    // allow 5s between SDP ok and TCP connected.
    setState(CONNLEVEL_50_SDP_DONE, CONNMGR_TIMER_5s);
}

void connMgr_TcpOk(void)
{
    if (connMgr_state > CONNLEVEL_80_TCP_RUNNING)
        return;

    // allow 5s between TCP connect and APPL-ok
    setState(CONNLEVEL_80_TCP_RUNNING, CONNMGR_TIMER_5s);
}

void connMgr_ApplOk(uint8_t timeout_in_seconds)
{
    setState(CONNLEVEL_100_APPL_RUNNING, timeout_in_seconds * CONNMGR_CYCLES_PER_SECOND);
}

void connMgr_Restart(void)
{
    setState(CONNLEVEL_5_ETH_LINK_PRESENT, 0 /* no timeout */);
}
