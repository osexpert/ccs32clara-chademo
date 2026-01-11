#include "ccs32_globals.h"


uint8_t mofi_state;
uint8_t mofi_stateDelay;

void modemFinder_Mainfunction(void) {
    if ((connMgr_getConnectionLevel() == CONNLEVEL_5_ETH_LINK_PRESENT) && (mofi_state == 0)) {
        /* We want the modem search only, if no connection is present at all. */
        addToTrace(MOD_MODEMFINDER, "[ModemFinder] Starting modem search");
        composeGetSwReq();
        myEthTransmit();
        setCheckpoint(6); /* "checking for local modem presence" */
        numberOfSoftwareVersionResponses = 0; /* we want to count the modems. Start from zero. */
        mofi_stateDelay = 15; /* 0.5s should be sufficient to receive the software versions from the modems */
        mofi_state = 1;
        return;
    }
    if (mofi_state == 1) {
        /* waiting for responses of the modems */
        if (mofi_stateDelay > 0) {
            mofi_stateDelay--;
            return;
        }
        /* waiting time is expired. Lets look how many responses we got. */
        addToTrace(MOD_MODEMFINDER, "[ModemFinder] Number of modems %d", numberOfSoftwareVersionResponses);
        if (numberOfSoftwareVersionResponses > 0) {
            // connMgr_ModemFinderOk(numberOfSoftwareVersionResponses);
            // This seems wrong....IF we for some reason find more than 1 modem here (I sometimes do), we will skip SLAC and go directly to CONNLEVEL_20_TWO_MODEMS_FOUND!
            // So always say we found just 1 (local) modem here, else SLAC can sometimes be skipped (typicalle when plug is left in car, and charging restarted)
            connMgr_ModemLocalOk();
        }
        mofi_stateDelay = 15; /* 0.5s to show the number of modems, before we start a new search if necessary */
        mofi_state = 2;
        return;
    }
    if (mofi_state == 2) {
        /* just waiting, to give the user time to read the result. */
        if (mofi_stateDelay > 0) {
            mofi_stateDelay--;
            return;
        }
        mofi_state = 0; /* back to idle state */
    }
}
