/* Homeplug message handling */

#include "ccs32_globals.h"



#define CM_SET_KEY  0x6008
#define CM_GET_KEY  0x600C
#define CM_SC_JOIN  0x6010
#define CM_CHAN_EST  0x6014
#define CM_TM_UPDATE  0x6018
#define CM_AMP_MAP  0x601C
#define CM_BRG_INFO  0x6020
#define CM_CONN_NEW  0x6024
#define CM_CONN_REL  0x6028
#define CM_CONN_MOD  0x602C
#define CM_CONN_INFO  0x6030
#define CM_STA_CAP  0x6034
#define CM_NW_INFO  0x6038
#define CM_GET_BEACON  0x603C
#define CM_HFID  0x6040
#define CM_MME_ERROR  0x6044
#define CM_NW_STATS  0x6048
#define CM_SLAC_PARAM  0x6064
#define CM_START_ATTEN_CHAR  0x6068
#define CM_ATTEN_CHAR  0x606C
#define CM_PKCS_CERT  0x6070
#define CM_MNBC_SOUND  0x6074
#define CM_VALIDATE  0x6078
#define CM_SLAC_MATCH  0x607C
#define CM_SLAC_USER_DATA  0x6080
#define CM_ATTEN_PROFILE  0x6084
#define CM_GET_SW  0xA000

#define MMTYPE_REQ  0x0000
#define MMTYPE_CNF  0x0001
#define MMTYPE_IND  0x0002
#define MMTYPE_RSP  0x0003

#define STATE_INITIAL                      0
#define STATE_SEND_SLAC_PARAM_REQ          1
#define STATE_WAITING_FOR_SLAC_PARAM_CNF   2
#define STATE_SLAC_PARAM_CNF_RECEIVED      3
#define STATE_START_ATTEN_CHAR             4
#define STATE_SOUNDING                     5
#define STATE_WAIT_FOR_ATTEN_CHAR_IND      6
#define STATE_SEND_SLAC_MATCH_REQ          7
#define STATE_WAITING_FOR_SLAC_MATCH_CNF   8
#define STATE_WAITING_FOR_SET_KEY_CNF      9

#define iAmPev 1 /* This project is intended only for PEV mode at the moment. */
#define iAmEvse 0

static const uint8_t MAC_BROADCAST[6] = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff};
uint8_t myMAC[6] = {0xFE, 0xED, 0xBE, 0xEF, 0xAF, 0xFE}; // FEED BEEF *
uint8_t evseMac[6];

static uint8_t NID[7];
static uint8_t NMK[16];
static uint8_t pevSequenceState;
static uint16_t pevSequenceCyclesInState;
static uint16_t cyclesSinceStartAttenChar;
static uint16_t pevTotalCycles;
static uint8_t sdpDelayCycles;
static uint8_t slacDelayCycles;
static uint8_t nRemainingStartAttenChar;
static uint8_t remainingNumberOfSounds;
static uint8_t SdpRepetitionCounter;
static uint8_t sdp_state;
static uint16_t AttenCharIndCount;
static uint8_t LowestAvgAtten;

/********** local prototypes *****************************************/
static void composeAttenCharRsp(const uint8_t* destMac);
static void slac_enterState(int n);
static void composeSetKey();
static void runSlacStateMachine();

/*********************************************************************************/
/* Extracting the EtherType from a received message. */
uint16_t getEtherType(uint8_t* messagebufferbytearray)
{
    uint16_t etherType = (messagebufferbytearray[12] << 8) | messagebufferbytearray[13];
    return etherType;
}

static void cleanTransmitBuffer(void)
{
    /* fill the complete ethernet transmit buffer with 0x00 */
    memset(myethtransmitbuffer, 0, MY_ETH_TRANSMIT_BUFFER_LEN);
}

static uint16_t getManagementMessageType(void)
{
    /* calculates the MMTYPE (base value + lower two bits), see Table 11-2 of homeplug spec */
    return (myethreceivebuffer[16] << 8) | myethreceivebuffer[15];
}

void composeGetSwReq(void)
{
    /* GET_SW.REQ request, as used by the win10 laptop */
    myethtransmitbufferLen = 60;
    cleanTransmitBuffer();
    memcpy(&myethtransmitbuffer[0], MAC_BROADCAST, 6); // Destination MAC
    memcpy(&myethtransmitbuffer[6], myMAC, 6); // Source MAC
    /* Protocol */
    myethtransmitbuffer[12] = 0x88; // Protocol HomeplugAV
    myethtransmitbuffer[13] = 0xE1; //
    myethtransmitbuffer[14] = 0x00; // version
    myethtransmitbuffer[15] = 0x00; // GET_SW.REQ
    myethtransmitbuffer[16] = 0xA0; //

    myethtransmitbuffer[17] = 0x00; // 17-19 Vendor OUI
    myethtransmitbuffer[18] = 0xB0; //
    myethtransmitbuffer[19] = 0x52; //
}

static void composeSlacParamReq(void)
{
    /* SLAC_PARAM request, as it was recorded 2021-12-17 WP charger 2 */
    myethtransmitbufferLen = 60;
    cleanTransmitBuffer();
    memcpy(&myethtransmitbuffer[0], MAC_BROADCAST, 6); // Destination MAC
    memcpy(&myethtransmitbuffer[6], myMAC, 6); // Source MAC
    // Protocol
    myethtransmitbuffer[12] = 0x88; // Protocol HomeplugAV
    myethtransmitbuffer[13] = 0xE1; //
    myethtransmitbuffer[14] = 0x01; // version
    myethtransmitbuffer[15] = 0x64; // SLAC_PARAM.REQ
    myethtransmitbuffer[16] = 0x60; //
    myethtransmitbuffer[17] = 0x00; // 2 bytes fragmentation information. 0000 means: unfragmented.
    myethtransmitbuffer[18] = 0x00; //
    // MME
    auto mme = &myethtransmitbuffer[19];
    mme[0] = 0x00; // apptype
    mme[1] = 0x00; // sectype
    memcpy(&mme[2], myMAC, 6); // 2-9: 8 bytes runid. The Ioniq uses the PEV mac plus 00 00.
}

static void evaluateSlacParamCnf(void)
{
    /* As PEV, we receive the first response from the charger. */
    _global.ccsLifesign = true;
    addToTrace(MOD_HOMEPLUG, "[PEVSLAC] Checkpoint102: received SLAC_PARAM.CNF");
    setCheckpoint(102);
    if (iAmPev)
    {
        if (pevSequenceState == STATE_WAITING_FOR_SLAC_PARAM_CNF) //  we were waiting for the SlacParamCnf
        {
            auto mme = &myethreceivebuffer[19];

            // check runId 17-24 (8 bytes), but ignore the 2 last bytes (set to 0).
            if (memcmp(&mme[17], myMAC, 6) != 0) {
                addToTrace(MOD_HOMEPLUG, "[PEVSLAC] SLAC_PARAM.CNF runId mismatch. Ignore.");
                return; // Not our session (crosstalk), ignore
            }

            slac_enterState(STATE_SLAC_PARAM_CNF_RECEIVED);
        }
    }
}

static void composeStartAttenCharInd(void)
{
    /* reference: see wireshark interpreted frame from ioniq */
    myethtransmitbufferLen = 60;
    cleanTransmitBuffer();
    memcpy(&myethtransmitbuffer[0], MAC_BROADCAST, 6); // Destination MAC
    memcpy(&myethtransmitbuffer[6], myMAC, 6); // Source MAC
    // Protocol
    myethtransmitbuffer[12] = 0x88; // Protocol HomeplugAV
    myethtransmitbuffer[13] = 0xE1; //
    myethtransmitbuffer[14] = 0x01; // version
    myethtransmitbuffer[15] = 0x6A; // START_ATTEN_CHAR.IND
    myethtransmitbuffer[16] = 0x60; //
    myethtransmitbuffer[17] = 0x00; // 2 bytes fragmentation information. 0000 means: unfragmented.
    myethtransmitbuffer[18] = 0x00; //
    // MME
    auto mme = &myethtransmitbuffer[19];
    mme[0] = 0x00; // apptype
    mme[1] = 0x00; // sectype
    mme[2] = 0x0a; // number of sounds: 10
    mme[3] = 6; // timeout N*100ms. Normally 6, means in 600ms all sounds must have been tranmitted.
    mme[4] = 0x01; // response type
    memcpy(&mme[5], myMAC, 6); // 5-10: sound_forwarding_sta, MAC of the PEV
    memcpy(&mme[11], myMAC, 6); // 11-18: runid, filled with MAC of PEV and two bytes 00 00
}

static void composeNmbcSoundInd(void)
{
    /* reference: see wireshark interpreted frame from Ioniq */
    myethtransmitbufferLen = 71;
    cleanTransmitBuffer();
    memcpy(&myethtransmitbuffer[0], MAC_BROADCAST, 6); // Destination MAC
    memcpy(&myethtransmitbuffer[6], myMAC, 6); // Source MAC
    // Protocol
    myethtransmitbuffer[12] = 0x88; // Protocol HomeplugAV
    myethtransmitbuffer[13] = 0xE1; //
    myethtransmitbuffer[14] = 0x01; // version
    myethtransmitbuffer[15] = 0x76; // NMBC_SOUND.IND
    myethtransmitbuffer[16] = 0x60; //
    myethtransmitbuffer[17] = 0x00; // 2 bytes fragmentation information. 0000 means: unfragmented.
    myethtransmitbuffer[18] = 0x00; //
    // MME
    auto mme = &myethtransmitbuffer[19];
    mme[0] = 0x00; // apptype
    mme[1] = 0x00; // sectype
    // 2-18 sender ID, all 00
    mme[19] = remainingNumberOfSounds; // countdown. Remaining number of sounds. Starts with 9 and counts down to 0.
    memcpy(&mme[20], myMAC, 6); // 20-27: runid, filled with MAC of PEV and two bytes 00 00
    // 28-35: reserved, all 00
    memset(&mme[36], 0xFF, 16); // 36-51: random number. All 0xff in the ioniq message.
}

static void evaluateAttenCharInd(void)
{
    addToTrace(MOD_HOMEPLUG, "[PEVSLAC] received ATTEN_CHAR.IND");
    if (iAmPev == 1)
    {
        if (pevSequenceState == STATE_WAIT_FOR_ATTEN_CHAR_IND) // we were waiting for the AttenCharInd
        {
            uint8_t* sourceMac = &myethreceivebuffer[6]; // source MAC starts at offset 6
            auto mme = &myethreceivebuffer[19];

            // check runId 8-15 (8 bytes), but ignore the 2 last bytes (set to 0).
            if (memcmp(&mme[8], myMAC, 6) != 0) {
                addToTrace(MOD_HOMEPLUG, "[PEVSLAC] ATTEN_CHAR.IND runId mismatch. Ignore.");
                return; // Not our session (crosstalk), ignore
            }

            uint8_t numberOfSounds = mme[50]; // how many sounds did the charger hear? We sent 10, so it can't possibly be more:-)
            if (numberOfSounds == 0) {
                addToTrace(MOD_HOMEPLUG, "[PEVSLAC] numberOfSounds is 0. Ignore."); // [V2G3-A09-36]
                return;
            }

            uint8_t numGroups = mme[51]; // should always be 58, and at least not more:-)
            uint16_t sumAtten = 0;
            uint8_t validGroups = 0;
            for (uint8_t i = 0; i < min<uint8_t>(numGroups, 58); i++) // limit groups to 58
            {
                uint8_t val = mme[52 + i];
                if (val != 0xFF)  // 0xFF is nonsense and likely a charger bug (under- or overflow), so ignore it: https://arxiv.org/pdf/2404.06635
                {
                    sumAtten += val;
                    validGroups++;
                }
            }

            uint8_t avgAtten = (validGroups > 0) ? (sumAtten / validGroups) : (0xFF - min<uint8_t>(numberOfSounds, 10)); // in case no groups, failover to 0xFF - numberOfSounds (limit to 10)
            bool best = AttenCharIndCount == 0 || avgAtten < LowestAvgAtten;

            addToTrace(MOD_HOMEPLUG, "[PEVSLAC] charger MAC %02x:%02x:%02x:%02x:%02x:%02x sounds:%d groups:%d avgAtten:%d best:%d",
                sourceMac[0], sourceMac[1], sourceMac[2], sourceMac[3], sourceMac[4], sourceMac[5],
                numberOfSounds, numGroups, avgAtten, best);

            composeAttenCharRsp(sourceMac);
            addToTrace(MOD_HOMEPLUG, "[PEVSLAC] transmitting ATTEN_CHAR.RSP...");
            setCheckpoint(140);
            myEthTransmit();

            if (best)
            {
                LowestAvgAtten = avgAtten;
                memcpy(evseMac, sourceMac, 6); // store the MAC of the best (lowest attenuation) charger seen so far
            }

            AttenCharIndCount++;
        }
    }
}

static void composeAttenCharRsp(const uint8_t* destMac)
{
    /* reference: see wireshark interpreted frame from Ioniq */
    myethtransmitbufferLen = 70;
    cleanTransmitBuffer();
    memcpy(&myethtransmitbuffer[0], destMac, 6); // Destination MAC
    memcpy(&myethtransmitbuffer[6], myMAC, 6); // Source MAC
    // Protocol
    myethtransmitbuffer[12] = 0x88; // Protocol HomeplugAV
    myethtransmitbuffer[13] = 0xE1; //
    myethtransmitbuffer[14] = 0x01; // version
    myethtransmitbuffer[15] = 0x6F; // ATTEN_CHAR.RSP
    myethtransmitbuffer[16] = 0x60; //
    myethtransmitbuffer[17] = 0x00; // 2 bytes fragmentation information. 0000 means: unfragmented.
    myethtransmitbuffer[18] = 0x00; //
    // MME
    auto mme = &myethtransmitbuffer[19];
    mme[0] = 0x00; // apptype
    mme[1] = 0x00; // sectype
    memcpy(&mme[2], myMAC, 6); // 2-7: source MAC
    memcpy(&mme[8], myMAC, 6); // 8-15: runid. The PEV mac, plus 00 00.
    // 16-32: source_id, all 00
    // 33-49: resp_id, all 00
    // 50: result. 0 is ok
}

static void composeSlacMatchReq(void)
{
    /* reference: see wireshark interpreted frame from Ioniq */
    myethtransmitbufferLen = 85;
    cleanTransmitBuffer();
    memcpy(&myethtransmitbuffer[0], evseMac, 6); // Destination MAC
    memcpy(&myethtransmitbuffer[6], myMAC, 6); // Source MAC
    // Protocol
    myethtransmitbuffer[12] = 0x88; // Protocol HomeplugAV
    myethtransmitbuffer[13] = 0xE1; //
    myethtransmitbuffer[14] = 0x01; // version
    myethtransmitbuffer[15] = 0x7C; // SLAC_MATCH.REQ
    myethtransmitbuffer[16] = 0x60; //
    myethtransmitbuffer[17] = 0x00; // 2 bytes fragmentation information. 0000 means: unfragmented.
    myethtransmitbuffer[18] = 0x00; //
    // MME 
    // NOTE: in my copy of ISO 15118-3:2015 the indexes of fields PEV ID, EVSE ID, EVSE MAC are incorrect. You had one job...
    auto mme = &myethtransmitbuffer[19];
    mme[0] = 0x00; // apptype
    mme[1] = 0x00; // sectype
    mme[2] = 0x3E; // 2-3: length
    mme[3] = 0x00;
    // 4-20: pev_id, all 00
    memcpy(&mme[21], myMAC, 6); // 21-26: PEV MAC
    // 27-43: evse_id, all 00
    memcpy(&mme[44], evseMac, 6); // 44-49: EVSE MAC
    memcpy(&mme[50], myMAC, 6); // 50-57: runid. The PEV mac, plus 00 00.
    // 58-65: reserved, all 00
}

static void evaluateSlacMatchCnf(void)
{
    if (pevSequenceState != STATE_WAITING_FOR_SLAC_MATCH_CNF)
    {
        addToTrace(MOD_HOMEPLUG, "[PEVSLAC] received SLAC_MATCH.CNF in unexpected state %d, ignoring", pevSequenceState);
        return;
    }

    // The SLAC_MATCH.CNF contains the NMK and the NID.
    // We extract this information, so that we can use it for the CM_SET_KEY afterwards.
    // References: https://github.com/qca/open-plc-utils/blob/master/slac/evse_cm_slac_match.c
    // 2021-12-16_HPC_säule1_full_slac.pcapng
    if (iAmEvse == 1)
    {
        // If we are EVSE, nothing to do. We have sent the match.CNF by our own.
        // The SET_KEY was already done at startup.
    }
    else
    {
        auto mme = &myethreceivebuffer[19];

        // check runId (8 bytes), but ignore the 2 last bytes (set to 0).
        if (memcmp(&mme[50], myMAC, 6) != 0) {
            addToTrace(MOD_HOMEPLUG, "[PEVSLAC] SLAC_MATCH.CNF runId mismatch. Ignore.");
            return; // Not our session (crosstalk), ignore
        }

        memcpy(NID, &mme[66], 7);   // NID has 7 bytes
        memcpy(NMK, &mme[74], 16);
        addToTrace(MOD_HOMEPLUG, "[PEVSLAC] From SLAC_MATCH.CNF, got network membership key (NMK) and NID.");

        // use the extracted NMK and NID to set the key in the adaptor:
        composeSetKey();
        addToTrace(MOD_HOMEPLUG, "[PEVSLAC] Checkpoint170: transmitting SET_KEY.REQ");
        setCheckpoint(170);
        myEthTransmit();

        slac_enterState(STATE_WAITING_FOR_SET_KEY_CNF);
    }
}

static void composeSetKey(void)
{
    /* CM_SET_KEY.REQ request */
    /* From example trace from catphish https://openinverter.org/forum/viewtopic.php?p=40558&sid=9c23d8c3842e95c4cf42173996803241#p40558
       Table 11-88 in the homeplug_av21_specification_final_public.pdf */
    myethtransmitbufferLen = 60;
    cleanTransmitBuffer();
    memcpy(&myethtransmitbuffer[0], MAC_BROADCAST, 6); // Destination MAC
    memcpy(&myethtransmitbuffer[6], myMAC, 6); // Source MAC
    // Protocol
    myethtransmitbuffer[12] = 0x88; // Protocol HomeplugAV
    myethtransmitbuffer[13] = 0xE1; //
    myethtransmitbuffer[14] = 0x01; // version
    myethtransmitbuffer[15] = 0x08; // CM_SET_KEY.REQ
    myethtransmitbuffer[16] = 0x60; //
    myethtransmitbuffer[17] = 0x00; // frag_index
    myethtransmitbuffer[18] = 0x00; // frag_seqnum
    // MME
    auto mme = &myethtransmitbuffer[19];
    mme[0] = 0x01; // 0 key type. 0x01 is NMK.

    mme[1] = 0xaa; // 1-4 my nonce
    mme[2] = 0xaa;
    mme[3] = 0xaa;
    mme[4] = 0xaa;

    mme[5] = 0x00; // 5-8 your nonce
    mme[6] = 0x00;
    mme[7] = 0x00;
    mme[8] = 0x00;

    mme[9] = 0x04; // 9 nw pid. 0x04 is HLE protocol.

    mme[10] = 0x00; // 10-11 prn
    mme[11] = 0x00;
    mme[12] = 0x00; // 12 pmn
    mme[13] = 0x00; // 13 cco cap
    memcpy(&mme[14], NID, 7); // 14-20 nid
    // Network ID to be associated with the key distributed herein.
    // The 54 LSBs of this field contain the NID (refer to Section 3.4.3.1). The two MSBs shall be set to 0b00.
    mme[21] = 0x01; // 21 peks (payload encryption key select) Table 11-83. 0x01 is NMK.
    // with 0x0F we could choose "no key, payload is sent in the clear"
    memcpy(&mme[22], NMK, 16); // 22 to 37: NMK

#define variation 0
    mme[22] += variation; // to try different NMKs
}

static void evaluateSetKeyCnf(void)
{
    if (pevSequenceState != STATE_WAITING_FOR_SET_KEY_CNF)
    {
        addToTrace(MOD_HOMEPLUG, "[PEVSLAC] received SET_KEY.CNF in unexpected state %d, ignoring", pevSequenceState);
        return;
    }

    // The Setkey confirmation
    uint8_t result;
    // In spec, the result 0 means "success". But in reality, the 0 means: did not work. When it works,
    // then the LEDs are blinking (device is restarting), and the response is 1.
    // open-plc-utils do the same: https://github.com/qca/open-plc-utils/blob/358dfcf78bdaf7b0b13dcdf91cb1aae1789f2770/slac/evse_cm_set_key.c
    // if (! confirm->RESULT) return (slac_debug(session, session->exit, __func__, "Device refused request"));
	// So for some reason, they did not follow the spec:-)

    addToTrace(MOD_HOMEPLUG, "[PEVSLAC] received SET_KEY.CNF");
    result = myethreceivebuffer[19];
    if (result == 0)
    {
        //this would be a bad sign for local modem, but normal for remote
        addToTrace(MOD_HOMEPLUG, "[PEVSLAC] SET_KEY.CNF says 0: Device refused request.");
    }
    else
    {
        addToTrace(MOD_HOMEPLUG, "[PEVSLAC] SET_KEY.CNF says %d: Success.", result);
        slac_enterState(STATE_INITIAL); // this would have happened anyways, but for readability
        connMgr_setLevel(CONNLEVEL_15_SLAC_DONE_SDP_NEXT);
    }
}

void readModemVersions(void)
{
    composeGetSwReq();
    myEthTransmit();
}

void evaluateGetSwCnf(void)
{
    /* The GET_SW confirmation. This contains the software version of the homeplug modem.
       Reference: see wireshark interpreted frame from TPlink, Ioniq and Alpitronic charger */
    uint8_t i, x;
    addToTrace(MOD_HOMEPLUG, "[PEVSLAC] received GET_SW.CNF");
    uint8_t* sourceMac = &myethreceivebuffer[6];

    uint8_t verLen = myethreceivebuffer[22];
    if ((verLen > 0) && (verLen < 0x30))
    {
        char strVersion[200];

        for (i = 0; i < verLen; i++)
        {
            x = myethreceivebuffer[23 + i];
            if (x < 0x20)
            {
                x = 0x20;   /* make unprintable character to space. */
            }
            strVersion[i] = x;
        }
        strVersion[i] = 0;
        addToTrace(MOD_HOMEPLUG, "[PEVSLAC] modem MAC %02x:%02x:%02x:%02x:%02x:%02x software version %s",
            sourceMac[0], sourceMac[1], sourceMac[2], sourceMac[3], sourceMac[4], sourceMac[5],
            strVersion);
    }
}

void slac_enterState(int n)
{
    addToTrace(MOD_HOMEPLUG, "[PEVSLAC] from %d entering %d", pevSequenceState, n);
    pevSequenceState = n;
    pevSequenceCyclesInState = 0;
    if (n == STATE_INITIAL) pevTotalCycles = 0;
}

void runSlacSequencer()
{
    if (connMgr_getLevel() != CONNLEVEL_10_START_SLAC)
    {
        if (pevSequenceState != STATE_INITIAL) slac_enterState(STATE_INITIAL);
        return;
    }

    pevSequenceCyclesInState++;
    pevTotalCycles++;
    if (pevSequenceState >= STATE_START_ATTEN_CHAR)
        cyclesSinceStartAttenChar++;
    else
        cyclesSinceStartAttenChar = 0;

    if (pevTotalCycles > 500) // 15s timeout for SLAC in total.
    {
        addToTrace(MOD_HOMEPLUG, "[PEVSLAC] ERROR: Timeout");
        slac_enterState(STATE_INITIAL);
    }

    if (slacDelayCycles > 0)
    {
        slacDelayCycles--;
    }
    else
    {
        runSlacStateMachine();
    }
}

void runSlacStateMachine()
{
    // state machine
    if (pevSequenceState == STATE_INITIAL)
    {
        slac_enterState(STATE_SEND_SLAC_PARAM_REQ);
    }
    else if (pevSequenceState == STATE_SEND_SLAC_PARAM_REQ)
    {
        addToTrace(MOD_HOMEPLUG, "[PEVSLAC] Checkpoint100: Sending SLAC_PARAM.REQ...");
        setCheckpoint(100);
        composeSlacParamReq();
        myEthTransmit();
        slac_enterState(STATE_WAITING_FOR_SLAC_PARAM_CNF);
    }
    else if (pevSequenceState == STATE_WAITING_FOR_SLAC_PARAM_CNF) // Waiting for slac_param confirmation.
    {
        // [V2G3-A09-07] wait TT_match_response:200ms until any charger answer with SLAC_PARAM_CNF.
        if (pevSequenceCyclesInState > 7) // TT_match_response:200ms
        {
            addToTrace(MOD_HOMEPLUG, "[PEVSLAC] Timeout while waiting for SLAC_PARAM.CNF");
            slacDelayCycles = 33 - 7; // delay before retry, no need to flood
            slac_enterState(STATE_INITIAL);
        }
        // evaluateSlacParamCnf() calls slac_enterState(STATE_SLAC_PARAM_CNF_RECEIVED)
    }
    else if (pevSequenceState == STATE_SLAC_PARAM_CNF_RECEIVED) // slac_param confirmation was received.
    {
        // [V2G3-A09-25] wait TP_match_sequence:100ms from SLAC_PARAM.CNF to START_ATTEN_CHAR.
        if (pevSequenceCyclesInState > 3) // wait for 90ms + 1 state change
        {
            nRemainingStartAttenChar = 3; // There shall be 3 START_ATTEN_CHAR messages.
            slac_enterState(STATE_START_ATTEN_CHAR);
        }
    }
    else if (pevSequenceState == STATE_START_ATTEN_CHAR) // received SLAC_PARAM.CNF. Multiple transmissions of START_ATTEN_CHAR.
    {
        if (nRemainingStartAttenChar > 0)
        {
            nRemainingStartAttenChar -= 1;
            composeStartAttenCharInd();
            addToTrace(MOD_HOMEPLUG, "[PEVSLAC] transmitting START_ATTEN_CHAR.IND...");
            myEthTransmit();
        }
        else
        {
            // all three START_ATTEN_CHAR.IND are finished. Now we send 10 MNBC_SOUND.IND
            // Delay shall be 20ms to 50ms. So the normal 30ms call cycle is perfect.
            remainingNumberOfSounds = 10; // We shall transmit 10 sound messages.
            slac_enterState(STATE_SOUNDING);
        }
    }
    else if (pevSequenceState == STATE_SOUNDING) // Multiple transmissions of MNBC_SOUND.IND.
    {
        if (remainingNumberOfSounds > 0)
        {
            remainingNumberOfSounds -= 1;
            composeNmbcSoundInd();
            addToTrace(MOD_HOMEPLUG, "[PEVSLAC] transmitting MNBC_SOUND.IND..."); // original from ioniq is 40ms after the last START_ATTEN_CHAR.IND
            setCheckpoint(104);
            myEthTransmit();
            if (remainingNumberOfSounds == 0)
            {
                LowestAvgAtten = 0xFF; // reset to max
                AttenCharIndCount = 0;
                slac_enterState(STATE_WAIT_FOR_ATTEN_CHAR_IND); // move fast to the next state, so that a fast response is catched in the correct state
            }
            // Shall be 20ms to 50ms. So we set to 0 and the normal 30ms call cycle is perfect.
        }
    }
    else if (pevSequenceState == STATE_WAIT_FOR_ATTEN_CHAR_IND) // waiting for ATTEN_CHAR.IND
    {
        // [V2G3-A09-31]
        // TT_EV_atten_results: Time EV should wait for all ATTEN_CHAR.IND, from first START_ATTEN_CHAR.IND is sent: 1200ms
        // All chargers that sent SLAC_PARAM.CNF will normally report their ATTEN_CHAR.IND as well, but also allowed for a previosly "silent" charger
        // to report its ATTEN_CHAR.IND. The one with the lowest avgAtten should win. If a tie, there may be a complicated selection process that may include BCB-toggle.
        // The car may wait the full 1.2sec if it want, to be sure it collect ATTEN_CHAR.IND from all chargers.
        // In theory, neighbour chargers could listen and send as well (crosstalk). In reality, I only seen one answer (from the charger we are connected to).
        // original from ioniq is 860ms to 980ms from ATTEN_CHAR.RSP to SLAC_MATCH.REQ
        if (cyclesSinceStartAttenChar > 40) // 1.2sec
        {
            if (AttenCharIndCount > 0)
            {
                addToTrace(MOD_HOMEPLUG, "[PEVSLAC] Received %d ATTEN_CHAR.IND", AttenCharIndCount);
                slac_enterState(STATE_SEND_SLAC_MATCH_REQ);
            }
            else
            {
                addToTrace(MOD_HOMEPLUG, "[PEVSLAC] Timeout waiting for ATTEN_CHAR.IND");
                slac_enterState(STATE_INITIAL);
            }
        }
    }
    else if (pevSequenceState == STATE_SEND_SLAC_MATCH_REQ)
    {
        composeSlacMatchReq();
        addToTrace(MOD_HOMEPLUG, "[PEVSLAC] Checkpoint150: transmitting SLAC_MATCH.REQ...");
        setCheckpoint(150);
        myEthTransmit();
        slac_enterState(STATE_WAITING_FOR_SLAC_MATCH_CNF);
    }
    else if (pevSequenceState == STATE_WAITING_FOR_SLAC_MATCH_CNF)
    {
        if (pevSequenceCyclesInState > 7) // TT_match_response:200ms
        {
            addToTrace(MOD_HOMEPLUG, "[PEVSLAC] Timeout waiting for SLAC_MATCH.CNF");
            slac_enterState(STATE_INITIAL);
        }
        // evaluateSlacMatchCnf() will bring us further into STATE_WAITING_FOR_SET_KEY_CNF after it send SET_KEY.REQ
    }
    else if (pevSequenceState == STATE_WAITING_FOR_SET_KEY_CNF)
    {
        // This response time does not seem to be covered by the spec. Spec has TT_match_join:12s but this cover a much larger "area".
        // Worst case in my logs: 664ms
        if (pevSequenceCyclesInState > 33) // 1s
        {
            addToTrace(MOD_HOMEPLUG, "[PEVSLAC] Timeout waiting for SET_KEY.CNF");
            slac_enterState(STATE_INITIAL);
        }
        // evaluateSetKeyCnf() will call connMgr_SlacOk() and get us out of here and into SDP....or stay until ConnMgr timeout:-)
    }
    else
    {
        // invalid state is reached. As robustness measure, go to initial state.
        addToTrace(MOD_HOMEPLUG, "[PEVSLAC] ERROR: Invalid state reached");
        slac_enterState(STATE_INITIAL);
    }
}

static int sdpRecoveryState = 0;
static int sdpRecoveryDelay = 0;

void runSdpRecoveryStateMachine(void)
{
    if (connMgr_getLevel() != CONNLEVEL_5_SDP_RECOVERY)
    {
        sdpRecoveryState = 0;
        return;
    }

    if (connMgr_sdpDoneTrigger()) // SDP done before, so restarting, try fast path
    {
        if (sdpRecoveryState == 0)
        {
            addToTrace(MOD_HOMEPLUG, "[SDP] Try SDP recovery before SLAC...");
            ipv6_initiateSdpRequest();
            sdpRecoveryState = 1;
            sdpRecoveryDelay = 15; // 0.5s
        }
        else if (sdpRecoveryState == 1)
        {
            // AI suggest 300ms delay and total 1-1.5s, but currently just sending one request and wait for 500ms.
            if (sdpRecoveryDelay > 0)
            {
                sdpRecoveryDelay--;
            }
            else
            {
                // We are still here after 0.5 seconds, so SDP recovery did not fly. Go to slac.
                // Why would we not be here? evaluateUdpPayload would have taken us into connMgr_setLevel(CONNLEVEL_50_SDP_DONE_TCP_NEXT);
                // TODO: reset sdpDoneTrigger = false so we only try recovery once? Does it matter?
                connMgr_setLevel(CONNLEVEL_10_START_SLAC);
            }
        }
    }
    else // cold start, directly to SLAC
    {
        connMgr_setLevel(CONNLEVEL_10_START_SLAC);
    }
}

void runSdpStateMachine(void)
{
    if (connMgr_getLevel() != CONNLEVEL_15_SLAC_DONE_SDP_NEXT)
    {
        sdp_state = 0;
        return;
    }

    if (sdp_state == 0)
    {
        // Next step is to discover the chargers communication controller (SECC) using discovery protocol (SDP).
        addToTrace(MOD_HOMEPLUG, "[SDP] Checkpoint200: Starting SDP.");
        setCheckpoint(200);
        sdpDelayCycles = 0;
        SdpRepetitionCounter = 50; // prepare the number of retries for the SDP. The more the better.
        sdp_state = 1;
    }
    else if (sdp_state == 1) // SDP request transmission and waiting for SDP response.
    {
        /* The normal state transition in case of received SDP response is done in
           the IPv6 receive handler. This will inform the ConnectionManager, and we will stop here
           because of the increased ConnectionLevel. */
        if (sdpDelayCycles > 0)
        {
            // just waiting until next action
            sdpDelayCycles -= 1;
        }
        else if (SdpRepetitionCounter > 0)
        {
            // Reference: The Ioniq waits 4.1s from the slac_match.cnf to the SDP request.
            // Here we send the SdpRequest. Maybe too early, but we will retry if there is no response.
            ipv6_initiateSdpRequest();
            SdpRepetitionCounter -= 1;
            sdpDelayCycles = 15; // e.g. half-a-second delay until re-try of the SDP
        }
        else
        {
            // All repetitions are over, no SDP response was seen. Back to the beginning.
            addToTrace(MOD_HOMEPLUG, "[SDP] ERROR: Did not receive SDP response -> restart");
            sdp_state = 0;
            connMgr_restart();
        }
    }
}

void evaluateReceivedHomeplugPacket(void)
{
    if (connMgr_getLevel() >= CONNLEVEL_80_TCP_CONNECTED) {
        /* we have TCP traffic running, so we ignore all homeplug management packets. This
        makes us robust against cross-talk from other charging cables.
        Discussion here: https://github.com/uhi22/ccs32clara/issues/24 */
        addToTrace(MOD_HOMEPLUG, "[HOMEPLUG] Ignoring homeplug message, because high level communication is ongoing.");
        return;
    }
    switch (getManagementMessageType())
    {
    case CM_SLAC_MATCH | MMTYPE_CNF:
        evaluateSlacMatchCnf();
        break;
    case CM_SLAC_PARAM | MMTYPE_CNF:
        evaluateSlacParamCnf();
        break;
    case CM_ATTEN_CHAR | MMTYPE_IND:
        evaluateAttenCharInd();
        break;
    case CM_SET_KEY | MMTYPE_CNF:
        evaluateSetKeyCnf();
        break;
    case CM_GET_SW | MMTYPE_CNF:
        evaluateGetSwCnf();
        break;
    }
}

void setOurMac(uint8_t* newMac)
{
    memcpy(myMAC, newMac, 6);
}

const uint8_t* getOurMac()
{
    return myMAC;
}
