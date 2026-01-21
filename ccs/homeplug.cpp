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

#define STATE_INITIAL  0
#define STATE_SEND_SLAC_PARAM_REQ        1
#define STATE_WAITING_FOR_SLAC_PARAM_CNF   2
#define STATE_SLAC_PARAM_CNF_RECEIVED      3
#define STATE_BEFORE_START_ATTEN_CHAR      4
#define STATE_SOUNDING                     5
#define STATE_WAIT_FOR_ATTEN_CHAR_IND      6
#define STATE_ATTEN_CHAR_IND_RECEIVED      7
#define STATE_DELAY_BEFORE_MATCH           8
#define STATE_WAITING_FOR_SLAC_MATCH_CNF   9
#define STATE_WAITING_FOR_SET_KEY_CNF   10

#define iAmPev 1 /* This project is intended only for PEV mode at the moment. */
#define iAmEvse 0

static const uint8_t MAC_BROADCAST[6] = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff};
uint8_t myMAC[6] = {0xFE, 0xED, 0xBE, 0xEF, 0xAF, 0xFE};
uint8_t evseMac[6];
uint8_t numberOfSoftwareVersionResponses;

static uint8_t verLen;
static uint8_t sourceMac[6];
static uint8_t NID[7];
static uint8_t NMK[16];
static uint8_t pevSequenceState;
static uint16_t pevSequenceCyclesInState;
static uint16_t pevTotalCycles;
static uint16_t pevSequenceDelayCycles;
static uint8_t nRemainingStartAttenChar;
static uint8_t remainingNumberOfSounds;
static uint8_t AttenCharIndNumberOfSounds;
static uint8_t SdpRepetitionCounter;
static uint8_t sdp_state;
//static uint8_t nEvseModemMissingCounter;

/********** local prototypes *****************************************/
static void composeAttenCharRsp(void);
static void slac_enterState(int n);
static void composeSetKey(void);

/*********************************************************************************/
/* Extracting the EtherType from a received message. */
uint16_t getEtherType(uint8_t* messagebufferbytearray)
{
    uint16_t etherType = 0;
    etherType = messagebufferbytearray[12] * 256 + messagebufferbytearray[13];
    return etherType;
}

void fillSourceMac(const uint8_t* mac, uint8_t offset)
{
    /* at offset 6 in the ethernet frame, we have the source MAC.
       we can give a different offset, to re-use the MAC also in the data area */
    memcpy(&myethtransmitbuffer[offset], mac, 6);
}

void fillDestinationMac(const uint8_t* mac, uint8_t offset)
{
    /* at offset 0 in the ethernet frame, we have the destination MAC.
       we can give a different offset, to re-use the MAC also in the data area */
    memcpy(&myethtransmitbuffer[offset], mac, 6);
}

static void cleanTransmitBuffer(void)
{
    /* fill the complete ethernet transmit buffer with 0x00 */
    int i;
    for (i = 0; i < MY_ETH_TRANSMIT_BUFFER_LEN; i++)
    {
        myethtransmitbuffer[i] = 0;
    }
}

static void setNmkAt(uint8_t index)
{
    /* sets the Network Membership Key (NMK) at a certain position in the transmit buffer */
    uint8_t i;
    for (i = 0; i < 16; i++)
    {
        myethtransmitbuffer[index + i] = NMK[i]; // NMK
    }
}

static void setNidAt(uint8_t index)
{
    /* copies the network ID (NID, 7 bytes) into the wished position in the transmit buffer */
    uint8_t i;
    for (i = 0; i < 7; i++)
    {
        myethtransmitbuffer[index + i] = NID[i];
    }
}

static uint16_t getManagementMessageType(void)
{
    /* calculates the MMTYPE (base value + lower two bits), see Table 11-2 of homeplug spec */
    return (myethreceivebuffer[16] << 8) + myethreceivebuffer[15];
}

void composeGetSwReq(void)
{
    /* GET_SW.REQ request, as used by the win10 laptop */
    myethtransmitbufferLen = 60;
    cleanTransmitBuffer();
    /* Destination MAC */
    fillDestinationMac(MAC_BROADCAST, 0);
    /* Source MAC */
    fillSourceMac(myMAC, 6);
    /* Protocol */
    myethtransmitbuffer[12] = 0x88; // Protocol HomeplugAV
    myethtransmitbuffer[13] = 0xE1; //
    myethtransmitbuffer[14] = 0x00; // version
    myethtransmitbuffer[15] = 0x00; // GET_SW.REQ
    myethtransmitbuffer[16] = 0xA0; //
    myethtransmitbuffer[17] = 0x00; // Vendor OUI
    myethtransmitbuffer[18] = 0xB0; //
    myethtransmitbuffer[19] = 0x52; //
}

static void composeSlacParamReq(void)
{
    /* SLAC_PARAM request, as it was recorded 2021-12-17 WP charger 2 */
    myethtransmitbufferLen = 60;
    cleanTransmitBuffer();
    // Destination MAC
    fillDestinationMac(MAC_BROADCAST, 0);
    // Source MAC
    fillSourceMac(myMAC, 6);
    // Protocol
    myethtransmitbuffer[12] = 0x88; // Protocol HomeplugAV
    myethtransmitbuffer[13] = 0xE1; //
    myethtransmitbuffer[14] = 0x01; // version
    myethtransmitbuffer[15] = 0x64; // SLAC_PARAM.REQ
    myethtransmitbuffer[16] = 0x60; //
    myethtransmitbuffer[17] = 0x00; // 2 bytes fragmentation information. 0000 means: unfragmented.
    myethtransmitbuffer[18] = 0x00; //
    myethtransmitbuffer[19] = 0x00; //
    myethtransmitbuffer[20] = 0x00; //
    fillSourceMac(myMAC, 21); // 21 to 28: 8 bytes runid. The Ioniq uses the PEV mac plus 00 00.
    myethtransmitbuffer[27] = 0x00; //
    myethtransmitbuffer[28] = 0x00; //
    // rest is 00
}

static void evaluateSlacParamCnf(void)
{
    /* As PEV, we receive the first response from the charger. */
    addToTrace(MOD_HOMEPLUG, "[PEVSLAC] Checkpoint102: received SLAC_PARAM.CNF");
    setCheckpoint(102);
    if (iAmPev)
    {
        if (pevSequenceState == STATE_WAITING_FOR_SLAC_PARAM_CNF)   //  we were waiting for the SlacParamCnf
        {
            pevSequenceDelayCycles = 4; // original Ioniq is waiting 200ms
            slac_enterState(STATE_SLAC_PARAM_CNF_RECEIVED); // enter next state. Will be handled in the cyclic runSlacSequencer
        }
    }
}

static void composeStartAttenCharInd(void)
{
    /* reference: see wireshark interpreted frame from ioniq */
    myethtransmitbufferLen = 60;
    cleanTransmitBuffer();
    // Destination MAC
    fillDestinationMac(MAC_BROADCAST, 0);
    // Source MAC
    fillSourceMac(myMAC, 6);
    // Protocol
    myethtransmitbuffer[12] = 0x88; // Protocol HomeplugAV
    myethtransmitbuffer[13] = 0xE1; //
    myethtransmitbuffer[14] = 0x01; // version
    myethtransmitbuffer[15] = 0x6A; // START_ATTEN_CHAR.IND
    myethtransmitbuffer[16] = 0x60; //
    myethtransmitbuffer[17] = 0x00; // 2 bytes fragmentation information. 0000 means: unfragmented.
    myethtransmitbuffer[18] = 0x00; //
    myethtransmitbuffer[19] = 0x00; // apptype
    myethtransmitbuffer[20] = 0x00; // sectype
    myethtransmitbuffer[21] = 0x0a; // number of sounds: 10
    myethtransmitbuffer[22] = 6; // timeout N*100ms. Normally 6, means in 600ms all sounds must have been tranmitted.
    // Todo: As long we are a little bit slow, lets give 1000ms instead of 600, so that the
    // charger is able to catch it all.
    myethtransmitbuffer[23] = 0x01; // response type
    fillSourceMac(myMAC, 24); // 24 to 29: sound_forwarding_sta, MAC of the PEV
    fillSourceMac(myMAC, 30); // 30 to 37: runid, filled with MAC of PEV and two bytes 00 00
    // rest is 00
}

static void composeNmbcSoundInd(void)
{
    /* reference: see wireshark interpreted frame from Ioniq */
    uint8_t i;
    myethtransmitbufferLen = 71;
    cleanTransmitBuffer();
    //Destination MAC
    fillDestinationMac(MAC_BROADCAST, 0);
    // Source MAC
    fillSourceMac(myMAC, 6);
    // Protocol
    myethtransmitbuffer[12] = 0x88; // Protocol HomeplugAV
    myethtransmitbuffer[13] = 0xE1; //
    myethtransmitbuffer[14] = 0x01; // version
    myethtransmitbuffer[15] = 0x76; // NMBC_SOUND.IND
    myethtransmitbuffer[16] = 0x60; //
    myethtransmitbuffer[17] = 0x00; // 2 bytes fragmentation information. 0000 means: unfragmented.
    myethtransmitbuffer[18] = 0x00; //
    myethtransmitbuffer[19] = 0x00; // apptype
    myethtransmitbuffer[20] = 0x00; // sectype
    myethtransmitbuffer[21] = 0x00; // 21 to 37 sender ID, all 00
    myethtransmitbuffer[38] = remainingNumberOfSounds; // countdown. Remaining number of sounds. Starts with 9 and counts down to 0.
    fillSourceMac(myMAC, 39); // 39 to 46: runid, filled with MAC of PEV and two bytes 00 00
    myethtransmitbuffer[47] = 0x00; // 47 to 54: reserved, all 00
    //55 to 70: random number. All 0xff in the ioniq message.
    for (i = 55; i < 71; i++)   // i in range(55, 71):
    {
        myethtransmitbuffer[i] = 0xFF;
    }
}

static void evaluateAttenCharInd(void)
{
    uint8_t i;
    addToTrace(MOD_HOMEPLUG, "[PEVSLAC] received ATTEN_CHAR.IND");
    if (iAmPev == 1)
    {
        //addToTrace("[PEVSLAC] received AttenCharInd in state %d", pevSequenceState);
        if (pevSequenceState == STATE_WAIT_FOR_ATTEN_CHAR_IND)   // we were waiting for the AttenCharInd
        {
            //todo: Handle the case when we receive multiple responses from different chargers.
            //      Wait a certain time, and compare the attenuation profiles. Decide for the nearest charger.
            //Take the MAC of the charger from the frame, and store it for later use.
            for (i = 0; i < 6; i++)
            {
                evseMac[i] = myethreceivebuffer[6 + i]; // source MAC starts at offset 6
            }
            AttenCharIndNumberOfSounds = myethreceivebuffer[69];
            //addToTrace("[PEVSLAC] number of sounds reported by the EVSE (should be 10): %d", AttenCharIndNumberOfSounds);
            composeAttenCharRsp();
            addToTrace(MOD_HOMEPLUG, "[PEVSLAC] transmitting ATTEN_CHAR.RSP...");
            setCheckpoint(140);
            myEthTransmit();

            slac_enterState(STATE_ATTEN_CHAR_IND_RECEIVED); // enter next state. Will be handled in the cyclic runSlacSequencer
        }
    }
}

static void composeAttenCharRsp(void)
{
    /* reference: see wireshark interpreted frame from Ioniq */
    myethtransmitbufferLen = 70;
    cleanTransmitBuffer();
    // Destination MAC
    fillDestinationMac(evseMac, 0);
    // Source MAC
    fillSourceMac(myMAC, 6);
    // Protocol
    myethtransmitbuffer[12] = 0x88; // Protocol HomeplugAV
    myethtransmitbuffer[13] = 0xE1; //
    myethtransmitbuffer[14] = 0x01; // version
    myethtransmitbuffer[15] = 0x6F; // ATTEN_CHAR.RSP
    myethtransmitbuffer[16] = 0x60; //
    myethtransmitbuffer[17] = 0x00; // 2 bytes fragmentation information. 0000 means: unfragmented.
    myethtransmitbuffer[18] = 0x00; //
    myethtransmitbuffer[19] = 0x00; // apptype
    myethtransmitbuffer[20] = 0x00; // sectype
    fillSourceMac(myMAC, 21); // 21 to 26: source MAC
    fillDestinationMac(myMAC, 27); // 27 to 34: runid. The PEV mac, plus 00 00.
    // 35 to 51: source_id, all 00
    // 52 to 68: resp_id, all 00
    // 69: result. 0 is ok
}

static void composeSlacMatchReq(void)
{
    /* reference: see wireshark interpreted frame from Ioniq */
    myethtransmitbufferLen = 85;
    cleanTransmitBuffer();
    // Destination MAC
    fillDestinationMac(evseMac, 0);
    // Source MAC
    fillSourceMac(myMAC, 6);
    // Protocol
    myethtransmitbuffer[12] = 0x88; // Protocol HomeplugAV
    myethtransmitbuffer[13] = 0xE1; //
    myethtransmitbuffer[14] = 0x01; // version
    myethtransmitbuffer[15] = 0x7C; // SLAC_MATCH.REQ
    myethtransmitbuffer[16] = 0x60; //
    myethtransmitbuffer[17] = 0x00; // 2 bytes fragmentation information. 0000 means: unfragmented.
    myethtransmitbuffer[18] = 0x00; //
    myethtransmitbuffer[19] = 0x00; // apptype
    myethtransmitbuffer[20] = 0x00; // sectype
    myethtransmitbuffer[21] = 0x3E; // 21 to 22: length
    myethtransmitbuffer[22] = 0x00; //
    // 23 to 39: pev_id, all 00
    fillSourceMac(myMAC, 40); // 40 to 45: PEV MAC
    // 46 to 62: evse_id, all 00
    fillDestinationMac(evseMac, 63); // 63 to 68: EVSE MAC
    fillSourceMac(myMAC, 69); // 69 to 76: runid. The PEV mac, plus 00 00.
    // 77 to 84: reserved, all 00
}

static void evaluateSlacMatchCnf(void)
{
    if (pevSequenceState != STATE_WAITING_FOR_SLAC_MATCH_CNF)
    {
        addToTrace(MOD_HOMEPLUG, "[PEVSLAC] received SLAC_MATCH.CNF in unexpected state %d, ignoring", pevSequenceState);
        return;
    }

    uint8_t i;
    uint8_t blIsDestinationMacForMe;
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
        blIsDestinationMacForMe = 1;
        for (i = 0; i < 6; i++) {
            /* compare all 6 bytes of the destination MAC with our own MAC */
            if (myethreceivebuffer[i] != myMAC[i]) {
                blIsDestinationMacForMe = 0; /* any mismatch -> it is not for me */
            }
        }
        if (!blIsDestinationMacForMe) {
            addToTrace(MOD_HOMEPLUG, "[PEVSLAC] received SLAC_MATCH.CNF but with foreign destination MAC. Ignoring.");
        }
        else {
            addToTrace(MOD_HOMEPLUG, "[PEVSLAC] received SLAC_MATCH.CNF");
            for (i = 0; i < 7; i++)   // NID has 7 bytes
            {
                NID[i] = myethreceivebuffer[85 + i];
            }
            for (i = 0; i < 16; i++)
            {
                NMK[i] = myethreceivebuffer[93 + i];
            }
            addToTrace(MOD_HOMEPLUG, "[PEVSLAC] From SlacMatchCnf, got network membership key (NMK) and NID.");

            // use the extracted NMK and NID to set the key in the adaptor:
            composeSetKey();
            addToTrace(MOD_HOMEPLUG, "[PEVSLAC] Checkpoint170: transmitting SET_KEY.REQ");
            setCheckpoint(170);
            myEthTransmit();

            slac_enterState(STATE_WAITING_FOR_SET_KEY_CNF);
        }
    }
}

static void composeSetKey(void)
{
    /* CM_SET_KEY.REQ request */
    /* From example trace from catphish https://openinverter.org/forum/viewtopic.php?p=40558&sid=9c23d8c3842e95c4cf42173996803241#p40558
       Table 11-88 in the homeplug_av21_specification_final_public.pdf */
    myethtransmitbufferLen = 60;
    cleanTransmitBuffer();
    // Destination MAC
    fillDestinationMac(MAC_BROADCAST, 0);
    // Source MAC
    fillSourceMac(myMAC, 6);
    // Protocol
    myethtransmitbuffer[12] = 0x88; // Protocol HomeplugAV
    myethtransmitbuffer[13] = 0xE1; //
    myethtransmitbuffer[14] = 0x01; // version
    myethtransmitbuffer[15] = 0x08; // CM_SET_KEY.REQ
    myethtransmitbuffer[16] = 0x60; //
    myethtransmitbuffer[17] = 0x00; // frag_index
    myethtransmitbuffer[18] = 0x00; // frag_seqnum
    myethtransmitbuffer[19] = 0x01; // 0 key info type

    myethtransmitbuffer[20] = 0xaa; // 1 my nonce
    myethtransmitbuffer[21] = 0xaa; // 2
    myethtransmitbuffer[22] = 0xaa; // 3
    myethtransmitbuffer[23] = 0xaa; // 4

    myethtransmitbuffer[24] = 0x00; // 5 your nonce
    myethtransmitbuffer[25] = 0x00; // 6
    myethtransmitbuffer[26] = 0x00; // 7
    myethtransmitbuffer[27] = 0x00; // 8

    myethtransmitbuffer[28] = 0x04; // 9 nw info pid

    myethtransmitbuffer[29] = 0x00; // 10 info prn
    myethtransmitbuffer[30] = 0x00; // 11
    myethtransmitbuffer[31] = 0x00; // 12 pmn
    myethtransmitbuffer[32] = 0x00; // 13 cco cap
    setNidAt(33); // 14-20 nid  7 bytes from 33 to 39
    //          Network ID to be associated with the key distributed herein.
    //          The 54 LSBs of this field contain the NID (refer to Section 3.4.3.1). The
    //          two MSBs shall be set to 0b00.
    myethtransmitbuffer[40] = 0x01; // 21 peks (payload encryption key select) Table 11-83. 01 is NMK. We had 02 here, why???
    // with 0x0F we could choose "no key, payload is sent in the clear"
    setNmkAt(41);
#define variation 0
    myethtransmitbuffer[41] += variation; // to try different NMKs
    // and three remaining zeros
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
        addToTrace(MOD_HOMEPLUG, "[PEVSLAC] SetKeyCnf says 0: Device refused request.");
    }
    else
    {
        addToTrace(MOD_HOMEPLUG, "[PEVSLAC] SetKeyCnf says %d: Success.", result);
        slac_enterState(STATE_INITIAL); // this would have happened anyways, but for readability
        connMgr_setLevel(CONNLEVEL_15_SLAC_DONE_SDP_NEXT);
    }
}

#if false
static void composeGetKey(void)
{
    /* CM_GET_KEY.REQ request
       from https://github.com/uhi22/plctool2/blob/master/listen_to_eth.c
       and homeplug_av21_specification_final_public.pdf */
    myethtransmitbufferLen = 60;
    cleanTransmitBuffer();
    // Destination MAC
    fillDestinationMac(MAC_BROADCAST, 0);
    // Source MAC
    fillSourceMac(myMAC, 6);
    // Protocol
    myethtransmitbuffer[12] = 0x88; // Protocol HomeplugAV
    myethtransmitbuffer[13] = 0xE1;
    myethtransmitbuffer[14] = 0x01; // version
    myethtransmitbuffer[15] = 0x0C; // CM_GET_KEY.REQ https://github.com/uhi22/plctool2/blob/master/plc_homeplug.h
    myethtransmitbuffer[16] = 0x60; //
    myethtransmitbuffer[17] = 0x00; // 2 bytes fragmentation information. 0000 means: unfragmented.
    myethtransmitbuffer[18] = 0x00; //
    myethtransmitbuffer[19] = 0x00; // 0 Request Type 0=direct
    myethtransmitbuffer[20] = 0x01; // 1 RequestedKeyType only "NMK" is permitted over the H1 interface.
    //           value see HomeplugAV2.1 spec table 11-89. 1 means AES-128.

    setNidAt(21); // NID starts here (table 11-91 Homeplug spec is wrong. Verified by accepted command.)
    myethtransmitbuffer[28] = 0xaa; // 10-13 mynonce. The position at 28 is verified by the response of the devolo.
    myethtransmitbuffer[29] = 0xaa; //
    myethtransmitbuffer[30] = 0xaa; //
    myethtransmitbuffer[31] = 0xaa; //
    myethtransmitbuffer[32] = 0x04; // 14 PID. According to  ISO15118-3 fix value 4, "HLE protocol"
    myethtransmitbuffer[33] = 0x00; // 15-16 PRN Protocol run number
    myethtransmitbuffer[34] = 0x00; //
    myethtransmitbuffer[35] = 0x00; // 17 PMN Protocol message number
}
#endif

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
    numberOfSoftwareVersionResponses += 1;
    for (i = 0; i < 6; i++)
    {
        sourceMac[i] = myethreceivebuffer[6 + i];
    }

    verLen = myethreceivebuffer[22];
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
        addToTrace(MOD_HOMEPLUG, "[PEVSLAC] MAC %02x:%02x:xx:xx:xx:%02x software version %s",
            sourceMac[0], sourceMac[1], sourceMac[5],
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

void runSlacSequencer(void)
{
    if (connMgr_getLevel() != CONNLEVEL_10_START_SLAC)
    {
        if (pevSequenceState != STATE_INITIAL) slac_enterState(STATE_INITIAL);
        return;
    }

    pevSequenceCyclesInState++;
    pevTotalCycles++;

    // 15s timeout for SLAC in total.
    if (pevSequenceCyclesInState > 500)
    {
        addToTrace(MOD_HOMEPLUG, "[PEVSLAC] ERROR: Timeout");
        slac_enterState(STATE_INITIAL);
    }

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
    else if (pevSequenceState == STATE_WAITING_FOR_SLAC_PARAM_CNF)   // Waiting for slac_param confirmation.
    {
        if (pevSequenceCyclesInState > 33)
        {
            // No response for 1s, this is an error.
            addToTrace(MOD_HOMEPLUG, "[PEVSLAC] Timeout while waiting for SLAC_PARAM.CNF");
            slac_enterState(STATE_INITIAL);
        }
        // (the normal state transition is done in the reception handler)
    }
    else if (pevSequenceState == STATE_SLAC_PARAM_CNF_RECEIVED)   // slac_param confirmation was received.
    {
        pevSequenceDelayCycles = 1; //  1*30=30ms as preparation for the next state.
        //  Between the SLAC_PARAM.CNF and the first START_ATTEN_CHAR.IND the Ioniq waits 100ms.
        //  The allowed time TP_match_sequence is 0 to 100ms.
        //  Alpitronic and ABB chargers are more tolerant, they worked with a delay of approx
        //  250ms. In contrast, Supercharger and Compleo do not respond anymore if we
        //  wait so long.
        nRemainingStartAttenChar = 3; // There shall be 3 START_ATTEN_CHAR messages.
        slac_enterState(STATE_BEFORE_START_ATTEN_CHAR);
    }
    else if (pevSequenceState == STATE_BEFORE_START_ATTEN_CHAR)   // received SLAC_PARAM.CNF. Multiple transmissions of START_ATTEN_CHAR.
    {
        if (pevSequenceDelayCycles > 0)
        {
            pevSequenceDelayCycles -= 1;
        }
        // The delay time is over. Let's transmit.
        else if (nRemainingStartAttenChar > 0)
        {
            nRemainingStartAttenChar -= 1;
            composeStartAttenCharInd();
            addToTrace(MOD_HOMEPLUG, "[PEVSLAC] transmitting START_ATTEN_CHAR.IND...");
            myEthTransmit();
            pevSequenceDelayCycles = 0; // original from ioniq is 20ms between the START_ATTEN_CHAR. Shall be 20ms to 50ms. So we set to 0 and the normal 30ms call cycle is perfect.
        }
        else
        {
            // all three START_ATTEN_CHAR.IND are finished. Now we send 10 MNBC_SOUND.IND
            pevSequenceDelayCycles = 0; // original from ioniq is 40ms after the last START_ATTEN_CHAR.IND.
            // Shall be 20ms to 50ms. So we set to 0 and the normal 30ms call cycle is perfect.
            remainingNumberOfSounds = 10; // We shall transmit 10 sound messages.
            slac_enterState(STATE_SOUNDING);
        }
    }
    else if (pevSequenceState == STATE_SOUNDING)   // Multiple transmissions of MNBC_SOUND.IND.
    {
        if (pevSequenceDelayCycles > 0)
        {
            pevSequenceDelayCycles -= 1;
        }
        else if (remainingNumberOfSounds > 0)
        {
            remainingNumberOfSounds -= 1;
            composeNmbcSoundInd();
            addToTrace(MOD_HOMEPLUG, "[PEVSLAC] transmitting MNBC_SOUND.IND..."); // original from ioniq is 40ms after the last START_ATTEN_CHAR.IND
            setCheckpoint(104);
            myEthTransmit();
            if (remainingNumberOfSounds == 0)
            {
                slac_enterState(STATE_WAIT_FOR_ATTEN_CHAR_IND); // move fast to the next state, so that a fast response is catched in the correct state
            }
            pevSequenceDelayCycles = 0; // original from ioniq is 20ms between the messages.
            // Shall be 20ms to 50ms. So we set to 0 and the normal 30ms call cycle is perfect.
        }
    }
    else if (pevSequenceState == STATE_WAIT_FOR_ATTEN_CHAR_IND)   // waiting for ATTEN_CHAR.IND
    {
        // TODO: it is possible that we receive this message from multiple chargers. We need
        // to select the charger with the loudest reported signals.

        // TT_EV_atten_results: Time EV should wait for ATTEN_CHAR.IND, from first START_ATTEN_CHAR.IND is sent: 1200ms
        // Since TT_EV_atten_results include 2 more START_ATTEN_CHAR.IND and 10 x NMBC_SOUND.IND, waiting additional 1s should be plenty.
        if (pevSequenceCyclesInState > 33) // 1s
        {
            addToTrace(MOD_HOMEPLUG, "[PEVSLAC] Timeout waiting for ATTEN_CHAR.IND");
            slac_enterState(STATE_INITIAL);
        }
        // (the normal state transition is done in the reception handler)
    }
    else if (pevSequenceState == STATE_ATTEN_CHAR_IND_RECEIVED)   // ATTEN_CHAR.IND was received and the nearest charger decided and the ATTEN_CHAR.RSP was sent.
    {
        slac_enterState(STATE_DELAY_BEFORE_MATCH);
        pevSequenceDelayCycles = 30; // original from ioniq is 860ms to 980ms from ATTEN_CHAR.RSP to SLAC_MATCH.REQ
    }
    else if (pevSequenceState == STATE_DELAY_BEFORE_MATCH)   // Waiting time before SLAC_MATCH.REQ
    {
        if (pevSequenceDelayCycles > 0)
        {
            pevSequenceDelayCycles -= 1;
        }
        else
        {
            composeSlacMatchReq();
            addToTrace(MOD_HOMEPLUG, "[PEVSLAC] Checkpoint150: transmitting SLAC_MATCH.REQ...");
            setCheckpoint(150);
            myEthTransmit();
            slac_enterState(STATE_WAITING_FOR_SLAC_MATCH_CNF);
        }
    }
    else if (pevSequenceState == STATE_WAITING_FOR_SLAC_MATCH_CNF)
    {
        if (pevSequenceCyclesInState > 66) // 2s
        {
            addToTrace(MOD_HOMEPLUG, "[PEVSLAC] Timeout waiting for SLAC_MATCH.CNF");
            slac_enterState(STATE_INITIAL);
        }
        // evaluateSlacMatchCnf() will bring us further into STATE_WAITING_FOR_SET_KEY_CNF after it send SET_KEY.REQ
    }
    else if (pevSequenceState == STATE_WAITING_FOR_SET_KEY_CNF)
    {
        if (pevSequenceCyclesInState > 33) // 1s
        {
            addToTrace(MOD_HOMEPLUG, "[PEVSLAC] Timeout waiting for SET_KEY.CNF");
            slac_enterState(STATE_INITIAL);
        }
        // evaluateSefKeyCnf() will call connMgr_SlacOk() and get us out of here and into SDP....or stay until ConnMgr timeout:-)
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
            // AI suggest 300ms delay and total 1-?"1.5s, but currently just sending one request and wait for 500ms.
            if (sdpRecoveryDelay > 0)
            {
                sdpRecoveryDelay--;
            }
            else
            {
                // We are still here after 0.5 seconds, so SDP attempt did not fly. Go to slac.
                // Why would we not be here? evaluateUdpPayload would have taken us into connMgr_setLevel(CONNLEVEL_50_SDP_DONE_TCP_NEXT);
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
        pevSequenceDelayCycles = 0;
        SdpRepetitionCounter = 50; // prepare the number of retries for the SDP. The more the better.
        sdp_state = 1;
    }
    else if (sdp_state == 1)   // SDP request transmission and waiting for SDP response.
    {
        /* The normal state transition in case of received SDP response is done in
           the IPv6 receive handler. This will inform the ConnectionManager, and we will stop here
           because of the increased ConnectionLevel. */
        if (pevSequenceDelayCycles > 0)
        {
            // just waiting until next action
            pevSequenceDelayCycles -= 1;
        }
        else if (SdpRepetitionCounter > 0)
        {
            // Reference: The Ioniq waits 4.1s from the slac_match.cnf to the SDP request.
            // Here we send the SdpRequest. Maybe too early, but we will retry if there is no response.
            ipv6_initiateSdpRequest();
            SdpRepetitionCounter -= 1;
            pevSequenceDelayCycles = 15; // e.g. half-a-second delay until re-try of the SDP
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

static void evaluateGetKeyCnf(void) {}

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
    case CM_GET_KEY + MMTYPE_CNF:
        evaluateGetKeyCnf();
        break;
    case CM_SLAC_MATCH + MMTYPE_CNF:
        evaluateSlacMatchCnf();
        break;
    case CM_SLAC_PARAM + MMTYPE_CNF:
        evaluateSlacParamCnf();
        break;
    case CM_ATTEN_CHAR + MMTYPE_IND:
        evaluateAttenCharInd();
        break;
    case CM_SET_KEY + MMTYPE_CNF:
        evaluateSetKeyCnf();
        break;
    case CM_GET_SW + MMTYPE_CNF:
        evaluateGetSwCnf();
        break;
    }
}

void setOurMac(uint8_t* newMac)
{
    for (int i = 0; i < 6; i++)
        myMAC[i] = newMac[i];
}

const uint8_t* getOurMac()
{
    return myMAC;
}
