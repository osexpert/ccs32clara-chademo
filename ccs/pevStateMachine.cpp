#include "ccs32_globals.h"
#include "projectExiConnector.h"

#include "main.h"

// Time to wait between different message types (states)
#define MESSAGE_STATE_DELAY_CYCLES 8 // 240ms (Ionic seems to wait minimum 250ms between message types)

// Artificial time to wait after we recieve a Res until we send next Req (same message type loops)
#define MESAGE_LOOP_DELAY_CYCLES 4 // 120ms (Ionic seems to wait 80-90ms between Res and send next Req)

/* The Charging State Machine for the car */
//STATE_ENTRY(internalName, friendlyName, response timeout in s, state timeout in s)
//state timeout has 1 second extra, since we may do some waiting at the start of the message states
#define STATE_LIST \
   STATE_ENTRY(Start, Start, 0, 0) \
   STATE_ENTRY(Connected, Connected, 0, 0) \
   STATE_ENTRY(SupportedApplicationProtocolMsg, NegotiateProtocol, 2, 3) \
   STATE_ENTRY(SessionSetupMsg, SessionSetup, 2, 3) \
   STATE_ENTRY(ServiceDiscoveryMsg, ServiceDiscovery, 2, 3) \
   STATE_ENTRY(ServicePaymentSelectionMsg, PaymentSelection, 2, 3) \
   STATE_ENTRY(ContractAuthenticationMsg, ContractAuthentication, 2, 121) /* spec says 60, 120-150 seems to be recomended for slow backend authorization */ \
   STATE_ENTRY(ChargeParameterDiscoveryMsg, ChargeParameterDiscovery, 2, 31) /* Resp: was 5, but DIN says 2 */ /* State: was 60 but AI thinks 10-20 is more common */ \
   STATE_ENTRY(CableCheckMsg, CableCheck, 2, 61) \
   STATE_ENTRY(PreChargeMsg, PreCharge, 2, 31) /* spec is 7sec, but allow more */ \
   STATE_ENTRY(PowerDeliveryOnMsg, PowerDeliveryOn, 2, 3) /* Resp: DIN: timeout 2sec, ISO: timeout 5sec. But we use DIN. */ \
   STATE_ENTRY(CurrentDemandMsg, CurrentDemand, 1, 0) /* Resp: DIN/ISO: timeout 250ms, but use 1sec. */ \
   STATE_ENTRY(PowerDeliveryOffMsg, PowerDeliveryOff, 2, 3) /* Resp: DIN: timeout 2sec, ISO: timeout 5sec. But we use DIN. */ \
   STATE_ENTRY(WaitForCurrentDownAfterStateB, CurrentDown, 0, 0) \
   STATE_ENTRY(WaitForPowerRelayOff, RelayOff, 0, 10) /* wait max 10sec for adapterContactorOpened */ \
   STATE_ENTRY(WeldingDetectionMsg, WeldingDetection, 2, 11) \
   STATE_ENTRY(SessionStopMsg, SessionStop, 2, 3) \
   STATE_ENTRY(SafeShutDown, SafeShutDown, 0, 0) \
   STATE_ENTRY(SafeShutDownWaitForChargerShutdown, WaitForChargerShutdown, 0, 0) \
   STATE_ENTRY(Stop, Stop, 0, 0) \
   STATE_ENTRY(End, End, 0, 0)

//States enum
#define STATE_ENTRY(name, fname, response_timeout_sec, state_timeout_sec) PEV_STATE_##name,
enum pevstates {
    STATE_LIST
};
#undef STATE_ENTRY

//state function prototypes
#define STATE_ENTRY(name, fname, response_timeout_sec, state_timeout_sec) static void stateFunction##name();
STATE_LIST
#undef STATE_ENTRY

//State function array
#define STATE_ENTRY(name, fname, response_timeout_sec, state_timeout_sec) stateFunction##name,
static void(* const stateFunctions[])() = {
STATE_LIST
};
#undef STATE_ENTRY

#define CCS_CYCLE_MS 30
#define CCS_CYCLES_PER_SEC 33 // but 33 * 30 = 990ms, so 10ms lost
#define SEC_TO_CCS_CYCLES(sec)          ((sec) * 33)
#define SEC_TO_CCS_CYCLES_ROUND_UP(sec) ((sec) * 33 + ((sec) + 2) / 3)

//Timeout array
#define STATE_ENTRY(name, fname, response_timeout_sec, state_timeout_sec) SEC_TO_CCS_CYCLES_ROUND_UP(response_timeout_sec),
static const uint16_t response_timeouts[] = {
STATE_LIST
};
#undef STATE_ENTRY

//Timeout array
#define STATE_ENTRY(name, fname, response_timeout_sec, state_timeout_sec) SEC_TO_CCS_CYCLES_ROUND_UP(state_timeout_sec),
static const uint16_t state_timeouts[] = {
STATE_LIST
};
#undef STATE_ENTRY

//Enum string for data module
#define STATE_ENTRY(name, fname, response_timeout_sec, state_timeout_sec) __COUNTER__=fname,
const char* pevSttString = STRINGIFY(STATE_LIST);
#undef STATE_ENTRY

//String array for logging
#define STATE_ENTRY(name, fname, response_timeout_sec, state_timeout_sec) #fname,
const char* const pevSttLabels[] = { STATE_LIST };
#undef STATE_ENTRY

//#define MAX_NUMBER_OF_WELDING_DETECTION_ROUNDS 10 /* The process time is specified with 1.5s. Ten loops should be fine. */
#define MAX_VOLTAGE_TO_FINISH_WELDING_DETECTION 40 /* 40V is considered to be sufficiently low to not harm. The Ioniq already finishes at 65V. */

#define LEN_OF_EVCCID 6 /* The EVCCID is the MAC according to spec. Ioniq uses exactly these 6 byte. */



/*
<supportedAppProtocolReq xmlns="urn:iso:15118:2010:AppProtocol">
  <AppProtocol>
    <ProtocolNamespace>urn:din:70121:2012:MsgDef</ProtocolNamespace>
    <VersionMajor>1</VersionMajor>
    <VersionMinor>0</VersionMinor>
    <SchemaID>1</SchemaID>
    <Priority>1</Priority>
  </AppProtocol>
</supportedAppProtocolReq>
*/
static const uint8_t exiDemoSupportedApplicationProtocolRequestIoniq[] = { 0x80, 0x00, 0xdb, 0xab, 0x93, 0x71, 0xd3, 0x23, 0x4b, 0x71, 0xd1, 0xb9, 0x81, 0x89, 0x91, 0x89, 0xd1, 0x91, 0x81, 0x89, 0x91, 0xd2, 0x6b, 0x9b, 0x3a, 0x23, 0x2b, 0x30, 0x02, 0x00, 0x00, 0x04, 0x00, 0x40 };


static uint32_t pev_cyclesInState;
static int pev_cyclesSinceReq = -1; // unset
static uint8_t pev_DelayCycles;
static pevstates pev_state = PEV_STATE_Start;
static bool pev_sendMessagePending = false;
static int LastCurrentDemandResPresentVoltage;
static int LastTargetVoltage;
static int LastTargetCurrent;
static bool PrechargeDifferenceIsSmall;
static int sentCurrentDemandMsg;

static bool PresentVoltageDifferentFromTarget;
static bool PresentVoltageDifferentFromTarget_isSet;

static bool PresentCurrentDifferentFromTarget;
static bool PresentCurrentDifferentFromTarget_isSet;

static bool ChargeParameterDiscoveryCompletedTrigger;

/***local function prototypes *****************************************/

static void pev_enterState(pevstates n);

/*** functions ********************************************************/

static float combineValueAndMultiplier(int32_t val, int8_t multiplier)
{
    float x;
    x = val;
    while (multiplier > 0)
    {
        x = x * 10;
        multiplier--;
    }
    while (multiplier < 0)
    {
        x = x / 10;
        multiplier++;
    }
    return x;
}

static float combineValueAndMultiplier(dinPhysicalValueType v)
{
    return combineValueAndMultiplier(v.Value, v.Multiplier);
}

static void addV2GTPHeaderAndTransmit(const uint8_t* exiBuffer, uint8_t exiBufferLen)
{
    // takes the bytearray with exidata, and adds a header to it, according to the Vehicle-to-Grid-Transport-Protocol
    // V2GTP header has 8 bytes
    // 1 byte protocol version
    // 1 byte protocol version inverted
    // 2 bytes payload type
    // 4 byte payload length
    tcpPayload[0] = 0x01; // version
    tcpPayload[1] = 0xfe; // version inverted
    tcpPayload[2] = 0x80; // payload type. 0x8001 means "EXI data"
    tcpPayload[3] = 0x01; //
    tcpPayload[4] = (uint8_t)(exiBufferLen >> 24); // length 4 byte.
    tcpPayload[5] = (uint8_t)(exiBufferLen >> 16);
    tcpPayload[6] = (uint8_t)(exiBufferLen >> 8);
    tcpPayload[7] = (uint8_t)exiBufferLen;
    if (exiBufferLen + 8 < TCP_PAYLOAD_LEN)
    {
        memcpy(&tcpPayload[8], exiBuffer, exiBufferLen);
        tcpPayloadLen = 8 + exiBufferLen; /* 8 byte V2GTP header, plus the EXI data */
        tcp_transmit();
    }
    else
    {
        log(MOD_PEV, "Error: EXI does not fit into tcpPayload.");
    }
}

static void encodeAndTransmit(void)
{
    /* calls the EXI encoder, adds the V2GTP header and sends the result to ethernet */
    //log("before: g_errn=%d", g_errn);
    //log("global_streamEncPos=%d", global_streamEncPos);
    global_streamEncPos = 0;
    projectExiConnector_encode_DinExiDocument();
    //log("after: g_errn=%d", g_errn);
    //log("global_streamEncPos=%d", global_streamEncPos);
 //#ifdef VERBOSE_EXI_DECODER
    if (_global.moreLogging) {
        showAsHex(global_streamEnc.data, global_streamEncPos, "encoded exi");
    }
    //#endif
    addV2GTPHeaderAndTransmit(global_streamEnc.data, global_streamEncPos);
}

static void routeDecoderInputData(void)
{
    /* connect the data from the TCP to the exiDecoder */
    /* The TCP receive data consists of two parts: 1. The V2GTP header and 2. the EXI stream.
       The decoder wants only the EXI stream, so we skip the V2GTP header.
       In best case, we would check also the consistency of the V2GTP header here.
    */
    global_streamDec.data = &tcp_rxdata[V2GTP_HEADER_SIZE];
    global_streamDec.size = tcp_rxdataLen - V2GTP_HEADER_SIZE;
    //#ifdef VERBOSE_EXI_DECODER
    if (_global.moreLogging) {
        showAsHex(global_streamDec.data, global_streamDec.size, "decoder will see");
    }
    //#endif
       /* We have something to decode, this is a good sign that the connection is fine.
          Inform the ConnectionManager that everything is fine. */
//    connMgr_ApplOk(10);
}

template<typename Pred>
static bool pev_decodeResponse(Pred decode)
{
    if (tcp_rxdataLen > V2GTP_HEADER_SIZE)
    {
        routeDecoderInputData();
        decode();
        tcp_rxdataLen = 0; /* mark the input data as "consumed" */
        return true;
    }

    return false;
}

template<typename Pred>
static bool pev_decodeDinResponse(Pred isUsed)
{
    if (!pev_decodeResponse([] {  projectExiConnector_decode_DinExiDocument(); }))
        return false;

    bool res = isUsed();
    if (res) pev_cyclesSinceReq = -1; // unset
    return res;
}

template<typename Pred>
static bool pev_decodeAppResponse(Pred isUsed)
{
    if (!pev_decodeResponse([] {  projectExiConnector_decode_appHandExiDocument(); }))
        return false;

    bool res = isUsed();
    if (res) pev_cyclesSinceReq = -1; // unset
    return res;
}

static void send_message_again(uint8_t delay)
{
    pev_sendMessagePending = true;
    pev_DelayCycles = delay;
}

// return true when a request is ready
template<typename Pred1, typename Pred2, typename Pred3>
static bool messageHandler(uint8_t initialDelayCycles, Pred1 send, Pred2 isUsed, Pred3 decode)
{
    if (pev_cyclesInState < initialDelayCycles) return false;

    if (pev_sendMessagePending)
    {
        send();
        pev_sendMessagePending = false;
        pev_cyclesSinceReq = 0;
        return false;
    }

    if (!pev_decodeResponse(decode))//[] {  projectExiConnector_decode_DinExiDocument(); }))
        return false;

    bool res = isUsed();
    if (res) pev_cyclesSinceReq = -1; // unset
    return res;
}

template<typename Pred1, typename Pred2>
static bool dinMessageHandler(uint8_t initialDelayCycles, Pred1 send, Pred2 isUsed)
{
    return messageHandler(initialDelayCycles, send, isUsed, [] {  projectExiConnector_decode_DinExiDocument(); });
}

template<typename Pred1, typename Pred2>
static bool appMessageHandler(uint8_t initialDelayCycles, Pred1 send, Pred2 isUsed)
{
    return messageHandler(initialDelayCycles, send, isUsed, [] {  projectExiConnector_decode_appHandExiDocument(); });
}


/********* EXI creation functions ************************/
static void pev_sendSupportedAppProtocolReq()
{
    addV2GTPHeaderAndTransmit(exiDemoSupportedApplicationProtocolRequestIoniq, sizeof(exiDemoSupportedApplicationProtocolRequestIoniq));
}

static void pev_sendSessionSetupReq()
{
    projectExiConnector_prepare_DinExiDocument();
    dinDocEnc.V2G_Message.Body.SessionSetupReq_isUsed = 1u;
    init_dinSessionSetupReqType(&dinDocEnc.V2G_Message.Body.SessionSetupReq);
    /* In the session setup request, the session ID zero means: create a new session.
        The format (len 8, all zero) is taken from the original Ioniq behavior. */
    dinDocEnc.V2G_Message.Header.SessionID.bytes[0] = 0;
    dinDocEnc.V2G_Message.Header.SessionID.bytes[1] = 0;
    dinDocEnc.V2G_Message.Header.SessionID.bytes[2] = 0;
    dinDocEnc.V2G_Message.Header.SessionID.bytes[3] = 0;
    dinDocEnc.V2G_Message.Header.SessionID.bytes[4] = 0;
    dinDocEnc.V2G_Message.Header.SessionID.bytes[5] = 0;
    dinDocEnc.V2G_Message.Header.SessionID.bytes[6] = 0;
    dinDocEnc.V2G_Message.Header.SessionID.bytes[7] = 0;
    dinDocEnc.V2G_Message.Header.SessionID.bytesLen = 8;
    /* Takeover from https://github.com/uhi22/OpenV2Gx/commit/fc46c3ca802f08c57120a308f69fb4d1ce14f6b6 */
    /* The EVCCID. In the ISO they write, that this shall be the EVCC MAC. But the DIN
       reserves 8 bytes (dinSessionSetupReqType_EVCCID_BYTES_SIZE is 8). This does not match.
       The Ioniq (DIN) sets the bytesLen=6 and fills the 6 bytes with its own MAC. Let's assume this
       is the best way. */
    for (uint8_t i = 0; i < LEN_OF_EVCCID; i++)
    {
        dinDocEnc.V2G_Message.Body.SessionSetupReq.EVCCID.bytes[i] = getOurMac()[i];
    }
    dinDocEnc.V2G_Message.Body.SessionSetupReq.EVCCID.bytesLen = LEN_OF_EVCCID;
    encodeAndTransmit();
}

static void pev_sendServiceDiscoveryReq()
{
    projectExiConnector_prepare_DinExiDocument();
    dinDocEnc.V2G_Message.Body.ServiceDiscoveryReq_isUsed = 1u;
    init_dinServiceDiscoveryReqType(&dinDocEnc.V2G_Message.Body.ServiceDiscoveryReq);
    encodeAndTransmit();
}

static void pev_sendServicePaymentSelectionReq()
{
    projectExiConnector_prepare_DinExiDocument();
    dinDocEnc.V2G_Message.Body.ServicePaymentSelectionReq_isUsed = 1u;
    init_dinServicePaymentSelectionReqType(&dinDocEnc.V2G_Message.Body.ServicePaymentSelectionReq);
    /* the mandatory fields in ISO are SelectedPaymentOption and SelectedServiceList. Same in DIN. */
    dinDocEnc.V2G_Message.Body.ServicePaymentSelectionReq.SelectedPaymentOption = dinpaymentOptionType_ExternalPayment; /* not paying per car */
    dinDocEnc.V2G_Message.Body.ServicePaymentSelectionReq.SelectedServiceList.SelectedService.array[0].ServiceID = 1; /* todo: what ever this means. The Ioniq uses 1. */
    dinDocEnc.V2G_Message.Body.ServicePaymentSelectionReq.SelectedServiceList.SelectedService.arrayLen = 1; /* just one element in the array */
    encodeAndTransmit();
}

static void pev_sendContractAuthenticationReq()
{
    projectExiConnector_prepare_DinExiDocument();
    dinDocEnc.V2G_Message.Body.ContractAuthenticationReq_isUsed = 1u;
    init_dinContractAuthenticationReqType(&dinDocEnc.V2G_Message.Body.ContractAuthenticationReq);
    /* no other fields are manatory */
    encodeAndTransmit();
}

static void pev_sendChargeParameterDiscoveryReq(void)
{
    struct dinDC_EVChargeParameterType* cp;
    projectExiConnector_prepare_DinExiDocument();
    dinDocEnc.V2G_Message.Body.ChargeParameterDiscoveryReq_isUsed = 1u;
    init_dinChargeParameterDiscoveryReqType(&dinDocEnc.V2G_Message.Body.ChargeParameterDiscoveryReq);
    dinDocEnc.V2G_Message.Body.ChargeParameterDiscoveryReq.EVRequestedEnergyTransferType = dinEVRequestedEnergyTransferType_DC_extended;
    cp = &dinDocEnc.V2G_Message.Body.ChargeParameterDiscoveryReq.DC_EVChargeParameter;
    cp->DC_EVStatus.EVReady = 0;  /* What ever this means. The Ioniq sends 0 here in the ChargeParameterDiscoveryReq message. */
    //cp->DC_EVStatus.EVCabinConditioning_isUsed /* The Ioniq sends this with 1, but let's assume it is not mandatory. */
    //cp->DC_EVStatus.RESSConditioning_isUsed /* The Ioniq sends this with 1, but let's assume it is not mandatory. */
    cp->DC_EVStatus.EVRESSSOC = hardwareInterface_getSoc();
    cp->EVMaximumCurrentLimit.Value = _ccs_params.MaxCurrent;
    cp->EVMaximumCurrentLimit.Multiplier = 0; /* -3 to 3. The exponent for base of 10. */
    cp->EVMaximumCurrentLimit.Unit_isUsed = 1;
    cp->EVMaximumCurrentLimit.Unit = dinunitSymbolType_A;

    cp->EVMaximumPowerLimit_isUsed = 1; /* The Ioniq sends 1 here. */
    cp->EVMaximumPowerLimit.Value = _ccs_params.MaxPower * 10; /* maxpower is kW, then x10 x 100 by Multiplier */
    cp->EVMaximumPowerLimit.Multiplier = 2; /* 10^2 */
    cp->EVMaximumPowerLimit.Unit_isUsed = 1;
    cp->EVMaximumPowerLimit.Unit = dinunitSymbolType_W; /* Watt */

    cp->EVMaximumVoltageLimit.Value = _ccs_params.MaxVoltage;
    cp->EVMaximumVoltageLimit.Multiplier = 0; /* -3 to 3. The exponent for base of 10. */
    cp->EVMaximumVoltageLimit.Unit_isUsed = 1;
    cp->EVMaximumVoltageLimit.Unit = dinunitSymbolType_V;

    cp->EVEnergyCapacity_isUsed = 1;
    cp->EVEnergyCapacity.Value = 10000; /* Lets make it 100 kWh so it doesn't get in the way */
    cp->EVEnergyCapacity.Multiplier = 1;
    cp->EVEnergyCapacity.Unit_isUsed = 1;
    cp->EVEnergyCapacity.Unit = dinunitSymbolType_Wh; /* from Ioniq */

    cp->EVEnergyRequest_isUsed = 1;
    cp->EVEnergyRequest.Value = 10000; /* Lets make it 100 kWh so it doesn't get in the way */
    cp->EVEnergyRequest.Multiplier = 1;
    cp->EVEnergyRequest.Unit_isUsed = 1;
    cp->EVEnergyRequest.Unit = dinunitSymbolType_Wh; /* 9 from Ioniq */

    cp->FullSOC_isUsed = 1;
    cp->FullSOC = 100;
    cp->BulkSOC_isUsed = 1;
    cp->BulkSOC = 80;

    dinDocEnc.V2G_Message.Body.ChargeParameterDiscoveryReq.DC_EVChargeParameter_isUsed = 1;
    encodeAndTransmit();
}

static void pev_sendCableCheckReq(void)
{
    projectExiConnector_prepare_DinExiDocument();
    dinDocEnc.V2G_Message.Body.CableCheckReq_isUsed = 1u;
    init_dinCableCheckReqType(&dinDocEnc.V2G_Message.Body.CableCheckReq);
#define st dinDocEnc.V2G_Message.Body.CableCheckReq.DC_EVStatus
    st.EVReady = 1; /* 1 means true. We are ready. */
    st.EVErrorCode = dinDC_EVErrorCodeType_NO_ERROR;
    st.EVRESSSOC = hardwareInterface_getSoc(); /* Scaling is 1%. */
#undef st
    encodeAndTransmit();
    /* Since the response to the CableCheckRequest may need longer, inform the connection manager to be patient.
       This makes sure, that the timeout of the state machine comes before the timeout of the connectionManager, so
       that we enter the safe shutdown sequence as intended.
       (This is a takeover from https://github.com/uhi22/pyPLC/commit/08af8306c60d57c4c33221a0dbb25919371197f9 ) */
//    connMgr_ApplOk(31);
}

static void pev_sendPreChargeReq(uint16_t targetVoltage)
{
    projectExiConnector_prepare_DinExiDocument();
    dinDocEnc.V2G_Message.Body.PreChargeReq_isUsed = 1u;
    init_dinPreChargeReqType(&dinDocEnc.V2G_Message.Body.PreChargeReq);
#define st dinDocEnc.V2G_Message.Body.PreChargeReq.DC_EVStatus
    st.EVReady = 1; /* 1 means true. We are ready. */
    st.EVErrorCode = dinDC_EVErrorCodeType_NO_ERROR;
    st.EVRESSSOC = hardwareInterface_getSoc(); /* The SOC. Scaling is 1%. */
#undef st
#define tvolt dinDocEnc.V2G_Message.Body.PreChargeReq.EVTargetVoltage
    tvolt.Multiplier = 0; /* -3 to 3. The exponent for base of 10. */
    tvolt.Unit = dinunitSymbolType_V;
    tvolt.Unit_isUsed = 1;
    tvolt.Value = targetVoltage; /* The precharge target voltage. Scaling is 1V. */
#undef tvolt
#define tcurr dinDocEnc.V2G_Message.Body.PreChargeReq.EVTargetCurrent
    tcurr.Multiplier = 0; /* -3 to 3. The exponent for base of 10. */
    tcurr.Unit = dinunitSymbolType_A;
    tcurr.Unit_isUsed = 1;
    tcurr.Value = 1; /* 1A for precharging */
#undef tcurr
    encodeAndTransmit();
}

static void pev_sendPowerDeliveryReq(bool isOn)
{
    if (isOn) {
        // reset if set from previous session
        LastCurrentDemandResPresentVoltage = 0;
        PresentVoltageDifferentFromTarget = PresentVoltageDifferentFromTarget_isSet = false;
        PresentCurrentDifferentFromTarget = PresentCurrentDifferentFromTarget_isSet = false;
    }

    projectExiConnector_prepare_DinExiDocument();
    dinDocEnc.V2G_Message.Body.PowerDeliveryReq_isUsed = 1u;
#define req dinDocEnc.V2G_Message.Body.PowerDeliveryReq
    init_dinPowerDeliveryReqType(&req);
    req.ReadyToChargeState = isOn; /* 1=ON, 0=OFF */
    req.DC_EVPowerDeliveryParameter_isUsed = 1;
    req.DC_EVPowerDeliveryParameter.DC_EVStatus.EVReady = 1; /* 1 means true. We are ready. */
    req.DC_EVPowerDeliveryParameter.DC_EVStatus.EVErrorCode = dinDC_EVErrorCodeType_NO_ERROR;
    req.DC_EVPowerDeliveryParameter.DC_EVStatus.EVRESSSOC = hardwareInterface_getSoc();
    req.DC_EVPowerDeliveryParameter.ChargingComplete = 0; /* boolean. Charging not finished. */
    /* some "optional" fields seem to be mandatory, at least the Ioniq sends them, and the Compleo charger ignores the message if too short.
       See https://github.com/uhi22/OpenV2Gx/commit/db2c7addb0cae0e16175d666e736efd551f3e14d#diff-333579da65917bc52ef70369b576374d0ee5dbca47d2b1e3bedb6f062decacff
       Let's fill them:
    */
    req.DC_EVPowerDeliveryParameter.DC_EVStatus.EVCabinConditioning_isUsed = 1;
    req.DC_EVPowerDeliveryParameter.DC_EVStatus.EVCabinConditioning = 0;
    req.DC_EVPowerDeliveryParameter.DC_EVStatus.EVRESSConditioning_isUsed = 1;
    req.DC_EVPowerDeliveryParameter.DC_EVStatus.EVRESSConditioning = 0;
    req.DC_EVPowerDeliveryParameter.BulkChargingComplete_isUsed = 1;
    req.DC_EVPowerDeliveryParameter.BulkChargingComplete = 0;
#undef req
    encodeAndTransmit();
}

static void pev_sendCurrentDemandReq(void)
{
    projectExiConnector_prepare_DinExiDocument();
    dinDocEnc.V2G_Message.Body.CurrentDemandReq_isUsed = 1u;
    init_dinCurrentDemandReqType(&dinDocEnc.V2G_Message.Body.CurrentDemandReq);
    // DC_EVStatus
#define st dinDocEnc.V2G_Message.Body.CurrentDemandReq.DC_EVStatus
    st.EVReady = 1; /* 1 means true. We are ready. */
    st.EVErrorCode = dinDC_EVErrorCodeType_NO_ERROR;
    st.EVRESSSOC = hardwareInterface_getSoc();
#undef st
    // EVTargetVoltage
    uint16_t targetVoltage = hardwareInterface_getChargingTargetVoltage(); /* The charging target. Scaling is 1V. */
    uint16_t targetCurrent = hardwareInterface_getChargingTargetCurrent(); /* The charging target current. Scaling is 1A. */
    uint16_t EVMaximumVoltageLimit = _ccs_params.MaxVoltage;
    if (EVMaximumVoltageLimit > 1 && targetVoltage > EVMaximumVoltageLimit) {
        /* Some chargers run into emergency shutdown, if the requested or actual voltage is above the announced EVMaximumVoltageLimit. */
        targetVoltage = EVMaximumVoltageLimit;
        log(MOD_PEV, "Warning: TargetVoltage %dV is above EVMaximumVoltageLimit %dV, which may cause charger shutdown.",
            targetVoltage, EVMaximumVoltageLimit);
    }
#define req dinDocEnc.V2G_Message.Body.CurrentDemandReq
    req.EVTargetVoltage.Multiplier = 0;  /* -3 to 3. The exponent for base of 10. */
    req.EVTargetVoltage.Unit = dinunitSymbolType_V;
    req.EVTargetVoltage.Unit_isUsed = 1;
    req.EVTargetVoltage.Value = targetVoltage; /* The charging target voltage. Scaling is 1V. */
    LastTargetVoltage = targetVoltage;

    req.EVTargetCurrent.Multiplier = 0;  /* -3 to 3. The exponent for base of 10. */
    req.EVTargetCurrent.Unit = dinunitSymbolType_A;
    req.EVTargetCurrent.Unit_isUsed = 1;
    req.EVTargetCurrent.Value = targetCurrent; /* The charging target current. Scaling is 1A. */
    LastTargetCurrent = targetCurrent;

    req.ChargingComplete = 0; /* boolean. Is it fine that the PEV just sends a PowerDeliveryReq with STOP, if it decides to stop the charging? */

    req.BulkChargingComplete_isUsed = 1;
    req.BulkChargingComplete = 0; /* not complete */

    req.RemainingTimeToFullSoC_isUsed = 1;
    req.RemainingTimeToFullSoC.Multiplier = 0;  /* -3 to 3. The exponent for base of 10. */
    req.RemainingTimeToFullSoC.Unit = dinunitSymbolType_s;
    req.RemainingTimeToFullSoC.Unit_isUsed = 1;
    req.RemainingTimeToFullSoC.Value = 1200; /* seconds */

    req.RemainingTimeToBulkSoC_isUsed = 1;
    req.RemainingTimeToBulkSoC.Multiplier = 0;  /* -3 to 3. The exponent for base of 10. */
    req.RemainingTimeToBulkSoC.Unit = dinunitSymbolType_s;
    req.RemainingTimeToBulkSoC.Unit_isUsed = 1;
    req.RemainingTimeToBulkSoC.Value = 600; /* seconds */

    // Charger 'Plugit HUBE S' wont work without maxes
    req.EVMaximumVoltageLimit_isUsed = 1;
    req.EVMaximumVoltageLimit.Multiplier = 0;
    req.EVMaximumVoltageLimit.Unit = dinunitSymbolType_V;
    req.EVMaximumVoltageLimit.Unit_isUsed = 1;
    req.EVMaximumVoltageLimit.Value = _ccs_params.MaxVoltage;

    req.EVMaximumCurrentLimit_isUsed = 1;
    req.EVMaximumCurrentLimit.Multiplier = 0;
    req.EVMaximumCurrentLimit.Unit = dinunitSymbolType_A;
    req.EVMaximumCurrentLimit.Unit_isUsed = 1;
    req.EVMaximumCurrentLimit.Value = _ccs_params.MaxCurrent;

    // evgo-vehicle-oem-best-practices.pdf
    req.EVMaximumPowerLimit_isUsed = 1; /* The Ioniq sends 1 here. */
    req.EVMaximumPowerLimit.Value = _ccs_params.MaxPower * 10; /* maxpower is kW, then x10 x 100 by Multiplier */
    req.EVMaximumPowerLimit.Multiplier = 2; /* 10^2 */
    req.EVMaximumPowerLimit.Unit_isUsed = 1;
    req.EVMaximumPowerLimit.Unit = dinunitSymbolType_W; /* Watt */
#undef req
    encodeAndTransmit();
}

static void pev_sendWeldingDetectionReq(void)
{
    projectExiConnector_prepare_DinExiDocument();
    dinDocEnc.V2G_Message.Body.WeldingDetectionReq_isUsed = 1u;
    init_dinWeldingDetectionReqType(&dinDocEnc.V2G_Message.Body.WeldingDetectionReq);
#define st dinDocEnc.V2G_Message.Body.WeldingDetectionReq.DC_EVStatus
    st.EVReady = 0; /* 0 means not ready to charge (we are done charging) */
    st.EVErrorCode = dinDC_EVErrorCodeType_NO_ERROR;
    st.EVRESSSOC = hardwareInterface_getSoc();
#undef st
    encodeAndTransmit();
}

static void pev_sendSessionStopReq()
{
    projectExiConnector_prepare_DinExiDocument();
    dinDocEnc.V2G_Message.Body.SessionStopReq_isUsed = 1u;
    init_dinSessionStopType(&dinDocEnc.V2G_Message.Body.SessionStopReq);
    /* no other fields are mandatory */
    encodeAndTransmit();
}


/**** State functions ***************/
//Empty functions
static void stateFunctionStart() {}

static void stateFunctionConnected(void)
{
    // We have a freshly established TCP channel. We start the V2GTP/EXI communication now.
    // We just use the initial request message from the Ioniq. It contains one entry: DIN.
    _ccs_params.CurrentDemandStopReason = STOP_REASON_NONE;
    setCheckpoint(400);
    pev_enterState(PEV_STATE_SupportedApplicationProtocolMsg);
}

static void stateFunctionSupportedApplicationProtocolMsg()
{
    if (appMessageHandler(MESSAGE_STATE_DELAY_CYCLES, [] {
        log(MOD_PEV, "send SupportedApplicationProtocolReq");
        pev_sendSupportedAppProtocolReq();
        },
        [] { return aphsDoc.supportedAppProtocolRes_isUsed; }))
    {
        log(MOD_PEV, "Schema negotiated. ResponseCode:%d, SchemaID_isUsed:%d, SchemaID:%d",
            aphsDoc.supportedAppProtocolRes.ResponseCode,
            aphsDoc.supportedAppProtocolRes.SchemaID_isUsed,
            aphsDoc.supportedAppProtocolRes.SchemaID);
        setCheckpoint(500);
        pev_enterState(PEV_STATE_SessionSetupMsg);
    }
}

static void stateFunctionSessionSetupMsg()
{
    if (dinMessageHandler(MESSAGE_STATE_DELAY_CYCLES, [] {
        log(MOD_PEV, "send SessionSetupReq");
        pev_sendSessionSetupReq();
        },
        [] { return dinDocDec.V2G_Message.Body.SessionSetupRes_isUsed; }))
    {
        memcpy(sessionId, dinDocDec.V2G_Message.Header.SessionID.bytes, SESSIONID_LEN);
        sessionIdLen = dinDocDec.V2G_Message.Header.SessionID.bytesLen; /* store the received SessionID, we will need it later. */
        log_bytes(MOD_PEV, "SessionId", sessionId, sessionIdLen);
        pev_enterState(PEV_STATE_ServiceDiscoveryMsg);
    }
}

static void stateFunctionServiceDiscoveryMsg()
{
    if (dinMessageHandler(MESSAGE_STATE_DELAY_CYCLES, [] {
        log(MOD_PEV, "send ServiceDiscoveryReq");
        pev_sendServiceDiscoveryReq();
        },
        [] { return dinDocDec.V2G_Message.Body.ServiceDiscoveryRes_isUsed; }))
    {
        pev_enterState(PEV_STATE_ServicePaymentSelectionMsg);
    }
}

static void stateFunctionServicePaymentSelectionMsg()
{
    if (dinMessageHandler(MESSAGE_STATE_DELAY_CYCLES, [] {
        log(MOD_PEV, "send ServicePaymentSelectionReq");
        pev_sendServicePaymentSelectionReq();
        },
        [] { return dinDocDec.V2G_Message.Body.ServicePaymentSelectionRes_isUsed; }))
    {
        pev_enterState(PEV_STATE_ContractAuthenticationMsg);
    }
}

static void stateFunctionContractAuthenticationMsg()
{
    if (dinMessageHandler(MESSAGE_STATE_DELAY_CYCLES, [] {
        log(MOD_PEV, "send ContractAuthenticationReq");
        pev_sendContractAuthenticationReq();
        },
        [] { return dinDocDec.V2G_Message.Body.ContractAuthenticationRes_isUsed; }))
    {
        // In normal case, we can have two results here: either the Authentication is needed (the user
        // needs to authorize by RFID card or app, or something like this.
        // Or, the authorization is finished. This is shown by EVSEProcessing=Finished.
        if (dinDocDec.V2G_Message.Body.ContractAuthenticationRes.EVSEProcessing == dinEVSEProcessingType_Finished)
        {
            log(MOD_PEV, "Auth is Finished");
            ChargeParameterDiscoveryCompletedTrigger = false; // reset
            pev_enterState(PEV_STATE_ChargeParameterDiscoveryMsg);
        }
        else
        {
            send_message_again(MESAGE_LOOP_DELAY_CYCLES);
            // Stay in same state
        }
    }
}

static void stateFunctionChargeParameterDiscoveryMsg()
{
    if (dinMessageHandler(MESSAGE_STATE_DELAY_CYCLES, [] {
        log(MOD_PEV, "send ChargeParameterDiscoveryReq");
        pev_sendChargeParameterDiscoveryReq();
        },
        [] { return dinDocDec.V2G_Message.Body.ChargeParameterDiscoveryRes_isUsed; }))
    {
        // We can have two cases here:
        // (A) The charger needs more time to show the charge parameters.
        // (B) The charger finished to tell the charge parameters.
        if (dinDocDec.V2G_Message.Body.ChargeParameterDiscoveryRes.EVSEProcessing == dinEVSEProcessingType_Finished)
        {
            ChargeParameterDiscoveryCompletedTrigger = true;

#define dcparm dinDocDec.V2G_Message.Body.ChargeParameterDiscoveryRes.DC_EVSEChargeParameter
            int evseMaxVoltage = combineValueAndMultiplier(dcparm.EVSEMaximumVoltageLimit);
            int evseMaxCurrent = combineValueAndMultiplier(dcparm.EVSEMaximumCurrentLimit);
            int evseMinimumVoltage = combineValueAndMultiplier(dcparm.EVSEMinimumVoltageLimit);
#undef dcparm
            _ccs_params.EvseMinimumVoltage = evseMinimumVoltage;
            _ccs_params.EvseMaxVoltage = evseMaxVoltage;
            _ccs_params.EvseMaxCurrent = evseMaxCurrent;

            log(MOD_PEV, "ChargeParams are discovered: min:%dV max:%dV/%dA", evseMinimumVoltage, evseMaxVoltage, evseMaxCurrent);

            // pull the CP line to state C here:
            log(MOD_PEV, "Set StateC");
            hardwareInterface_setStateC();

            hardwareInterface_lockConnector();

            setCheckpoint(550);
            pev_enterState(PEV_STATE_CableCheckMsg);
        }
        else
        {
            send_message_again(MESAGE_LOOP_DELAY_CYCLES);
            // stay in same state
        }
    }
}

static void stateFunctionCableCheckMsg()
{
    if (dinMessageHandler(MESSAGE_STATE_DELAY_CYCLES, [] { // Ionic delay is 420ms, 250ms + StateC and locking connector. 240ms is sufficient for us.
        log(MOD_PEV, "send CableCheckReq");
        pev_sendCableCheckReq();
        },
        [] { return dinDocDec.V2G_Message.Body.CableCheckRes_isUsed; }))
    {
        uint8_t rc = dinDocDec.V2G_Message.Body.CableCheckRes.ResponseCode;
        uint8_t proc = dinDocDec.V2G_Message.Body.CableCheckRes.EVSEProcessing;
        _ccs_params.EvseVoltage = 0;
        // We have two cases here:
        // 1) The charger says "cable check is finished and cable ok", by setting ResponseCode=OK and EVSEProcessing=Finished.
        // 2) Else: The charger says "need more time or cable not ok". In this case, we just run into timeout and start from the beginning.
        if (rc == dinresponseCodeType_OK && proc == dinEVSEProcessingType_Finished)
        {
            log(MOD_PEV, "CableCheck is finished and ok");
            PrechargeDifferenceIsSmall = false; // reset
            pev_enterState(PEV_STATE_PreChargeMsg);
        }
        else if (rc == dinresponseCodeType_OK && proc == dinEVSEProcessingType_Ongoing)
        {
            send_message_again(MESAGE_LOOP_DELAY_CYCLES);
            // stay in same state
        }
        else // spec only mention the 2 cases above, assuming all other cases must be errors
        {
            log(MOD_PEV, "CableCheck error rc:%d proc:%d", rc, proc);
            pev_enterState(PEV_STATE_SafeShutDown);
        }
    }
}

static void stateFunctionPreChargeMsg()
{
    // DX: wait 2 sec. It is possible some chargers do not like precharge lasting longer than 7 seconds? This at least saves 2 :-)
    // Ionic has 300ms from CableCheck -> PreCharge, and we have 240ms + time in state stateFunctionWaitForPreChargeStart, so we are good:-)
    uint8_t delay = CONFIG_SX ? MESSAGE_STATE_DELAY_CYCLES : DX_CCS_WaitForPreChargeStart_MS / 30;
    if (dinMessageHandler(delay, [] {
        uint16_t batVtg = hardwareInterface_getBatteryVoltage();
        if (batVtg < _ccs_params.EvseMinimumVoltage) {
            // Unlikely that this can happen, and if it does, then precharge will never be satisfied and charger will go into timeout, so don't need to handle it specially
            log(MOD_PEV, "Warning: batteryVoltage:%d is less than evseMinimumVoltage:%d", batVtg, _ccs_params.EvseMinimumVoltage);
        }
        log(MOD_PEV, "send PreChargeReq:%dV", batVtg);
        pev_sendPreChargeReq(batVtg);
        },
        [] { return dinDocDec.V2G_Message.Body.PreChargeRes_isUsed; }))
    {
        _global.auto_power_off_timer_count_up_ms = 0;

        int evsePresentVoltage = combineValueAndMultiplier(dinDocDec.V2G_Message.Body.PreChargeRes.EVSEPresentVoltage);
        _ccs_params.EvseVoltage = evsePresentVoltage;

        log(MOD_PEV, "PreCharge:%dV", evsePresentVoltage);

        uint16_t inletVtg = hardwareInterface_getInletVoltage();
        uint16_t batVtg = hardwareInterface_getBatteryVoltage();

        if (not PrechargeDifferenceIsSmall)
        {
            if (ABS(inletVtg - batVtg) < PARAM_U_DELTA_MAX_FOR_END_OF_PRECHARGE)
            {
                log(MOD_PEV, "PreCharge difference is small (inlet:%dV batt:%dV)", inletVtg, batVtg);
                setCheckpoint(573);
                PrechargeDifferenceIsSmall = true;
            }
        }

        if (PrechargeDifferenceIsSmall && chademoInterface_preChargeCompleted())
        {
            log(MOD_PEV, "PreCharge completed");

            // Turn the power relay on.
            // Ionic timings show...it most likely is not done here, but after PowerDeliveryRes
            // But currently we only set a flag here, so it does not matter
            hardwareInterface_setPowerRelayOn();

            pev_enterState(PEV_STATE_PowerDeliveryOnMsg);
        }
        else
        {
            send_message_again(MESAGE_LOOP_DELAY_CYCLES);
            // stay in same state
        }
    }
}

static void stateFunctionPowerDeliveryOnMsg()
{
    if (dinMessageHandler(15, [] { // 450ms for contactors to close
        log(MOD_PEV, "send PowerDeliveryReq:true");
        pev_sendPowerDeliveryReq(true); /* true is ON */
        },
        [] { return dinDocDec.V2G_Message.Body.PowerDeliveryRes_isUsed; }))
    {
        if (dinDocDec.V2G_Message.Body.PowerDeliveryRes.ResponseCode == dinresponseCodeType_OK)
        {
            // Turn the power relay on (Ionic delays show that it probably happens here and not after PreCharge done, but both probably works)
            // But currently we only set a flag here, so it does not matter
            //hardwareInterface_setPowerRelayOn();

            sentCurrentDemandMsg = 0; // reset
            setCheckpoint(700);
            pev_enterState(PEV_STATE_CurrentDemandMsg);
        }
        else
        {
            // FAILED_PowerDeliveryNotApplied (17) seen in cases where precharge voltage was < 20V less than battery voltage.
            // Exception: it seems the charger dislike lower voltage (than battery) more than higher voltage (than battery):
            // Lower: huge current inrush agains charger. Car has no way to limit amps. Higher: inrush agains car, but charger is current limiting, so it will be max 1A (precharge current).
            log(MOD_PEV, "PowerDelivery failed rc:%d", dinDocDec.V2G_Message.Body.PowerDeliveryRes.ResponseCode);
            pev_enterState(PEV_STATE_SafeShutDown);
        }
    }
}

static void stateFunctionCurrentDemandMsg()
{
    if (dinMessageHandler(0, [] { // no delay, get into currentdemand asap so chademo chargingLoop does not have to wait for us.
        if (sentCurrentDemandMsg++ < 10) { // only show the 10 first, to get an idea of the timing (no need to spam)
            log(MOD_PEV, "send CurrentDemandReq");
        }
        pev_sendCurrentDemandReq();
        },
        [] { return dinDocDec.V2G_Message.Body.CurrentDemandRes_isUsed; }))
    {
        _global.auto_power_off_timer_count_up_ms = 0;

        /* as long as the battery is not full and no stop-demand from the user, we continue charging */
        _stopreasons currentDemandStopReason = STOP_REASON_NONE;
        if (dinDocDec.V2G_Message.Body.CurrentDemandRes.DC_EVSEStatus.EVSEStatusCode == dinDC_EVSEStatusCodeType_EVSE_Shutdown)
        {
            log(MOD_PEV, "User requested stop on charger side.");
            currentDemandStopReason = STOP_REASON_CHARGER_SHUTDOWN;
        }
        else if (dinDocDec.V2G_Message.Body.CurrentDemandRes.DC_EVSEStatus.EVSEStatusCode == dinDC_EVSEStatusCodeType_EVSE_Malfunction)
        {
            /* If the charger reports a malfunction, we stop the charging. */
            /* Issue reference: https://github.com/uhi22/ccs32clara/issues/29 */
            log(MOD_PEV, "Charger reported EVSE_Malfunction. A reason could be hitting the EVSEMinimumVoltageLimit or EVSEMaximumVoltageLimit.");
            currentDemandStopReason = STOP_REASON_CHARGER_EVSE_MALFUNCTION;
        }
        else if (dinDocDec.V2G_Message.Body.CurrentDemandRes.DC_EVSEStatus.EVSEStatusCode == dinDC_EVSEStatusCodeType_EVSE_EmergencyShutdown)
        {
            /* If the charger reports an emergency, we stop the charging. */
            log(MOD_PEV, "Charger reported EmergencyShutdown.");
            currentDemandStopReason = STOP_REASON_CHARGER_EMERGENCY_SHUTDOWN;
        }
        else if (hardwareInterface_stopChargeRequested())
        {
            log(MOD_PEV, "User requested stop on car side.");
            currentDemandStopReason = STOP_REASON_POWER_OFF_PENDING;
        }
        else if (hardwareInterface_getIsBatteryFull())
        {
            log(MOD_PEV, "Battery is full.");
            currentDemandStopReason = STOP_REASON_BATTERY_FULL;
        }

        if (currentDemandStopReason != STOP_REASON_NONE)
        {
            _ccs_params.CurrentDemandStopReason = currentDemandStopReason;
            pev_enterState(PEV_STATE_PowerDeliveryOffMsg);
        }
        else
        {
            /* continue charging loop */
            int evsePresentVoltage = combineValueAndMultiplier(dinDocDec.V2G_Message.Body.CurrentDemandRes.EVSEPresentVoltage);
            uint16_t evsePresentCurrent = combineValueAndMultiplier(dinDocDec.V2G_Message.Body.CurrentDemandRes.EVSEPresentCurrent);

            if (dinDocDec.V2G_Message.Body.CurrentDemandRes.EVSEMaximumCurrentLimit_isUsed) {
                int evseMaxCurrent = combineValueAndMultiplier(dinDocDec.V2G_Message.Body.CurrentDemandRes.EVSEMaximumCurrentLimit);
                _ccs_params.EvseMaxCurrentInCurrentDemandRes = evseMaxCurrent;
            }
            else {
                _ccs_params.EvseMaxCurrentInCurrentDemandRes = 0;
            }

            _ccs_params.EvseVoltage = evsePresentVoltage;
            _ccs_params.EvseCurrent = evsePresentCurrent;
            LastCurrentDemandResPresentVoltage = evsePresentVoltage;

            if (evsePresentVoltage != LastTargetVoltage) PresentVoltageDifferentFromTarget = true;
            PresentVoltageDifferentFromTarget_isSet = true;

            if (evsePresentCurrent != LastTargetCurrent) PresentCurrentDifferentFromTarget = true;
            PresentCurrentDifferentFromTarget_isSet = true;

            send_message_again(0); // no delay in CurrentDemand loop
            // stay in same state
        }
    }
}

static void stateFunctionPowerDeliveryOffMsg()
{
    if (dinMessageHandler(MESSAGE_STATE_DELAY_CYCLES, [] {
        /* we can immediately send the powerDeliveryStopRequest, while we are under full current.
           sequence explained here: https://github.com/uhi22/pyPLC#detailled-investigation-about-the-normal-end-of-the-charging-session */
        log(MOD_PEV, "send PowerDeliveryReq:false");
        pev_sendPowerDeliveryReq(false); 
        },
        [] { return dinDocDec.V2G_Message.Body.PowerDeliveryRes_isUsed; }))
    {
        log(MOD_PEV, "Set StateB and give charger some time to detect it and ramp down the current");
        hardwareInterface_setStateB(); /* ISO Figure 107: The PEV shall set stateB after receiving PowerDeliveryRes and before WeldingDetectionReq */
        pev_enterState(PEV_STATE_WaitForCurrentDownAfterStateB); /* We give the charger some time to detect the StateB and fully ramp down the current */
    }
}

static void stateFunctionWaitForCurrentDownAfterStateB(void)
{
    /* During normal end of the charging session, we have set the StateB, and want to give the charger some time to ramp down the current completely,
       before we are opening the contactors. 10*30ms=300ms for charger shutdown should be more than sufficient, because somewhere was a requirement with 20ms between StateB until current is down. The Ioniq uses 300ms. */
    if (pev_cyclesInState < 10) { // 300ms
        return;
    }

    /* Time is over. Current flow should have been stopped by the charger. Let's open the contactors and send a weldingDetectionRequest, to find out whether the voltage drops. */
    hardwareInterface_setPowerRelayOff();
    pev_enterState(PEV_STATE_WaitForPowerRelayOff);
}

static void stateFunctionWaitForPowerRelayOff()
{
    // Wait for chademo to ACK _ccs_params.ContactorClosed we sat in hardwareInterface_setPowerRelayOff()
    if (chademoInterface_adapterContactorOpened())
    {
        pev_enterState(PEV_STATE_WeldingDetectionMsg);
    }
}

static void stateFunctionWeldingDetectionMsg()
{
    if (dinMessageHandler(0, [] { // no delay, stateFunctionWaitForCurrentDownAfterStateB already waited 300 + 2 state transitions x 30ms
        log(MOD_PEV, "send WeldingDetectionReq");
        pev_sendWeldingDetectionReq();
        },
        [] { return dinDocDec.V2G_Message.Body.WeldingDetectionRes_isUsed; }))
    {
        /* The charger measured the voltage on the cable, and gives us the value. In the first
            round will show a quite high voltage, because the contactors are just opening. We
            need to repeat the requests, until the voltage is at a non-dangerous level. */
        int evsePresentVoltage = combineValueAndMultiplier(dinDocDec.V2G_Message.Body.WeldingDetectionRes.EVSEPresentVoltage);
        _ccs_params.EvseVoltage = evsePresentVoltage;
        log(MOD_PEV, "WeldingDetection:%dV", evsePresentVoltage);
        bool voltageIsLow = evsePresentVoltage < MAX_VOLTAGE_TO_FINISH_WELDING_DETECTION;

        // Charger still says it has the same voltage as when we were last charging.
        // If we were welded, the measured voltage should be battery voltage, and its very unlikely that this would be exactly the same as last charging voltage.
        // It should most certainly be lower, as the charging voltage is always higher than the battery voltage.
        // So it seems the charger is lying to us and just send us its last known voltage.
        bool voltageIsLastChargingVoltageAfter2sec = evsePresentVoltage == LastCurrentDemandResPresentVoltage && pev_cyclesInState > 66;
        if (voltageIsLastChargingVoltageAfter2sec)
            log(MOD_PEV, "WeldingDetection voltage is last charging voltage after 2s: charger is probably lying.");

        if (voltageIsLow || voltageIsLastChargingVoltageAfter2sec)
        {
            log(MOD_PEV, "WeldingDetection finished");
            pev_enterState(PEV_STATE_SessionStopMsg);
        }
        else 
        {
            send_message_again(MESAGE_LOOP_DELAY_CYCLES);
            // stay in same state
        }
    }
}

static void stateFunctionSessionStopMsg()
{
    if (dinMessageHandler(MESSAGE_STATE_DELAY_CYCLES, [] {
        log(MOD_PEV, "send SessionStopReq");
        pev_sendSessionStopReq();
        },
        [] { return dinDocDec.V2G_Message.Body.SessionStopRes_isUsed; }))
    {
        tcp_disconnect();
        log(MOD_PEV, "Charging is finished");
        pev_enterState(PEV_STATE_Stop);
    }
}

static void stateFunctionSafeShutDown()
{
    /* Here we end, if we run into a timeout in the state machine (or other error before we reach CurrentDemand). */
    /* Initiate the safe-shutdown-sequence. */
    log(MOD_PEV, "Safe-shutdown-sequence: setting StateB");
    hardwareInterface_setStateB(); /* setting CP line to B disables in the charger the current flow. */
    pev_enterState(PEV_STATE_SafeShutDownWaitForChargerShutdown);
}

static void stateFunctionSafeShutDownWaitForChargerShutdown(void)
{
    /* wait 66*30ms=2s for charger shutdown, to give the charger the time to stop the current. */
    if (pev_cyclesInState < 66) { // 2sec
        return;
    }
    /* Now the current flow is stopped by the charger. We can safely open the contactors: */
    hardwareInterface_setPowerRelayOff();

    tcp_disconnect(); /* Set StateB is our last communication with the charger during safe shutdown, and after waiting for StateB to be fully processed by charger, close tcp as well. */
    pev_enterState(PEV_STATE_Stop);
}

static void stateFunctionStop(void)
{
    /* Set voltage to 0 so rest of system sees safe state */
    _ccs_params.EvseVoltage = 0;

    hardwareInterface_unlockConnector();

    // I guess we would want to retry if something failed, but only if we did not complete ChargeParameterDiscovery,
    // because this trigger stateC and connector locking (energizing state). Something failing after this is probably hardware/high voltage related, and will not "heal".
    if (not ChargeParameterDiscoveryCompletedTrigger && not hardwareInterface_stopChargeRequested())
    {
        log(MOD_PEV, "Did not complete ChargeParameterDiscovery -> restart");
        pev_enterState(PEV_STATE_Start);
        connMgr_restart();
    }
    else
    {
        pev_enterState(PEV_STATE_End);
    }
}

static void stateFunctionEnd(void)
{
    // terminal state. No code should exist here. We can never leave this state
}

static void pev_enterState(pevstates n)
{
    if (n == pev_state) // set same state will mess up pev_cyclesInState and also pointless
        log(MOD_PEV, "Error: set same state %s", pevSttLabels[n]);

    log(MOD_PEV, "=> set state %s", pevSttLabels[n]);

    pev_cyclesInState = 0;
    pev_DelayCycles = 0; // delayCycles are within a state only
    pev_sendMessagePending = true; // trigger send of Req-message when entering next state
    pev_state = n;
    _ccs_params.state = n;
    
    // messages no longer crosses states
    pev_cyclesSinceReq = -1; // unset
}

static uint8_t pev_stateIsTooLong(void)
{
    return state_timeouts[pev_state] > 0 && pev_cyclesInState > state_timeouts[pev_state];
}

static uint8_t pev_responseIsTooLong(void)
{
    return response_timeouts[pev_state] > 0 && pev_cyclesSinceReq > response_timeouts[pev_state];
}

/******* The statemachine dispatcher *******************/
static void pev_runFsm(void)
{
    // TODO: check power off pending for all states, not only during current demand?

    if (connMgr_getLevel() < CONNLEVEL_80_TCP_CONNECTED && pev_state == PEV_STATE_Start)
    {
        /* No TCP and we are still in Start. Nothing to do here. */
        return;
    }

    if (connMgr_getLevel() == CONNLEVEL_80_TCP_CONNECTED && pev_state == PEV_STATE_Start)
    {
        /* We have TCP and we are in Start. This is the trigger for us. */
        pev_enterState(PEV_STATE_Connected);
        connMgr_setLevel(CONNLEVEL_100_APPL_RUNNING);
    }

    if (pev_DelayCycles > 0)
    {
        pev_DelayCycles -= 1;
    }
    else
    {
        stateFunctions[pev_state](); //call state function
    }

    if (pev_state != PEV_STATE_CurrentDemandMsg) //only in currentDemand we have meaningful current values
        _ccs_params.EvseCurrent = 0;

    bool stop = false;
    if (pev_responseIsTooLong())
    {
        log(MOD_PEV, "Response timeout in state %s", pevSttLabels[pev_state]);
        stop = true;
    }
    if (pev_stateIsTooLong())
    {
        log(MOD_PEV, "Timeout in state %s", pevSttLabels[pev_state]);
        stop = true;
    }

    if (state_timeouts[pev_state] > 0 && not tcp_isConnected())
    {
        log(MOD_PEV, "Tcp connection lost in timeoutable state %s", pevSttLabels[pev_state]);
        stop = true;
    }

    if (hardwareInterface_stopChargeRequested()
        && hardwareInterface_isConnectorLocked()
        && pev_state < PEV_STATE_CurrentDemandMsg
        )
    {
        log(MOD_PEV, "Stop charging requested before CurrentDemand (%s)", pevSttLabels[pev_state]);
        stop = true;
    }

    if (stop)
    {
        // Make sure we set a CurrentDemandStopReason if we pull the rug on CurrentDemand
        if (pev_state == PEV_STATE_CurrentDemandMsg && _ccs_params.CurrentDemandStopReason == STOP_REASON_NONE)
            _ccs_params.CurrentDemandStopReason = STOP_REASON_TIMEOUT;

        pev_enterState(PEV_STATE_SafeShutDown);
	}
}

/************ public interfaces *****************************************/

void pevStateMachine_reset()
{
    pev_enterState(PEV_STATE_Start);
}

/* The cyclic main function of the PEV charging state machine.
   Called each 30ms. */
void pevStateMachine_Mainfunction(void)
{
    // run the state machine:
    pev_cyclesInState += 1; // for timeout handling, count how long we are in a state
    if (pev_cyclesSinceReq != -1) pev_cyclesSinceReq += 1; // how long since req sent
    pev_runFsm();
}

bool chademoInterface_ccsInStateEnd() {
    return pev_state == PEV_STATE_End;
}

bool chademoInterface_ccsPresentVoltageMirrorsTarget() {
    return PresentVoltageDifferentFromTarget_isSet && not PresentVoltageDifferentFromTarget;
}

bool chademoInterface_ccsPresentCurrentMirrorsTarget() {
    return PresentCurrentDifferentFromTarget_isSet && not PresentCurrentDifferentFromTarget;
}

bool chademoInterface_ccsInStateWaitForPreChargeStart() {
    //This would return true even if SafeShutdown and restart...
    //return pev_state > PEV_STATE_WaitForCableCheckResponse;

    // ...while this one should only be true when we are in this state, and should allow ccs statemachine restarts
    return pev_state == PEV_STATE_PreChargeMsg;
}

int chademoInterface_ccsCurrentDemandPos()
{
    if (pev_state < PEV_STATE_CurrentDemandMsg)
        return -1;
    else if (pev_state == PEV_STATE_CurrentDemandMsg)
        return 0;
    else
        return 1;
}
