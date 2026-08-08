#include "ccs32_globals.h"
#include "projectExiConnector.h"

#include "main.h"

#define SEC_TO_CCS_CYCLES(sec) ((sec) * 1000 / 30)

/* The Charging State Machine for the car */
//STATE_ENTRY(internalName, friendlyName, loop_timeout_sec in s)
#define STATE_LIST \
   STATE_ENTRY(Start, Start, 0) \
   STATE_ENTRY(Connected, Connected, 0) \
   STATE_ENTRY(WaitForSupportedApplicationProtocolResponse, NegotiateProtocol, 2) \
   STATE_ENTRY(WaitForSessionSetupResponse, SessionSetup, 2) \
   STATE_ENTRY(WaitForServiceDiscoveryResponse, ServiceDiscovery, 2) \
   STATE_ENTRY(WaitForServicePaymentSelectionResponse, PaymentSelection, 2) \
   STATE_ENTRY(WaitForContractAuthenticationResponse, ContractAuthentication, 2) \
   STATE_ENTRY(WaitForChargeParameterDiscoveryResponse, ChargeParameterDiscovery, 2) /* was 5 */ /* On some charger models, the chargeParameterDiscovery needs more than a second. Wait at least 5s. */ \
   STATE_ENTRY(WaitForCableCheckResponse, CableCheck, 2) /* was30 */ \
   STATE_ENTRY(WaitForPreChargeStart, PreChargeStart, 0) /* was 10 */ \
   STATE_ENTRY(WaitForPreChargeResponse, PreCharge, 2) /* was 30*/ \
   STATE_ENTRY(WaitForContactorsClosed, ContactorsClosed, 0) /* was 5*/ \
   STATE_ENTRY(WaitForPowerDeliveryOnResponse, PowerDeliveryOn, 2) /* was 6 PowerDelivery may need some time. Wait at least 6s. On Compleo charger, observed more than 1s until response. specified performance time is 4.5s (ISO) */\
   STATE_ENTRY(WaitForCurrentDemandResponse, CurrentDemand, 1) /* was 5. Test with 5s timeout. Just experimental. The specified performance time is 25ms (ISO), the specified timeout 250ms. */\
   STATE_ENTRY(WaitForPowerDeliveryOffResponse, PowerDeliveryOff, 2) /*  was 6 PowerDelivery may need some time. Wait at least 6s. On Compleo charger, observed more than 1s until response. specified performance time is 4.5s (ISO) */\
   STATE_ENTRY(WaitForCurrentDownAfterStateB, CurrentDown, 0) /* was 60 */ \
   STATE_ENTRY(WaitForPowerRelayOff, RelayOff, 10) /* was60*/ \
   STATE_ENTRY(WaitForWeldingDetectionResponse, WeldingDetection, 2) \
   STATE_ENTRY(WaitForSessionStopResponse, SessionStop, 2) \
   STATE_ENTRY(SafeShutDown, SafeShutDown, 0) \
   STATE_ENTRY(SafeShutDownWaitForChargerShutdown, WaitForChargerShutdown, 0) \
   STATE_ENTRY(Stop, Stop, 0) \
   STATE_ENTRY(End, End, 0)

//States enum
#define STATE_ENTRY(name, fname, loop_timeout_sec) PEV_STATE_##name,
enum pevstates {
    STATE_LIST
};
#undef STATE_ENTRY

//state function prototypes
#define STATE_ENTRY(name, fname, loop_timeout_sec) static void stateFunction##name();
STATE_LIST
#undef STATE_ENTRY

//State function array
#define STATE_ENTRY(name, fname, loop_timeout_sec) stateFunction##name,
static void(* const stateFunctions[])() = {
STATE_LIST
};
#undef STATE_ENTRY

//Timeout array
#define STATE_ENTRY(name, fname, loop_timeout_sec) SEC_TO_CCS_CYCLES(loop_timeout_sec),
static const uint16_t loop_timeouts[] = {
STATE_LIST
};
#undef STATE_ENTRY

//Enum string for data module
#define STATE_ENTRY(name, fname, loop_timeout_sec) __COUNTER__=fname,
const char* pevSttString = STRINGIFY(STATE_LIST);
#undef STATE_ENTRY

//String array for logging
#define STATE_ENTRY(name, fname, loop_timeout_sec) #fname,
const char* const pevSttLabels[] = { STATE_LIST };
#undef STATE_ENTRY

#define MAX_NUMBER_OF_WELDING_DETECTION_ROUNDS 10 /* The process time is specified with 1.5s. Ten loops should be fine. */
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


static uint16_t pev_cyclesInLoop;
static uint32_t pev_cyclesInState;
static uint8_t pev_DelayCycles;
static pevstates pev_state = PEV_STATE_Start;
static uint16_t pev_numberOfContractAuthenticationReq;
static uint16_t pev_numberOfChargeParameterDiscoveryReq;
static uint16_t pev_numberOfCableCheckReq;
static int LastCurrentDemandResPresentVoltage;
static int LastTargetVoltage;
static int LastTargetCurrent;
static bool PrechargeDifferenceIsSmall;
static uint8_t numberOfWeldingDetectionRounds;

static bool PresentVoltageDifferentFromTarget;
static bool PresentVoltageDifferentFromTarget_isSet;

static bool PresentCurrentDifferentFromTarget;
static bool PresentCurrentDifferentFromTarget_isSet;

static bool ChargeParameterDiscoveryCompletedTrigger;

/***local function prototypes *****************************************/

static uint8_t pev_isTooLong(void);
static void pev_enterState(pevstates n);
static void pev_loopState();

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
        addToTrace(MOD_PEV, "Error: EXI does not fit into tcpPayload.");
    }
}

static void encodeAndTransmit(void)
{
    /* calls the EXI encoder, adds the V2GTP header and sends the result to ethernet */
    //addToTrace("before: g_errn=%d", g_errn);
    //addToTrace("global_streamEncPos=%d", global_streamEncPos);
    global_streamEncPos = 0;
    projectExiConnector_encode_DinExiDocument();
    //addToTrace("after: g_errn=%d", g_errn);
    //addToTrace("global_streamEncPos=%d", global_streamEncPos);
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

static inline bool consume_din()
{
    if (tcp_rxdataLen > V2GTP_HEADER_SIZE)
    {
        routeDecoderInputData();
        projectExiConnector_decode_DinExiDocument();
        tcp_rxdataLen = 0; /* mark the input data as "consumed" */
        return true;
    }
    return false;
}

#define CONSUME_DIN_MESSAGE(msg) (consume_din() && msg##_isUsed) \

static inline bool consume_app()
{
    if (tcp_rxdataLen > V2GTP_HEADER_SIZE)
    {
        routeDecoderInputData();
        projectExiConnector_decode_appHandExiDocument();
        tcp_rxdataLen = 0; /* mark the input data as "consumed" */
        return true;
    }
    return false;
}

#define CONSUME_APP_MESSAGE(msg) (consume_app() && msg##_isUsed) \

/********* EXI creation functions ************************/
static void pev_sendSessionSetupReq()
{
    uint8_t i;
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
    for (i = 0; i < LEN_OF_EVCCID; i++)
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

static void pev_SendContractAuthenticationReq()
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
    addToTrace(MOD_PEV, "Send PreChargeReq:%dV", targetVoltage);

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
        addToTrace(MOD_PEV, "Warning: TargetVoltage %dV is above EVMaximumVoltageLimit %dV, which may cause charger shutdown.",
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

static void stateFunctionConnected()
{
    // We have a freshly established TCP channel. We start the V2GTP/EXI communication now.
    // We just use the initial request message from the Ioniq. It contains one entry: DIN.
    addToTrace(MOD_PEV, "Checkpoint400: Sending the initial SupportedApplicationProtocolReq");
    setCheckpoint(400);

    /*
    constexpr char protocolNamespace[] = "urn:din:70121:2012:MsgDef";
    constexpr size_t protocolNamespaceLen = sizeof(protocolNamespace) - 1; // exclusive nullterm
    memcpy(
        aphsDoc.supportedAppProtocolReq.AppProtocol.array[0].ProtocolNamespace.characters,
        protocolNamespace,
        protocolNamespaceLen);
    aphsDoc.supportedAppProtocolReq.AppProtocol.array[0].ProtocolNamespace.charactersLen = protocolNamespaceLen;
    aphsDoc.supportedAppProtocolReq.AppProtocol.array[0].VersionNumberMajor = 1;
    aphsDoc.supportedAppProtocolReq.AppProtocol.array[0].VersionNumberMinor = 0;
    aphsDoc.supportedAppProtocolReq.AppProtocol.array[0].SchemaID = 1;
    aphsDoc.supportedAppProtocolReq.AppProtocol.array[0].Priority = 1;
    aphsDoc.supportedAppProtocolReq.AppProtocol.arrayLen = 1;
    */

    addV2GTPHeaderAndTransmit(exiDemoSupportedApplicationProtocolRequestIoniq, sizeof(exiDemoSupportedApplicationProtocolRequestIoniq));
    _ccs_params.CurrentDemandStopReason = STOP_REASON_NONE;
    pev_enterState(PEV_STATE_WaitForSupportedApplicationProtocolResponse);
}

static void stateFunctionWaitForSupportedApplicationProtocolResponse()
{
    if (CONSUME_APP_MESSAGE(aphsDoc.supportedAppProtocolRes))
    {
        addToTrace(MOD_PEV, "In state WaitForSupportedApplicationProtocolResponse");
        /* it is the correct response */
        addToTrace(MOD_PEV, "supportedAppProtocolRes ResponseCode:%d, SchemaID_isUsed:%d, SchemaID:%d",
            aphsDoc.supportedAppProtocolRes.ResponseCode,
            aphsDoc.supportedAppProtocolRes.SchemaID_isUsed,
            aphsDoc.supportedAppProtocolRes.SchemaID);
        addToTrace(MOD_PEV, "Checkpoint403: Schema negotiated. And Checkpoint500: Will send SessionSetupReq");
        setCheckpoint(500);
        pev_sendSessionSetupReq();
        pev_enterState(PEV_STATE_WaitForSessionSetupResponse);
    }
}

static void stateFunctionWaitForSessionSetupResponse()
{
    if (CONSUME_DIN_MESSAGE(dinDocDec.V2G_Message.Body.SessionSetupRes))
    {
        addToTrace(MOD_PEV, "In state WaitForSessionSetupResponse");
        memcpy(sessionId, dinDocDec.V2G_Message.Header.SessionID.bytes, SESSIONID_LEN);
        sessionIdLen = dinDocDec.V2G_Message.Header.SessionID.bytesLen; /* store the received SessionID, we will need it later. */
        addToTrace_bytes(MOD_PEV, "Checkpoint506: The Evse decided for SessionId", sessionId, sessionIdLen);
        setCheckpoint(506);
        addToTrace(MOD_PEV, "Will send ServiceDiscoveryReq");
        setCheckpoint(510);
        pev_sendServiceDiscoveryReq();
        pev_enterState(PEV_STATE_WaitForServiceDiscoveryResponse);
    }
}

static void stateFunctionWaitForServiceDiscoveryResponse()
{
    if (CONSUME_DIN_MESSAGE(dinDocDec.V2G_Message.Body.ServiceDiscoveryRes))
    {
        addToTrace(MOD_PEV, "In state WaitForServiceDiscoveryResponse");
        addToTrace(MOD_PEV, "Will send ServicePaymentSelectionReq");
        setCheckpoint(520);
        pev_sendServicePaymentSelectionReq();
        pev_enterState(PEV_STATE_WaitForServicePaymentSelectionResponse);
    }
}

static void stateFunctionWaitForServicePaymentSelectionResponse()
{
    if (CONSUME_DIN_MESSAGE(dinDocDec.V2G_Message.Body.ServicePaymentSelectionRes))
    {
        addToTrace(MOD_PEV, "In state WaitForServicePaymentSelectionResponse");
        addToTrace(MOD_PEV, "Checkpoint530: Will send ContractAuthenticationReq");
        setCheckpoint(530);
        pev_SendContractAuthenticationReq();
        pev_numberOfContractAuthenticationReq = 1; // This is the first request.
        pev_enterState(PEV_STATE_WaitForContractAuthenticationResponse);
    }
}

static void stateFunctionWaitForContractAuthenticationResponse()
{
    if (pev_cyclesInLoop < SEC_TO_CCS_CYCLES(1))   // The first second in the state just do nothing.
    {
        return;
    }
    if (CONSUME_DIN_MESSAGE(dinDocDec.V2G_Message.Body.ContractAuthenticationRes))
    {
        addToTrace(MOD_PEV, "In state WaitForContractAuthenticationResponse");
        // In normal case, we can have two results here: either the Authentication is needed (the user
        // needs to authorize by RFID card or app, or something like this.
        // Or, the authorization is finished. This is shown by EVSEProcessing=Finished.
        if (dinDocDec.V2G_Message.Body.ContractAuthenticationRes.EVSEProcessing == dinEVSEProcessingType_Finished)
        {
            addToTrace(MOD_PEV, "Checkpoint538 and 540: Auth is Finished. Will send ChargeParameterDiscoveryReq");
            setCheckpoint(540);
            pev_sendChargeParameterDiscoveryReq();
            pev_numberOfChargeParameterDiscoveryReq = 1; // first message
            ChargeParameterDiscoveryCompletedTrigger = false; // reset
            pev_enterState(PEV_STATE_WaitForChargeParameterDiscoveryResponse);
        }
        else
        {
            // Not (yet) finished.
            if (pev_cyclesInState > SEC_TO_CCS_CYCLES(120))//  pev_numberOfContractAuthenticationReq >= 120)   // approx 120 seconds, maybe the user searches two minutes for his RFID card...
            {
                addToTrace(MOD_PEV, "Authentication lasted too long. Giving up.");
                pev_enterState(PEV_STATE_SafeShutDown);
            }
            else
            {
                // Try again.
                pev_numberOfContractAuthenticationReq += 1; // count the number of tries.
                addToTrace(MOD_PEV, "Not (yet) finished. Will again send ContractAuthenticationReq #%d", pev_numberOfContractAuthenticationReq);
                pev_SendContractAuthenticationReq();                //was encodeAndTransmit();
                // We just stay in the same state, until the timeout elapses.
                pev_loopState();
            }
        }
    }
}

static void stateFunctionWaitForChargeParameterDiscoveryResponse()
{
    if (pev_cyclesInLoop < SEC_TO_CCS_CYCLES(1))   // The first second in the state just do nothing.
    {
        return;
    }
    if (CONSUME_DIN_MESSAGE(dinDocDec.V2G_Message.Body.ChargeParameterDiscoveryRes))
    {
        addToTrace(MOD_PEV, "In state WaitForChargeParameterDiscoveryResponse");
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

            addToTrace(MOD_PEV, "Checkpoint550: ChargeParams are discovered: min:%dV max:%dV/%dA. Will change to state C.",
                evseMinimumVoltage, evseMaxVoltage, evseMaxCurrent);

            setCheckpoint(550);
            // pull the CP line to state C here:
            hardwareInterface_setStateC();
            addToTrace(MOD_PEV, "Checkpoint555: Locking the connector.");
            hardwareInterface_lockConnector();

            addToTrace(MOD_PEV, "Checkpoint560: Send CableCheckReq.");
            setCheckpoint(560);
            pev_sendCableCheckReq();
            pev_numberOfCableCheckReq = 1; // This is the first request.
            pev_enterState(PEV_STATE_WaitForCableCheckResponse);
        }
        else
        {
            // Not (yet) finished.
            if (pev_cyclesInState > SEC_TO_CCS_CYCLES(60))// pev_numberOfChargeParameterDiscoveryReq >= 60)
            {
                /* approx 60 seconds, should be sufficient for the charger to find its parameters.
                    ... The ISO allows up to 55s reaction time and 60s timeout for "ongoing". Taken over from
                        https://github.com/uhi22/pyPLC/commit/01c7c069fd4e7b500aba544ae4cfce6774f7344a */
                addToTrace(MOD_PEV, "ChargeParameterDiscovery lasted too long:%d Giving up.", pev_numberOfChargeParameterDiscoveryReq);
                pev_enterState(PEV_STATE_SafeShutDown);
            }
            else
            {
                // Try again.
                pev_numberOfChargeParameterDiscoveryReq += 1; // count the number of tries.
                addToTrace(MOD_PEV, "Not (yet) finished. Will again send ChargeParameterDiscoveryReq #%d", pev_numberOfChargeParameterDiscoveryReq);
                pev_sendChargeParameterDiscoveryReq();
                // we stay in the same state
                pev_loopState();
            }
        }
    }
}

static void stateFunctionWaitForCableCheckResponse()
{
    uint8_t rc, proc;
    if (pev_cyclesInLoop < SEC_TO_CCS_CYCLES(1))   // The first second in the state just do nothing.
    {
        return;
    }
    if (CONSUME_DIN_MESSAGE(dinDocDec.V2G_Message.Body.CableCheckRes))
    {
        rc = dinDocDec.V2G_Message.Body.CableCheckRes.ResponseCode;
        proc = dinDocDec.V2G_Message.Body.CableCheckRes.EVSEProcessing;
        _ccs_params.EvseVoltage = 0;
        // We have two cases here:
        // 1) The charger says "cable check is finished and cable ok", by setting ResponseCode=OK and EVSEProcessing=Finished.
        // 2) Else: The charger says "need more time or cable not ok". In this case, we just run into timeout and start from the beginning.
        if (rc == dinresponseCodeType_OK && proc == dinEVSEProcessingType_Finished)
        {
            addToTrace(MOD_PEV, "The EVSE says that the CableCheck is finished and ok.");
            pev_enterState(PEV_STATE_WaitForPreChargeStart);
        }
        else if (rc == dinresponseCodeType_OK && proc == dinEVSEProcessingType_Ongoing)
        {
            if (pev_cyclesInState > SEC_TO_CCS_CYCLES(60))// pev_numberOfCableCheckReq > 60)   /* approx 60s should be sufficient for cable check. The ISO allows up to 55s reaction time and 60s timeout for "ongoing". Taken over from https://github.com/uhi22/pyPLC/commit/01c7c069fd4e7b500aba544ae4cfce6774f7344a */
            {
                addToTrace(MOD_PEV, "CableCheck lasted too long:%d Giving up.", pev_numberOfCableCheckReq);
                pev_enterState(PEV_STATE_SafeShutDown);
            }
            else
            {
                // cable check not yet finished -> try again
                pev_numberOfCableCheckReq += 1;
                addToTrace(MOD_PEV, "Will again send CableCheckReq");
                pev_sendCableCheckReq();
                // stay in the same state
                pev_loopState();
            }
        }
        else // spec only mention the 2 cases above, assuming all other cases must be errors
        {
            addToTrace(MOD_PEV, "CableCheck error rc:%d proc:%d", rc, proc);
            pev_enterState(PEV_STATE_SafeShutDown);
        }
    }
}

static void stateFunctionWaitForPreChargeStart()
{
    // wait 2 sec. It is possible some chargers do not like precharge lasting longer than 7 seconds? This at least saves 2 :-)
    // Its "impossible" that chademo uses less than 2 seconds until reaching _preChargeDoneButStalled, so it should be safe to wait 2 sec here
    // without worry about chademo needing to wait unnecesary for _preChargeDoneButStalled.
    // How long can we wait before sending the first PreChargeReq? It seems undefined in spec.
    if (CONFIG_SX ? true : pev_cyclesInState > (DX_CCS_WaitForPreChargeStart_MS / 30))
    {
        uint16_t batVtg = hardwareInterface_getBatteryVoltage();

        if (batVtg < _ccs_params.EvseMinimumVoltage) {
            // Unlikely that this can happen, and if it does, then precharge will never be satisfied and charger will go into timeout, so don't need to handle it specially
            addToTrace(MOD_PEV, "Warning: batteryVoltage:%d is less than evseMinimumVoltage:%d", batVtg, _ccs_params.EvseMinimumVoltage);
        }

        PrechargeDifferenceIsSmall = false; // reset

        addToTrace(MOD_PEV, "Will send PreChargeReq");
        setCheckpoint(570);
        pev_sendPreChargeReq(batVtg);
        //        connMgr_ApplOk(31); /* PreChargeResponse may need longer. Inform the connection manager to be patient.
                //                    (This is a takeover from https://github.com/uhi22/pyPLC/commit/08af8306c60d57c4c33221a0dbb25919371197f9 ) */
        pev_enterState(PEV_STATE_WaitForPreChargeResponse);
    }
}

static void stateFunctionWaitForPreChargeResponse()
{
    if (CONSUME_DIN_MESSAGE(dinDocDec.V2G_Message.Body.PreChargeRes))
    {
        _global.auto_power_off_timer_count_up_ms = 0;

        int evsePresentVoltage = combineValueAndMultiplier(dinDocDec.V2G_Message.Body.PreChargeRes.EVSEPresentVoltage);
        _ccs_params.EvseVoltage = evsePresentVoltage;

        addToTrace(MOD_PEV, "PreCharge response:%dV", evsePresentVoltage);
        setCheckpoint(571);

        uint16_t inletVtg = hardwareInterface_getInletVoltage();
        uint16_t batVtg = hardwareInterface_getBatteryVoltage();

        if (not PrechargeDifferenceIsSmall)
        {
            if (ABS(inletVtg - batVtg) < PARAM_U_DELTA_MAX_FOR_END_OF_PRECHARGE)
            {
                addToTrace(MOD_PEV, "PreCharge difference is small (inlet:%dV batt:%dV)", inletVtg, batVtg);
                setCheckpoint(572);
                PrechargeDifferenceIsSmall = true;
            }
        }

        if (PrechargeDifferenceIsSmall)
        {
            // This check may be random. pev_runFsm already handle stop. But I guess it does not hurt with an extra check.
            if (chademoInterface_chargingLoopPos() > 0)
            {
                // Not sure if this can happen, but in case, chademo must have ended/failed already
                addToTrace(MOD_PEV, "Error: Can not complete precharge -> chademo is past ChargingLoop");
                pev_enterState(PEV_STATE_SafeShutDown);
                return;
            }
            
            if (chademoInterface_preChargeCompleted())
            {
                addToTrace(MOD_PEV, "PreCharge completed.");
                setCheckpoint(573);
                // Turn the power relay on.
                hardwareInterface_setPowerRelayOn();
                pev_enterState(PEV_STATE_WaitForContactorsClosed);
                return;
            }
        }

        // send again
        pev_sendPreChargeReq(batVtg);
        pev_DelayCycles = 15; // wait with the next evaluation approx half a second
        pev_loopState();
    }
}

static void stateFunctionWaitForContactorsClosed()
{
    if (pev_cyclesInState < 15)
    {
        /* simplified solution for waiting for the contactors: Since the contactors anyway have no feedback whether
           they are really closed, we just use a time-based approach. In
           https://github.com/uhi22/ccs32clara/issues/22 we see that it takes ~350ms until both contactors have
           current, so we wait here 15 cycles * 30ms = 450ms, and additional delay will be caused by the
           powerDeliveryRequest/Response and the currentDemandRequest/Response. So this should give sufficient
           time to close the contactors until the charger really provides current. */
        return;
    }
    addToTrace(MOD_PEV, "Contactors assumingly finished closing. Sending PowerDeliveryReq.");
    pev_sendPowerDeliveryReq(true); /* true is ON */
    setCheckpoint(600);
    pev_enterState(PEV_STATE_WaitForPowerDeliveryOnResponse);
}

static void stateFunctionWaitForPowerDeliveryOnResponse()
{
    if (CONSUME_DIN_MESSAGE(dinDocDec.V2G_Message.Body.PowerDeliveryRes))
    {
        if (dinDocDec.V2G_Message.Body.PowerDeliveryRes.ResponseCode == dinresponseCodeType_OK)
        {
            addToTrace(MOD_PEV, "Checkpoint700: Starting the charging loop with CurrentDemandReq");
            setCheckpoint(700);
            pev_sendCurrentDemandReq();
            pev_enterState(PEV_STATE_WaitForCurrentDemandResponse);
        }
        else
        {
            // FAILED_PowerDeliveryNotApplied (17) seen in cases where precharge voltage was < 20V less than battery voltage,
            // even if precharge is continued and the precharge voltage rised to battery voltage. This is strange
            // (not that it is failing, it may create huge inrush current against the charger, when precharge voltage is lower),
            // but it is strange that it is not failing during the precharge itself, but during PowerDelivery. This made it harder
            // to guess why it failed, but after experimenting, this seems to be the most likely cause.
            // This means: precharge can not be abused to adjust the voltage after closing contactors, the voltage must be adjusted before closing contactors.
            // Exception: it seems the charger dislike lower voltage (than battery) more than higher voltage (than battery):
            // Lower: huge current inrush agains charger. Car has no way to limit amps. Higher: inrush agains car, but charger is current limiting, so it will be max 1A (precharge current).
            addToTrace(MOD_PEV, "PowerDelivery failed rc:%d", dinDocDec.V2G_Message.Body.PowerDeliveryRes.ResponseCode);
            pev_enterState(PEV_STATE_SafeShutDown);
        }
    }
}

static void stateFunctionWaitForCurrentDemandResponse()
{
    if (CONSUME_DIN_MESSAGE(dinDocDec.V2G_Message.Body.CurrentDemandRes))
    {
        _global.auto_power_off_timer_count_up_ms = 0;

        /* as long as the battery is not full and no stop-demand from the user, we continue charging */
        _stopreasons currentDemandStopReason = STOP_REASON_NONE;
        if (dinDocDec.V2G_Message.Body.CurrentDemandRes.DC_EVSEStatus.EVSEStatusCode == dinDC_EVSEStatusCodeType_EVSE_Shutdown)
        {
            /* https://github.com/uhi22/pyPLC#example-flow, checkpoint 790: If the user stops the
               charging session on the charger, we get a CurrentDemandResponse with
               DC_EVSEStatus.EVSEStatusCode = 2 "EVSE_Shutdown" (observed on Compleo. To be tested
               on other chargers. */
            addToTrace(MOD_PEV, "User requested stop on charger side.");
            setCheckpoint(790);
            currentDemandStopReason = STOP_REASON_CHARGER_SHUTDOWN;
        }
        else if (dinDocDec.V2G_Message.Body.CurrentDemandRes.DC_EVSEStatus.EVSEStatusCode == dinDC_EVSEStatusCodeType_EVSE_Malfunction)
        {
            /* If the charger reports a malfunction, we stop the charging. */
            /* Issue reference: https://github.com/uhi22/ccs32clara/issues/29 */
            addToTrace(MOD_PEV, "Charger reported EVSE_Malfunction. A reason could be hitting the EVSEMinimumVoltageLimit or EVSEMaximumVoltageLimit.");
            currentDemandStopReason = STOP_REASON_CHARGER_EVSE_MALFUNCTION;
        }
        else if (dinDocDec.V2G_Message.Body.CurrentDemandRes.DC_EVSEStatus.EVSEStatusCode == dinDC_EVSEStatusCodeType_EVSE_EmergencyShutdown)
        {
            /* If the charger reports an emergency, we stop the charging. */
            addToTrace(MOD_PEV, "Charger reported EmergencyShutdown.");
            currentDemandStopReason = STOP_REASON_CHARGER_EMERGENCY_SHUTDOWN;
        }
        else if (hardwareInterface_stopChargeRequested())
        {
            addToTrace(MOD_PEV, "User requested stop on car side. Sending PowerDeliveryReq Stop.");
            currentDemandStopReason = STOP_REASON_POWER_OFF_PENDING;
        }
        else if (hardwareInterface_getIsBatteryFull())
        {
            addToTrace(MOD_PEV, "Battery is full. Sending PowerDeliveryReq Stop.");
            currentDemandStopReason = STOP_REASON_BATTERY_FULL;
        }

        if (currentDemandStopReason != STOP_REASON_NONE)
        {
            _ccs_params.CurrentDemandStopReason = currentDemandStopReason;
            setCheckpoint(800);
            pev_sendPowerDeliveryReq(false); /* we can immediately send the powerDeliveryStopRequest, while we are under full current.
                                            sequence explained here: https://github.com/uhi22/pyPLC#detailled-investigation-about-the-normal-end-of-the-charging-session */
            pev_enterState(PEV_STATE_WaitForPowerDeliveryOffResponse);
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

            setCheckpoint(710);
            pev_sendCurrentDemandReq();
            pev_loopState();
        }
    }
}

static void stateFunctionWaitForPowerDeliveryOffResponse()
{
    if (CONSUME_DIN_MESSAGE(dinDocDec.V2G_Message.Body.PowerDeliveryRes))
    {
        /* We requested "OFF". This is while the charging session is ending.
        When we received this response, the charger had up to 1.5s time to ramp down
        the current. On Compleo, there are really 1.5s until we get this response.
        See https://github.com/uhi22/pyPLC#detailled-investigation-about-the-normal-end-of-the-charging-session */
        setCheckpoint(810);
        /* set the CP line to B */
        hardwareInterface_setStateB(); /* ISO Figure 107: The PEV shall set stateB after receiving PowerDeliveryRes and before WeldingDetectionReq */
        addToTrace(MOD_PEV, "Giving the charger some time to detect StateB and ramp down the current.");
        pev_enterState(PEV_STATE_WaitForCurrentDownAfterStateB); /* We give the charger some time to detect the StateB and fully ramp down the current */
    }
}

static void stateFunctionWaitForCurrentDownAfterStateB()
{
    /* During normal end of the charging session, we have set the StateB, and want to give the charger some time to ramp down the current completely, before we are opening the contactors. */
    /* 15*30ms=450ms for charger shutdown. Should be more than sufficient, because somewhere was a requirement with 20ms between StateB until current is down. The Ioniq uses 300ms. */
    if (pev_cyclesInState < 10) {
        /* just waiting */
        return;
    }
    /* Time is over. Current flow should have been stopped by the charger. Let's open the contactors and send a weldingDetectionRequest, to find out whether the voltage drops. */
    hardwareInterface_setPowerRelayOff();
    setCheckpoint(850);
    pev_enterState(PEV_STATE_WaitForPowerRelayOff);
}

static void stateFunctionWaitForPowerRelayOff()
{
    // Wait for chademo to ACK _ccs_params.ContactorClosed we sat in hardwareInterface_setPowerRelayOff()
    if (chademoInterface_adapterContactorOpened())
    {
        addToTrace(MOD_PEV, "Starting WeldingDetection");

        /* We do not need a waiting time before sending the weldingDetectionRequest, because the weldingDetection
        will be anyway in a loop. So the first round will see a high voltage (because the contactor mechanically needed
        some time to open, but this is no problem, the next samples will see decreasing voltage in normal case. */
        numberOfWeldingDetectionRounds = 0;
        pev_sendWeldingDetectionReq();
        pev_enterState(PEV_STATE_WaitForWeldingDetectionResponse);
    }
}

static void stateFunctionWaitForWeldingDetectionResponse()
{
    if (CONSUME_DIN_MESSAGE(dinDocDec.V2G_Message.Body.WeldingDetectionRes))
    {
        addToTrace(MOD_PEV, "In state WaitForWeldingDetectionRes");
        /* The charger measured the voltage on the cable, and gives us the value. In the first
           round will show a quite high voltage, because the contactors are just opening. We
           need to repeat the requests, until the voltage is at a non-dangerous level. */
        int evsePresentVoltage = combineValueAndMultiplier(dinDocDec.V2G_Message.Body.WeldingDetectionRes.EVSEPresentVoltage);
        _ccs_params.EvseVoltage = evsePresentVoltage;
        addToTrace(MOD_PEV, "EVSEPresentVoltage %dV", evsePresentVoltage);
        bool voltageIsLow = evsePresentVoltage < MAX_VOLTAGE_TO_FINISH_WELDING_DETECTION;
        if (voltageIsLow || numberOfWeldingDetectionRounds > MAX_NUMBER_OF_WELDING_DETECTION_ROUNDS) {

            if (not voltageIsLow) {
                if (evsePresentVoltage == LastCurrentDemandResPresentVoltage) {
                    // Charger still says it has the same voltage as when we were last charging.
                    // If we were welded, the measured voltage should be battery voltage, and its very unlikely that this would be exactly the same as last charging voltage.
                    // It should most certainly be lower, as the charging voltage is always higher than the battery voltage.
                    // So it seems the charger is lying to us and just send us its last known voltage.
                    addToTrace(MOD_PEV, "WeldingDetection voltage equals last charging voltage: charger is probably lying.");
                }
                else {
                    /* even after multiple welding detection requests/responses, the voltage did not fall as expected.
                    This may be due to two hanging/welded contactors or an issue of the charging station. */
                    addToTrace(MOD_PEV, "WeldingDetection: ERROR: Did not reach low voltage: contactors probably welded.");
                }
            }

            addToTrace(MOD_PEV, "WeldingDetection finished. Sending SessionStopReq");
            setCheckpoint(900);
            pev_sendSessionStopReq();
            pev_enterState(PEV_STATE_WaitForSessionStopResponse);
        }
        /* The voltage on the cable is still high, so we make another round with the WeldingDetection. */
        else {
            /* max number of rounds not yet reached */
            numberOfWeldingDetectionRounds++; /* https://github.com/uhi22/ccs32clara/issues/55
                                                 Count the number of welding detection rounds. To be clarified, whether
                                                 a certain time or number of rounds make sense to cover all use cases with
                                                 different chargers etc */
            addToTrace(MOD_PEV, "WeldingDetection: voltage still too high. Sending again WeldingDetectionReq:%d", numberOfWeldingDetectionRounds);
            pev_sendWeldingDetectionReq();
            pev_loopState();
        }
    }
}

static void stateFunctionWaitForSessionStopResponse()
{
    if (CONSUME_DIN_MESSAGE(dinDocDec.V2G_Message.Body.SessionStopRes))
    {
        addToTrace(MOD_PEV, "In state WaitForSessionStopRes");
        tcp_disconnect();
        addToTrace(MOD_PEV, "Charging is finished");
        pev_enterState(PEV_STATE_Stop);
    }
}

static void stateFunctionSafeShutDown()
{
    /* Here we end, if we run into a timeout in the state machine (or other error before we reach CurrentDemand). */
    /* Initiate the safe-shutdown-sequence. */
    addToTrace(MOD_PEV, "Safe-shutdown-sequence: setting state B");
    setCheckpoint(1100);
    hardwareInterface_setStateB(); /* setting CP line to B disables in the charger the current flow. */
    tcp_disconnect(); /* Set StateB is our last communication with the charger during safe shutdown, so close tcp as well. */
    pev_enterState(PEV_STATE_SafeShutDownWaitForChargerShutdown);
}

static void stateFunctionSafeShutDownWaitForChargerShutdown()
{
    /* wait 2sec, to give the charger the time to stop the current. */
    if (pev_cyclesInState < SEC_TO_CCS_CYCLES(2))
    {
        return;
    }
    /* Now the current flow is stopped by the charger. We can safely open the contactors: */
    addToTrace(MOD_PEV, "Safe-shutdown-sequence: opening contactors");
    setCheckpoint(1300);
    hardwareInterface_setPowerRelayOff();
    pev_enterState(PEV_STATE_Stop);
}

static void stateFunctionStop()
{
    /* Set voltage to 0 so rest of system sees safe state */
    _ccs_params.EvseVoltage = 0;

    hardwareInterface_unlockConnector();

    // I guess we would want to retry if something failed, but only if we did not complete ChargeParameterDiscovery,
    // because this trigger stateC and connector locking (energizing state). Something failing after this is probably hardware/high voltage related, and will not "heal".
    if (not ChargeParameterDiscoveryCompletedTrigger && not hardwareInterface_stopChargeRequested())
    {
        addToTrace(MOD_PEV, "Did not complete ChargeParameterDiscovery -> restart");
        pev_enterState(PEV_STATE_Start);
        connMgr_restart();
    }
    else
    {
        pev_enterState(PEV_STATE_End);
    }
}

static void stateFunctionEnd()
{
    // terminal state. No code should exist here. We can never leave this state
}

static inline void pev_loopState()
{
    pev_enterState(pev_state);
}

static void pev_enterState(pevstates n)
{
    if (n != pev_state)
    {
        addToTrace(MOD_PEV, "[PEV] entering %s", pevSttLabels[n]);
        pev_cyclesInState = 0;
    }

    pev_state = n;
    pev_cyclesInLoop = 0; // since pev_enterState called
    _ccs_params.opmode = n;
}

static uint8_t pev_isTooLong(void)
{
    return loop_timeouts[pev_state] > 0 && pev_cyclesInLoop > loop_timeouts[pev_state];
}

/******* The statemachine dispatcher *******************/
static void pev_runFsm(void)
{
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
        pev_DelayCycles--; /* just waiting */
    }
    else
    {
        stateFunctions[pev_state](); // call state function
    }

    if (pev_state != PEV_STATE_WaitForCurrentDemandResponse) //only in currentDemand we have meaningful current values
        _ccs_params.EvseCurrent = 0;

    bool stop = false;
    if (pev_isTooLong())
    {
        addToTrace(MOD_PEV, "Timeout in state %s", pevSttLabels[pev_state]);
        stop = true;
    }

    if (loop_timeouts[pev_state] > 0 && not tcp_isConnected())
    {
        addToTrace(MOD_PEV, "Tcp connection lost in timeoutable state %s", pevSttLabels[pev_state]);
        stop = true;
    }

    if (hardwareInterface_stopChargeRequested() // powerOffPending?
        && hardwareInterface_isConnectorLocked() // why check for connector lock? old logic related to late plug insertion?
        && pev_state < PEV_STATE_WaitForCurrentDemandResponse
        )
    {
        addToTrace(MOD_PEV, "Stop charging requested before CurrentDemand (%s)", pevSttLabels[pev_state]);
        stop = true;
    }

    if (stop)
    {
        // Make sure we set a CurrentDemandStopReason if we pull the rug on CurrentDemand
        if (pev_state == PEV_STATE_WaitForCurrentDemandResponse && _ccs_params.CurrentDemandStopReason == STOP_REASON_NONE)
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
    pev_cyclesInLoop += 1; // for timeout handling, count how long we are in a loop
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
    return pev_state == PEV_STATE_WaitForPreChargeStart;
}

int chademoInterface_ccsCurrentDemandPos()
{
    if (pev_state < PEV_STATE_WaitForCurrentDemandResponse)
        return -1;
    else if (pev_state == PEV_STATE_WaitForCurrentDemandResponse)
        return 0;
    else
        return 1;
}
