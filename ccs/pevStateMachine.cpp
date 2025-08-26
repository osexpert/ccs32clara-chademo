#include "ccs32_globals.h"
#include "projectExiConnector.h"

/* The Charging State Machine for the car */
//STATE_ENTRY(internalName, friendlyName, timeout in s)
#define STATE_LIST \
   STATE_ENTRY(NotYetInitialized, Off, 0) \
   STATE_ENTRY(Connecting, Connecting, 0) \
   STATE_ENTRY(Connected, Connected, 0) \
   STATE_ENTRY(WaitForSupportedApplicationProtocolResponse, NegotiateProtocol, 2) \
   STATE_ENTRY(WaitForSessionSetupResponse, SessionSetup, 2) \
   STATE_ENTRY(WaitForServiceDiscoveryResponse, ServiceDiscovery, 2) \
   STATE_ENTRY(WaitForServicePaymentSelectionResponse, PaymentSelection, 2) \
   STATE_ENTRY(WaitForContractAuthenticationResponse, ContractAuthentication, 2) \
   STATE_ENTRY(WaitForChargeParameterDiscoveryResponse, ChargeParameterDiscovery, 5) /* On some charger models, the chargeParameterDiscovery needs more than a second. Wait at least 5s. */ \
   STATE_ENTRY(WaitForConnectorLock, ConnectorLock, 2) \
   STATE_ENTRY(WaitForCableCheckResponse, CableCheck, 30) \
   STATE_ENTRY(WaitForPreChargeResponse, PreCharge, 30) \
   STATE_ENTRY(WaitForContactorsClosed, ContactorsClosed, 5) \
   STATE_ENTRY(WaitForPowerDeliveryResponse, PowerDelivery, 6) /* PowerDelivery may need some time. Wait at least 6s. On Compleo charger, observed more than 1s until response. specified performance time is 4.5s (ISO) */\
   STATE_ENTRY(WaitForCurrentDemandResponse, CurrentDemand, 5) /* Test with 5s timeout. Just experimental. The specified performance time is 25ms (ISO), the specified timeout 250ms. */\
   STATE_ENTRY(WaitForCurrentDownAfterStateB, WaitCurrentDown, 0) \
   STATE_ENTRY(WaitForWeldingDetectionResponse, WeldingDetection, 2) \
   STATE_ENTRY(WaitForSessionStopResponse, SessionStop, 2) \
   STATE_ENTRY(UnrecoverableError, Error, 0) \
   STATE_ENTRY(SequenceTimeout, Timeout, 0) \
   STATE_ENTRY(SafeShutDownWaitForChargerShutdown, WaitForChargerShutdown, 0) \
   STATE_ENTRY(SafeShutDownWaitForContactorsOpen, WaitForContactorsOpen, 0) \
   STATE_ENTRY(End, End, 0)

//States enum
#define STATE_ENTRY(name, fname, timeout) PEV_STATE_##name,
enum pevstates {
STATE_LIST
};
#undef STATE_ENTRY

//state function prototypes
#define STATE_ENTRY(name, fname, timeout) static void stateFunction##name();
STATE_LIST
#undef STATE_ENTRY

//State function array
#define STATE_ENTRY(name, fname, timeout) stateFunction##name,
static void(*const stateFunctions[])() = {
STATE_LIST
};
#undef STATE_ENTRY

//Timeout array
#define STATE_ENTRY(name, fname, timeout) timeout * 33,
static const uint16_t timeouts[] = {
STATE_LIST
};
#undef STATE_ENTRY

//Enum string for data module
#define STATE_ENTRY(name, fname, timeout) __COUNTER__=fname,
const char* pevSttString = STRINGIFY(STATE_LIST);
#undef STATE_ENTRY

//String array for logging
#define STATE_ENTRY(name, fname, timeout) #fname,
const char pevSttLabels[][MAX_LABEL_LEN] = { STATE_LIST };
#undef STATE_ENTRY

#define MAX_VOLTAGE_TO_FINISH_WELDING_DETECTION 40 /* 40V is considered to be sufficiently low to not harm. The Ioniq already finishes at 65V. */
#define MAX_NUMBER_OF_WELDING_DETECTION_ROUNDS 10 /* The process time is specified with 1.5s. Ten loops should be fine. */

#define LEN_OF_EVCCID 6 /* The EVCCID is the MAC according to spec. Ioniq uses exactly these 6 byte. */


static const uint8_t exiDemoSupportedApplicationProtocolRequestIoniq[]= {0x80, 0x00, 0xdb, 0xab, 0x93, 0x71, 0xd3, 0x23, 0x4b, 0x71, 0xd1, 0xb9, 0x81, 0x89, 0x91, 0x89, 0xd1, 0x91, 0x81, 0x89, 0x91, 0xd2, 0x6b, 0x9b, 0x3a, 0x23, 0x2b, 0x30, 0x02, 0x00, 0x00, 0x04, 0x00, 0x40 };


static uint16_t pev_cyclesInState;
static uint8_t pev_DelayCycles;
static pevstates pev_state=PEV_STATE_NotYetInitialized;
static uint16_t pev_numberOfContractAuthenticationReq;
static uint16_t pev_numberOfChargeParameterDiscoveryReq;
static uint16_t pev_numberOfCableCheckReq;
static uint8_t pev_wasPowerDeliveryRequestedOn;
static int EVSEPresentVoltage;
static int LastChargingVoltage;
static uint16_t EVSEMinimumVoltage;
static uint8_t numberOfWeldingDetectionRounds;

/***local function prototypes *****************************************/

static uint8_t pev_isTooLong(void);
static void pev_enterState(pevstates n);

/*** functions ********************************************************/

static float combineValueAndMultiplier(int32_t val, int8_t multiplier)
{
   float x;
   x = val;
   while (multiplier>0)
   {
      x=x*10;
      multiplier--;
   }
   while (multiplier<0)
   {
      x=x/10;
      multiplier++;
   }
   return x;
}

static float combineValueAndMultiplier(dinPhysicalValueType v)
{
   return combineValueAndMultiplier(v.Value, v.Multiplier);
}

static void addV2GTPHeaderAndTransmit(const uint8_t *exiBuffer, uint8_t exiBufferLen)
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
   if (exiBufferLen+8<TCP_PAYLOAD_LEN)
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
#ifdef VERBOSE_EXI_DECODER
   showAsHex(global_streamEnc.data, global_streamEncPos, "encoded exi");
#endif
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
#ifdef VERBOSE_EXI_DECODER
   showAsHex(global_streamDec.data, global_streamDec.size, "decoder will see");
#endif
   /* We have something to decode, this is a good sign that the connection is fine.
      Inform the ConnectionManager that everything is fine. */
   connMgr_ApplOk(10);
}

/********* EXI creation functions ************************/
static void pev_sendChargeParameterDiscoveryReq(void)
{
   struct dinDC_EVChargeParameterType *cp;
   projectExiConnector_prepare_DinExiDocument();
   dinDocEnc.V2G_Message.Body.ChargeParameterDiscoveryReq_isUsed = 1u;
   init_dinChargeParameterDiscoveryReqType(&dinDocEnc.V2G_Message.Body.ChargeParameterDiscoveryReq);
   dinDocEnc.V2G_Message.Body.ChargeParameterDiscoveryReq.EVRequestedEnergyTransferType = dinEVRequestedEnergyTransferType_DC_extended;
   cp = &dinDocEnc.V2G_Message.Body.ChargeParameterDiscoveryReq.DC_EVChargeParameter;
   cp->DC_EVStatus.EVReady = 0;  /* What ever this means. The Ioniq sends 0 here in the ChargeParameterDiscoveryReq message. */
   //cp->DC_EVStatus.EVCabinConditioning_isUsed /* The Ioniq sends this with 1, but let's assume it is not mandatory. */
   //cp->DC_EVStatus.RESSConditioning_isUsed /* The Ioniq sends this with 1, but let's assume it is not mandatory. */
   cp->DC_EVStatus.EVRESSSOC = hardwareInterface_getSoc();
   cp->EVMaximumCurrentLimit.Value = Param::GetInt(Param::MaxCurrent);
   cp->EVMaximumCurrentLimit.Multiplier = 0; /* -3 to 3. The exponent for base of 10. */
   cp->EVMaximumCurrentLimit.Unit_isUsed = 1;
   cp->EVMaximumCurrentLimit.Unit = dinunitSymbolType_A;

   cp->EVMaximumPowerLimit_isUsed = 1; /* The Ioniq sends 1 here. */
   cp->EVMaximumPowerLimit.Value = Param::GetInt(Param::MaxPower) * 10; /* maxpower is kW, then x10 x 100 by Multiplier */
   cp->EVMaximumPowerLimit.Multiplier = 2; /* 10^2 */
   cp->EVMaximumPowerLimit.Unit_isUsed = 1;
   cp->EVMaximumPowerLimit.Unit = dinunitSymbolType_W; /* Watt */

   cp->EVMaximumVoltageLimit.Value = Param::GetInt(Param::MaxVoltage);
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
   connMgr_ApplOk(31);
}

static void pev_sendPreChargeReq(void)
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
   tvolt.Value = hardwareInterface_getAccuVoltage(); /* The precharge target voltage. Scaling is 1V. */
#undef tvolt
#define tcurr dinDocEnc.V2G_Message.Body.PreChargeReq.EVTargetCurrent
   tcurr.Multiplier = 0; /* -3 to 3. The exponent for base of 10. */
   tcurr.Unit = dinunitSymbolType_A;
   tcurr.Unit_isUsed = 1;
   tcurr.Value = 1; /* 1A for precharging */
#undef tcurr
   encodeAndTransmit();
}

static void pev_sendPowerDeliveryReq(uint8_t isOn)
{
   pev_wasPowerDeliveryRequestedOn = isOn;
   if (isOn) {
       // reset if set from previous session
       LastChargingVoltage = 0;
   }

   projectExiConnector_prepare_DinExiDocument();
   dinDocEnc.V2G_Message.Body.PowerDeliveryReq_isUsed = 1u;
   init_dinPowerDeliveryReqType(&dinDocEnc.V2G_Message.Body.PowerDeliveryReq);
   dinDocEnc.V2G_Message.Body.PowerDeliveryReq.ReadyToChargeState = isOn; /* 1=ON, 0=OFF */
   dinDocEnc.V2G_Message.Body.PowerDeliveryReq.DC_EVPowerDeliveryParameter_isUsed = 1;
   dinDocEnc.V2G_Message.Body.PowerDeliveryReq.DC_EVPowerDeliveryParameter.DC_EVStatus.EVReady = 1; /* 1 means true. We are ready. */
   dinDocEnc.V2G_Message.Body.PowerDeliveryReq.DC_EVPowerDeliveryParameter.DC_EVStatus.EVErrorCode = dinDC_EVErrorCodeType_NO_ERROR;
   dinDocEnc.V2G_Message.Body.PowerDeliveryReq.DC_EVPowerDeliveryParameter.DC_EVStatus.EVRESSSOC = hardwareInterface_getSoc();
   dinDocEnc.V2G_Message.Body.PowerDeliveryReq.DC_EVPowerDeliveryParameter.ChargingComplete = 0; /* boolean. Charging not finished. */
   /* some "optional" fields seem to be mandatory, at least the Ioniq sends them, and the Compleo charger ignores the message if too short.
      See https://github.com/uhi22/OpenV2Gx/commit/db2c7addb0cae0e16175d666e736efd551f3e14d#diff-333579da65917bc52ef70369b576374d0ee5dbca47d2b1e3bedb6f062decacff
      Let's fill them:
   */
   dinDocEnc.V2G_Message.Body.PowerDeliveryReq.DC_EVPowerDeliveryParameter.DC_EVStatus.EVCabinConditioning_isUsed = 1;
   dinDocEnc.V2G_Message.Body.PowerDeliveryReq.DC_EVPowerDeliveryParameter.DC_EVStatus.EVCabinConditioning = 0;
   dinDocEnc.V2G_Message.Body.PowerDeliveryReq.DC_EVPowerDeliveryParameter.DC_EVStatus.EVRESSConditioning_isUsed = 1;
   dinDocEnc.V2G_Message.Body.PowerDeliveryReq.DC_EVPowerDeliveryParameter.DC_EVStatus.EVRESSConditioning = 0;
   dinDocEnc.V2G_Message.Body.PowerDeliveryReq.DC_EVPowerDeliveryParameter.BulkChargingComplete_isUsed  = 1;
   dinDocEnc.V2G_Message.Body.PowerDeliveryReq.DC_EVPowerDeliveryParameter.BulkChargingComplete = 0;
   encodeAndTransmit();
}

static inline void setPhysicalValue(dinPhysicalValueType* pv, int16_t value, dinunitSymbolType unit, int8_t multiplier)
{
    pv->Value = value;
    pv->Unit_isUsed = 1;
    pv->Unit = unit;
    pv->Multiplier = multiplier; /* -3 to 3. The exponent for base of 10. */
}

static void pev_sendCurrentDemandReq(void)
{
   uint16_t UTarget, EVMaximumVoltageLimit;
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
   UTarget = hardwareInterface_getChargingTargetVoltage(); /* The charging target. Scaling is 1V. */
   EVMaximumVoltageLimit = Param::GetInt(Param::MaxVoltage);
   if ((UTarget+1>EVMaximumVoltageLimit) && (EVMaximumVoltageLimit>1)) {
       /* Some chargers run into emergency shutdown, if the requested or actual voltage is above the
          announced EVMaximumVoltageLimit. That's why we limit here the request. Nevertheless, this
          is more a workaround than a solution, because a physical overshoot may still lead to
          emergency shutdown. The solution is to choose an appropriate value of Param::MaxVoltage. */
       UTarget=EVMaximumVoltageLimit-1;
       addToTrace(MOD_PEV, "Warning: TargetVoltage %dV is near to EVMaximumVoltageLimit %dV, which may cause charger shutdown.",
              UTarget, EVMaximumVoltageLimit);
   }
#define tvolt dinDocEnc.V2G_Message.Body.CurrentDemandReq.EVTargetVoltage
   tvolt.Multiplier = 0;  /* -3 to 3. The exponent for base of 10. */
   tvolt.Unit = dinunitSymbolType_V;
   tvolt.Unit_isUsed = 1;
   tvolt.Value = UTarget; /* The charging target voltage. Scaling is 1V. */
#undef tvolt
   // EVTargetCurrent
#define tcurr dinDocEnc.V2G_Message.Body.CurrentDemandReq.EVTargetCurrent
   tcurr.Multiplier = 0;  /* -3 to 3. The exponent for base of 10. */
   tcurr.Unit = dinunitSymbolType_A;
   tcurr.Unit_isUsed = 1;
   tcurr.Value = hardwareInterface_getChargingTargetCurrent(); /* The charging target current. Scaling is 1A. */
#undef tcurr
   dinDocEnc.V2G_Message.Body.CurrentDemandReq.ChargingComplete = 0; /* boolean. Todo: Do we need to take this from command line? Or is it fine
    that the PEV just sends a PowerDeliveryReq with STOP, if it decides to stop the charging? */
   dinDocEnc.V2G_Message.Body.CurrentDemandReq.BulkChargingComplete_isUsed = 1u;
   dinDocEnc.V2G_Message.Body.CurrentDemandReq.BulkChargingComplete = 0u; /* not complete */
   dinDocEnc.V2G_Message.Body.CurrentDemandReq.RemainingTimeToFullSoC_isUsed = 1u;
   dinDocEnc.V2G_Message.Body.CurrentDemandReq.RemainingTimeToFullSoC.Multiplier = 0;  /* -3 to 3. The exponent for base of 10. */
   dinDocEnc.V2G_Message.Body.CurrentDemandReq.RemainingTimeToFullSoC.Unit = dinunitSymbolType_s;
   dinDocEnc.V2G_Message.Body.CurrentDemandReq.RemainingTimeToFullSoC.Unit_isUsed = 1;
   dinDocEnc.V2G_Message.Body.CurrentDemandReq.RemainingTimeToFullSoC.Value = 1200; /* seconds */

   dinDocEnc.V2G_Message.Body.CurrentDemandReq.RemainingTimeToBulkSoC_isUsed = 1u;
   dinDocEnc.V2G_Message.Body.CurrentDemandReq.RemainingTimeToBulkSoC.Multiplier = 0;  /* -3 to 3. The exponent for base of 10. */
   dinDocEnc.V2G_Message.Body.CurrentDemandReq.RemainingTimeToBulkSoC.Unit = dinunitSymbolType_s;
   dinDocEnc.V2G_Message.Body.CurrentDemandReq.RemainingTimeToBulkSoC.Unit_isUsed = 1;
   dinDocEnc.V2G_Message.Body.CurrentDemandReq.RemainingTimeToBulkSoC.Value = 600; /* seconds */

   // Charger 'Plugit HUBE S' wont work without maxes
#define req dinDocEnc.V2G_Message.Body.CurrentDemandReq
   req.EVMaximumVoltageLimit_isUsed = 1;
   setPhysicalValue(&req.EVMaximumVoltageLimit, Param::GetInt(Param::MaxVoltage), dinunitSymbolType_V, 0);

   req.EVMaximumCurrentLimit_isUsed = 1;
   setPhysicalValue(&req.EVMaximumCurrentLimit, Param::GetInt(Param::MaxCurrent), dinunitSymbolType_A, 0);
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

/**** State functions ***************/
//Empty functions
static void stateFunctionNotYetInitialized() {}
static void stateFunctionConnecting() {}

static void stateFunctionConnected(void)
{
   // We have a freshly established TCP channel. We start the V2GTP/EXI communication now.
   // We just use the initial request message from the Ioniq. It contains one entry: DIN.
   addToTrace(MOD_PEV, "Checkpoint400: Sending the initial SupportedApplicationProtocolReq");
   setCheckpoint(400);
   addV2GTPHeaderAndTransmit(exiDemoSupportedApplicationProtocolRequestIoniq, sizeof(exiDemoSupportedApplicationProtocolRequestIoniq));
   Param::SetInt(Param::StopReason, STOP_REASON_NONE);
   pev_enterState(PEV_STATE_WaitForSupportedApplicationProtocolResponse);
}

static void stateFunctionWaitForSupportedApplicationProtocolResponse(void)
{
   uint8_t i;
   if (tcp_rxdataLen>V2GTP_HEADER_SIZE)
   {
      addToTrace(MOD_PEV, "In state WaitForSupportedApplicationProtocolResponse");
      routeDecoderInputData();
      projectExiConnector_decode_appHandExiDocument();
      tcp_rxdataLen = 0; /* mark the input data as "consumed" */
      if (aphsDoc.supportedAppProtocolRes_isUsed)
      {
         /* it is the correct response */
         addToTrace(MOD_PEV, "supportedAppProtocolRes ResponseCode:%d, SchemaID_isUsed:%d, SchemaID:%d",
                aphsDoc.supportedAppProtocolRes.ResponseCode,
                aphsDoc.supportedAppProtocolRes.SchemaID_isUsed,
                aphsDoc.supportedAppProtocolRes.SchemaID);
         publishStatus("Schema negotiated", "");
         addToTrace(MOD_PEV, "Checkpoint403: Schema negotiated. And Checkpoint500: Will send SessionSetupReq");
         setCheckpoint(500);
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
         for (i=0; i<LEN_OF_EVCCID; i++)
         {
            dinDocEnc.V2G_Message.Body.SessionSetupReq.EVCCID.bytes[i] = getOurMac()[i];
         }
         dinDocEnc.V2G_Message.Body.SessionSetupReq.EVCCID.bytesLen = LEN_OF_EVCCID;
         encodeAndTransmit();
         pev_enterState(PEV_STATE_WaitForSessionSetupResponse);
      }
   }
   if (pev_isTooLong())
      ErrorMessage::Post(ERR_PLCTIMEOUT);
}

static void stateFunctionWaitForSessionSetupResponse(void)
{
   if (tcp_rxdataLen>V2GTP_HEADER_SIZE)
   {
      addToTrace(MOD_PEV, "In state WaitForSessionSetupResponse");
      routeDecoderInputData();
      projectExiConnector_decode_DinExiDocument();
      tcp_rxdataLen = 0; /* mark the input data as "consumed" */
      //addToTrace("after decoding: g_errn=%d", g_errn);
      //addToTrace("global_streamDecPos=%d", global_streamDecPos);
      if (dinDocDec.V2G_Message.Body.SessionSetupRes_isUsed)
      {
         memcpy(sessionId, dinDocDec.V2G_Message.Header.SessionID.bytes, SESSIONID_LEN);
         sessionIdLen = dinDocDec.V2G_Message.Header.SessionID.bytesLen; /* store the received SessionID, we will need it later. */
         addToTrace_bytes(MOD_PEV, "Checkpoint506: The Evse decided for SessionId", sessionId, sessionIdLen);
         setCheckpoint(506);
         publishStatus("Session established", "");
         addToTrace(MOD_PEV, "Will send ServiceDiscoveryReq");
         projectExiConnector_prepare_DinExiDocument();
         dinDocEnc.V2G_Message.Body.ServiceDiscoveryReq_isUsed = 1u;
         init_dinServiceDiscoveryReqType(&dinDocEnc.V2G_Message.Body.ServiceDiscoveryReq);
         setCheckpoint(510);
         encodeAndTransmit();
         pev_enterState(PEV_STATE_WaitForServiceDiscoveryResponse);
      }
   }
}

static void stateFunctionWaitForServiceDiscoveryResponse(void)
{
   if (tcp_rxdataLen>V2GTP_HEADER_SIZE)
   {
      addToTrace(MOD_PEV, "In state WaitForServiceDiscoveryResponse");
      routeDecoderInputData();
      projectExiConnector_decode_DinExiDocument();
      tcp_rxdataLen = 0; /* mark the input data as "consumed" */
      if (dinDocDec.V2G_Message.Body.ServiceDiscoveryRes_isUsed)
      {
         publishStatus("ServDisc done", "");
         addToTrace(MOD_PEV, "Will send ServicePaymentSelectionReq");
         projectExiConnector_prepare_DinExiDocument();
         dinDocEnc.V2G_Message.Body.ServicePaymentSelectionReq_isUsed = 1u;
         init_dinServicePaymentSelectionReqType(&dinDocEnc.V2G_Message.Body.ServicePaymentSelectionReq);
         /* the mandatory fields in ISO are SelectedPaymentOption and SelectedServiceList. Same in DIN. */
         dinDocEnc.V2G_Message.Body.ServicePaymentSelectionReq.SelectedPaymentOption = dinpaymentOptionType_ExternalPayment; /* not paying per car */
         dinDocEnc.V2G_Message.Body.ServicePaymentSelectionReq.SelectedServiceList.SelectedService.array[0].ServiceID = 1; /* todo: what ever this means. The Ioniq uses 1. */
         dinDocEnc.V2G_Message.Body.ServicePaymentSelectionReq.SelectedServiceList.SelectedService.arrayLen = 1; /* just one element in the array */
         setCheckpoint(520);
         encodeAndTransmit();
         pev_enterState(PEV_STATE_WaitForServicePaymentSelectionResponse);
      }
   }
}

static void stateFunctionWaitForServicePaymentSelectionResponse(void)
{
   if (tcp_rxdataLen>V2GTP_HEADER_SIZE)
   {
      addToTrace(MOD_PEV, "In state WaitForServicePaymentSelectionResponse");
      routeDecoderInputData();
      projectExiConnector_decode_DinExiDocument();
      tcp_rxdataLen = 0; /* mark the input data as "consumed" */
      if (dinDocDec.V2G_Message.Body.ServicePaymentSelectionRes_isUsed)
      {
         publishStatus("ServPaySel done", "");
         addToTrace(MOD_PEV, "Checkpoint530: Will send ContractAuthenticationReq");
         setCheckpoint(530);
         projectExiConnector_prepare_DinExiDocument();
         dinDocEnc.V2G_Message.Body.ContractAuthenticationReq_isUsed = 1u;
         init_dinContractAuthenticationReqType(&dinDocEnc.V2G_Message.Body.ContractAuthenticationReq);
         /* no other fields are manatory */
         encodeAndTransmit();
         pev_numberOfContractAuthenticationReq = 1; // This is the first request.
         pev_enterState(PEV_STATE_WaitForContractAuthenticationResponse);
      }
   }
}

static void stateFunctionWaitForContractAuthenticationResponse(void)
{
   if (pev_cyclesInState<30)   // The first second in the state just do nothing.
   {
      return;
   }
   if (tcp_rxdataLen>V2GTP_HEADER_SIZE)
   {
      addToTrace(MOD_PEV, "In state WaitForContractAuthenticationResponse");
      routeDecoderInputData();
      projectExiConnector_decode_DinExiDocument();
      tcp_rxdataLen = 0; /* mark the input data as "consumed" */
      if (dinDocDec.V2G_Message.Body.ContractAuthenticationRes_isUsed)
      {
         // In normal case, we can have two results here: either the Authentication is needed (the user
         // needs to authorize by RFID card or app, or something like this.
         // Or, the authorization is finished. This is shown by EVSEProcessing=Finished.
         if (dinDocDec.V2G_Message.Body.ContractAuthenticationRes.EVSEProcessing == dinEVSEProcessingType_Finished)
         {
            publishStatus("Auth finished", "");
            addToTrace(MOD_PEV, "Checkpoint538 and 540: Auth is Finished. Will send ChargeParameterDiscoveryReq");
            setCheckpoint(540);
            pev_sendChargeParameterDiscoveryReq();
            pev_numberOfChargeParameterDiscoveryReq = 1; // first message
            pev_enterState(PEV_STATE_WaitForChargeParameterDiscoveryResponse);
         }
         else
         {
            // Not (yet) finished.
            if (pev_numberOfContractAuthenticationReq>=120)   // approx 120 seconds, maybe the user searches two minutes for his RFID card...
            {
               addToTrace(MOD_PEV, "Authentication lasted too long. Giving up.");
               pev_enterState(PEV_STATE_SequenceTimeout);
            }
            else
            {
               // Try again.
               pev_numberOfContractAuthenticationReq += 1; // count the number of tries.
               publishStatus("Waiting f Auth", "");
               addToTrace(MOD_PEV, "Not (yet) finished. Will again send ContractAuthenticationReq #%d", pev_numberOfContractAuthenticationReq);
               encodeAndTransmit();
               // We just stay in the same state, until the timeout elapses.
               pev_enterState(PEV_STATE_WaitForContractAuthenticationResponse);
            }
         }
      }
   }
}

static void stateFunctionWaitForChargeParameterDiscoveryResponse(void)
{
   if (pev_cyclesInState<30)   // The first second in the state just do nothing.
   {
      return;
   }
   if (tcp_rxdataLen>V2GTP_HEADER_SIZE)
   {
	  Param::SetInt(Param::PortState, PS_CHARGING_DC);  //Set Port state to DC Fastcharging -as this is an attempt !!! TBC if this is the right place for it.
      addToTrace(MOD_PEV, "In state WaitForChargeParameterDiscoveryResponse");
      routeDecoderInputData();
      projectExiConnector_decode_DinExiDocument();
      tcp_rxdataLen = 0; /* mark the input data as "consumed" */
      if (dinDocDec.V2G_Message.Body.ChargeParameterDiscoveryRes_isUsed)
      {
         // We can have two cases here:
         // (A) The charger needs more time to show the charge parameters.
         // (B) The charger finished to tell the charge parameters.
         if (dinDocDec.V2G_Message.Body.ChargeParameterDiscoveryRes.EVSEProcessing == dinEVSEProcessingType_Finished)
         {
            publishStatus("ChargeParams discovered", "");
            addToTrace(MOD_PEV, "Checkpoint550: ChargeParams are discovered.. Will change to state C.");
#define dcparm dinDocDec.V2G_Message.Body.ChargeParameterDiscoveryRes.DC_EVSEChargeParameter
            int evseMaxVoltage = combineValueAndMultiplier(dcparm.EVSEMaximumVoltageLimit);
            int evseMaxCurrent = combineValueAndMultiplier(dcparm.EVSEMaximumCurrentLimit);
            EVSEMinimumVoltage = combineValueAndMultiplier(dcparm.EVSEMinimumVoltageLimit);
#undef dcparm
            Param::SetInt(Param::EvseMaxVoltage, evseMaxVoltage);
            Param::SetInt(Param::EvseMaxCurrent, evseMaxCurrent);

            setCheckpoint(550);
            // pull the CP line to state C here:
            hardwareInterface_setStateC();
            addToTrace(MOD_PEV, "Checkpoint555: Locking the connector.");
            hardwareInterface_triggerConnectorLocking();
            //If we are not ready for charging, don't go past this state -> will time out
            if (hardwareInterface_stopChargeRequested())
            {
               addToTrace(MOD_PEV, "Stopping due to stoprequest.");
               pev_enterState(PEV_STATE_WaitForServiceDiscoveryResponse);
            }
            else
            {
               pev_enterState(PEV_STATE_WaitForConnectorLock);
            }
         }
         else
         {
            // Not (yet) finished.
            if (pev_numberOfChargeParameterDiscoveryReq>=60)
            {
               /* approx 60 seconds, should be sufficient for the charger to find its parameters.
                   ... The ISO allows up to 55s reaction time and 60s timeout for "ongoing". Taken over from
                       https://github.com/uhi22/pyPLC/commit/01c7c069fd4e7b500aba544ae4cfce6774f7344a */
               addToTrace(MOD_PEV, "ChargeParameterDiscovery lasted too long:%d Giving up.", pev_numberOfChargeParameterDiscoveryReq);
               pev_enterState(PEV_STATE_SequenceTimeout);
            }
            else
            {
               // Try again.
               pev_numberOfChargeParameterDiscoveryReq += 1; // count the number of tries.
               publishStatus("disc ChargeParams", "");
               addToTrace(MOD_PEV, "Not (yet) finished. Will again send ChargeParameterDiscoveryReq #%d", pev_numberOfChargeParameterDiscoveryReq);
               pev_sendChargeParameterDiscoveryReq();
               // we stay in the same state
               pev_enterState(PEV_STATE_WaitForChargeParameterDiscoveryResponse);
            }
         }
      }
   }
}

static void stateFunctionWaitForConnectorLock(void)
{
   if (hardwareInterface_isConnectorLocked())
   {
      addToTrace(MOD_PEV, "Checkpoint560: Connector Lock confirmed. Will send CableCheckReq.");
      setCheckpoint(560);
      pev_sendCableCheckReq();
      pev_numberOfCableCheckReq = 1; // This is the first request.
      pev_enterState(PEV_STATE_WaitForCableCheckResponse);
   }
   if (pev_isTooLong())
      ErrorMessage::Post(ERR_LOCKTIMEOUT);
}

static void stateFunctionWaitForCableCheckResponse(void)
{
   uint8_t rc, proc;
   if (pev_cyclesInState<30)   // The first second in the state just do nothing.
   {
      return;
   }
   if (tcp_rxdataLen>V2GTP_HEADER_SIZE)
   {
      //addToTrace_bytes(MOD_PEV, "In state WaitForCableCheckResponse, received:", tcp_rxdata, tcp_rxdataLen);
      routeDecoderInputData();
      projectExiConnector_decode_DinExiDocument();
      tcp_rxdataLen = 0; /* mark the input data as "consumed" */
      if (dinDocDec.V2G_Message.Body.CableCheckRes_isUsed)
      {
         rc = dinDocDec.V2G_Message.Body.CableCheckRes.ResponseCode;
         proc = dinDocDec.V2G_Message.Body.CableCheckRes.EVSEProcessing;
         Param::SetInt(Param::EvseVoltage, 0);
         //addToTrace("The CableCheck result is %d %d", rc, proc);
         // We have two cases here:
         // 1) The charger says "cable check is finished and cable ok", by setting ResponseCode=OK and EVSEProcessing=Finished.
         // 2) Else: The charger says "need more time or cable not ok". In this case, we just run into timeout and start from the beginning.
         if ((proc==dinEVSEProcessingType_Finished) && (rc==dinresponseCodeType_OK))
         {
            publishStatus("CbleChck done", "");
            addToTrace(MOD_PEV, "The EVSE says that the CableCheck is finished and ok.");
            addToTrace(MOD_PEV, "Will send PreChargeReq");
            setCheckpoint(570);
            pev_sendPreChargeReq();
            connMgr_ApplOk(31); /* PreChargeResponse may need longer. Inform the connection manager to be patient.
                                (This is a takeover from https://github.com/uhi22/pyPLC/commit/08af8306c60d57c4c33221a0dbb25919371197f9 ) */
            pev_enterState(PEV_STATE_WaitForPreChargeResponse);
         }
         else
         {
            if (pev_numberOfCableCheckReq>60)   /* approx 60s should be sufficient for cable check. The ISO allows up to 55s reaction time and 60s timeout for "ongoing". Taken over from https://github.com/uhi22/pyPLC/commit/01c7c069fd4e7b500aba544ae4cfce6774f7344a */
            {
               addToTrace(MOD_PEV, "CableCheck lasted too long:%d Giving up.", pev_numberOfCableCheckReq);
               pev_enterState(PEV_STATE_SequenceTimeout);
            }
            else
            {
               // cable check not yet finished or finished with bad result -> try again
               pev_numberOfCableCheckReq += 1;
               publishStatus("CbleChck ongoing", "");
               addToTrace(MOD_PEV, "Will again send CableCheckReq");
               pev_sendCableCheckReq();
               // stay in the same state
               pev_enterState(PEV_STATE_WaitForCableCheckResponse);
            }
         }
      }
   }
}

static void stateFunctionWaitForPreChargeResponse(void)
{
   if (pev_DelayCycles>0)
   {
      pev_DelayCycles-=1;
      return;
   }
   if (tcp_rxdataLen>V2GTP_HEADER_SIZE)
   {
      //addToTrace_bytes(MOD_PEV, "In state WaitForPreChargeResponse, received:", tcp_rxdata, tcp_rxdataLen);
      routeDecoderInputData();
      projectExiConnector_decode_DinExiDocument();
      tcp_rxdataLen = 0; /* mark the input data as "consumed" */
      if (dinDocDec.V2G_Message.Body.PreChargeRes_isUsed)
      {
         addToTrace(MOD_PEV, "PreCharge aknowledge received.");
         EVSEPresentVoltage = combineValueAndMultiplier(dinDocDec.V2G_Message.Body.PreChargeRes.EVSEPresentVoltage);
         Param::SetInt(Param::EvseVoltage, EVSEPresentVoltage);

         uint16_t inletVtg = hardwareInterface_getInletVoltage();
         uint16_t batVtg = hardwareInterface_getAccuVoltage();

         addToTrace(MOD_PEV, "PreCharge aknowledge received. Inlet %dV, accu %dV, uMin %dV", inletVtg, batVtg, EVSEMinimumVoltage);
         if ((ABS(inletVtg - batVtg) < PARAM_U_DELTA_MAX_FOR_END_OF_PRECHARGE) && (batVtg > EVSEMinimumVoltage) && chademoInterface_preChargeCompleted())
         {
            addToTrace(MOD_PEV, "Difference between accu voltage and inlet voltage is small.");
            publishStatus("PreCharge done", "");
            // Turn the power relay on.
            hardwareInterface_setPowerRelayOn();
            pev_DelayCycles = 15; /* 15*30ms, explanation see below */
            pev_enterState(PEV_STATE_WaitForContactorsClosed);
          }
         else
         {
            publishStatus("PreChrge ongoing", String(u) + "V");
            addToTrace(MOD_PEV, "Difference too big. Continuing PreCharge.");
            pev_sendPreChargeReq();
            pev_DelayCycles=15; // wait with the next evaluation approx half a second
         }
      }
   }
   if (pev_isTooLong())
      ErrorMessage::Post(ERR_PRECTIMEOUT);
}

static void stateFunctionWaitForContactorsClosed(void)
{
   if (pev_DelayCycles>0)
   {
      /* simplified solution for waiting for the contactors: Since the contactors anyway have no feedback whether
         they are really closed, we just use a time-based approach. In
         https://github.com/uhi22/ccs32clara/issues/22 we see that it takes ~350ms until both contactors have
         current, so we wait here 15 cycles * 30ms = 450ms, and additional delay will be caused by the
         powerDeliveryRequest/Response and the currentDemandRequest/Response. So this should give sufficient
         time to close the contactors until the charger really provides current. */
      pev_DelayCycles--;
      return;
   }
   addToTrace(MOD_PEV, "Contactors assumingly finished closing. Sending PowerDeliveryReq.");
   pev_sendPowerDeliveryReq(1); /* 1 is ON */
   setCheckpoint(600);
   pev_enterState(PEV_STATE_WaitForPowerDeliveryResponse);
}


static void stateFunctionWaitForPowerDeliveryResponse(void)
{
   if (tcp_rxdataLen>V2GTP_HEADER_SIZE)
   {
      //addToTrace_bytes(MOD_PEV, "In state WaitForPowerDeliveryRes, received:", tcp_rxdata, tcp_rxdataLen);
      routeDecoderInputData();
      projectExiConnector_decode_DinExiDocument();
      tcp_rxdataLen = 0; /* mark the input data as "consumed" */
      if (dinDocDec.V2G_Message.Body.PowerDeliveryRes_isUsed)
      {
         // chatgpt: FAILED_PowerDeliveryNotApplied may require looping until ok?
         addToTrace(MOD_PEV, "PowerDeliveryRes ResponseCode:%d", dinDocDec.V2G_Message.Body.PowerDeliveryRes.ResponseCode);

         if (pev_wasPowerDeliveryRequestedOn)
         {
            publishStatus("PwrDelvy ON success", "");
            addToTrace(MOD_PEV, "Checkpoint700: Starting the charging loop with CurrentDemandReq");
            setCheckpoint(700);
            pev_sendCurrentDemandReq();
            pev_enterState(PEV_STATE_WaitForCurrentDemandResponse);
         }
         else
         {
            /* We requested "OFF". This is while the charging session is ending.
            When we received this response, the charger had up to 1.5s time to ramp down
            the current. On Compleo, there are really 1.5s until we get this response.
            See https://github.com/uhi22/pyPLC#detailled-investigation-about-the-normal-end-of-the-charging-session */
            publishStatus("PwrDelvry OFF success", "");
            setCheckpoint(810);
            /* set the CP line to B */
            hardwareInterface_setStateB(); /* ISO Figure 107: The PEV shall set stateB after receiving PowerDeliveryRes and before WeldingDetectionReq */
            addToTrace(MOD_PEV, "Giving the charger some time to detect StateB and ramp down the current.");
            pev_DelayCycles = 10; /* 15*30ms=450ms for charger shutdown. Should be more than sufficient, because somewhere was a requirement with 20ms between StateB until current is down. The Ioniq uses 300ms. */
            pev_enterState(PEV_STATE_WaitForCurrentDownAfterStateB); /* We give the charger some time to detect the StateB and fully ramp down the current */
         }
      }
   }
}

static void stateFunctionWaitForCurrentDownAfterStateB(void) 
{
    /* During normal end of the charging session, we have set the StateB, and
       want to give the charger some time to ramp down the current completely,
       before we are opening the contactors. */
    if (pev_DelayCycles>0) {
        /* just waiting */
        pev_DelayCycles--;

    } else if (chademoInterface_carContactorsOpened()) {
        /* Time is over. Current flow should have been stopped by the charger. Let's open the contactors and send a weldingDetectionRequest, to find out whether the voltage drops. */
        addToTrace(MOD_PEV, "Starting WeldingDetection");
        hardwareInterface_setPowerRelayOff();
        setCheckpoint(850);
        /* We do not need a waiting time before sending the weldingDetectionRequest, because the weldingDetection
        will be anyway in a loop. So the first round will see a high voltage (because the contactor mechanically needed
        some time to open, but this is no problem, the next samples will see decreasing voltage in normal case. */
        numberOfWeldingDetectionRounds = 0;
        pev_sendWeldingDetectionReq();
        pev_enterState(PEV_STATE_WaitForWeldingDetectionResponse);
    }
}

static void stateFunctionWaitForCurrentDemandResponse(void)
{
   if (tcp_rxdataLen>V2GTP_HEADER_SIZE)
   {
      //addToTrace_bytes(MOD_PEV, "In state WaitForCurrentDemandRes, received:", tcp_rxdata, tcp_rxdataLen);
      routeDecoderInputData();
      //addToTrace(MOD_PEV, "step1 %d", tcp_rxdataLen);
      projectExiConnector_decode_DinExiDocument();
      //addToTrace(MOD_PEV, "step2 %d %d", g_errn, global_streamDecPos);
      tcp_rxdataLen = 0; /* mark the input data as "consumed" */
      if (dinDocDec.V2G_Message.Body.CurrentDemandRes_isUsed)
      {
         /* as long as the accu is not full and no stop-demand from the user, we continue charging */
         bool pev_isUserStopRequestOnChargerSide = false;
         bool pev_stopRequestFromCharger = false;
         if (dinDocDec.V2G_Message.Body.CurrentDemandRes.DC_EVSEStatus.EVSEStatusCode == dinDC_EVSEStatusCodeType_EVSE_Shutdown)
         {
            /* https://github.com/uhi22/pyPLC#example-flow, checkpoint 790: If the user stops the
               charging session on the charger, we get a CurrentDemandResponse with
               DC_EVSEStatus.EVSEStatusCode = 2 "EVSE_Shutdown" (observed on Compleo. To be tested
               on other chargers. */
            addToTrace(MOD_PEV, "Checkpoint790: Charging is terminated from charger side.");
            setCheckpoint(790);
            pev_isUserStopRequestOnChargerSide = true;
            Param::SetInt(Param::StopReason, STOP_REASON_CHARGER_SHUTDOWN);
         }
         else if (dinDocDec.V2G_Message.Body.CurrentDemandRes.DC_EVSEStatus.EVSEStatusCode == dinDC_EVSEStatusCodeType_EVSE_Malfunction)
         {
            /* If the charger reports a malfunction, we stop the charging. */
            /* Issue reference: https://github.com/uhi22/ccs32clara/issues/29 */
            addToTrace(MOD_PEV, "Charger reported EVSE_Malfunction. A reason could be hitting the EVMaximumVoltageLimit or EVSEMaximumVoltageLimit.");
            pev_stopRequestFromCharger = true;
            Param::SetInt(Param::StopReason, STOP_REASON_CHARGER_EVSE_MALFUNCTION);
         }
         else if (dinDocDec.V2G_Message.Body.CurrentDemandRes.DC_EVSEStatus.EVSEStatusCode == dinDC_EVSEStatusCodeType_EVSE_EmergencyShutdown)
         {
            /* If the charger reports an emergency, we stop the charging. */
            addToTrace(MOD_PEV, "Charger reported EmergencyShutdown.");
            pev_stopRequestFromCharger = true;
            Param::SetInt(Param::StopReason, STOP_REASON_CHARGER_EMERGENCY_SHUTDOWN);
         }

         /* If the pushbutton is pressed longer than 0.5s or enable is set to off, we interpret this as charge stop request. */
         bool pev_isUserStopRequestOnCarSide = hardwareInterface_stopChargeRequested();
         bool pev_isAccuFull = hardwareInterface_getIsAccuFull();
         if (pev_isAccuFull || pev_isUserStopRequestOnCarSide || pev_isUserStopRequestOnChargerSide || pev_stopRequestFromCharger)
         {
            if (pev_isAccuFull)
            {
               publishStatus("Accu full", "");
               addToTrace(MOD_PEV, "Accu is full. Sending PowerDeliveryReq Stop.");
               Param::SetInt(Param::StopReason, STOP_REASON_ACCU_FULL);
            }
            else if (pev_isUserStopRequestOnCarSide)
            {
               publishStatus("User req stop on car side", "");
               addToTrace(MOD_PEV, "User requested stop on car side. Sending PowerDeliveryReq Stop.");
            }
            else if (pev_isUserStopRequestOnChargerSide)
            {
               publishStatus("User req stop on charger side", "");
               addToTrace(MOD_PEV, "User requested stop on charger side. Sending PowerDeliveryReq Stop.");
            }

            setCheckpoint(800);
            pev_sendPowerDeliveryReq(0); /* we can immediately send the powerDeliveryStopRequest, while we are under full current.
                                            sequence explained here: https://github.com/uhi22/pyPLC#detailled-investigation-about-the-normal-end-of-the-charging-session */
            pev_enterState(PEV_STATE_WaitForPowerDeliveryResponse);
         }
         else
         {
            /* continue charging loop */
            EVSEPresentVoltage = combineValueAndMultiplier(dinDocDec.V2G_Message.Body.CurrentDemandRes.EVSEPresentVoltage);
            LastChargingVoltage = EVSEPresentVoltage;
            uint16_t evsePresentCurrent = combineValueAndMultiplier(dinDocDec.V2G_Message.Body.CurrentDemandRes.EVSEPresentCurrent);
            //publishStatus("Charging", String(u) + "V", String(hardwareInterface_getSoc()) + "%");
            Param::SetInt(Param::EvseVoltage, EVSEPresentVoltage);
            Param::SetInt(Param::EvseCurrent, evsePresentCurrent);
            setCheckpoint(710);
            pev_sendCurrentDemandReq();
            pev_enterState(PEV_STATE_WaitForCurrentDemandResponse);
         }
      }
   }
}

static void stateFunctionWaitForWeldingDetectionResponse(void)
{
   if (tcp_rxdataLen>V2GTP_HEADER_SIZE)
   {
      addToTrace(MOD_PEV, "In state WaitForWeldingDetectionRes");
      routeDecoderInputData();
      projectExiConnector_decode_DinExiDocument();
      tcp_rxdataLen = 0; /* mark the input data as "consumed" */
      if (dinDocDec.V2G_Message.Body.WeldingDetectionRes_isUsed)
      {
          /* The charger measured the voltage on the cable, and gives us the value. In the first
             round will show a quite high voltage, because the contactors are just opening. We
             need to repeat the requests, until the voltage is at a non-dangerous level. */
          EVSEPresentVoltage = combineValueAndMultiplier(dinDocDec.V2G_Message.Body.WeldingDetectionRes.EVSEPresentVoltage);
          Param::SetInt(Param::EvseVoltage, EVSEPresentVoltage);
          addToTrace(MOD_PEV, "EVSEPresentVoltage %dV", EVSEPresentVoltage);
          bool voltageIsLow = EVSEPresentVoltage < MAX_VOLTAGE_TO_FINISH_WELDING_DETECTION;
          if (voltageIsLow || numberOfWeldingDetectionRounds > MAX_NUMBER_OF_WELDING_DETECTION_ROUNDS) {

              publishStatus("WeldingDet done", "");

              if (voltageIsLow == false) {
                  if (EVSEPresentVoltage == LastChargingVoltage) {
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
                      ErrorMessage::Post(ERR_RELAYWELDED);
                  }
              }

              addToTrace(MOD_PEV, "WeldingDetection finished. Sending SessionStopReq");
              projectExiConnector_prepare_DinExiDocument();
              dinDocEnc.V2G_Message.Body.SessionStopReq_isUsed = 1u;
              init_dinSessionStopType(&dinDocEnc.V2G_Message.Body.SessionStopReq);
              /* no other fields are mandatory */
              setCheckpoint(900);
              encodeAndTransmit();
              addToTrace(MOD_PEV, "Unlocking the connector");
              /* unlock the connectore here. Better here than later, to avoid endless locked connector in case of broken SessionStopResponse. */
              hardwareInterface_triggerConnectorUnlocking();
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
              pev_enterState(PEV_STATE_WaitForWeldingDetectionResponse);
          }
      }
   }
}

static void stateFunctionWaitForSessionStopResponse(void)
{
   if (tcp_rxdataLen>V2GTP_HEADER_SIZE)
   {
      addToTrace(MOD_PEV, "In state WaitForSessionStopRes");
      routeDecoderInputData();
      projectExiConnector_decode_DinExiDocument();
      tcp_rxdataLen = 0; /* mark the input data as "consumed" */
      if (dinDocDec.V2G_Message.Body.SessionStopRes_isUsed)
      {
         // req -508
         tcp_disconnect();
         publishStatus("Stopped normally", "");
         addToTrace(MOD_PEV, "Charging is finished");
         pev_enterState(PEV_STATE_End);
      }
   }
}

static void stateFunctionSequenceTimeout(void)
{
   /* Here we end, if we run into a timeout in the state machine. */
   publishStatus("ERROR Timeout", "");
   /* Initiate the safe-shutdown-sequence. */
   addToTrace(MOD_PEV, "Safe-shutdown-sequence: setting state B");
   setCheckpoint(1100);
   hardwareInterface_setStateB(); /* setting CP line to B disables in the charger the current flow. */
   pev_DelayCycles = 66; /* 66*30ms=2s for charger shutdown */
   pev_enterState(PEV_STATE_SafeShutDownWaitForChargerShutdown);
}

static void stateFunctionUnrecoverableError(void)
{
   /* Here we end, if the EVSE reported an error code, which terminates the charging session. */
   publishStatus("ERROR reported", "");
   /* Initiate the safe-shutdown-sequence. */
   addToTrace(MOD_PEV, "Safe-shutdown-sequence: setting state B");
   ErrorMessage::Post(ERR_EVSEFAULT);
   setCheckpoint(1200);
   hardwareInterface_setStateB(); /* setting CP line to B disables in the charger the current flow. */
   pev_DelayCycles = 66; /* 66*30ms=2s for charger shutdown */
   pev_enterState(PEV_STATE_SafeShutDownWaitForChargerShutdown);
}

static void stateFunctionSafeShutDownWaitForChargerShutdown(void)
{
   /* wait state, to give the charger the time to stop the current. */
   if (pev_DelayCycles>0)
   {
      pev_DelayCycles--;
      return;
   }
   /* Now the current flow is stopped by the charger. We can safely open the contactors: */
   addToTrace(MOD_PEV, "Safe-shutdown-sequence: opening contactors");
   setCheckpoint(1300);
   hardwareInterface_setPowerRelayOff();
   pev_DelayCycles = 33; /* 33*30ms=1s for opening the contactors */
   pev_enterState(PEV_STATE_SafeShutDownWaitForContactorsOpen);
}

static void stateFunctionSafeShutDownWaitForContactorsOpen(void)
{
   /* wait state, to give the contactors the time to open. */
   if (pev_DelayCycles>0)
   {
      pev_DelayCycles--;
      return;
   }
   /* Finally, when we have no current and no voltage, unlock the connector */
   setCheckpoint(1400);
   addToTrace(MOD_PEV, "Safe-shutdown-sequence: unlocking the connector");
   hardwareInterface_triggerConnectorUnlocking();
   /* This is the end of the safe-shutdown-sequence. */
   pev_enterState(PEV_STATE_End);
}

static void stateFunctionEnd(void)
{
   /* Just stay here, until we get re-initialized after a new SLAC/SDP. */
}

static void pev_enterState(pevstates n)
{
   //addToTrace("[PEV] from %d entering %d", pev_state, n);
   pev_state = n;
   pev_cyclesInState = 0;
   Param::SetInt(Param::opmode, n);
}

static uint8_t pev_isTooLong(void)
{
   return timeouts[pev_state] > 0 && pev_cyclesInState > timeouts[pev_state];
}

/******* The statemachine dispatcher *******************/
static void pev_runFsm(void)
{
   if (connMgr_getConnectionLevel()<CONNLEVEL_80_TCP_RUNNING)
   {
      /* If we have no TCP to the charger, nothing to do here. Just wait for the link. */
      if (pev_state!=PEV_STATE_NotYetInitialized)
      {
         pev_enterState(PEV_STATE_NotYetInitialized);
         hardwareInterface_setStateB();
         hardwareInterface_setPowerRelayOff();
      }
      return;
   }
   if (connMgr_getConnectionLevel()==CONNLEVEL_80_TCP_RUNNING)
   {
      /* We have a TCP connection. This is the trigger for us. */
      if (pev_state == PEV_STATE_NotYetInitialized) pev_enterState(PEV_STATE_Connected);
   }

   stateFunctions[pev_state](); //call state function

   if (pev_state != PEV_STATE_WaitForCurrentDemandResponse) //only in currentDemand we have meaningful current values
      Param::SetInt(Param::EvseCurrent, 0);

   if (pev_isTooLong())
   {
      addToTrace(MOD_PEV, "Timeout in dispatcher.");
      pev_enterState(PEV_STATE_SequenceTimeout);
   }
}

/************ public interfaces *****************************************/


/* The cyclic main function of the PEV charging state machine.
   Called each 30ms. */
void pevStateMachine_Mainfunction(void)
{
   // run the state machine:
   pev_cyclesInState += 1; // for timeout handling, count how long we are in a state
   pev_runFsm();
}

bool chademoInterface_isCcsInStateEnd(){
    return Param::GetInt(Param::opmode) == PEV_STATE_End;
}

