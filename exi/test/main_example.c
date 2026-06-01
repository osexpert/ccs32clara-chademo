/*
 * Copyright (C) 2007-2018 Siemens AG
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published
 * by the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 */

/*******************************************************************
 *
 * @author Daniel.Peintner.EXT@siemens.com
 * @author Sebastian.Kaebisch@siemens.com
 * @version 0.9.4
 * @contact Richard.Kuntschke@siemens.com
 *
 *
 ********************************************************************/



#include <stdio.h>
#include <stdarg.h>
#include <stdlib.h>

#include <string.h>

#include "EXITypes.h"

#include "appHandEXIDatatypes.h"
#include "appHandEXIDatatypesEncoder.h"
#include "appHandEXIDatatypesDecoder.h"

/* Activate support for DIN */
#include "dinEXIDatatypes.h"
#if DEPLOY_DIN_CODEC == SUPPORT_YES
#include "dinEXIDatatypesEncoder.h"
#include "dinEXIDatatypesDecoder.h"
#endif /* DEPLOY_DIN_CODEC == SUPPORT_YES */

/* Activate support for XMLDSIG */
#include "xmldsigEXIDatatypes.h"
#if DEPLOY_XMLDSIG_CODEC == SUPPORT_YES
#include "xmldsigEXIDatatypesEncoder.h"
#include "xmldsigEXIDatatypesDecoder.h"
#endif /* DEPLOY_XMLDSIG_CODEC == SUPPORT_YES */

/* Activate support for ISO1 */
#include "iso1EXIDatatypes.h"
#if DEPLOY_ISO1_CODEC == SUPPORT_YES
#include "iso1EXIDatatypesEncoder.h"
#include "iso1EXIDatatypesDecoder.h"
#endif /* DEPLOY_ISO1_CODEC == SUPPORT_YES */

/* Activate support for ISO2 */
#include "iso2EXIDatatypes.h"
#if DEPLOY_ISO2_CODEC == SUPPORT_YES
#include "iso2EXIDatatypesEncoder.h"
#include "iso2EXIDatatypesDecoder.h"
#endif /* DEPLOY_ISO2_CODEC == SUPPORT_YES */

#include "v2gtp.h"

#define BUFFER_SIZE 256
uint8_t buffer1[BUFFER_SIZE];
uint8_t buffer2[BUFFER_SIZE];

#define ERROR_UNEXPECTED_REQUEST_MESSAGE -601
#define ERROR_UNEXPECTED_SESSION_SETUP_RESP_MESSAGE -602
#define ERROR_UNEXPECTED_SERVICE_DISCOVERY_RESP_MESSAGE -602
#define ERROR_UNEXPECTED_SERVICE_DETAILS_RESP_MESSAGE -603
#define ERROR_UNEXPECTED_PAYMENT_SERVICE_SELECTION_RESP_MESSAGE -604
#define ERROR_UNEXPECTED_PAYMENT_DETAILS_RESP_MESSAGE -605
#define ERROR_UNEXPECTED_AUTHORIZATION_RESP_MESSAGE -606
#define ERROR_UNEXPECTED_CHARGE_PARAMETER_DISCOVERY_RESP_MESSAGE -607
#define ERROR_UNEXPECTED_POWER_DELIVERY_RESP_MESSAGE -608
#define ERROR_UNEXPECTED_CHARGING_STATUS_RESP_MESSAGE -609
#define ERROR_UNEXPECTED_METERING_RECEIPT_RESP_MESSAGE -610
#define ERROR_UNEXPECTED_SESSION_STOP_RESP_MESSAGE -611
#define ERROR_UNEXPECTED_CABLE_CHECK_RESP_MESSAGE -612
#define ERROR_UNEXPECTED_PRE_CHARGE_RESP_MESSAGE -612
#define ERROR_UNEXPECTED_CURRENT_DEMAND_RESP_MESSAGE -613
#define ERROR_UNEXPECTED_WELDING_DETECTION_RESP_MESSAGE -614


static int writeStringToEXIString(char* string, exi_string_character_t* exiString) {
	int pos = 0;
	while(string[pos]!='\0')
	{
		exiString[pos] = string[pos];
		pos++;
	}

	return pos;
}

static void printASCIIString(exi_string_character_t* string, uint16_t len) {
	unsigned int i;
	for(i=0; i<len; i++) {
		printf("%c",(char)string[i]);
	}
	printf("\n");
}


static void printBinaryArray(uint8_t* byte, uint16_t len) {
	unsigned int i;
	for(i=0; i<len; i++) {
		printf("%d ",byte[i]);
	}
	printf("\n");
}

static void copyBytes(uint8_t* from, uint16_t len, uint8_t* to) {
	int i;
	for(i=0; i<len; i++) {
		to[i] = from[i];
	}
}


/** Example implementation of the app handshake protocol for the EVSE side  */
static int appHandshakeHandler(bitstream_t* iStream, bitstream_t* oStream) {
	struct appHandEXIDocument appHandResp;
	int i;
	struct appHandEXIDocument exiDoc;
	int errn = 0;
	uint32_t payloadLengthDec;


	if ( (errn = read_v2gtpHeader(iStream->data, &payloadLengthDec)) == 0) {
		*iStream->pos = V2GTP_HEADER_LENGTH;
		if( (errn = decode_appHandExiDocument(iStream, &exiDoc)) ) {
			/* an error occured */
			return errn;
		}

	}





	printf("EVSE side: List of application handshake protocols of the EV \n");

	for(i=0;i<exiDoc.supportedAppProtocolReq.AppProtocol.arrayLen;i++) {
		printf("\tProtocol entry #=%d\n",(i+1));
		printf("\t\tProtocolNamespace=");
		printASCIIString(exiDoc.supportedAppProtocolReq.AppProtocol.array[i].ProtocolNamespace.characters, exiDoc.supportedAppProtocolReq.AppProtocol.array[i].ProtocolNamespace.charactersLen);
		printf("\t\tVersion=%d.%d\n", exiDoc.supportedAppProtocolReq.AppProtocol.array[i].VersionNumberMajor, exiDoc.supportedAppProtocolReq.AppProtocol.array[i].VersionNumberMinor);
		printf("\t\tSchemaID=%d\n", exiDoc.supportedAppProtocolReq.AppProtocol.array[i].SchemaID);
		printf("\t\tPriority=%d\n", exiDoc.supportedAppProtocolReq.AppProtocol.array[i].Priority);
	}

	/* prepare response handshake response:
	 * it is assumed, we support the 15118 1.0 version :-) */
	init_appHandEXIDocument(&appHandResp);
	appHandResp.supportedAppProtocolRes_isUsed = 1u;
	appHandResp.supportedAppProtocolRes.ResponseCode = appHandresponseCodeType_OK_SuccessfulNegotiation;
	appHandResp.supportedAppProtocolRes.SchemaID = exiDoc.supportedAppProtocolReq.AppProtocol.array[0].SchemaID; /* signal the protocol by the provided schema id*/
	appHandResp.supportedAppProtocolRes.SchemaID_isUsed = 1u;

	*oStream->pos = V2GTP_HEADER_LENGTH;
	if( (errn = encode_appHandExiDocument(oStream, &appHandResp)) == 0) {
		errn = write_v2gtpHeader(oStream->data, (*oStream->pos)-V2GTP_HEADER_LENGTH, V2GTP_EXI_TYPE);
	}


	return errn;
}


static int appHandshake()
{
	bitstream_t stream1;
	bitstream_t stream2;

	uint32_t payloadLengthDec;
	size_t pos1 = V2GTP_HEADER_LENGTH; /* v2gtp header */
	size_t pos2 = 0;

	struct appHandEXIDocument handshake;
	struct appHandEXIDocument handshakeResp;

	int errn = 0;

	char* ns0 = "urn:iso:15118:2:2010:MsgDef";
	char* ns1 = "urn:din:70121:2012:MsgDef";

	stream1.size = BUFFER_SIZE;
	stream1.data = buffer1;
	stream1.pos = &pos1;

	stream2.size = BUFFER_SIZE;
	stream2.data = buffer2;
	stream2.pos = &pos2;

	init_appHandEXIDocument(&handshake);

	printf("EV side: setup data for the supported application handshake request message\n");

	/* set up ISO/IEC 15118 Version 1.0 information */
	handshake.supportedAppProtocolReq_isUsed = 1u;
	handshake.supportedAppProtocolReq.AppProtocol.arrayLen = 2; /* we have only two protocols implemented */

	handshake.supportedAppProtocolReq.AppProtocol.array[0].ProtocolNamespace.charactersLen =
			writeStringToEXIString(ns0, handshake.supportedAppProtocolReq.AppProtocol.array[0].ProtocolNamespace.characters);
	handshake.supportedAppProtocolReq.AppProtocol.array[0].SchemaID = 1;
	handshake.supportedAppProtocolReq.AppProtocol.array[0].VersionNumberMajor = 1;
	handshake.supportedAppProtocolReq.AppProtocol.array[0].VersionNumberMinor = 0;
	handshake.supportedAppProtocolReq.AppProtocol.array[0].Priority = 1;

	handshake.supportedAppProtocolReq.AppProtocol.array[1].ProtocolNamespace.charactersLen =
			writeStringToEXIString(ns1, handshake.supportedAppProtocolReq.AppProtocol.array[1].ProtocolNamespace.characters);
	handshake.supportedAppProtocolReq.AppProtocol.array[1].SchemaID = 2;
	handshake.supportedAppProtocolReq.AppProtocol.array[1].VersionNumberMajor = 1;
	handshake.supportedAppProtocolReq.AppProtocol.array[1].VersionNumberMinor = 0;
	handshake.supportedAppProtocolReq.AppProtocol.array[1].Priority = 2;

	/* send app handshake request */
	if( (errn = encode_appHandExiDocument(&stream1, &handshake)) == 0) {
		if ( write_v2gtpHeader(stream1.data, pos1-V2GTP_HEADER_LENGTH, V2GTP_EXI_TYPE) == 0 ) {
			printf("EV side: send message to the EVSE\n");
		}
	}

	if (errn == 0) {
		/* read app handshake request & generate response */
		errn = appHandshakeHandler(&stream1, &stream2);
	}

	if (errn == 0) {
		/* check response */
		if ( (errn = read_v2gtpHeader(stream2.data, &payloadLengthDec)) == 0) {
			pos2 = V2GTP_HEADER_LENGTH;

			if(decode_appHandExiDocument(&stream2, &handshakeResp) == 0) {
				printf("EV side: Response of the EVSE \n");
				if(handshakeResp.supportedAppProtocolRes.ResponseCode == appHandresponseCodeType_OK_SuccessfulNegotiation) {
					printf("\t\tResponseCode=OK_SuccessfulNegotiation\n");
					printf("\t\tSchemaID=%d\n",handshakeResp.supportedAppProtocolRes.SchemaID);
				}
			}
		}

	}

	if (errn != 0) {
		printf("appHandshake error %d \n", errn);
	}


	return errn;

}





#if DEPLOY_ISO2_CODEC == SUPPORT_YES

static void printEVSEStatus2(struct iso2EVSEStatusType* status)
{
	printf("\tEVSEStatus:\n");
	printf("\t\tEVSENotification=%d\n", status->EVSENotification);
	printf("\t\tNotificationMaxDelay=%d\n", status->NotificationMaxDelay);
}

/* serializes EXI stream and adds V2G TP header */
static int serialize2EXI2Stream(struct iso2EXIDocument* exiIn, bitstream_t* stream) {
	int errn;
	*stream->pos = V2GTP_HEADER_LENGTH;  /* v2gtp header */
	if( (errn = encode_iso2ExiDocument(stream, exiIn)) == 0) {
		errn = write_v2gtpHeader(stream->data, (*stream->pos)-V2GTP_HEADER_LENGTH, V2GTP_EXI_TYPE);
	}
	return errn;
}


/* deserializes V2G TP header and decodes right away EXI stream */
static int deserialize2Stream2EXI(bitstream_t* streamIn, struct iso2EXIDocument* exi) {
	int errn;
	uint32_t payloadLength;

	*streamIn->pos = 0;
	if ( (errn = read_v2gtpHeader(streamIn->data, &payloadLength)) == 0) {
		*streamIn->pos += V2GTP_HEADER_LENGTH;

		errn = decode_iso2ExiDocument(streamIn, exi);
	}
	return errn;
}

static int sessionSetup2(struct iso2EXIDocument* exiIn, struct iso2EXIDocument* exiOut) {
	printf("EVSE side: sessionSetup called\n"  );
	printf("\tReceived data:\n");
	printf("\tHeader SessionID=");
	printBinaryArray(exiIn->V2G_Message.Header.SessionID.bytes, exiIn->V2G_Message.Header.SessionID.bytesLen);
	printf("\t\t EVCCID=%d\n", exiIn->V2G_Message.Body.SessionSetupReq.EVCCID.bytes[0]);

	exiOut->V2G_Message_isUsed = 1u;

	/* generate an unique sessionID */
	init_iso2MessageHeaderType(&exiOut->V2G_Message.Header);
	exiOut->V2G_Message.Header.SessionID.bytes[0] = 1;
	exiOut->V2G_Message.Header.SessionID.bytes[1] = 2;
	exiOut->V2G_Message.Header.SessionID.bytes[2] = 3;
	exiOut->V2G_Message.Header.SessionID.bytes[3] = 4;
	exiOut->V2G_Message.Header.SessionID.bytes[4] = 5;
	exiOut->V2G_Message.Header.SessionID.bytes[5] = 6;
	exiOut->V2G_Message.Header.SessionID.bytes[6] = 7;
	exiOut->V2G_Message.Header.SessionID.bytes[7] = 8;
	exiOut->V2G_Message.Header.SessionID.bytesLen = 8;

	/* Prepare data for EV */
	init_iso2BodyType(&exiOut->V2G_Message.Body);

	exiOut->V2G_Message.Body.SessionSetupRes_isUsed = 1u;
	init_iso2SessionSetupResType(&exiOut->V2G_Message.Body.SessionSetupRes);

	exiOut->V2G_Message.Body.SessionSetupRes.ResponseCode = iso2responseCodeType_OK;
	exiOut->V2G_Message.Body.SessionSetupRes.EVSEID.characters[0] = 65;
	exiOut->V2G_Message.Body.SessionSetupRes.EVSEID.characters[1] = 85;
	exiOut->V2G_Message.Body.SessionSetupRes.EVSEID.characters[2] = 86;
	exiOut->V2G_Message.Body.SessionSetupRes.EVSEID.characters[3] = 87;
	exiOut->V2G_Message.Body.SessionSetupRes.EVSEID.characters[4] = 88;
	exiOut->V2G_Message.Body.SessionSetupRes.EVSEID.characters[5] = 89;
	exiOut->V2G_Message.Body.SessionSetupRes.EVSEID.characters[6] = 90;
	exiOut->V2G_Message.Body.SessionSetupRes.EVSEID.charactersLen = 7;
	exiOut->V2G_Message.Body.SessionSetupRes.EVSETimeStamp_isUsed = 1u;
	exiOut->V2G_Message.Body.SessionSetupRes.EVSETimeStamp = 123456789;

	return 0;
}

static int serviceDiscovery2(struct iso2EXIDocument* exiIn, struct iso2EXIDocument* exiOut) {
	int i;

	printf("EVSE side: serviceDiscovery called\n"  );
	printf("\tReceived data:\n");
	printf("\tHeader SessionID=");
	printBinaryArray(exiIn->V2G_Message.Header.SessionID.bytes, exiIn->V2G_Message.Header.SessionID.bytesLen);
	if(exiIn->V2G_Message.Body.ServiceDiscoveryReq.SupportedServiceIDs_isUsed) {
		for(i=0;i<exiIn->V2G_Message.Body.ServiceDiscoveryReq.SupportedServiceIDs.ServiceID.arrayLen; i++) {
			printf("\t\tSupportedServiceID=%d\n", exiIn->V2G_Message.Body.ServiceDiscoveryReq.SupportedServiceIDs.ServiceID.array[i]);
		}
	}

	/* Prepare data for EV */
	exiOut->V2G_Message_isUsed = 1u;
	init_iso2BodyType(&exiOut->V2G_Message.Body);

	exiOut->V2G_Message.Body.ServiceDiscoveryRes_isUsed = 1u;
	init_iso2ServiceDiscoveryResType(&exiOut->V2G_Message.Body.ServiceDiscoveryRes);


	exiOut->V2G_Message.Body.ServiceDiscoveryRes.VASList_isUsed = 0u;  /* we do not provide VAS */
	exiOut->V2G_Message.Body.ServiceDiscoveryRes.ResponseCode = iso2responseCodeType_OK;

	exiOut->V2G_Message.Body.ServiceDiscoveryRes.PaymentOptionList.PaymentOption.array[0] = iso2paymentOptionType_ExternalPayment; /* EVSE handles the payment */
	exiOut->V2G_Message.Body.ServiceDiscoveryRes.PaymentOptionList.PaymentOption.array[1] = iso2paymentOptionType_Contract;
	exiOut->V2G_Message.Body.ServiceDiscoveryRes.PaymentOptionList.PaymentOption.arrayLen = 2;

	exiOut->V2G_Message.Body.ServiceDiscoveryRes.EnergyTransferServiceList.Service.arrayLen = 1;
	exiOut->V2G_Message.Body.ServiceDiscoveryRes.EnergyTransferServiceList.Service.array[0].ServiceID = 1; /* ID of the charge service */
	exiOut->V2G_Message.Body.ServiceDiscoveryRes.EnergyTransferServiceList.Service.array[0].FreeService = 1;

	exiOut->V2G_Message.Body.ServiceDiscoveryRes.VASList_isUsed = 0u; /* no value added service requested */

	return 0;
}


static int serviceDetail2(struct iso2EXIDocument* exiIn, struct iso2EXIDocument* exiOut) {

	printf("EVSE side: serviceDetail called\n"  );
	printf("\tReceived data:\n");
	printf("\tHeader SessionID=");
	printBinaryArray(exiIn->V2G_Message.Header.SessionID.bytes, exiIn->V2G_Message.Header.SessionID.bytesLen);
	printf("\t\t ServiceDetailID=%d\n",exiIn->V2G_Message.Body.ServiceDetailReq.ServiceID);


	/* Prepare data for EV */
	exiOut->V2G_Message_isUsed = 1u;
	init_iso2BodyType(&exiOut->V2G_Message.Body);

	exiOut->V2G_Message.Body.ServiceDetailRes_isUsed= 1u;
	init_iso2ServiceDetailResType(&exiOut->V2G_Message.Body.ServiceDetailRes);

	exiOut->V2G_Message.Body.ServiceDetailRes.ServiceID = 1234;

	exiOut->V2G_Message.Body.ServiceDetailRes.ServiceParameterList_isUsed = 1u;
	exiOut->V2G_Message.Body.ServiceDetailRes.ServiceParameterList.ParameterSet.arrayLen = 2;

	/* Parameter Set 1*/
	exiOut->V2G_Message.Body.ServiceDetailRes.ServiceParameterList.ParameterSet.array[0].ParameterSetID = 1;
	exiOut->V2G_Message.Body.ServiceDetailRes.ServiceParameterList.ParameterSet.array[0].Parameter.arrayLen = 2;
	exiOut->V2G_Message.Body.ServiceDetailRes.ServiceParameterList.ParameterSet.array[0].Parameter.array[0].Name.charactersLen = 8;
	exiOut->V2G_Message.Body.ServiceDetailRes.ServiceParameterList.ParameterSet.array[0].Parameter.array[0].Name.characters[0] = 'P';
	exiOut->V2G_Message.Body.ServiceDetailRes.ServiceParameterList.ParameterSet.array[0].Parameter.array[0].Name.characters[1] = 'r';
	exiOut->V2G_Message.Body.ServiceDetailRes.ServiceParameterList.ParameterSet.array[0].Parameter.array[0].Name.characters[2] = 'o';
	exiOut->V2G_Message.Body.ServiceDetailRes.ServiceParameterList.ParameterSet.array[0].Parameter.array[0].Name.characters[3] = 't';
	exiOut->V2G_Message.Body.ServiceDetailRes.ServiceParameterList.ParameterSet.array[0].Parameter.array[0].Name.characters[4]= 'o';
	exiOut->V2G_Message.Body.ServiceDetailRes.ServiceParameterList.ParameterSet.array[0].Parameter.array[0].Name.characters[5] = 'c';
	exiOut->V2G_Message.Body.ServiceDetailRes.ServiceParameterList.ParameterSet.array[0].Parameter.array[0].Name.characters[6] = 'o';
	exiOut->V2G_Message.Body.ServiceDetailRes.ServiceParameterList.ParameterSet.array[0].Parameter.array[0].Name.characters[7] = 'l';
	exiOut->V2G_Message.Body.ServiceDetailRes.ServiceParameterList.ParameterSet.array[0].Parameter.array[0].intValue = 15119;
	exiOut->V2G_Message.Body.ServiceDetailRes.ServiceParameterList.ParameterSet.array[0].Parameter.array[0].intValue_isUsed = 1u;

	exiOut->V2G_Message.Body.ServiceDetailRes.ServiceParameterList.ParameterSet.array[0].Parameter.array[1].Name.charactersLen = 4;
	exiOut->V2G_Message.Body.ServiceDetailRes.ServiceParameterList.ParameterSet.array[0].Parameter.array[1].Name.characters[0] = 'N';
	exiOut->V2G_Message.Body.ServiceDetailRes.ServiceParameterList.ParameterSet.array[0].Parameter.array[1].Name.characters[1] = 'a';
	exiOut->V2G_Message.Body.ServiceDetailRes.ServiceParameterList.ParameterSet.array[0].Parameter.array[1].Name.characters[2] = 'm';
	exiOut->V2G_Message.Body.ServiceDetailRes.ServiceParameterList.ParameterSet.array[0].Parameter.array[1].Name.characters[3] = 'e';

	exiOut->V2G_Message.Body.ServiceDetailRes.ServiceParameterList.ParameterSet.array[0].Parameter.array[1].stringValue_isUsed = 1u;
	exiOut->V2G_Message.Body.ServiceDetailRes.ServiceParameterList.ParameterSet.array[0].Parameter.array[1].stringValue.charactersLen = 3;
	exiOut->V2G_Message.Body.ServiceDetailRes.ServiceParameterList.ParameterSet.array[0].Parameter.array[1].stringValue.characters[0] = 'V';
	exiOut->V2G_Message.Body.ServiceDetailRes.ServiceParameterList.ParameterSet.array[0].Parameter.array[1].stringValue.characters[1] = '2';
	exiOut->V2G_Message.Body.ServiceDetailRes.ServiceParameterList.ParameterSet.array[0].Parameter.array[1].stringValue.characters[2] = 'G';

	/* Parameter Set 2 */
	exiOut->V2G_Message.Body.ServiceDetailRes.ServiceParameterList.ParameterSet.array[1].ParameterSetID = 2;
	exiOut->V2G_Message.Body.ServiceDetailRes.ServiceParameterList.ParameterSet.array[1].Parameter.arrayLen = 1;
	exiOut->V2G_Message.Body.ServiceDetailRes.ServiceParameterList.ParameterSet.array[1].Parameter.array[0].Name.charactersLen = 7;
	exiOut->V2G_Message.Body.ServiceDetailRes.ServiceParameterList.ParameterSet.array[1].Parameter.array[0].Name.characters[0] = 'C';
	exiOut->V2G_Message.Body.ServiceDetailRes.ServiceParameterList.ParameterSet.array[1].Parameter.array[0].Name.characters[1] = 'h';
	exiOut->V2G_Message.Body.ServiceDetailRes.ServiceParameterList.ParameterSet.array[1].Parameter.array[0].Name.characters[2] = 'a';
	exiOut->V2G_Message.Body.ServiceDetailRes.ServiceParameterList.ParameterSet.array[1].Parameter.array[0].Name.characters[3] = 'n';
	exiOut->V2G_Message.Body.ServiceDetailRes.ServiceParameterList.ParameterSet.array[1].Parameter.array[0].Name.characters[4] = 'n';
	exiOut->V2G_Message.Body.ServiceDetailRes.ServiceParameterList.ParameterSet.array[1].Parameter.array[0].Name.characters[5] = 'e';
	exiOut->V2G_Message.Body.ServiceDetailRes.ServiceParameterList.ParameterSet.array[1].Parameter.array[0].Name.characters[6] = 'l';
	exiOut->V2G_Message.Body.ServiceDetailRes.ServiceParameterList.ParameterSet.array[1].Parameter.array[0].physicalValue_isUsed = 1u;
	exiOut->V2G_Message.Body.ServiceDetailRes.ServiceParameterList.ParameterSet.array[1].Parameter.array[0].physicalValue.Value = 1234;

	exiOut->V2G_Message.Body.ServiceDetailRes.ServiceParameterList.ParameterSet.array[1].Parameter.array[0].physicalValue.Exponent = 1;
	exiOut->V2G_Message.Body.ServiceDetailRes.ServiceParameterList.ParameterSet.array[1].Parameter.array[0].physicalValue.Value = 2;

	exiOut->V2G_Message.Body.ServiceDetailRes.ResponseCode = iso2responseCodeType_OK;

	return 0;
}


static int paymentServiceSelection2(struct iso2EXIDocument* exiIn, struct iso2EXIDocument* exiOut) {
	int i;

	printf("EVSE side: paymentServiceSelection called\n"  );
	printf("\tReceived data:\n");
	printf("\tHeader SessionID=");
	printBinaryArray(exiIn->V2G_Message.Header.SessionID.bytes, exiIn->V2G_Message.Header.SessionID.bytesLen);

	if(exiIn->V2G_Message.Body.PaymentServiceSelectionReq.SelectedPaymentOption == iso2paymentOptionType_ExternalPayment)  {
		printf("\t\t SelectedPaymentOption=ExternalPayment\n");
	}

	if(exiIn->V2G_Message.Body.PaymentServiceSelectionReq.SelectedVASList_isUsed) {
		for(i=0; i<exiIn->V2G_Message.Body.PaymentServiceSelectionReq.SelectedVASList.SelectedService.arrayLen;i++)
		{
			printf("\t\t ServiceID=%d\n", exiIn->V2G_Message.Body.PaymentServiceSelectionReq.SelectedVASList.SelectedService.array[i].ServiceID);
			printf("\t\t ParameterSetID=%d\n", exiIn->V2G_Message.Body.PaymentServiceSelectionReq.SelectedVASList.SelectedService.array[i].ParameterSetID);
		}
	}

	/* Prepare data for EV */
	exiOut->V2G_Message_isUsed = 1u;
	init_iso2BodyType(&exiOut->V2G_Message.Body);

	exiOut->V2G_Message.Body.PaymentServiceSelectionRes_isUsed= 1u;
	init_iso2PaymentServiceSelectionResType(&exiOut->V2G_Message.Body.PaymentServiceSelectionRes);

	exiOut->V2G_Message.Body.ServiceDetailRes.ResponseCode = iso2responseCodeType_OK;

	return 0;
}


static int paymentDetails2(struct iso2EXIDocument* exiIn, struct iso2EXIDocument* exiOut) {

	printf("EVSE side: paymentDetails called\n"  );
	printf("\tReceived data:\n");

	printf("\t\t eMAID=%d\n", exiIn->V2G_Message.Body.PaymentDetailsReq.eMAID.characters[0]);
	printf("\t\t ID=%c%c\n", exiIn->V2G_Message.Body.PaymentDetailsReq.ContractSignatureCertChain.Id.characters[0], exiIn->V2G_Message.Body.PaymentDetailsReq.ContractSignatureCertChain.Id.characters[1]);
	printf("\t\t Certificate=%c%c\n", exiIn->V2G_Message.Body.PaymentDetailsReq.ContractSignatureCertChain.Certificate.bytes[0],  exiIn->V2G_Message.Body.PaymentDetailsReq.ContractSignatureCertChain.Certificate.bytes[1]);
	printf("\t\t SubCertificate 1=%c%c\n", exiIn->V2G_Message.Body.PaymentDetailsReq.ContractSignatureCertChain.SubCertificates.Certificate.array[0].bytes[0], exiIn->V2G_Message.Body.PaymentDetailsReq.ContractSignatureCertChain.SubCertificates.Certificate.array[0].bytes[1]);
	printf("\t\t SubCertificate 2=%c%c\n", exiIn->V2G_Message.Body.PaymentDetailsReq.ContractSignatureCertChain.SubCertificates.Certificate.array[1].bytes[0], exiIn->V2G_Message.Body.PaymentDetailsReq.ContractSignatureCertChain.SubCertificates.Certificate.array[1].bytes[1]);

	/* Prepare data for EV */
	exiOut->V2G_Message_isUsed = 1u;
	init_iso2BodyType(&exiOut->V2G_Message.Body);

	exiOut->V2G_Message.Body.PaymentDetailsRes_isUsed = 1u;
	init_iso2PaymentDetailsResType(&exiOut->V2G_Message.Body.PaymentDetailsRes);

	exiOut->V2G_Message.Body.PaymentDetailsRes.ResponseCode = iso2responseCodeType_OK;
	exiOut->V2G_Message.Body.PaymentDetailsRes.GenChallenge.bytesLen = 1;
	exiOut->V2G_Message.Body.PaymentDetailsRes.GenChallenge.bytes[0] = 1;
	exiOut->V2G_Message.Body.PaymentDetailsRes.EVSETimeStamp = 123456;

	return 0;
}


static int authorization2(struct iso2EXIDocument* exiIn, struct iso2EXIDocument* exiOut) {

	printf("EVSE: Authorization called\n"  );
	printf("\tReceived data:\n");

	if(exiIn->V2G_Message.Body.AuthorizationReq.GenChallenge_isUsed) {
		printf("\t\t\t GenChallenge=%d\n", exiIn->V2G_Message.Body.AuthorizationReq.GenChallenge.bytes[0]);
	}
	if(exiIn->V2G_Message.Body.AuthorizationReq.Id_isUsed ) {
		printf("\t\t\t ID=%c%c%c\n", exiIn->V2G_Message.Body.AuthorizationReq.Id.characters[0], exiIn->V2G_Message.Body.AuthorizationReq.Id.characters[1], exiIn->V2G_Message.Body.AuthorizationReq.Id.characters[2]);
	}


	/* Prepare data for EV */
	exiOut->V2G_Message_isUsed = 1u;
	init_iso2BodyType(&exiOut->V2G_Message.Body);

	exiOut->V2G_Message.Body.AuthorizationRes_isUsed = 1u;
	init_iso2AuthorizationResType(&exiOut->V2G_Message.Body.AuthorizationRes);

	exiOut->V2G_Message.Body.AuthorizationRes.ResponseCode = iso2responseCodeType_OK;
	exiOut->V2G_Message.Body.AuthorizationRes.EVSEProcessing = iso2EVSEProcessingType_Finished;

	return 0;
}


static int chargeParameterDiscovery2(struct iso2EXIDocument* exiIn, struct iso2EXIDocument* exiOut) {

	printf("EVSE side: chargeParameterDiscovery called\n"  );
	printf("\tReceived data:\n");

	if(exiIn->V2G_Message.Body.ChargeParameterDiscoveryReq.AC_EVBidirectionalParameter_isUsed) {
		printf("\t\t DepartureTime=%d\n", exiIn->V2G_Message.Body.ChargeParameterDiscoveryReq.AC_EVBidirectionalParameter.DepartureTime);
		printf("\t\t EVMaximumChargeCurrent=%d\n", exiIn->V2G_Message.Body.ChargeParameterDiscoveryReq.AC_EVBidirectionalParameter.EVMaximumChargeCurrent.Value);
	}

	/* Prepare data for EV */
	exiOut->V2G_Message_isUsed = 1u;
	init_iso2BodyType(&exiOut->V2G_Message.Body);

	exiOut->V2G_Message.Body.ChargeParameterDiscoveryRes_isUsed = 1u;
	init_iso2ChargeParameterDiscoveryResType(&exiOut->V2G_Message.Body.ChargeParameterDiscoveryRes);

	exiOut->V2G_Message.Body.ChargeParameterDiscoveryRes.ResponseCode = iso2responseCodeType_OK_CertificateExpiresSoon;
	exiOut->V2G_Message.Body.ChargeParameterDiscoveryRes.EVSEProcessing = iso2EVSEProcessingType_Ongoing;
	exiOut->V2G_Message.Body.ChargeParameterDiscoveryRes.EVSEEnergyTransferParameter_isUsed = 1u;
	/*exiOut->V2G_Message.Body.ChargeParameterDiscoveryRes.EVSEEnergyTransferParameter = 0;*/
	return 0;
}


static int powerDelivery2(struct iso2EXIDocument* exiIn, struct iso2EXIDocument* exiOut) {
	printf("EVSE side: powerDelivery called\n"  );
	printf("\tReceived data:\n");

	printf("\t\t  ChargeProgress=%d\n", exiIn->V2G_Message.Body.PowerDeliveryReq.ChargeProgress);
	printf("\t\t  SAScheduleTupleID=%d\n", exiIn->V2G_Message.Body.PowerDeliveryReq.SAScheduleTupleID);

	/* Prepare data for EV */
	exiOut->V2G_Message_isUsed = 1u;
	init_iso2BodyType(&exiOut->V2G_Message.Body);

	exiOut->V2G_Message.Body.PowerDeliveryRes_isUsed = 1u;
	init_iso2PowerDeliveryResType(&exiOut->V2G_Message.Body.PowerDeliveryRes);

	exiOut->V2G_Message.Body.PowerDeliveryRes.ResponseCode = iso2responseCodeType_OK;

	exiOut->V2G_Message.Body.PowerDeliveryRes.EVSEStatus_isUsed = 1;
	exiOut->V2G_Message.Body.PowerDeliveryRes.EVSEStatus.EVSENotification = iso2EVSENotificationType_StopCharging;
	exiOut->V2G_Message.Body.PowerDeliveryRes.EVSEStatus.NotificationMaxDelay=12;

	exiOut->V2G_Message.Body.PowerDeliveryRes.EVSEProcessing = iso2EVSEProcessingType_Ongoing_WaitingForCustomerInteraction;

	return 0;
}


static int chargingStatus2(struct iso2EXIDocument* exiIn, struct iso2EXIDocument* exiOut) {

	printf("EVSE side: chargingStatus called\n"  );

	/* Prepare data for EV */
	exiOut->V2G_Message_isUsed = 1u;
	init_iso2BodyType(&exiOut->V2G_Message.Body);

	exiOut->V2G_Message.Body.ChargingStatusRes_isUsed = 1u;
	init_iso2ChargingStatusResType(&exiOut->V2G_Message.Body.ChargingStatusRes);


	exiOut->V2G_Message.Body.ChargingStatusRes.ResponseCode = iso2responseCodeType_OK;
	exiOut->V2G_Message.Body.ChargingStatusRes.EVSEID.characters[0]= 'A';
	exiOut->V2G_Message.Body.ChargingStatusRes.EVSEID.charactersLen =1;

	exiOut->V2G_Message.Body.ChargingStatusRes.EVSEStatus.EVSENotification = iso2EVSENotificationType_ReNegotiation;
	exiOut->V2G_Message.Body.ChargingStatusRes.EVSEStatus.NotificationMaxDelay=123;
	exiOut->V2G_Message.Body.ChargingStatusRes.ReceiptRequired = 1;
	exiOut->V2G_Message.Body.ChargingStatusRes.ReceiptRequired_isUsed = 1;

	return 0;
}


static int meteringReceipt2(struct iso2EXIDocument* exiIn, struct iso2EXIDocument* exiOut) {

	printf("EVSE side: meteringReceipt called\n"  );
	printf("\tReceived data:\n");

	printf("\t\t ID=%c%c%c\n", exiIn->V2G_Message.Body.MeteringReceiptReq.Id.characters[0], exiIn->V2G_Message.Body.MeteringReceiptReq.Id.characters[1], exiIn->V2G_Message.Body.MeteringReceiptReq.Id.characters[2]);
	printf("\t\t SAScheduleTupleID=%d\n", exiIn->V2G_Message.Body.MeteringReceiptReq.SAScheduleTupleID);
	printf("\t\t SessionID=%d\n", exiIn->V2G_Message.Body.MeteringReceiptReq.SessionID.bytes[1]);
	printf("\t\t MeterInfo.MeterStatus=%d\n", exiIn->V2G_Message.Body.MeteringReceiptReq.MeterInfo.MeterStatus);
	printf("\t\t MeterInfo.MeterID=%d\n",		exiIn->V2G_Message.Body.MeteringReceiptReq.MeterInfo.MeterID.characters[0]);

	/* Prepare data for EV */
	exiOut->V2G_Message_isUsed = 1u;
	init_iso2BodyType(&exiOut->V2G_Message.Body);

	exiOut->V2G_Message.Body.MeteringReceiptRes_isUsed = 1u;
	init_iso2MeteringReceiptResType(&exiOut->V2G_Message.Body.MeteringReceiptRes);

	exiOut->V2G_Message.Body.MeteringReceiptRes.ResponseCode = iso2responseCodeType_FAILED;

	return 0;
}

static int sessionStop2(struct iso2EXIDocument* exiIn, struct iso2EXIDocument* exiOut) {

	printf("EVSE side: sessionStop called\n"  );
	printf("\tReceived data:\n");
	printf("\tHeader SessionID=");
	printBinaryArray(exiIn->V2G_Message.Header.SessionID.bytes, exiIn->V2G_Message.Header.SessionID.bytesLen);
	printf("\t\t ChargingSession=%d\n", exiIn->V2G_Message.Body.SessionStopReq.ChargingSession);

	/* Prepare data for EV */
	exiOut->V2G_Message_isUsed = 1u;
	init_iso2BodyType(&exiOut->V2G_Message.Body);

	exiOut->V2G_Message.Body.SessionStopRes_isUsed = 1u;
	init_iso2SessionStopResType(&exiOut->V2G_Message.Body.SessionStopRes);

	exiOut->V2G_Message.Body.SessionStopRes.ResponseCode = iso2responseCodeType_OK;

	return 0;
}

static int cableCheck2(struct iso2EXIDocument* exiIn, struct iso2EXIDocument* exiOut) {

	printf("EVSE side: cableCheck called\n"  );

	/* Prepare data for EV */
	exiOut->V2G_Message_isUsed = 1u;
	init_iso2BodyType(&exiOut->V2G_Message.Body);

	exiOut->V2G_Message.Body.CableCheckRes_isUsed = 1u;
	init_iso2CableCheckResType(&exiOut->V2G_Message.Body.CableCheckRes);

	exiOut->V2G_Message.Body.CableCheckRes.ResponseCode = iso2responseCodeType_OK;

	exiOut->V2G_Message.Body.CableCheckRes.EVSEStatus.NotificationMaxDelay = 1234;
	exiOut->V2G_Message.Body.CableCheckRes.EVSEStatus.EVSENotification= iso2EVSENotificationType_ReNegotiation;

	exiOut->V2G_Message.Body.CableCheckRes.EVSEProcessing = iso2EVSEProcessingType_Finished;

	return 0;
}

static int preCharge2(struct iso2EXIDocument* exiIn, struct iso2EXIDocument* exiOut) {

	printf("EVSE side: preCharge called\n"  );
	printf("\tReceived data:\n");

	printf("\t\t EVTargetCurrent=%d (%d)\n", exiIn->V2G_Message.Body.PreChargeReq.EVTargetCurrent.Value, exiIn->V2G_Message.Body.PreChargeReq.EVTargetCurrent.Exponent);
	printf("\t\t EVTargetVoltage=%d (%d)\n", exiIn->V2G_Message.Body.PreChargeReq.EVTargetVoltage.Value, exiIn->V2G_Message.Body.PreChargeReq.EVTargetVoltage.Exponent);

	/* Prepare data for EV */
	exiOut->V2G_Message_isUsed = 1u;
	init_iso2BodyType(&exiOut->V2G_Message.Body);

	exiOut->V2G_Message.Body.PreChargeRes_isUsed = 1u;
	init_iso2PreChargeResType(&exiOut->V2G_Message.Body.PreChargeRes);

	exiOut->V2G_Message.Body.PreChargeRes.ResponseCode = iso2responseCodeType_OK;

	exiOut->V2G_Message.Body.PreChargeRes.EVSEStatus.EVSENotification = iso2EVSENotificationType_StopCharging;
	exiOut->V2G_Message.Body.PreChargeRes.EVSEStatus.NotificationMaxDelay= 1234;

	exiOut->V2G_Message.Body.PreChargeRes.EVSEPresentVoltage.Exponent = 3;
	exiOut->V2G_Message.Body.PreChargeRes.EVSEPresentVoltage.Value= 456;

	return 0;
}



static int create_response_message2(struct iso2EXIDocument* exiIn, struct iso2EXIDocument* exiOut) {
	int errn = ERROR_UNEXPECTED_REQUEST_MESSAGE;

	/* create response message as EXI document */
	if(exiIn->V2G_Message_isUsed) {
		init_iso2EXIDocument(exiOut);
		if (exiIn->V2G_Message.Body.SessionSetupReq_isUsed) {
			errn = sessionSetup2(exiIn, exiOut);
		} else if (exiIn->V2G_Message.Body.ServiceDiscoveryReq_isUsed) {
			errn = serviceDiscovery2(exiIn, exiOut);
		} else if (exiIn->V2G_Message.Body.ServiceDetailReq_isUsed) {
			errn = serviceDetail2(exiIn, exiOut);
		} else if (exiIn->V2G_Message.Body.PaymentServiceSelectionReq_isUsed) {
			errn = paymentServiceSelection2(exiIn, exiOut);
		} else if (exiIn->V2G_Message.Body.PaymentDetailsReq_isUsed) {
			errn = paymentDetails2(exiIn, exiOut);
		} else if (exiIn->V2G_Message.Body.AuthorizationReq_isUsed) {
			errn = authorization2(exiIn, exiOut);
		} else if (exiIn->V2G_Message.Body.ChargeParameterDiscoveryReq_isUsed) {
			errn = chargeParameterDiscovery2(exiIn, exiOut);
		} else if (exiIn->V2G_Message.Body.PowerDeliveryReq_isUsed) {
			errn = powerDelivery2(exiIn, exiOut);
		} else if (exiIn->V2G_Message.Body.ChargingStatusReq_isUsed) {
			errn = chargingStatus2(exiIn, exiOut);
		} else if (exiIn->V2G_Message.Body.MeteringReceiptReq_isUsed) {
			errn = meteringReceipt2(exiIn, exiOut);
		} else if (exiIn->V2G_Message.Body.SessionStopReq_isUsed) {
			errn = sessionStop2(exiIn, exiOut);
		} else if (exiIn->V2G_Message.Body.CableCheckReq_isUsed) {
			errn = cableCheck2(exiIn, exiOut);
		} else if (exiIn->V2G_Message.Body.PreChargeReq_isUsed) {
			errn = preCharge2(exiIn, exiOut);
		}
	}

	return errn;
}

/* Adapt this to your system setup! */
/* In this situation EV and EVSE is the same party */
static int request_response2(struct iso2EXIDocument* exiIn, struct iso2EXIDocument* exiOut) {
	int errn;

	bitstream_t stream1;
	bitstream_t stream2;

	size_t pos1;
	size_t pos2;

	stream1.size = BUFFER_SIZE;
	stream1.data = buffer1;
	stream1.pos = &pos1;

	stream2.size = BUFFER_SIZE;
	stream2.data = buffer2;
	stream2.pos = &pos2;

	/* EV side */
	errn = serialize2EXI2Stream(exiIn, &stream1);

	/* --> Start of EVSE side */
	/* deserialize request message */
	if (errn == 0) {
		errn = deserialize2Stream2EXI(&stream1, exiOut);
	}
	/* create response message */
	if (errn == 0) {
		errn = create_response_message2(exiOut, exiIn);
	}
	/* serialize response message */
	if (errn == 0) {
		errn = serialize2EXI2Stream(exiIn, &stream2);
	}
	/* <-- End of EVSE side */


	/* EV side */
	/* deserialize response message */
	if (errn == 0) {
		errn = deserialize2Stream2EXI(&stream2, exiOut);
	}

	return errn;
}


static int charging2()
{
	int errn = 0;
	int i, j;

	struct iso2EXIDocument exiIn;
	struct iso2EXIDocument exiOut;

	struct iso2ServiceDetailResType serviceDetailRes;
	struct iso2PaymentServiceSelectionResType paymentServiceSelectionRes;
	struct iso2PaymentDetailsResType paymentDetailsRes;

	/* setup header information */
	init_iso2EXIDocument(&exiIn);
	exiIn.V2G_Message_isUsed = 1u;
	init_iso2MessageHeaderType(&exiIn.V2G_Message.Header);
	exiIn.V2G_Message.Header.SessionID.bytes[0] = 0; /* sessionID is always '0' at the beginning (the response contains the valid sessionID)*/
	exiIn.V2G_Message.Header.SessionID.bytes[1] = 0;
	exiIn.V2G_Message.Header.SessionID.bytes[2] = 0;
	exiIn.V2G_Message.Header.SessionID.bytes[3] = 0;
	exiIn.V2G_Message.Header.SessionID.bytes[4] = 0;
	exiIn.V2G_Message.Header.SessionID.bytes[5] = 0;
	exiIn.V2G_Message.Header.SessionID.bytes[6] = 0;
	exiIn.V2G_Message.Header.SessionID.bytes[7] = 0;
	exiIn.V2G_Message.Header.SessionID.bytesLen = 8;
	exiIn.V2G_Message.Header.Signature_isUsed = 0u;


	/************************
	 * sessionSetup *
	 ************************/
	init_iso2BodyType(&exiIn.V2G_Message.Body);
	exiIn.V2G_Message.Body.SessionSetupReq_isUsed = 1u;

	init_iso2SessionSetupReqType(&exiIn.V2G_Message.Body.SessionSetupReq);

	exiIn.V2G_Message.Body.SessionSetupReq.EVCCID.bytesLen = 1;
	exiIn.V2G_Message.Body.SessionSetupReq.EVCCID.bytes[0] = 10;

	printf("EV side: call EVSE sessionSetup");

	errn = request_response2(&exiIn, &exiOut);

	if(errn == 0) {
		/* check, if this is the right response message */
		if(exiOut.V2G_Message.Body.SessionSetupRes_isUsed) {
			/* show results of EVSEs answer message */
			printf("EV side: received response message from EVSE\n");
			printf("\tHeader SessionID=");
			printBinaryArray(exiOut.V2G_Message.Header.SessionID.bytes, exiOut.V2G_Message.Header.SessionID.bytesLen);
			printf("\tResponseCode=%d\n", exiOut.V2G_Message.Body.SessionSetupRes.ResponseCode);
			printf("\tEVSEID=%d\n",	exiOut.V2G_Message.Body.SessionSetupRes.EVSEID.characters[1]);
			printf("\tEVSETimeStamp=%li\n", (long int)exiOut.V2G_Message.Body.SessionSetupRes.EVSETimeStamp);
		} else {
			errn = ERROR_UNEXPECTED_SESSION_SETUP_RESP_MESSAGE;
			return errn;
		}
	} else {
		return errn;
	}


	/*******************************************
	 * serviceDiscovery *
	 *******************************************/
	init_iso2BodyType(&exiIn.V2G_Message.Body);
	exiIn.V2G_Message.Body.ServiceDiscoveryReq_isUsed = 1u;

	init_iso2ServiceDiscoveryReqType(&exiIn.V2G_Message.Body.ServiceDiscoveryReq);

	exiIn.V2G_Message.Body.ServiceDiscoveryReq.SupportedServiceIDs_isUsed = 1u;
	exiIn.V2G_Message.Body.ServiceDiscoveryReq.SupportedServiceIDs.ServiceID.arrayLen = 1;
	exiIn.V2G_Message.Body.ServiceDiscoveryReq.SupportedServiceIDs.ServiceID.array[0] = iso2serviceCategoryType_Internet;

	printf("EV side: call EVSE serviceDiscovery");

	errn = request_response2(&exiIn, &exiOut);

	if(errn == 0) {
		/* check, if this is the right response message */
		if(exiOut.V2G_Message.Body.ServiceDiscoveryRes_isUsed) {
			/* show results of EVSEs answer message */
			printf("EV side: received response message from EVSE\n");
			printf("\tHeader SessionID=");
			printBinaryArray(exiOut.V2G_Message.Header.SessionID.bytes, exiOut.V2G_Message.Header.SessionID.bytesLen);
			printf("\t ResponseCode=%d\n",  exiOut.V2G_Message.Body.ServiceDiscoveryRes.ResponseCode);

			printf("\t Service ResponseCode=%d\n",	exiOut.V2G_Message.Body.ServiceDiscoveryRes.ResponseCode);

			/*printf("\t ServiceID=%d\n",	exiOut.V2G_Message.Body.ServiceDiscoveryRes.ChargeService.ServiceID);
			printf("\t ServiceName=");
			printASCIIString(serviceDiscoveryRes.ChargeService.ServiceName.characters, serviceDiscoveryRes.ChargeService.ServiceName.charactersLen);
			if(serviceDiscoveryRes.PaymentOptionList.PaymentOption.array[1] == v2gpaymentOptionType_Contract) {
				printf("\t PaymentOption=Contract_paymentOptionType\n");
			}
			if(serviceDiscoveryRes.ChargeService.FreeService==1) {
				printf("\t ChargeService.FreeService=True\n");
			}
			if(serviceDiscoveryRes.ChargeService.SupportedEnergyTransferMode.EnergyTransferMode.array[0] == v2gEnergyTransferModeType_DC_combo_core) {
				printf("\t EnergyTransferMode=AC_single_DC_core\n");
			}
			if(serviceDiscoveryRes.ChargeService.SupportedEnergyTransferMode.EnergyTransferMode.array[1] == v2gEnergyTransferModeType_AC_single_phase_core) {
				printf("\t EnergyTransferMode=AC_single_phase_core_EnergyTransferModeType\n");
			}
			printf("\t Value added service list:\n");
			for(i=0;i<serviceDiscoveryRes.ServiceList.Service.arrayLen;i++)
			{
				printf("\n\t\t ServiceID=%d\n",	serviceDiscoveryRes.ServiceList.Service.array[i].ServiceID);
				printf("\t\t ServiceName=");
				printASCIIString(serviceDiscoveryRes.ServiceList.Service.array[i].ServiceName.characters, exiOut.V2G_Message.Body.ServiceDiscoveryRes.ServiceList.Service.array[i].ServiceName.charactersLen );
				if(serviceDiscoveryRes.ServiceList.Service.array[i].ServiceCategory == v2gserviceCategoryType_Internet) {
					printf("\t\t ServiceCategory=Internet\n");
				}
				if(serviceDiscoveryRes.ServiceList.Service.array[i].FreeService==1) {
					printf("\t\t FreeService=True\n");
				}
			}*/

		} else {
			errn = ERROR_UNEXPECTED_SERVICE_DISCOVERY_RESP_MESSAGE;
			return errn;
		}
	} else {
		return errn;
	}


	/*********************************
	 * ServiceDetails *
	 *********************************/
	init_iso2BodyType(&exiIn.V2G_Message.Body);
	exiIn.V2G_Message.Body.ServiceDetailReq_isUsed = 1u;

	init_iso2ServiceDetailReqType(&exiIn.V2G_Message.Body.ServiceDetailReq);

	exiIn.V2G_Message.Body.ServiceDetailReq.ServiceID = 22; /* Value Added Server ID */

	printf("EV side: call EVSE ServiceDetail \n");

	errn = request_response2(&exiIn, &exiOut);

	if(errn == 0) {
		/* check, if this is the right response message */
		if(exiOut.V2G_Message.Body.ServiceDetailRes_isUsed) {
			serviceDetailRes = exiOut.V2G_Message.Body.ServiceDetailRes;
			/* show results of EVSEs answer message */
			printf("EV side: received response message from EVSE\n");
			printf("\tHeader SessionID=");
			printBinaryArray(exiOut.V2G_Message.Header.SessionID.bytes, exiOut.V2G_Message.Header.SessionID.bytesLen);
			printf("\t ResponseCode=%d\n",  exiOut.V2G_Message.Body.ServiceDiscoveryRes.ResponseCode);
			printf("\t ServiceID=%d\n",	exiOut.V2G_Message.Body.ServiceDetailRes.ServiceID);

			if(serviceDetailRes.ServiceParameterList_isUsed) {
				printf("\t\tLength=%d\n", serviceDetailRes.ServiceParameterList.ParameterSet.arrayLen );/*TEST*/

				for(i=0; i<serviceDetailRes.ServiceParameterList.ParameterSet.arrayLen; i++)
				{
					printf("\t\tServiceSetID=%d\n", serviceDetailRes.ServiceParameterList.ParameterSet.array[i].ParameterSetID);
					printf("\t\tParameters=%d\n", serviceDetailRes.ServiceParameterList.ParameterSet.array[i].Parameter.arrayLen);

					for(j=0; j<serviceDetailRes.ServiceParameterList.ParameterSet.array[i].Parameter.arrayLen; j++)
					{
						printf("\t\t\t %d: ParameterName=", j+1);
						printASCIIString(serviceDetailRes.ServiceParameterList.ParameterSet.array[i].Parameter.array[j].Name.characters, exiOut.V2G_Message.Body.ServiceDetailRes.ServiceParameterList.ParameterSet.array[i].Parameter.array[j].Name.charactersLen);

						/*if(serviceDetailRes.ServiceParameterList.ParameterSet.array[i].Parameter.array[j].shortValue_isUsed == 1u) {
							printf("\t\t\t %d: StringValue=", j+1);
							printASCIIString(serviceDetailRes.ServiceParameterList.ParameterSet.array[i].Parameter.array[j].stringValue.characters, serviceDetailRes.ServiceParameterList.ParameterSet.array[i].Parameter.array[j].stringValue.charactersLen);
						} else if(serviceDetailRes.ServiceParameterList.ParameterSet.array[i].Parameter.array[j].intValue_isUsed == 1u) {
							printf("\t\t\t %d: IntValue=%d\n", j+1, serviceDetailRes.ServiceParameterList.ParameterSet.array[i].Parameter.array[j].intValue);
						} else if(serviceDetailRes.ServiceParameterList.ParameterSet.array[i].Parameter.array[j].physicalValue_isUsed == 1u) {
							printf("\t\t\t %d: PhysicalValue=%d (%d)\n",  j+1, serviceDetailRes.ServiceParameterList.ParameterSet.array[i].Parameter.array[j].physicalValue.Value, serviceDetailRes.ServiceParameterList.ParameterSet.array[i].Parameter.array[j].physicalValue.Multiplier);
						}*/
					}
				}
			}
		} else {
			errn = ERROR_UNEXPECTED_SERVICE_DETAILS_RESP_MESSAGE;
			return errn;
		}
	} else {
		return errn;
	}


	/*******************************************
	 * ServicePaymentSelection *
	 *******************************************/
	init_iso2BodyType(&exiIn.V2G_Message.Body);
	exiIn.V2G_Message.Body.PaymentServiceSelectionReq_isUsed = 1u;

	init_iso2PaymentServiceSelectionReqType(&exiIn.V2G_Message.Body.PaymentServiceSelectionReq);

	exiIn.V2G_Message.Body.PaymentServiceSelectionReq.SelectedPaymentOption = iso2paymentOptionType_ExternalPayment;
	exiIn.V2G_Message.Body.PaymentServiceSelectionReq.SelectedVASList_isUsed = 0u;
	exiIn.V2G_Message.Body.PaymentServiceSelectionReq.SelectedEnergyTransferService.ServiceID = 1;
	exiIn.V2G_Message.Body.PaymentServiceSelectionReq.SelectedEnergyTransferService.ParameterSetID = 4;

	printf("EV side: call EVSE ServicePaymentSelection \n");

	errn = request_response2(&exiIn, &exiOut);

	if(errn == 0) {
		/* check, if this is the right response message */
		if(exiOut.V2G_Message.Body.PaymentServiceSelectionRes_isUsed) {
			paymentServiceSelectionRes = exiOut.V2G_Message.Body.PaymentServiceSelectionRes;

			/* show results of EVSEs answer message */
			printf("EV side: received response message from EVSE\n");
			if(exiOut.V2G_Message.Body.PaymentServiceSelectionRes.EVSEStatus_isUsed) {
				printf("\tHeader SessionID=");
				printBinaryArray(exiOut.V2G_Message.Header.SessionID.bytes, exiOut.V2G_Message.Header.SessionID.bytesLen);
			}
			printf("\t ResponseCode=%d\n",  paymentServiceSelectionRes.ResponseCode);

		} else {
			errn = ERROR_UNEXPECTED_PAYMENT_SERVICE_SELECTION_RESP_MESSAGE;
			return errn;
		}
	} else {
		return errn;
	}


	/**********************************
	 * PaymentDetails *
	 **********************************/
	init_iso2BodyType(&exiIn.V2G_Message.Body);
	exiIn.V2G_Message.Body.PaymentDetailsReq_isUsed = 1u;

	init_iso2PaymentDetailsReqType(&exiIn.V2G_Message.Body.PaymentDetailsReq);

	exiIn.V2G_Message.Body.PaymentDetailsReq.eMAID.characters[0] = 1;
	exiIn.V2G_Message.Body.PaymentDetailsReq.eMAID.characters[1] = 123;
	exiIn.V2G_Message.Body.PaymentDetailsReq.eMAID.charactersLen =2;

	exiIn.V2G_Message.Body.PaymentDetailsReq.ContractSignatureCertChain.Certificate.bytes[0] = 'C';
	exiIn.V2G_Message.Body.PaymentDetailsReq.ContractSignatureCertChain.Certificate.bytes[1] = 'e';
	exiIn.V2G_Message.Body.PaymentDetailsReq.ContractSignatureCertChain.Certificate.bytesLen = 2;
	exiIn.V2G_Message.Body.PaymentDetailsReq.ContractSignatureCertChain.SubCertificates_isUsed = 1u;
	exiIn.V2G_Message.Body.PaymentDetailsReq.ContractSignatureCertChain.SubCertificates.Certificate.array[0].bytes[0] = 'S';
	exiIn.V2G_Message.Body.PaymentDetailsReq.ContractSignatureCertChain.SubCertificates.Certificate.array[0].bytes[1] = 'u';
	exiIn.V2G_Message.Body.PaymentDetailsReq.ContractSignatureCertChain.SubCertificates.Certificate.array[0].bytesLen = 2;
	exiIn.V2G_Message.Body.PaymentDetailsReq.ContractSignatureCertChain.SubCertificates.Certificate.array[1].bytes[0] = 'S';
	exiIn.V2G_Message.Body.PaymentDetailsReq.ContractSignatureCertChain.SubCertificates.Certificate.array[1].bytes[1] = 'u';
	exiIn.V2G_Message.Body.PaymentDetailsReq.ContractSignatureCertChain.SubCertificates.Certificate.array[1].bytes[2] = '2';
	exiIn.V2G_Message.Body.PaymentDetailsReq.ContractSignatureCertChain.SubCertificates.Certificate.array[1].bytesLen = 3;
	exiIn.V2G_Message.Body.PaymentDetailsReq.ContractSignatureCertChain.SubCertificates.Certificate.arrayLen =2;
	exiIn.V2G_Message.Body.PaymentDetailsReq.ContractSignatureCertChain.Id_isUsed = 1u;
	exiIn.V2G_Message.Body.PaymentDetailsReq.ContractSignatureCertChain.Id.charactersLen = 2;
	exiIn.V2G_Message.Body.PaymentDetailsReq.ContractSignatureCertChain.Id.characters[0] = 'I';
	exiIn.V2G_Message.Body.PaymentDetailsReq.ContractSignatureCertChain.Id.characters[0] = 'd';

	printf("EV side: call EVSE ServiceDetail \n");

	errn = request_response2(&exiIn, &exiOut);

	if(errn == 0) {
		/* check, if this is the right response message */
		if(exiOut.V2G_Message.Body.PaymentDetailsRes_isUsed) {

			paymentDetailsRes = exiOut.V2G_Message.Body.PaymentDetailsRes;

			printf("EV side: received response message from EVSE\n");
			/* show results of EVSEs answer message */
			printf("\tHeader SessionID=");
			printBinaryArray(exiOut.V2G_Message.Header.SessionID.bytes, exiOut.V2G_Message.Header.SessionID.bytesLen);
			printf("\t ResponseCode=%d\n",  paymentDetailsRes.ResponseCode);
			printf("\tEVSETimeStamp=%li\n",  (long int) paymentDetailsRes.EVSETimeStamp);
			printf("\tGenChallenge=%d\n",   paymentDetailsRes.GenChallenge.bytes[0]);

		} else {
			errn = ERROR_UNEXPECTED_PAYMENT_DETAILS_RESP_MESSAGE;
			return errn;
		}
	} else {
		return errn;
	}




	/*******************************************
	 * Authorization *
	 *******************************************/
	init_iso2BodyType(&exiIn.V2G_Message.Body);
	exiIn.V2G_Message.Body.AuthorizationReq_isUsed = 1u;

	init_iso2AuthorizationReqType(&exiIn.V2G_Message.Body.AuthorizationReq);

	copyBytes(paymentDetailsRes.GenChallenge.bytes, paymentDetailsRes.GenChallenge.bytesLen, exiIn.V2G_Message.Body.AuthorizationReq.GenChallenge.bytes);
	exiIn.V2G_Message.Body.AuthorizationReq.GenChallenge.bytesLen = paymentDetailsRes.GenChallenge.bytesLen;
	exiIn.V2G_Message.Body.AuthorizationReq.GenChallenge_isUsed = 1u; /* no challenge needed here*/
	exiIn.V2G_Message.Body.AuthorizationReq.Id_isUsed = 1u; /* no signature needed here */
	exiIn.V2G_Message.Body.AuthorizationReq.Id.charactersLen = 3;
	exiIn.V2G_Message.Body.AuthorizationReq.Id.characters[0] = 'I';
	exiIn.V2G_Message.Body.AuthorizationReq.Id.characters[1] = 'd';
	exiIn.V2G_Message.Body.AuthorizationReq.Id.characters[2] = '2';

	printf("EV side: call EVSE Authorization \n");


	errn = request_response2(&exiIn, &exiOut);

	if(errn == 0) {
		/* check, if this is the right response message */
		if(exiOut.V2G_Message.Body.AuthorizationRes_isUsed) {

			/* show results of EVSEs answer message */
			printf("EV side: received response message from EVSE\n");
			printf("\tHeader SessionID=");
			printBinaryArray(exiOut.V2G_Message.Header.SessionID.bytes, exiOut.V2G_Message.Header.SessionID.bytesLen);
			printf("\t ResponseCode=%d\n",  exiOut.V2G_Message.Body.AuthorizationRes.ResponseCode);

			if(exiOut.V2G_Message.Body.AuthorizationRes.EVSEProcessing == iso2EVSEProcessingType_Finished) {
				printf("\t EVSEProcessing=Finished\n");
			}
		} else {
			errn = ERROR_UNEXPECTED_AUTHORIZATION_RESP_MESSAGE;
			return errn;
		}
	} else {
		return errn;
	}




	/*******************************************
	 * chargeParameterDiscovery *
	 *******************************************/
	init_iso2BodyType(&exiIn.V2G_Message.Body);
	exiIn.V2G_Message.Body.ChargeParameterDiscoveryReq_isUsed = 1u;

	init_iso2ChargeParameterDiscoveryReqType(&exiIn.V2G_Message.Body.ChargeParameterDiscoveryReq);

	/* we use here AC based charging parameters */
	exiIn.V2G_Message.Body.ChargeParameterDiscoveryReq.MaxSupportingPoints_isUsed = 1u;
	exiIn.V2G_Message.Body.ChargeParameterDiscoveryReq.MaxSupportingPoints = 1234;

	exiIn.V2G_Message.Body.ChargeParameterDiscoveryReq.AC_EVBidirectionalParameter_isUsed = 1u;
	exiIn.V2G_Message.Body.ChargeParameterDiscoveryReq.AC_EVBidirectionalParameter.DepartureTime = 12345;

	exiIn.V2G_Message.Body.ChargeParameterDiscoveryReq.AC_EVBidirectionalParameter.EVMaximumChargePower.Exponent = 0;
	exiIn.V2G_Message.Body.ChargeParameterDiscoveryReq.AC_EVBidirectionalParameter.EVMaximumChargePower.Value = 100;

	exiIn.V2G_Message.Body.ChargeParameterDiscoveryReq.AC_EVBidirectionalParameter.EVMaximumChargeCurrent.Exponent = 0;
	exiIn.V2G_Message.Body.ChargeParameterDiscoveryReq.AC_EVBidirectionalParameter.EVMaximumChargeCurrent.Value = 400;

	exiIn.V2G_Message.Body.ChargeParameterDiscoveryReq.AC_EVBidirectionalParameter.EVMinimumChargeCurrent.Exponent = 0;
	exiIn.V2G_Message.Body.ChargeParameterDiscoveryReq.AC_EVBidirectionalParameter.EVMinimumChargeCurrent.Value = 200;

	exiIn.V2G_Message.Body.ChargeParameterDiscoveryReq.AC_EVBidirectionalParameter.EVMaximumVoltage.Exponent = 0;
	exiIn.V2G_Message.Body.ChargeParameterDiscoveryReq.AC_EVBidirectionalParameter.EVMaximumVoltage.Value = 400;

	exiIn.V2G_Message.Body.ChargeParameterDiscoveryReq.AC_EVBidirectionalParameter.EVMaximumDischargePower.Exponent = 0;
	exiIn.V2G_Message.Body.ChargeParameterDiscoveryReq.AC_EVBidirectionalParameter.EVMaximumDischargePower.Value = 200;

	exiIn.V2G_Message.Body.ChargeParameterDiscoveryReq.AC_EVBidirectionalParameter.EVMaximumDischargeCurrent.Exponent = 0;
	exiIn.V2G_Message.Body.ChargeParameterDiscoveryReq.AC_EVBidirectionalParameter.EVMaximumDischargeCurrent.Value = 400;

	exiIn.V2G_Message.Body.ChargeParameterDiscoveryReq.AC_EVBidirectionalParameter.EVMinimumDischargeCurrent.Exponent = 0;
	exiIn.V2G_Message.Body.ChargeParameterDiscoveryReq.AC_EVBidirectionalParameter.EVMinimumDischargeCurrent.Value = 200;

	printf("EV side: call EVSE chargeParameterDiscovery");

	errn = request_response2(&exiIn, &exiOut);

	if(errn == 0) {
		/* check, if this is the right response message */
		if(exiOut.V2G_Message.Body.ChargeParameterDiscoveryRes_isUsed) {

			/* show results of EVSEs answer message */
			printf("EV side: received response message from EVSE\n");
			printf("\tHeader SessionID=");
			printBinaryArray(exiOut.V2G_Message.Header.SessionID.bytes, exiOut.V2G_Message.Header.SessionID.bytesLen);
			printf("\t ResponseCode=%d\n", exiOut.V2G_Message.Body.ChargeParameterDiscoveryRes.ResponseCode);

			/*printACEVSEStatus(&(exiOut.V2G_Message.Body.ChargeParameterDiscoveryRes.AC_EVSEChargeParameter.AC_EVSEStatus));
			printf("\t EVSEProcessing=%d\n", exiOut.V2G_Message.Body.ChargeParameterDiscoveryRes.EVSEProcessing);
			printf("\t EVSEMaxCurrent=%d\n", exiOut.V2G_Message.Body.ChargeParameterDiscoveryRes.AC_EVSEChargeParameter.EVSEMaxCurrent.Value);
			printf("\t EVSENominalVoltage=%d\n", exiOut.V2G_Message.Body.ChargeParameterDiscoveryRes.AC_EVSEChargeParameter.EVSENominalVoltage.Value);*/
		} else {
			errn = ERROR_UNEXPECTED_CHARGE_PARAMETER_DISCOVERY_RESP_MESSAGE;
			return errn;
		}
	} else {
		return errn;
	}





	/*****************************
	 * cableCheck *
	 *****************************/
	init_iso2BodyType(&exiIn.V2G_Message.Body);
	exiIn.V2G_Message.Body.CableCheckReq_isUsed = 1u;

	/*init_v2gCableCheckReqType(&exiIn.V2G_Message.Body.CableCheckReq);*/

	printf("EV side: call EVSE cableCheck \n");

	errn = request_response2(&exiIn, &exiOut);

	if(errn == 0) {
		/* check, if this is the right response message */
		if(exiOut.V2G_Message.Body.CableCheckRes_isUsed) {

			/* show results of EVSEs answer message */
			printf("EV side: received response message from EVSE\n");
			printf("\tHeader SessionID=");
			printBinaryArray(exiOut.V2G_Message.Header.SessionID.bytes, exiOut.V2G_Message.Header.SessionID.bytesLen);
			printf("\t ResponseCode=%d\n", exiOut.V2G_Message.Body.CableCheckRes.ResponseCode);

			if(exiOut.V2G_Message.Body.CableCheckRes.EVSEProcessing==iso2EVSEProcessingType_Finished) {
				printf("\tEVSEProcessing=Finished\n");
			}

			printEVSEStatus2(&(exiOut.V2G_Message.Body.CableCheckRes.EVSEStatus));
		} else {
			errn = ERROR_UNEXPECTED_CABLE_CHECK_RESP_MESSAGE;
			return errn;
		}
	} else {
		return errn;
	}



	/*****************************
	 * preCharge *
	 *****************************/
	init_iso2BodyType(&exiIn.V2G_Message.Body);
	exiIn.V2G_Message.Body.PreChargeReq_isUsed = 1u;

	init_iso2PreChargeReqType(&exiIn.V2G_Message.Body.PreChargeReq);

	exiIn.V2G_Message.Body.PreChargeReq.EVTargetCurrent.Exponent = 1;
	exiIn.V2G_Message.Body.PreChargeReq.EVTargetCurrent.Value = 234;

	exiIn.V2G_Message.Body.PreChargeReq.EVTargetVoltage.Exponent = 1;
	exiIn.V2G_Message.Body.PreChargeReq.EVTargetVoltage.Value = 100;

	printf("EV side: call EVSE preCharge \n");

	errn = request_response2(&exiIn, &exiOut);

	if(errn == 0) {
		/* check, if this is the right response message */
		if(exiOut.V2G_Message.Body.PreChargeRes_isUsed) {

			/* show results of EVSEs answer message */
			printf("EV side: received response message from EVSE\n");
			printf("\tHeader SessionID=");
			printBinaryArray(exiOut.V2G_Message.Header.SessionID.bytes, exiOut.V2G_Message.Header.SessionID.bytesLen);
			printf("\t ResponseCode=%d\n", exiOut.V2G_Message.Body.PreChargeRes.ResponseCode);

			printEVSEStatus2(&exiOut.V2G_Message.Body.PreChargeRes.EVSEStatus);
			printf("\tEVSEPresentVoltage=%d (%d %d)\n", exiOut.V2G_Message.Body.PreChargeRes.EVSEPresentVoltage.Value, exiOut.V2G_Message.Body.PreChargeRes.EVSEPresentVoltage.Value, exiOut.V2G_Message.Body.PreChargeRes.EVSEPresentVoltage.Exponent);

		} else {
			errn = ERROR_UNEXPECTED_PRE_CHARGE_RESP_MESSAGE;
			return errn;
		}
	} else {
		return errn;
	}



	/*********************************
	 * PowerDelivery *
	 *********************************/

	init_iso2BodyType(&exiIn.V2G_Message.Body);
	exiIn.V2G_Message.Body.PowerDeliveryReq_isUsed = 1u;

	init_iso2PowerDeliveryReqType(&exiIn.V2G_Message.Body.PowerDeliveryReq);

	exiIn.V2G_Message.Body.PowerDeliveryReq.ChargeProgress = iso2chargeProgressType_Start;
	exiIn.V2G_Message.Body.PowerDeliveryReq.SAScheduleTupleID_isUsed = 1u;
	exiIn.V2G_Message.Body.PowerDeliveryReq.SAScheduleTupleID = exiOut.V2G_Message.Body.ChargeParameterDiscoveryRes.SAScheduleList.SAScheduleTuple.array[0].SAScheduleTupleID;

	printf("EV side: call EVSE powerDelivery \n");

	errn = request_response2(&exiIn, &exiOut);

	if(errn == 0) {
		/* check, if this is the right response message */
		if(exiOut.V2G_Message.Body.PowerDeliveryRes_isUsed) {

			/* show results of EVSEs answer message */
			printf("EV side: received response message from EVSE\n");
			printf("\tHeader SessionID=");
			printBinaryArray(exiOut.V2G_Message.Header.SessionID.bytes, exiOut.V2G_Message.Header.SessionID.bytesLen);
			printf("\t ResponseCode=%d\n", exiOut.V2G_Message.Body.PowerDeliveryRes.ResponseCode);

			/*printACEVSEStatus(&(exiOut.V2G_Message.Body.PowerDeliveryRes.AC_EVSEStatus));*/
		} else {
			errn = ERROR_UNEXPECTED_POWER_DELIVERY_RESP_MESSAGE;
			return errn;
		}
	} else {
		return errn;
	}



	/*********************************
	 * Setup data for chargingStatus *
	 *********************************/

	init_iso2BodyType(&exiIn.V2G_Message.Body);
	exiIn.V2G_Message.Body.ChargingStatusReq_isUsed = 1u;

	init_iso2ChargingStatusReqType(&exiIn.V2G_Message.Body.ChargingStatusReq);
	exiIn.V2G_Message.Body.ChargingStatusReq.EVTargetEnergyRequest.Exponent = 2;
	exiIn.V2G_Message.Body.ChargingStatusReq.EVTargetEnergyRequest.Value = 100;

	printf("EV side: call EVSE chargingStatus \n");

	errn = request_response2(&exiIn, &exiOut);

	if(errn == 0) {
		/* check, if this is the right response message */
		if(exiOut.V2G_Message.Body.ChargingStatusRes_isUsed) {

			/* show results of EVSEs answer message */
			printf("EV side: received response message from EVSE\n");
			printf("\tHeader SessionID=");
			printBinaryArray(exiOut.V2G_Message.Header.SessionID.bytes, exiOut.V2G_Message.Header.SessionID.bytesLen);
			printf("\t ResponseCode=%d\n", exiOut.V2G_Message.Body.ChargingStatusRes.ResponseCode);

			/*printACEVSEStatus(&(exiOut.V2G_Message.Body.ChargingStatusRes.AC_EVSEStatus));

			printf("\tReceiptRequired=%d\n", exiOut.V2G_Message.Body.ChargingStatusRes.ReceiptRequired);
			printf("\tEVSEID=%d\n", exiOut.V2G_Message.Body.ChargingStatusRes.EVSEID.characters[0]);
			printf("\tSAScheduleTupleID=%d\n", exiOut.V2G_Message.Body.ChargingStatusRes.SAScheduleTupleID);
			printf("\tEVSEMaxCurrent=%d (%d %d)\n", exiOut.V2G_Message.Body.ChargingStatusRes.EVSEMaxCurrent.Value, exiOut.V2G_Message.Body.ChargingStatusRes.EVSEMaxCurrent.Unit, exiOut.V2G_Message.Body.ChargingStatusRes.EVSEMaxCurrent.Multiplier);
			printf("\tisused.MeterInfo=%d\n",		exiOut.V2G_Message.Body.ChargingStatusRes.MeterInfo_isUsed);
			printf("\t\tMeterInfo.MeterID=%d\n",		exiOut.V2G_Message.Body.ChargingStatusRes.MeterInfo.MeterID.characters[0]);
			printf("\t\tMeterInfo.MeterReading.Value=%li\n",		(long int)exiOut.V2G_Message.Body.ChargingStatusRes.MeterInfo.MeterReading);
			printf("\t\tMeterInfo.MeterStatus=%d\n",		exiOut.V2G_Message.Body.ChargingStatusRes.MeterInfo.MeterStatus);
			printf("\t\tMeterInfo.TMeter=%li\n",		(long int)exiOut.V2G_Message.Body.ChargingStatusRes.MeterInfo.TMeter);
			printf("\t\tMeterInfo.SigMeterReading.data=%d\n",		exiOut.V2G_Message.Body.ChargingStatusRes.MeterInfo.SigMeterReading.bytes[0]);*/
		} else {
			errn = ERROR_UNEXPECTED_CHARGING_STATUS_RESP_MESSAGE;
			return errn;
		}
	} else {
		return errn;
	}




	/***********************************
	 * MeteringReceipt *
	 ***********************************/

	init_iso2BodyType(&exiIn.V2G_Message.Body);
	exiIn.V2G_Message.Body.MeteringReceiptReq_isUsed = 1u;

	init_iso2MeteringReceiptReqType(&exiIn.V2G_Message.Body.MeteringReceiptReq);


	exiIn.V2G_Message.Body.MeteringReceiptReq.Id.characters[0]='I';
	exiIn.V2G_Message.Body.MeteringReceiptReq.Id.characters[1]='d';
	exiIn.V2G_Message.Body.MeteringReceiptReq.Id.characters[2]='3';
	exiIn.V2G_Message.Body.MeteringReceiptReq.Id.charactersLen =3;

	exiIn.V2G_Message.Body.MeteringReceiptReq.SessionID.bytes[0] = 22;
	exiIn.V2G_Message.Body.MeteringReceiptReq.SessionID.bytesLen = 1;

	init_iso2MeterInfoType(&exiIn.V2G_Message.Body.MeteringReceiptReq.MeterInfo);
	exiIn.V2G_Message.Body.MeteringReceiptReq.MeterInfo.MeterID.characters[0] = 'M';
	exiIn.V2G_Message.Body.MeteringReceiptReq.MeterInfo.MeterID.characters[1] = 'i';
	exiIn.V2G_Message.Body.MeteringReceiptReq.MeterInfo.MeterID.characters[2] = 'd';
	exiIn.V2G_Message.Body.MeteringReceiptReq.MeterInfo.MeterID.charactersLen = 3;

	printf("EV side: call EVSE meteringReceipt \n");

	errn = request_response2(&exiIn, &exiOut);

	if(errn == 0) {
		/* check, if this is the right response message */
		if(exiOut.V2G_Message.Body.MeteringReceiptRes_isUsed) {

			/* show results of EVSEs answer message */
			printf("EV side: received response message from EVSE\n");
			printf("\tHeader SessionID=");
			printBinaryArray(exiOut.V2G_Message.Header.SessionID.bytes, exiOut.V2G_Message.Header.SessionID.bytesLen);
			printf("\t ResponseCode=%d\n", exiOut.V2G_Message.Body.MeteringReceiptRes.ResponseCode);

		} else {
			errn = ERROR_UNEXPECTED_METERING_RECEIPT_RESP_MESSAGE;
			return errn;
		}
	} else {
		return errn;
	}



	/***********************************
	 * SessionStop *
	 ***********************************/


	init_iso2BodyType(&exiIn.V2G_Message.Body);
	exiIn.V2G_Message.Body.SessionStopReq_isUsed = 1u;

	init_iso2SessionStopReqType(&exiIn.V2G_Message.Body.SessionStopReq);
	exiIn.V2G_Message.Body.SessionStopReq.ChargingSession = iso2chargingSessionType_Pause;

	printf("EV side: call EVSE stopSession \n");

	errn = request_response2(&exiIn, &exiOut);

	if(errn == 0) {
		/* check, if this is the right response message */
		if(exiOut.V2G_Message.Body.SessionStopRes_isUsed) {

			/* show results of EVSEs answer message */
			printf("EV side: received response message from EVSE\n");
			printf("\tHeader SessionID=");
			printBinaryArray(exiOut.V2G_Message.Header.SessionID.bytes, exiOut.V2G_Message.Header.SessionID.bytesLen);
			printf("\t ResponseCode=%d\n", exiOut.V2G_Message.Body.SessionStopRes.ResponseCode);

		} else {
			errn = ERROR_UNEXPECTED_SESSION_STOP_RESP_MESSAGE;
			return errn;
		}
	} else {
		return errn;
	}



	return errn;

}
#endif /* DEPLOY_ISO2_CODEC == SUPPORT_YES */






#if DEPLOY_ISO1_CODEC == SUPPORT_YES

static void printEVSEStatus1(struct iso1DC_EVSEStatusType* status)
{
	printf("\tEVSEStatus:\n");
	printf("\t\tEVSENotification=%d\n", status->EVSENotification);
	printf("\t\tNotificationMaxDelay=%d\n", status->NotificationMaxDelay);
}


/* serializes EXI stream and adds V2G TP header */
static int serialize1EXI2Stream(struct iso1EXIDocument* exiIn, bitstream_t* stream) {
	int errn;
	*stream->pos = V2GTP_HEADER_LENGTH;  /* v2gtp header */
	if( (errn = encode_iso1ExiDocument(stream, exiIn)) == 0) {
		errn = write_v2gtpHeader(stream->data, (*stream->pos)-V2GTP_HEADER_LENGTH, V2GTP_EXI_TYPE);
	}
	return errn;
}



/* deserializes V2G TP header and decodes right away EXI stream */
static int deserialize1Stream2EXI(bitstream_t* streamIn, struct iso1EXIDocument* exi) {
	int errn;
	uint32_t payloadLength;

	*streamIn->pos = 0;
	if ( (errn = read_v2gtpHeader(streamIn->data, &payloadLength)) == 0) {
		*streamIn->pos += V2GTP_HEADER_LENGTH;

		errn = decode_iso1ExiDocument(streamIn, exi);
	}
	return errn;
}



static int sessionSetup1(struct iso1EXIDocument* exiIn, struct iso1EXIDocument* exiOut) {
	printf("EVSE side: sessionSetup called\n"  );
	printf("\tReceived data:\n");
	printf("\tHeader SessionID=");
	printBinaryArray(exiIn->V2G_Message.Header.SessionID.bytes, exiIn->V2G_Message.Header.SessionID.bytesLen);
	printf("\t\t EVCCID=%d\n", exiIn->V2G_Message.Body.SessionSetupReq.EVCCID.bytes[0]);

	exiOut->V2G_Message_isUsed = 1u;

	/* generate an unique sessionID */
	init_iso1MessageHeaderType(&exiOut->V2G_Message.Header);
	exiOut->V2G_Message.Header.SessionID.bytes[0] = 1;
	exiOut->V2G_Message.Header.SessionID.bytes[1] = 2;
	exiOut->V2G_Message.Header.SessionID.bytes[2] = 3;
	exiOut->V2G_Message.Header.SessionID.bytes[3] = 4;
	exiOut->V2G_Message.Header.SessionID.bytes[4] = 5;
	exiOut->V2G_Message.Header.SessionID.bytes[5] = 6;
	exiOut->V2G_Message.Header.SessionID.bytes[6] = 7;
	exiOut->V2G_Message.Header.SessionID.bytes[7] = 8;
	exiOut->V2G_Message.Header.SessionID.bytesLen = 8;

	/* Prepare data for EV */
	init_iso1BodyType(&exiOut->V2G_Message.Body);

	exiOut->V2G_Message.Body.SessionSetupRes_isUsed = 1u;
	init_iso1SessionSetupResType(&exiOut->V2G_Message.Body.SessionSetupRes);

	exiOut->V2G_Message.Body.SessionSetupRes.ResponseCode = iso1responseCodeType_OK;
	exiOut->V2G_Message.Body.SessionSetupRes.EVSEID.characters[0] = 65;
	exiOut->V2G_Message.Body.SessionSetupRes.EVSEID.characters[1] = 85;
	exiOut->V2G_Message.Body.SessionSetupRes.EVSEID.characters[2] = 86;
	exiOut->V2G_Message.Body.SessionSetupRes.EVSEID.characters[3] = 87;
	exiOut->V2G_Message.Body.SessionSetupRes.EVSEID.characters[4] = 88;
	exiOut->V2G_Message.Body.SessionSetupRes.EVSEID.characters[5] = 89;
	exiOut->V2G_Message.Body.SessionSetupRes.EVSEID.characters[6] = 90;
	exiOut->V2G_Message.Body.SessionSetupRes.EVSEID.charactersLen = 7;
	exiOut->V2G_Message.Body.SessionSetupRes.EVSETimeStamp_isUsed = 1u;
	exiOut->V2G_Message.Body.SessionSetupRes.EVSETimeStamp = 123456789;

	return 0;
}



static int serviceDetail1(struct iso1EXIDocument* exiIn, struct iso1EXIDocument* exiOut) {

	printf("EVSE side: serviceDetail called\n"  );
	printf("\tReceived data:\n");
	printf("\tHeader SessionID=");
	printBinaryArray(exiIn->V2G_Message.Header.SessionID.bytes, exiIn->V2G_Message.Header.SessionID.bytesLen);
	printf("\t\t ServiceDetailID=%d\n",exiIn->V2G_Message.Body.ServiceDetailReq.ServiceID);


	/* Prepare data for EV */
	exiOut->V2G_Message_isUsed = 1u;
	init_iso1BodyType(&exiOut->V2G_Message.Body);

	exiOut->V2G_Message.Body.ServiceDetailRes_isUsed= 1u;
	init_iso1ServiceDetailResType(&exiOut->V2G_Message.Body.ServiceDetailRes);

	exiOut->V2G_Message.Body.ServiceDetailRes.ServiceID = 1234;

	exiOut->V2G_Message.Body.ServiceDetailRes.ServiceParameterList_isUsed = 1u;
	exiOut->V2G_Message.Body.ServiceDetailRes.ServiceParameterList.ParameterSet.arrayLen = 2;

	/* Parameter Set 1*/
	exiOut->V2G_Message.Body.ServiceDetailRes.ServiceParameterList.ParameterSet.array[0].ParameterSetID = 1;
	exiOut->V2G_Message.Body.ServiceDetailRes.ServiceParameterList.ParameterSet.array[0].Parameter.arrayLen = 2;
	exiOut->V2G_Message.Body.ServiceDetailRes.ServiceParameterList.ParameterSet.array[0].Parameter.array[0].Name.charactersLen = 8;
	exiOut->V2G_Message.Body.ServiceDetailRes.ServiceParameterList.ParameterSet.array[0].Parameter.array[0].Name.characters[0] = 'P';
	exiOut->V2G_Message.Body.ServiceDetailRes.ServiceParameterList.ParameterSet.array[0].Parameter.array[0].Name.characters[1] = 'r';
	exiOut->V2G_Message.Body.ServiceDetailRes.ServiceParameterList.ParameterSet.array[0].Parameter.array[0].Name.characters[2] = 'o';
	exiOut->V2G_Message.Body.ServiceDetailRes.ServiceParameterList.ParameterSet.array[0].Parameter.array[0].Name.characters[3] = 't';
	exiOut->V2G_Message.Body.ServiceDetailRes.ServiceParameterList.ParameterSet.array[0].Parameter.array[0].Name.characters[4]= 'o';
	exiOut->V2G_Message.Body.ServiceDetailRes.ServiceParameterList.ParameterSet.array[0].Parameter.array[0].Name.characters[5] = 'c';
	exiOut->V2G_Message.Body.ServiceDetailRes.ServiceParameterList.ParameterSet.array[0].Parameter.array[0].Name.characters[6] = 'o';
	exiOut->V2G_Message.Body.ServiceDetailRes.ServiceParameterList.ParameterSet.array[0].Parameter.array[0].Name.characters[7] = 'l';
	exiOut->V2G_Message.Body.ServiceDetailRes.ServiceParameterList.ParameterSet.array[0].Parameter.array[0].intValue = 15119;
	exiOut->V2G_Message.Body.ServiceDetailRes.ServiceParameterList.ParameterSet.array[0].Parameter.array[0].intValue_isUsed = 1u;

	exiOut->V2G_Message.Body.ServiceDetailRes.ServiceParameterList.ParameterSet.array[0].Parameter.array[1].Name.charactersLen = 4;
	exiOut->V2G_Message.Body.ServiceDetailRes.ServiceParameterList.ParameterSet.array[0].Parameter.array[1].Name.characters[0] = 'N';
	exiOut->V2G_Message.Body.ServiceDetailRes.ServiceParameterList.ParameterSet.array[0].Parameter.array[1].Name.characters[1] = 'a';
	exiOut->V2G_Message.Body.ServiceDetailRes.ServiceParameterList.ParameterSet.array[0].Parameter.array[1].Name.characters[2] = 'm';
	exiOut->V2G_Message.Body.ServiceDetailRes.ServiceParameterList.ParameterSet.array[0].Parameter.array[1].Name.characters[3] = 'e';

	exiOut->V2G_Message.Body.ServiceDetailRes.ServiceParameterList.ParameterSet.array[0].Parameter.array[1].stringValue_isUsed = 1u;
	exiOut->V2G_Message.Body.ServiceDetailRes.ServiceParameterList.ParameterSet.array[0].Parameter.array[1].stringValue.charactersLen = 3;
	exiOut->V2G_Message.Body.ServiceDetailRes.ServiceParameterList.ParameterSet.array[0].Parameter.array[1].stringValue.characters[0] = 'V';
	exiOut->V2G_Message.Body.ServiceDetailRes.ServiceParameterList.ParameterSet.array[0].Parameter.array[1].stringValue.characters[1] = '2';
	exiOut->V2G_Message.Body.ServiceDetailRes.ServiceParameterList.ParameterSet.array[0].Parameter.array[1].stringValue.characters[2] = 'G';

	/* Parameter Set 2 */
	exiOut->V2G_Message.Body.ServiceDetailRes.ServiceParameterList.ParameterSet.array[1].ParameterSetID = 2;
	exiOut->V2G_Message.Body.ServiceDetailRes.ServiceParameterList.ParameterSet.array[1].Parameter.arrayLen = 1;
	exiOut->V2G_Message.Body.ServiceDetailRes.ServiceParameterList.ParameterSet.array[1].Parameter.array[0].Name.charactersLen = 7;
	exiOut->V2G_Message.Body.ServiceDetailRes.ServiceParameterList.ParameterSet.array[1].Parameter.array[0].Name.characters[0] = 'C';
	exiOut->V2G_Message.Body.ServiceDetailRes.ServiceParameterList.ParameterSet.array[1].Parameter.array[0].Name.characters[1] = 'h';
	exiOut->V2G_Message.Body.ServiceDetailRes.ServiceParameterList.ParameterSet.array[1].Parameter.array[0].Name.characters[2] = 'a';
	exiOut->V2G_Message.Body.ServiceDetailRes.ServiceParameterList.ParameterSet.array[1].Parameter.array[0].Name.characters[3] = 'n';
	exiOut->V2G_Message.Body.ServiceDetailRes.ServiceParameterList.ParameterSet.array[1].Parameter.array[0].Name.characters[4] = 'n';
	exiOut->V2G_Message.Body.ServiceDetailRes.ServiceParameterList.ParameterSet.array[1].Parameter.array[0].Name.characters[5] = 'e';
	exiOut->V2G_Message.Body.ServiceDetailRes.ServiceParameterList.ParameterSet.array[1].Parameter.array[0].Name.characters[6] = 'l';
	exiOut->V2G_Message.Body.ServiceDetailRes.ServiceParameterList.ParameterSet.array[1].Parameter.array[0].physicalValue_isUsed = 1u;
	exiOut->V2G_Message.Body.ServiceDetailRes.ServiceParameterList.ParameterSet.array[1].Parameter.array[0].physicalValue.Value = 1234;

	exiOut->V2G_Message.Body.ServiceDetailRes.ServiceParameterList.ParameterSet.array[1].Parameter.array[0].physicalValue.Multiplier = 1;
	exiOut->V2G_Message.Body.ServiceDetailRes.ServiceParameterList.ParameterSet.array[1].Parameter.array[0].physicalValue.Value = 2;

	exiOut->V2G_Message.Body.ServiceDetailRes.ResponseCode = iso1responseCodeType_OK;

	return 0;
}



static int authorization1(struct iso1EXIDocument* exiIn, struct iso1EXIDocument* exiOut) {

	printf("EVSE: Authorization called\n"  );
	printf("\tReceived data:\n");

	if(exiIn->V2G_Message.Body.AuthorizationReq.GenChallenge_isUsed) {
		printf("\t\t\t GenChallenge=%d\n", exiIn->V2G_Message.Body.AuthorizationReq.GenChallenge.bytes[0]);
	}
	if(exiIn->V2G_Message.Body.AuthorizationReq.Id_isUsed ) {
		printf("\t\t\t ID=%c%c%c\n", exiIn->V2G_Message.Body.AuthorizationReq.Id.characters[0], exiIn->V2G_Message.Body.AuthorizationReq.Id.characters[1], exiIn->V2G_Message.Body.AuthorizationReq.Id.characters[2]);
	}


	/* Prepare data for EV */
	exiOut->V2G_Message_isUsed = 1u;
	init_iso1BodyType(&exiOut->V2G_Message.Body);

	exiOut->V2G_Message.Body.AuthorizationRes_isUsed = 1u;
	init_iso1AuthorizationResType(&exiOut->V2G_Message.Body.AuthorizationRes);

	exiOut->V2G_Message.Body.AuthorizationRes.ResponseCode = iso1responseCodeType_OK;
	exiOut->V2G_Message.Body.AuthorizationRes.EVSEProcessing = iso1EVSEProcessingType_Finished;

	return 0;
}


static int powerDelivery1(struct iso1EXIDocument* exiIn, struct iso1EXIDocument* exiOut) {
	printf("EVSE side: powerDelivery called\n"  );
	printf("\tReceived data:\n");

	printf("\t\t  ChargeProgress=%d\n", exiIn->V2G_Message.Body.PowerDeliveryReq.ChargeProgress);
	printf("\t\t  SAScheduleTupleID=%d\n", exiIn->V2G_Message.Body.PowerDeliveryReq.SAScheduleTupleID);

	/* Prepare data for EV */
	exiOut->V2G_Message_isUsed = 1u;
	init_iso1BodyType(&exiOut->V2G_Message.Body);

	exiOut->V2G_Message.Body.PowerDeliveryRes_isUsed = 1u;
	init_iso1PowerDeliveryResType(&exiOut->V2G_Message.Body.PowerDeliveryRes);

	exiOut->V2G_Message.Body.PowerDeliveryRes.ResponseCode = iso1responseCodeType_OK;

	exiOut->V2G_Message.Body.PowerDeliveryRes.EVSEStatus_isUsed = 1;
	exiOut->V2G_Message.Body.PowerDeliveryRes.EVSEStatus.EVSENotification = iso1EVSENotificationType_StopCharging;
	exiOut->V2G_Message.Body.PowerDeliveryRes.EVSEStatus.NotificationMaxDelay=12;

	return 0;
}


static int chargingStatus1(struct iso1EXIDocument* exiIn, struct iso1EXIDocument* exiOut) {

	printf("EVSE side: chargingStatus called\n"  );

	/* Prepare data for EV */
	exiOut->V2G_Message_isUsed = 1u;
	init_iso1BodyType(&exiOut->V2G_Message.Body);

	exiOut->V2G_Message.Body.ChargingStatusRes_isUsed = 1u;
	init_iso1ChargingStatusResType(&exiOut->V2G_Message.Body.ChargingStatusRes);


	exiOut->V2G_Message.Body.ChargingStatusRes.ResponseCode = iso1responseCodeType_OK;
	exiOut->V2G_Message.Body.ChargingStatusRes.EVSEID.characters[0]= 'A';
	exiOut->V2G_Message.Body.ChargingStatusRes.EVSEID.charactersLen =1;

	exiOut->V2G_Message.Body.ChargingStatusRes.ReceiptRequired = 1;
	exiOut->V2G_Message.Body.ChargingStatusRes.ReceiptRequired_isUsed = 1;

	return 0;
}


static int meteringReceipt1(struct iso1EXIDocument* exiIn, struct iso1EXIDocument* exiOut) {

	printf("EVSE side: meteringReceipt called\n"  );
	printf("\tReceived data:\n");

	printf("\t\t ID=%c%c%c\n", exiIn->V2G_Message.Body.MeteringReceiptReq.Id.characters[0], exiIn->V2G_Message.Body.MeteringReceiptReq.Id.characters[1], exiIn->V2G_Message.Body.MeteringReceiptReq.Id.characters[2]);
	printf("\t\t SAScheduleTupleID=%d\n", exiIn->V2G_Message.Body.MeteringReceiptReq.SAScheduleTupleID);
	printf("\t\t SessionID=%d\n", exiIn->V2G_Message.Body.MeteringReceiptReq.SessionID.bytes[1]);
	printf("\t\t MeterInfo.MeterStatus=%d\n", exiIn->V2G_Message.Body.MeteringReceiptReq.MeterInfo.MeterStatus);
	printf("\t\t MeterInfo.MeterID=%d\n",		exiIn->V2G_Message.Body.MeteringReceiptReq.MeterInfo.MeterID.characters[0]);

	/* Prepare data for EV */
	exiOut->V2G_Message_isUsed = 1u;
	init_iso1BodyType(&exiOut->V2G_Message.Body);

	exiOut->V2G_Message.Body.MeteringReceiptRes_isUsed = 1u;
	init_iso1MeteringReceiptResType(&exiOut->V2G_Message.Body.MeteringReceiptRes);

	exiOut->V2G_Message.Body.MeteringReceiptRes.ResponseCode = iso1responseCodeType_FAILED;

	return 0;
}

static int sessionStop1(struct iso1EXIDocument* exiIn, struct iso1EXIDocument* exiOut) {

	printf("EVSE side: sessionStop called\n"  );
	printf("\tReceived data:\n");
	printf("\tHeader SessionID=");
	printBinaryArray(exiIn->V2G_Message.Header.SessionID.bytes, exiIn->V2G_Message.Header.SessionID.bytesLen);
	printf("\t\t ChargingSession=%d\n", exiIn->V2G_Message.Body.SessionStopReq.ChargingSession);

	/* Prepare data for EV */
	exiOut->V2G_Message_isUsed = 1u;
	init_iso1BodyType(&exiOut->V2G_Message.Body);

	exiOut->V2G_Message.Body.SessionStopRes_isUsed = 1u;
	init_iso1SessionStopResType(&exiOut->V2G_Message.Body.SessionStopRes);

	exiOut->V2G_Message.Body.SessionStopRes.ResponseCode = iso1responseCodeType_OK;

	return 0;
}

static int cableCheck1(struct iso1EXIDocument* exiIn, struct iso1EXIDocument* exiOut) {

	printf("EVSE side: cableCheck called\n"  );

	/* Prepare data for EV */
	exiOut->V2G_Message_isUsed = 1u;
	init_iso1BodyType(&exiOut->V2G_Message.Body);

	exiOut->V2G_Message.Body.CableCheckRes_isUsed = 1u;
	init_iso1CableCheckResType(&exiOut->V2G_Message.Body.CableCheckRes);

	exiOut->V2G_Message.Body.CableCheckRes.ResponseCode = iso1responseCodeType_OK;

	exiOut->V2G_Message.Body.CableCheckRes.DC_EVSEStatus.NotificationMaxDelay = 1234;
	exiOut->V2G_Message.Body.CableCheckRes.DC_EVSEStatus.EVSENotification= iso1EVSENotificationType_ReNegotiation;

	exiOut->V2G_Message.Body.CableCheckRes.EVSEProcessing = iso1EVSEProcessingType_Finished;

	return 0;
}

static int preCharge1(struct iso1EXIDocument* exiIn, struct iso1EXIDocument* exiOut) {

	printf("EVSE side: preCharge called\n"  );
	printf("\tReceived data:\n");

	printf("\t\t EVTargetCurrent=%d (%d)\n", exiIn->V2G_Message.Body.PreChargeReq.EVTargetCurrent.Value, exiIn->V2G_Message.Body.PreChargeReq.EVTargetCurrent.Multiplier);
	printf("\t\t EVTargetVoltage=%d (%d)\n", exiIn->V2G_Message.Body.PreChargeReq.EVTargetVoltage.Value, exiIn->V2G_Message.Body.PreChargeReq.EVTargetVoltage.Multiplier);

	/* Prepare data for EV */
	exiOut->V2G_Message_isUsed = 1u;
	init_iso1BodyType(&exiOut->V2G_Message.Body);

	exiOut->V2G_Message.Body.PreChargeRes_isUsed = 1u;
	init_iso1PreChargeResType(&exiOut->V2G_Message.Body.PreChargeRes);

	exiOut->V2G_Message.Body.PreChargeRes.ResponseCode = iso1responseCodeType_OK;

	exiOut->V2G_Message.Body.PreChargeRes.DC_EVSEStatus.EVSENotification = iso1EVSENotificationType_StopCharging;
	exiOut->V2G_Message.Body.PreChargeRes.DC_EVSEStatus.NotificationMaxDelay= 1234;

	exiOut->V2G_Message.Body.PreChargeRes.EVSEPresentVoltage.Multiplier = 3;
	exiOut->V2G_Message.Body.PreChargeRes.EVSEPresentVoltage.Value= 456;

	return 0;
}





static int create_response_message1(struct iso1EXIDocument* exiIn, struct iso1EXIDocument* exiOut) {
	int errn = ERROR_UNEXPECTED_REQUEST_MESSAGE;

	/* create response message as EXI document */
	if(exiIn->V2G_Message_isUsed) {
		init_iso1EXIDocument(exiOut);
		if (exiIn->V2G_Message.Body.SessionSetupReq_isUsed) {
			errn = sessionSetup1(exiIn, exiOut);
		} else if (exiIn->V2G_Message.Body.ServiceDetailReq_isUsed) {
			errn = serviceDetail1(exiIn, exiOut);
		/*} else if (exiIn->V2G_Message.Body.PaymentDetailsReq_isUsed) {
			errn = paymentDetails1(exiIn, exiOut);*/
		} else if (exiIn->V2G_Message.Body.AuthorizationReq_isUsed) {
			errn = authorization1(exiIn, exiOut);
		} else if (exiIn->V2G_Message.Body.PowerDeliveryReq_isUsed) {
			errn = powerDelivery1(exiIn, exiOut);
		} else if (exiIn->V2G_Message.Body.ChargingStatusReq_isUsed) {
			errn = chargingStatus1(exiIn, exiOut);
		} else if (exiIn->V2G_Message.Body.MeteringReceiptReq_isUsed) {
			errn = meteringReceipt1(exiIn, exiOut);
		} else if (exiIn->V2G_Message.Body.SessionStopReq_isUsed) {
			errn = sessionStop1(exiIn, exiOut);
		} else if (exiIn->V2G_Message.Body.CableCheckReq_isUsed) {
			errn = cableCheck1(exiIn, exiOut);
		} else if (exiIn->V2G_Message.Body.PreChargeReq_isUsed) {
			errn = preCharge1(exiIn, exiOut);
		}
	}

	return errn;
}


/* Adapt this to your system setup! */
/* In this situation EV and EVSE is the same party */
static int request_response1(struct iso1EXIDocument* exiIn, struct iso1EXIDocument* exiOut) {
	int errn;

	bitstream_t stream1;
	bitstream_t stream2;

	size_t pos1;
	size_t pos2;

	stream1.size = BUFFER_SIZE;
	stream1.data = buffer1;
	stream1.pos = &pos1;

	stream2.size = BUFFER_SIZE;
	stream2.data = buffer2;
	stream2.pos = &pos2;

	/* EV side */
	errn = serialize1EXI2Stream(exiIn, &stream1);

	/* --> Start of EVSE side */
	/* deserialize request message */
	if (errn == 0) {
		errn = deserialize1Stream2EXI(&stream1, exiOut);
	}
	/* create response message */
	if (errn == 0) {
		errn = create_response_message1(exiOut, exiIn);
	}
	/* serialize response message */
	if (errn == 0) {
		errn = serialize1EXI2Stream(exiIn, &stream2);
	}
	/* <-- End of EVSE side */


	/* EV side */
	/* deserialize response message */
	if (errn == 0) {
		errn = deserialize1Stream2EXI(&stream2, exiOut);
	}

	return errn;
}

static int charging1()
{
	int errn = 0;
	int i, j;

	struct iso1EXIDocument exiIn;
	struct iso1EXIDocument exiOut;

	struct iso1ServiceDetailResType serviceDetailRes;
	struct iso1PaymentDetailsResType paymentDetailsRes;

	/* setup header information */
	init_iso1EXIDocument(&exiIn);
	exiIn.V2G_Message_isUsed = 1u;
	init_iso1MessageHeaderType(&exiIn.V2G_Message.Header);
	exiIn.V2G_Message.Header.SessionID.bytes[0] = 0; /* sessionID is always '0' at the beginning (the response contains the valid sessionID)*/
	exiIn.V2G_Message.Header.SessionID.bytes[1] = 0;
	exiIn.V2G_Message.Header.SessionID.bytes[2] = 0;
	exiIn.V2G_Message.Header.SessionID.bytes[3] = 0;
	exiIn.V2G_Message.Header.SessionID.bytes[4] = 0;
	exiIn.V2G_Message.Header.SessionID.bytes[5] = 0;
	exiIn.V2G_Message.Header.SessionID.bytes[6] = 0;
	exiIn.V2G_Message.Header.SessionID.bytes[7] = 0;
	exiIn.V2G_Message.Header.SessionID.bytesLen = 8;
	exiIn.V2G_Message.Header.Signature_isUsed = 0u;


	/************************
	 * sessionSetup *
	 ************************/
	init_iso1BodyType(&exiIn.V2G_Message.Body);
	exiIn.V2G_Message.Body.SessionSetupReq_isUsed = 1u;

	init_iso1SessionSetupReqType(&exiIn.V2G_Message.Body.SessionSetupReq);

	exiIn.V2G_Message.Body.SessionSetupReq.EVCCID.bytesLen = 1;
	exiIn.V2G_Message.Body.SessionSetupReq.EVCCID.bytes[0] = 10;

	printf("EV side: call EVSE sessionSetup");

	errn = request_response1(&exiIn, &exiOut);

	if(errn == 0) {
		/* check, if this is the right response message */
		if(exiOut.V2G_Message.Body.SessionSetupRes_isUsed) {
			/* show results of EVSEs answer message */
			printf("EV side: received response message from EVSE\n");
			printf("\tHeader SessionID=");
			printBinaryArray(exiOut.V2G_Message.Header.SessionID.bytes, exiOut.V2G_Message.Header.SessionID.bytesLen);
			printf("\tResponseCode=%d\n", exiOut.V2G_Message.Body.SessionSetupRes.ResponseCode);
			printf("\tEVSEID=%d\n",	exiOut.V2G_Message.Body.SessionSetupRes.EVSEID.characters[1]);
			printf("\tEVSETimeStamp=%li\n", (long int)exiOut.V2G_Message.Body.SessionSetupRes.EVSETimeStamp);
		} else {
			errn = ERROR_UNEXPECTED_SESSION_SETUP_RESP_MESSAGE;
			return errn;
		}
	} else {
		return errn;
	}



	/*********************************
	 * ServiceDetails *
	 *********************************/
	init_iso1BodyType(&exiIn.V2G_Message.Body);
	exiIn.V2G_Message.Body.ServiceDetailReq_isUsed = 1u;

	init_iso1ServiceDetailReqType(&exiIn.V2G_Message.Body.ServiceDetailReq);

	exiIn.V2G_Message.Body.ServiceDetailReq.ServiceID = 22; /* Value Added Server ID */

	printf("EV side: call EVSE ServiceDetail \n");

	errn = request_response1(&exiIn, &exiOut);

	if(errn == 0) {
		/* check, if this is the right response message */
		if(exiOut.V2G_Message.Body.ServiceDetailRes_isUsed) {
			serviceDetailRes = exiOut.V2G_Message.Body.ServiceDetailRes;
			/* show results of EVSEs answer message */
			printf("EV side: received response message from EVSE\n");
			printf("\tHeader SessionID=");
			printBinaryArray(exiOut.V2G_Message.Header.SessionID.bytes, exiOut.V2G_Message.Header.SessionID.bytesLen);
			printf("\t ResponseCode=%d\n",  exiOut.V2G_Message.Body.ServiceDiscoveryRes.ResponseCode);
			printf("\t ServiceID=%d\n",	exiOut.V2G_Message.Body.ServiceDetailRes.ServiceID);

			if(serviceDetailRes.ServiceParameterList_isUsed) {
				printf("\t\tLength=%d\n", serviceDetailRes.ServiceParameterList.ParameterSet.arrayLen );/*TEST*/

				for(i=0; i<serviceDetailRes.ServiceParameterList.ParameterSet.arrayLen; i++)
				{
					printf("\t\tServiceSetID=%d\n", serviceDetailRes.ServiceParameterList.ParameterSet.array[i].ParameterSetID);
					printf("\t\tParameters=%d\n", serviceDetailRes.ServiceParameterList.ParameterSet.array[i].Parameter.arrayLen);

					for(j=0; j<serviceDetailRes.ServiceParameterList.ParameterSet.array[i].Parameter.arrayLen; j++)
					{
						printf("\t\t\t %d: ParameterName=", j+1);
						printASCIIString(serviceDetailRes.ServiceParameterList.ParameterSet.array[i].Parameter.array[j].Name.characters, exiOut.V2G_Message.Body.ServiceDetailRes.ServiceParameterList.ParameterSet.array[i].Parameter.array[j].Name.charactersLen);

						/*if(serviceDetailRes.ServiceParameterList.ParameterSet.array[i].Parameter.array[j].shortValue_isUsed == 1u) {
							printf("\t\t\t %d: StringValue=", j+1);
							printASCIIString(serviceDetailRes.ServiceParameterList.ParameterSet.array[i].Parameter.array[j].stringValue.characters, serviceDetailRes.ServiceParameterList.ParameterSet.array[i].Parameter.array[j].stringValue.charactersLen);
						} else if(serviceDetailRes.ServiceParameterList.ParameterSet.array[i].Parameter.array[j].intValue_isUsed == 1u) {
							printf("\t\t\t %d: IntValue=%d\n", j+1, serviceDetailRes.ServiceParameterList.ParameterSet.array[i].Parameter.array[j].intValue);
						} else if(serviceDetailRes.ServiceParameterList.ParameterSet.array[i].Parameter.array[j].physicalValue_isUsed == 1u) {
							printf("\t\t\t %d: PhysicalValue=%d (%d)\n",  j+1, serviceDetailRes.ServiceParameterList.ParameterSet.array[i].Parameter.array[j].physicalValue.Value, serviceDetailRes.ServiceParameterList.ParameterSet.array[i].Parameter.array[j].physicalValue.Multiplier);
						}*/
					}
				}
			}
		} else {
			errn = ERROR_UNEXPECTED_SERVICE_DETAILS_RESP_MESSAGE;
			return errn;
		}
	} else {
		return errn;
	}




	/*******************************************
	 * Authorization *
	 *******************************************/
	init_iso1BodyType(&exiIn.V2G_Message.Body);
	exiIn.V2G_Message.Body.AuthorizationReq_isUsed = 1u;

	init_iso1AuthorizationReqType(&exiIn.V2G_Message.Body.AuthorizationReq);

	copyBytes(paymentDetailsRes.GenChallenge.bytes, paymentDetailsRes.GenChallenge.bytesLen, exiIn.V2G_Message.Body.AuthorizationReq.GenChallenge.bytes);
	exiIn.V2G_Message.Body.AuthorizationReq.GenChallenge.bytesLen = paymentDetailsRes.GenChallenge.bytesLen;
	exiIn.V2G_Message.Body.AuthorizationReq.GenChallenge_isUsed = 1u; /* no challenge needed here*/
	exiIn.V2G_Message.Body.AuthorizationReq.Id_isUsed = 1u; /* no signature needed here */
	exiIn.V2G_Message.Body.AuthorizationReq.Id.charactersLen = 3;
	exiIn.V2G_Message.Body.AuthorizationReq.Id.characters[0] = 'I';
	exiIn.V2G_Message.Body.AuthorizationReq.Id.characters[1] = 'd';
	exiIn.V2G_Message.Body.AuthorizationReq.Id.characters[2] = '2';

	printf("EV side: call EVSE Authorization \n");


	errn = request_response1(&exiIn, &exiOut);

	if(errn == 0) {
		/* check, if this is the right response message */
		if(exiOut.V2G_Message.Body.AuthorizationRes_isUsed) {

			/* show results of EVSEs answer message */
			printf("EV side: received response message from EVSE\n");
			printf("\tHeader SessionID=");
			printBinaryArray(exiOut.V2G_Message.Header.SessionID.bytes, exiOut.V2G_Message.Header.SessionID.bytesLen);
			printf("\t ResponseCode=%d\n",  exiOut.V2G_Message.Body.AuthorizationRes.ResponseCode);

			if(exiOut.V2G_Message.Body.AuthorizationRes.EVSEProcessing == iso1EVSEProcessingType_Finished) {
				printf("\t EVSEProcessing=Finished\n");
			}
		} else {
			errn = ERROR_UNEXPECTED_AUTHORIZATION_RESP_MESSAGE;
			return errn;
		}
	} else {
		return errn;
	}





	/*****************************
	 * cableCheck *
	 *****************************/
	init_iso1BodyType(&exiIn.V2G_Message.Body);
	exiIn.V2G_Message.Body.CableCheckReq_isUsed = 1u;

	/*init_v2gCableCheckReqType(&exiIn.V2G_Message.Body.CableCheckReq);*/

	printf("EV side: call EVSE cableCheck \n");

	errn = request_response1(&exiIn, &exiOut);

	if(errn == 0) {
		/* check, if this is the right response message */
		if(exiOut.V2G_Message.Body.CableCheckRes_isUsed) {

			/* show results of EVSEs answer message */
			printf("EV side: received response message from EVSE\n");
			printf("\tHeader SessionID=");
			printBinaryArray(exiOut.V2G_Message.Header.SessionID.bytes, exiOut.V2G_Message.Header.SessionID.bytesLen);
			printf("\t ResponseCode=%d\n", exiOut.V2G_Message.Body.CableCheckRes.ResponseCode);

			if(exiOut.V2G_Message.Body.CableCheckRes.EVSEProcessing==iso1EVSEProcessingType_Finished) {
				printf("\tEVSEProcessing=Finished\n");
			}

			printEVSEStatus1(&(exiOut.V2G_Message.Body.CableCheckRes.DC_EVSEStatus));
		} else {
			errn = ERROR_UNEXPECTED_CABLE_CHECK_RESP_MESSAGE;
			return errn;
		}
	} else {
		return errn;
	}



	/*****************************
	 * preCharge *
	 *****************************/
	init_iso1BodyType(&exiIn.V2G_Message.Body);
	exiIn.V2G_Message.Body.PreChargeReq_isUsed = 1u;

	init_iso1PreChargeReqType(&exiIn.V2G_Message.Body.PreChargeReq);

	exiIn.V2G_Message.Body.PreChargeReq.EVTargetCurrent.Multiplier = 1;
	exiIn.V2G_Message.Body.PreChargeReq.EVTargetCurrent.Value = 234;

	exiIn.V2G_Message.Body.PreChargeReq.EVTargetVoltage.Multiplier = 1;
	exiIn.V2G_Message.Body.PreChargeReq.EVTargetVoltage.Value = 100;

	printf("EV side: call EVSE preCharge \n");

	errn = request_response1(&exiIn, &exiOut);

	if(errn == 0) {
		/* check, if this is the right response message */
		if(exiOut.V2G_Message.Body.PreChargeRes_isUsed) {

			/* show results of EVSEs answer message */
			printf("EV side: received response message from EVSE\n");
			printf("\tHeader SessionID=");
			printBinaryArray(exiOut.V2G_Message.Header.SessionID.bytes, exiOut.V2G_Message.Header.SessionID.bytesLen);
			printf("\t ResponseCode=%d\n", exiOut.V2G_Message.Body.PreChargeRes.ResponseCode);

			printEVSEStatus1(&exiOut.V2G_Message.Body.PreChargeRes.DC_EVSEStatus);
			printf("\tEVSEPresentVoltage=%d (%d %d)\n", exiOut.V2G_Message.Body.PreChargeRes.EVSEPresentVoltage.Value, exiOut.V2G_Message.Body.PreChargeRes.EVSEPresentVoltage.Value, exiOut.V2G_Message.Body.PreChargeRes.EVSEPresentVoltage.Multiplier);

		} else {
			errn = ERROR_UNEXPECTED_PRE_CHARGE_RESP_MESSAGE;
			return errn;
		}
	} else {
		return errn;
	}



	/*********************************
	 * PowerDelivery *
	 *********************************/

	init_iso1BodyType(&exiIn.V2G_Message.Body);
	exiIn.V2G_Message.Body.PowerDeliveryReq_isUsed = 1u;

	init_iso1PowerDeliveryReqType(&exiIn.V2G_Message.Body.PowerDeliveryReq);

	exiIn.V2G_Message.Body.PowerDeliveryReq.ChargeProgress = iso1chargeProgressType_Start;
	exiIn.V2G_Message.Body.PowerDeliveryReq.SAScheduleTupleID = exiOut.V2G_Message.Body.ChargeParameterDiscoveryRes.SAScheduleList.SAScheduleTuple.array[0].SAScheduleTupleID;

	printf("EV side: call EVSE powerDelivery \n");

	errn = request_response1(&exiIn, &exiOut);

	if(errn == 0) {
		/* check, if this is the right response message */
		if(exiOut.V2G_Message.Body.PowerDeliveryRes_isUsed) {

			/* show results of EVSEs answer message */
			printf("EV side: received response message from EVSE\n");
			printf("\tHeader SessionID=");
			printBinaryArray(exiOut.V2G_Message.Header.SessionID.bytes, exiOut.V2G_Message.Header.SessionID.bytesLen);
			printf("\t ResponseCode=%d\n", exiOut.V2G_Message.Body.PowerDeliveryRes.ResponseCode);

			/*printACEVSEStatus(&(exiOut.V2G_Message.Body.PowerDeliveryRes.AC_EVSEStatus));*/
		} else {
			errn = ERROR_UNEXPECTED_POWER_DELIVERY_RESP_MESSAGE;
			return errn;
		}
	} else {
		return errn;
	}



	/*********************************
	 * Setup data for chargingStatus *
	 *********************************/

	init_iso1BodyType(&exiIn.V2G_Message.Body);
	exiIn.V2G_Message.Body.ChargingStatusReq_isUsed = 1u;

	init_iso1ChargingStatusReqType(&exiIn.V2G_Message.Body.ChargingStatusReq);

	printf("EV side: call EVSE chargingStatus \n");

	errn = request_response1(&exiIn, &exiOut);

	if(errn == 0) {
		/* check, if this is the right response message */
		if(exiOut.V2G_Message.Body.ChargingStatusRes_isUsed) {

			/* show results of EVSEs answer message */
			printf("EV side: received response message from EVSE\n");
			printf("\tHeader SessionID=");
			printBinaryArray(exiOut.V2G_Message.Header.SessionID.bytes, exiOut.V2G_Message.Header.SessionID.bytesLen);
			printf("\t ResponseCode=%d\n", exiOut.V2G_Message.Body.ChargingStatusRes.ResponseCode);

			/*printACEVSEStatus(&(exiOut.V2G_Message.Body.ChargingStatusRes.AC_EVSEStatus));

			printf("\tReceiptRequired=%d\n", exiOut.V2G_Message.Body.ChargingStatusRes.ReceiptRequired);
			printf("\tEVSEID=%d\n", exiOut.V2G_Message.Body.ChargingStatusRes.EVSEID.characters[0]);
			printf("\tSAScheduleTupleID=%d\n", exiOut.V2G_Message.Body.ChargingStatusRes.SAScheduleTupleID);
			printf("\tEVSEMaxCurrent=%d (%d %d)\n", exiOut.V2G_Message.Body.ChargingStatusRes.EVSEMaxCurrent.Value, exiOut.V2G_Message.Body.ChargingStatusRes.EVSEMaxCurrent.Unit, exiOut.V2G_Message.Body.ChargingStatusRes.EVSEMaxCurrent.Multiplier);
			printf("\tisused.MeterInfo=%d\n",		exiOut.V2G_Message.Body.ChargingStatusRes.MeterInfo_isUsed);
			printf("\t\tMeterInfo.MeterID=%d\n",		exiOut.V2G_Message.Body.ChargingStatusRes.MeterInfo.MeterID.characters[0]);
			printf("\t\tMeterInfo.MeterReading.Value=%li\n",		(long int)exiOut.V2G_Message.Body.ChargingStatusRes.MeterInfo.MeterReading);
			printf("\t\tMeterInfo.MeterStatus=%d\n",		exiOut.V2G_Message.Body.ChargingStatusRes.MeterInfo.MeterStatus);
			printf("\t\tMeterInfo.TMeter=%li\n",		(long int)exiOut.V2G_Message.Body.ChargingStatusRes.MeterInfo.TMeter);
			printf("\t\tMeterInfo.SigMeterReading.data=%d\n",		exiOut.V2G_Message.Body.ChargingStatusRes.MeterInfo.SigMeterReading.bytes[0]);*/
		} else {
			errn = ERROR_UNEXPECTED_CHARGING_STATUS_RESP_MESSAGE;
			return errn;
		}
	} else {
		return errn;
	}


	/***********************************
	 * SessionStop *
	 ***********************************/


	init_iso1BodyType(&exiIn.V2G_Message.Body);
	exiIn.V2G_Message.Body.SessionStopReq_isUsed = 1u;

	init_iso1SessionStopReqType(&exiIn.V2G_Message.Body.SessionStopReq);
	exiIn.V2G_Message.Body.SessionStopReq.ChargingSession = iso1chargingSessionType_Pause;

	printf("EV side: call EVSE stopSession \n");

	errn = request_response1(&exiIn, &exiOut);

	if(errn == 0) {
		/* check, if this is the right response message */
		if(exiOut.V2G_Message.Body.SessionStopRes_isUsed) {

			/* show results of EVSEs answer message */
			printf("EV side: received response message from EVSE\n");
			printf("\tHeader SessionID=");
			printBinaryArray(exiOut.V2G_Message.Header.SessionID.bytes, exiOut.V2G_Message.Header.SessionID.bytesLen);
			printf("\t ResponseCode=%d\n", exiOut.V2G_Message.Body.SessionStopRes.ResponseCode);

		} else {
			errn = ERROR_UNEXPECTED_SESSION_STOP_RESP_MESSAGE;
			return errn;
		}
	} else {
		return errn;
	}



	return errn;

}
#endif /* DEPLOY_ISO1_CODEC == SUPPORT_YES */




#if DEPLOY_DIN_CODEC == SUPPORT_YES
static int din_test() {
	int errn = 0;

	struct dinEXIDocument exiDin1;
	struct dinEXIDocument exiDin2;

	bitstream_t stream1;
	bitstream_t stream2;
	size_t pos1 = 0;
	size_t pos2 = 0;

	stream1.size = BUFFER_SIZE;
	stream1.data = buffer1;
	stream1.pos = &pos1;

	stream2.size = BUFFER_SIZE;
	stream2.data = buffer2;
	stream2.pos = &pos2;

	/* SetupSessionReq  */
	/* BMW: 80 9A 00 11 D0 20 00 03 C1 FC 30 00 43 F8 00 */
	buffer1[0] = 0x80;
	buffer1[1] = 0x9A;
	buffer1[2] = 0x00;
	buffer1[3] = 0x11;
	buffer1[4] = 0xD0;
	buffer1[5] = 0x20;
	buffer1[6] = 0x00;
	buffer1[7] = 0x03;
	buffer1[8] = 0xC1;
	buffer1[9] = 0xFC;
	buffer1[10] = 0x30;
	buffer1[11] = 0x00;
	buffer1[12] = 0x43;
	buffer1[13] = 0xF8;
	buffer1[14] = 0x00;

	errn =  decode_dinExiDocument(&stream1, &exiDin1);

	if(errn != 0) {
		printf("\n\nDIN test error %d!\n", errn);
		return errn;
	} else if (pos1 != 15) {
		printf("\n\nDIN warning. not all bytes read!\n");
		errn = -1;
		return errn;
	}


	/* SetupSessionReq  */
	/* Chevy: 80 9A 02 00 00 00 00 00 00 00 00 11 D0 18 00 60 8C 44 09 94 00 */
	buffer2[0] = 0x80;
	buffer2[1] = 0x9A;
	buffer2[2] = 0x02;
	buffer2[3] = 0x00;
	buffer2[4] = 0x00;
	buffer2[5] = 0x00;
	buffer2[6] = 0x00;
	buffer2[7] = 0x00;
	buffer2[8] = 0x00;
	buffer2[9] = 0x00;
	buffer2[10] = 0x00;
	buffer2[11] = 0x11;
	buffer2[12] = 0xD0;
	buffer2[13] = 0x18;
	buffer2[14] = 0x00;
	buffer2[15] = 0x60;
	buffer2[16] = 0x8C;
	buffer2[17] = 0x44;
	buffer2[18] = 0x09;
	buffer2[19] = 0x94;
	buffer2[20] = 0x00;

	errn =  decode_dinExiDocument(&stream2, &exiDin2);

	if(errn != 0) {
		printf("\n\nDIN test error %d!\n", errn);
		return errn;
	} else if (pos2 != 21) {
		printf("\n\nDIN warning. not all bytes read!\n");
		errn = -1;
		return errn;
	}


	/* sessionStopReq  */
	pos2 = 0; /* reset position */

	/* V: 0x80, 0x9a, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0x91, 0xf0  */
	buffer2[0] = 0x80;
	buffer2[1] = 0x9a;
	buffer2[2] = 0x02;
	buffer2[3] = 0x00;
	buffer2[4] = 0x00;
	buffer2[5] = 0x00;
	buffer2[6] = 0x00;
	buffer2[7] = 0x00;
	buffer2[8] = 0x00;
	buffer2[9] = 0x00;
	buffer2[10] = 0x03;
	buffer2[11] = 0x91;
	buffer2[12] = 0xf0;

	errn =  decode_dinExiDocument(&stream2, &exiDin2);

	if(errn != 0) {
		printf("\n\nDIN test error %d!\n", errn);
		return errn;
	} else if (pos2 != 13) {
		printf("\n\nDIN warning. not all bytes read!\n");
		errn = -1;
		return errn;
	} else if(exiDin2.V2G_Message_isUsed == 0 || exiDin2.V2G_Message.Body.SessionStopReq_isUsed == 0) {
		printf("\n\nDIN warning. no sessionStopReq message!\n");
		errn = -1;
		return errn;
	}


	if(errn == 0) {
		printf("DIN test passed\n");
	} else {
		printf("DIN test error %d!\n", errn);
	}

	return errn;
}
#endif /* DEPLOY_DIN_CODEC == SUPPORT_YES */



#if DEPLOY_XMLDSIG_CODEC == SUPPORT_YES
#if DEPLOY_ISO_CODEC_FRAGMENT == SUPPORT_YES
static int xmldsig_test() {
	int errn = 0, i;

	bitstream_t stream1;
	size_t pos1 = 0;
	stream1.size = BUFFER_SIZE;
	stream1.data = buffer1;
	stream1.pos = &pos1;

	bitstream_t stream2;
	size_t pos2 = 0;
	stream2.size = BUFFER_SIZE;
	stream2.data = buffer2;
	stream2.pos = &pos2;

	struct iso2EXIFragment exiV2G_AR;
	struct xmldsigEXIFragment exiXMLDSIG_SI;

	int sizeIsoStream1 = 25;
	int isoStream1[] = {0x80, 0x04, 0x01, 0x52, 0x51, 0x0C, 0x40, 0x82, 0x9B, 0x7B, 0x6B, 0x29, 0x02, 0x93, 0x0B, 0x73, 0x23, 0x7B, 0x69, 0x02, 0x23, 0x0B, 0xA3, 0x09, 0xE8};

	int sizeIsoStream2 = 209;
	int isoStream2[] = {0x80, 0x81, 0x12, 0xB4, 0x3A, 0x3A, 0x38, 0x1D, 0x17, 0x97, 0xBB, 0xBB, 0xBB, 0x97, 0x3B, 0x99, 0x97, 0x37, 0xB9, 0x33, 0x97, 0xAA, 0x29, 0x17, 0xB1, 0xB0, 0xB7, 0x37, 0xB7, 0x34, 0xB1, 0xB0, 0xB6, 0x16, 0xB2, 0xBC, 0x34, 0x97, 0xA1, 0xAB, 0x43, 0xA3, 0xA3, 0x81, 0xD1, 0x79, 0x7B, 0xBB, 0xBB, 0xB9, 0x73, 0xB9, 0x99, 0x73, 0x7B, 0x93, 0x39, 0x79, 0x91, 0x81, 0x81, 0x89, 0x79, 0x81, 0xA1, 0x7B, 0xC3, 0x6B, 0x63, 0x23, 0x9B, 0x4B, 0x39, 0x6B, 0x6B, 0x7B, 0x93, 0x29, 0x1B, 0x2B, 0x1B, 0x23, 0x9B, 0x09, 0x6B, 0x9B, 0x43, 0x09, 0x91, 0xA9, 0xB2, 0x20, 0x62, 0x34, 0x94, 0x43, 0x10, 0x25, 0x68, 0x74, 0x74, 0x70, 0x3A, 0x2F, 0x2F, 0x77, 0x77, 0x77, 0x2E, 0x77, 0x33, 0x2E, 0x6F, 0x72, 0x67, 0x2F, 0x54, 0x52, 0x2F, 0x63, 0x61, 0x6E, 0x6F, 0x6E, 0x69, 0x63, 0x61, 0x6C, 0x2D, 0x65, 0x78, 0x69, 0x2F, 0x48, 0x52, 0xD0, 0xE8, 0xE8, 0xE0, 0x74, 0x5E, 0x5E, 0xEE, 0xEE, 0xEE, 0x5C, 0xEE, 0x66, 0x5C, 0xDE, 0xE4, 0xCE, 0x5E, 0x64, 0x60, 0x60, 0x62, 0x5E, 0x60, 0x68, 0x5E, 0xF0, 0xDA, 0xD8, 0xCA, 0xDC, 0xC6, 0x46, 0xE6, 0xD0, 0xC2, 0x64, 0x6A, 0x6C, 0x84, 0x1A, 0x36, 0xBC, 0x07, 0xA0, 0x0C, 0xB7, 0xDC, 0xAD, 0x66, 0x2F, 0x30, 0x88, 0xA6, 0x0A, 0x3D, 0x6A, 0x99, 0x43, 0x1F, 0x81, 0xC1, 0x22, 0xC2, 0xE9, 0xF1, 0x67, 0x8E, 0xF5, 0x31, 0xE9, 0x55, 0x23, 0x70};


	uint8_t digestValue[] = {0xD1, 0xB5, 0xE0, 0x3D, 0x00, 0x65, 0xBE, 0xE5, 0x6B, 0x31, 0x79, 0x84, 0x45, 0x30, 0x51, 0xEB, 0x54, 0xCA, 0x18, 0xFC, 0x0E, 0x09, 0x16, 0x17, 0x4F, 0x8B, 0x3C, 0x77, 0xA9, 0x8F, 0x4A, 0xA9}; /* 32 Bytes */


	/*
	<v2gci_b:AuthorizationReq xmlns:v2gci_b="urn:iso:15118:2:2013:MsgBody" v2gci_b:Id="ID1">
	    <v2gci_b:GenChallenge>U29tZSBSYW5kb20gRGF0YQ==</v2gci_b:GenChallenge>
	</v2gci_b:AuthorizationReq>
	*/

	init_iso2EXIFragment(&exiV2G_AR);
	exiV2G_AR.AuthorizationReq_isUsed = 1u;
	init_iso2AuthorizationReqType(&exiV2G_AR.AuthorizationReq);
	exiV2G_AR.AuthorizationReq.Id_isUsed = 1;
	exiV2G_AR.AuthorizationReq.Id.charactersLen = 3;
	exiV2G_AR.AuthorizationReq.Id.characters[0] = 'I';
	exiV2G_AR.AuthorizationReq.Id.characters[1] = 'D';
	exiV2G_AR.AuthorizationReq.Id.characters[2] = '1';
	exiV2G_AR.AuthorizationReq.GenChallenge_isUsed = 1;
	/* base64 U29tZSBSYW5kb20gRGF0YQ== */
	exiV2G_AR.AuthorizationReq.GenChallenge.bytesLen = 16;
	exiV2G_AR.AuthorizationReq.GenChallenge.bytes[0] = 0x53;
	exiV2G_AR.AuthorizationReq.GenChallenge.bytes[1] = 0x6F;
	exiV2G_AR.AuthorizationReq.GenChallenge.bytes[2] = 0x6D;
	exiV2G_AR.AuthorizationReq.GenChallenge.bytes[3] = 0x65;
	exiV2G_AR.AuthorizationReq.GenChallenge.bytes[4] = 0x20;
	exiV2G_AR.AuthorizationReq.GenChallenge.bytes[5] = 0x52;
	exiV2G_AR.AuthorizationReq.GenChallenge.bytes[6] = 0x61;
	exiV2G_AR.AuthorizationReq.GenChallenge.bytes[7] = 0x6E;
	exiV2G_AR.AuthorizationReq.GenChallenge.bytes[8] = 0x64;
	exiV2G_AR.AuthorizationReq.GenChallenge.bytes[9] = 0x6F;
	exiV2G_AR.AuthorizationReq.GenChallenge.bytes[10] = 0x6D;
	exiV2G_AR.AuthorizationReq.GenChallenge.bytes[11] = 0x20;
	exiV2G_AR.AuthorizationReq.GenChallenge.bytes[12] = 0x44;
	exiV2G_AR.AuthorizationReq.GenChallenge.bytes[13] = 0x61;
	exiV2G_AR.AuthorizationReq.GenChallenge.bytes[14] = 0x74;
	exiV2G_AR.AuthorizationReq.GenChallenge.bytes[15] = 0x61;

	/* encode fragment with ISO schema */
	errn = encode_iso2ExiFragment(&stream1, &exiV2G_AR);

	if((*stream1.pos) != sizeIsoStream1) {
		errn = -1;
		printf("EXI1 stream length does not match !\n");
		return errn;
	} else {
		for(i=0; i<sizeIsoStream1; i++) {
			if(stream1.data[i] != isoStream1[i]) {
				errn = -1;
				printf("EXI1 stream does not match at position %d !\n", i);
				return errn;
			}
		}
	}

	/* TODO Create Hash for stream 1 etc ... */
	/* SHA-256 is "0bXgPQBlvuVrMXmERTBR61TKGPwOCRYXT4s8d6mPSqk=" */

	/*
	<xmlsig:SignedInfo xmlns:xmlsig="http://www.w3.org/2000/09/xmldsig#" >
	    <xmlsig:CanonicalizationMethod Algorithm="http://www.w3.org/TR/canonical-exi/"/>
	    <xmlsig:SignatureMethod
	        Algorithm="http://www.w3.org/2001/04/xmldsig-more#ecdsa-sha256"/>
	    <xmlsig:Reference URI="#ID1">
	        <xmlsig:Transforms>
	            <xmlsig:Transform Algorithm="http://www.w3.org/TR/canonical-exi/"/>
	        </xmlsig:Transforms>
	        <xmlsig:DigestMethod Algorithm="http://www.w3.org/2001/04/xmlenc#sha256"/>
	        <xmlsig:DigestValue>0bXgPQBlvuVrMXmERTBR61TK
	            GPwOCRYXT4s8d6mPSqk=</xmlsig:DigestValue>
	    </xmlsig:Reference>
	</xmlsig:SignedInfo>
	*/

	/* encode SignedInfo element with xmldsig schema */

	const char arrayCanonicalEXI[35] = {"http://www.w3.org/TR/canonical-exi/"};
	const char arrayxmldsigSHA256[51] = {"http://www.w3.org/2001/04/xmldsig-more#ecdsa-sha256"};
	const char arrayxmlencSHA256[39] = {"http://www.w3.org/2001/04/xmlenc#sha256"};

	init_xmldsigEXIFragment(&exiXMLDSIG_SI);
	exiXMLDSIG_SI.SignedInfo_isUsed = 1;
	init_xmldsigSignedInfoType(&exiXMLDSIG_SI.SignedInfo);
	{
		init_xmldsigCanonicalizationMethodType(&exiXMLDSIG_SI.SignedInfo.CanonicalizationMethod);
		exiXMLDSIG_SI.SignedInfo.CanonicalizationMethod.Algorithm.charactersLen = 35;
		strncpy(exiXMLDSIG_SI.SignedInfo.CanonicalizationMethod.Algorithm.characters, arrayCanonicalEXI, 35);

		exiXMLDSIG_SI.SignedInfo.SignatureMethod.HMACOutputLength_isUsed = 0;
		exiXMLDSIG_SI.SignedInfo.SignatureMethod.Algorithm.charactersLen = 51;
		strncpy(exiXMLDSIG_SI.SignedInfo.SignatureMethod.Algorithm.characters, arrayxmldsigSHA256, 51);

		exiXMLDSIG_SI.SignedInfo.Reference.arrayLen = 1;
		 /* "#ID1" */
		exiXMLDSIG_SI.SignedInfo.Reference.array[0].URI_isUsed = 1;
		exiXMLDSIG_SI.SignedInfo.Reference.array[0].URI.charactersLen = 4;
		exiXMLDSIG_SI.SignedInfo.Reference.array[0].URI.characters[0] = '#';
		exiXMLDSIG_SI.SignedInfo.Reference.array[0].URI.characters[1] = 'I';
		exiXMLDSIG_SI.SignedInfo.Reference.array[0].URI.characters[2] = 'D';
		exiXMLDSIG_SI.SignedInfo.Reference.array[0].URI.characters[3] = '1';
		/* "http://www.w3.org/TR/canonical-exi/" */
		exiXMLDSIG_SI.SignedInfo.Reference.array[0].Transforms_isUsed = 1;
		exiXMLDSIG_SI.SignedInfo.Reference.array[0].Transforms.Transform.arrayLen = 1;
		exiXMLDSIG_SI.SignedInfo.Reference.array[0].Transforms.Transform.array[0].Algorithm.charactersLen = 35;
		strncpy(exiXMLDSIG_SI.SignedInfo.Reference.array[0].Transforms.Transform.array[0].Algorithm.characters, arrayCanonicalEXI, 35); /* Will copy 35 characters from arrayCanonicalEXI to characters */
		exiXMLDSIG_SI.SignedInfo.Reference.array[0].Transforms.Transform.array[0].XPath.arrayLen = 0;

		exiXMLDSIG_SI.SignedInfo.Reference.array[0].DigestMethod.Algorithm.charactersLen = 39;
		strncpy(exiXMLDSIG_SI.SignedInfo.Reference.array[0].DigestMethod.Algorithm.characters, arrayxmlencSHA256, 39);

		/* "0bXgPQBlvuVrMXmERTBR61TKGPwOCRYXT4s8d6mPSqk=" --> 16 Bytes 536F6D652052616E646F6D2044617461 */
		exiXMLDSIG_SI.SignedInfo.Reference.array[0].DigestValue.bytesLen = 32;
		memcpy(exiXMLDSIG_SI.SignedInfo.Reference.array[0].DigestValue.bytes, digestValue, 32);
	}

	errn = encode_xmldsigExiFragment(&stream2, &exiXMLDSIG_SI);

	if((*stream2.pos) != sizeIsoStream2) {
		errn = -1;
		printf("EXI2 stream length does not match !\n");
		return errn;
	} else {
		for(i=0; i<sizeIsoStream2; i++) {
			if(stream2.data[i] != isoStream2[i]) {
				errn = -1;
				printf("EXI2 stream does not match at position %d !\n", i);
				return errn;
			}
		}
	}


	if(errn == 0) {
		printf("XMLDSIG test passed\n");
	} else {
		printf("XMLDSIG test error %d!\n", errn);
	}

	return errn;
}
#endif /* DEPLOY_ISO_CODEC_FRAGMENT */
#endif /* DEPLOY_XMLDSIG_CODEC == SUPPORT_YES */


int main_example(int argc, char *argv[]) {
	int errn = 0;

	printf("+++ Start application handshake protocol example +++\n\n");
	errn = appHandshake();
	printf("+++ Terminate application handshake protocol example with errn = %d +++\n\n", errn);

	if(errn != 0) {
		printf("\n\nHandshake error %d!\n", errn);
		return errn;
	}


#if DEPLOY_ISO1_CODEC == SUPPORT_YES
	printf("+++ Start V2G client / service example for charging (ISO1) +++\n\n");
	errn = charging1();
	printf("\n+++Terminate V2G Client / Service example for charging with errn = %d +++\n\n", errn);
	if(errn != 0) {
		printf("\n\ncharging error %d!\n", errn);
		return errn;
	}
#endif /* DEPLOY_ISO1_CODEC == SUPPORT_YES */


#if DEPLOY_ISO2_CODEC == SUPPORT_YES
	printf("+++ Start V2G client / service example for charging (ISO2) +++\n\n");
	errn = charging2();
	printf("\n+++Terminate V2G Client / Service example for charging with errn = %d +++\n\n", errn);
	if(errn != 0) {
		printf("\n\ncharging error %d!\n", errn);
		return errn;
	}
#endif /* DEPLOY_ISO2_CODEC == SUPPORT_YES */


#if DEPLOY_DIN_CODEC == SUPPORT_YES
	printf("+++ Start simple DIN test +++\n");
	errn = din_test();
	printf("+++ Terminate simple DIN test with errn = %d +++\n\n", errn);
	if(errn != 0) {
		printf("\nDIN test error %d!\n", errn);
		return errn;
	}
#endif /* DEPLOY_DIN_CODEC == SUPPORT_YES */

#if DEPLOY_XMLDSIG_CODEC == SUPPORT_YES
#if DEPLOY_ISO_CODEC_FRAGMENT == SUPPORT_YES
	printf("+++ Start simple XMLDSIG test +++\n");
	errn = xmldsig_test();
	printf("+++ Terminate simple XMLDSIG test with errn = %d +++\n\n", errn);
	if(errn != 0) {
		printf("\nXMLDSIG test error %d!\n", errn);
		return errn;
	}
#endif /* DEPLOY_ISO_CODEC_FRAGMENT */
#endif /* DEPLOY_XMLDSIG_CODEC == SUPPORT_YES */


	return errn;
}


