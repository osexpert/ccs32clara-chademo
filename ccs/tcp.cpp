#include "ccs32_globals.h"

/* Todo: implement a retry strategy, to cover the situation that single packets are lost on the way. */

#define NEXT_TCP 0x06 /* the next protocol is TCP */

#define TCP_FLAG_FIN 0x01
#define TCP_FLAG_SYN 0x02
#define TCP_FLAG_RST 0x04
#define TCP_FLAG_PSH 0x08
#define TCP_FLAG_ACK 0x10
#define TCP_TRANSMIT_PACKET_LEN 200

//#define TCP_ACK_TIMEOUT_MS 100 /* if for 100ms no ACK is received, we retry the transmission */
//A safer setting is TCP_ACK_TIMEOUT_MS 1000 with 3–5 retries using exponential backoff, aligning with CCS’s ISO 15118 requirements and typical network conditions.
#define TCP_ACK_TIMEOUT_MS 500
//Total timeout : ~7.5 seconds(0.5s + 1s + 2s + 4s), suitable for ISO 15118.
//todo: exponential backoff?
#define TCP_MAX_NUMBER_OF_RETRANSMISSIONS 8 //40 /* allow 40 retries with 100ms cycle, to bridge 4 seconds of broken line. */

#define TCP_ACTIVITY_TIMER_START (5*33) /* 5 seconds */

#define TCP_STATE_CLOSED 0
#define TCP_STATE_SYN_SENT 1
#define TCP_STATE_ESTABLISHED 2

static uint16_t tcpActivityTimer;
static uint32_t lastUnackTransmissionTime = 0;
static uint8_t retryCounter = 0;
static uint8_t TcpIpRequestLen;
static uint8_t* TcpIpRequest = &myethtransmitbuffer[14];
static uint8_t TcpTransmitPacketLen;
static uint8_t* TcpTransmitPacket = &TcpIpRequest[40];

uint8_t tcpHeaderLen;
uint8_t tcpPayloadLen;
uint8_t* tcpPayload = &TcpTransmitPacket[20];
uint8_t tcp_rxdataLen=0;
uint8_t tcp_rxdata[TCP_RX_DATA_LEN]; /* dedicated receive buffer for TCP data with correct ports */

static uint8_t tcpState = TCP_STATE_CLOSED;

static uint32_t TcpSeqNr=200; /* a "random" start sequence number */
static uint32_t TcpAckNr;
static uint32_t tcp_debug_totalRetryCounter;

//static uint16_t highestWrongPort = 0;

/*** local function prototypes ****************************************************/

static void tcp_packRequestIntoEthernet(void);
static void tcp_packRequestIntoIp(void);
static void tcp_prepareTcpHeader(uint8_t tcpFlag);
static void tcp_sendAck(void);
static void tcp_sendFirstAck(void);
//static void tcp_sendReset_reply(void);

/*** functions *********************************************************************/
uint32_t tcp_getTotalNumberOfRetries(void) {
  return tcp_debug_totalRetryCounter;
}

void evaluateTcpPacket(void)
{
   uint8_t flags;
   uint32_t remoteSeqNr;
   uint32_t remoteAckNr;
   uint16_t sourcePort, destinationPort, pLen, hdrLen, tmpPayloadLen;

   /* todo: check the IP addresses, checksum etc */
   nTcpPacketsReceived++;

   pLen =  (((uint16_t)myethreceivebuffer[18])<<8) +  myethreceivebuffer[19]; /* length of the IP payload */
   hdrLen=(myethreceivebuffer[66]>>4) * 4; /* header length in byte */

   //log_v("pLen=%d, hdrLen=%d", pLen, hdrLen);
   if (pLen>=hdrLen)
   {
      tmpPayloadLen = pLen - hdrLen;
   }
   else
   {
      tmpPayloadLen = 0; /* no TCP payload data */
   }

   sourcePort =      (((uint16_t)myethreceivebuffer[54])<<8) +  myethreceivebuffer[55];
   destinationPort = (((uint16_t)myethreceivebuffer[56])<<8) +  myethreceivebuffer[57];
   if ((sourcePort != seccTcpPort) || (destinationPort != evccPort))
   {
       // At my Tesla v2, when coming back days later after a previous (failed) attempt, it give we answers to messages I sent way back when, ports higher than I am currently at.
       // Seems they implemented tcp with a q per mac? Anyways...since we reset now, this may help to clear the q.
       // edit: Yes, it seems to help. I did not get reset of wrong ported packages to work thou, but should be possible too.
       // But also...to reduce overlapping/conflicting/reuse of ports, use higher ports than the one we got.

//       if (destinationPort > highestWrongPort)
  //         highestWrongPort = destinationPort;

      //addToTrace(MOD_TCP, "[TCP] wrong port.");
      printf("[TCP] wrong port src:%d/%d dst:%d/%d\r\n", sourcePort, seccTcpPort, destinationPort, evccPort);

//      tcp_sendReset_reply();

      return; /* wrong port */
   }

   // Parse sequence and ACK numbers
   remoteSeqNr =
      (((uint32_t)myethreceivebuffer[58])<<24) +
      (((uint32_t)myethreceivebuffer[59])<<16) +
      (((uint32_t)myethreceivebuffer[60])<<8) +
      (((uint32_t)myethreceivebuffer[61]));
   remoteAckNr =
      (((uint32_t)myethreceivebuffer[62])<<24) +
      (((uint32_t)myethreceivebuffer[63])<<16) +
      (((uint32_t)myethreceivebuffer[64])<<8) +
      (((uint32_t)myethreceivebuffer[65]));
   flags = myethreceivebuffer[67];

   // Handle SYN+ACK (connection setup)
   if ((flags & (TCP_FLAG_SYN | TCP_FLAG_ACK)) == (TCP_FLAG_SYN | TCP_FLAG_ACK))   /* This is the connection setup response from the server. */
   {
      if (tcpState == TCP_STATE_SYN_SENT)
      {
         TcpSeqNr = remoteAckNr; /* The sequence number of our next transmit packet is given by the received ACK number. */
         TcpAckNr = remoteSeqNr+1; /* The ACK number of our next transmit packet is one more than the received seq number. */
         setCheckpoint(303);
         tcpState = TCP_STATE_ESTABLISHED;
         tcp_sendFirstAck();
         connMgr_TcpOk();
         addToTrace(MOD_TCP, "[TCP] connected.");
      }
      return;
   }

   /* It is no connection setup. We can have the following situations here: */
   if (tcpState != TCP_STATE_ESTABLISHED)
   {
      /* received something while the connection is closed. Just ignore it. */
      addToTrace(MOD_TCP, "[TCP] ignore, not connected.");
      return;
   }

   // seems unused?
   tcpActivityTimer = TCP_ACTIVITY_TIMER_START;

   /* It can be an ACK, or a data package, or a combination of both. We treat the ACK and the data independent from each other,
     to treat each combination. */
   //log_v("L=%d", tmpPayloadLen);
   if ((tmpPayloadLen>0) && (tmpPayloadLen<TCP_RX_DATA_LEN))
   {
      /* This is a data transfer packet. */
      tcp_rxdataLen = tmpPayloadLen;
      /* myethreceivebuffer[74] is the first payload byte. */
      /* Fix for https://github.com/uhi22/ccs32clara/issues/15. We explicitely need to copy the data here,
      because the application will look asynchronously on the tcp_rxdata, and if in between some other received
      data would end up in the myethreceivebuffer (e.g. neighbour solicitation, or TCP traffic with other ports),
      the overwritten myethreceivebuffer would lead to not seeing the tcp_rxdata anymore, if this would be just
      a pointer to myethreceivebuffer[74]. */
      memcpy(tcp_rxdata, &myethreceivebuffer[74], tcp_rxdataLen);  /* provide the received data to the application */
      connMgr_TcpOk();
      TcpAckNr = remoteSeqNr+tcp_rxdataLen; /* The ACK number of our next transmit packet is tcp_rxdataLen more than the received seq number. */
      tcp_sendAck();

      addToTrace(MOD_TCPTRAFFIC, "Data received: ", tcp_rxdata, tcp_rxdataLen);
   }

   // Handle ACK
   if (flags & TCP_FLAG_ACK)
   {
      nTcpPacketsReceived+=1000;
      TcpSeqNr = remoteAckNr; /* The sequence number of our next transmit packet is given by the received ACK number. */
      lastUnackTransmissionTime = 0; /* mark timer as "not used", because we received the ACK */
      retryCounter = 0;
   }

   // Handle RST
   if (flags & TCP_FLAG_RST)
   {
       addToTrace(MOD_TCP, "[TCP] RST received, closing connection");
       tcpState = TCP_STATE_CLOSED;
   }

   // Handle FIN
   if (flags & TCP_FLAG_FIN)
   {
       addToTrace(MOD_TCP, "[TCP] FIN received, sending final ACK");
       TcpAckNr = remoteSeqNr + 1;
       tcp_sendAck();
       tcpState = TCP_STATE_CLOSED;
   }
}

void tcp_connect(void)
{
   addToTrace(MOD_TCP, "[TCP] Checkpoint301: connecting");
   setCheckpoint(301);
   TcpTransmitPacket[20] = 0x02; /* options: 12 bytes, just copied from the Win10 notebook trace */
   TcpTransmitPacket[21] = 0x04;
   TcpTransmitPacket[22] = 0x05;
   TcpTransmitPacket[23] = 0xA0;

   TcpTransmitPacket[24] = 0x01;
   TcpTransmitPacket[25] = 0x03;
   TcpTransmitPacket[26] = 0x03;
   TcpTransmitPacket[27] = 0x08;

   TcpTransmitPacket[28] = 0x01;
   TcpTransmitPacket[29] = 0x01;
   TcpTransmitPacket[30] = 0x04;
   TcpTransmitPacket[31] = 0x02;

   tcpHeaderLen = 32; /* 20 bytes normal header, plus 12 bytes options */
   tcpPayloadLen = 0;   /* only the TCP header, no data is in the connect message. */
   tcp_prepareTcpHeader(TCP_FLAG_SYN);
   tcp_packRequestIntoIp();
   tcpState = TCP_STATE_SYN_SENT;
   tcpActivityTimer=TCP_ACTIVITY_TIMER_START;
}

static void tcp_sendFirstAck(void)
{
   addToTrace(MOD_TCP, "[TCP] sending first ACK");
   tcpHeaderLen = 20; /* 20 bytes normal header, no options */
   tcpPayloadLen = 0;   /* only the TCP header, no data is in the first ACK message. */
   tcp_prepareTcpHeader(TCP_FLAG_ACK);
   tcp_packRequestIntoIp();
}

static void tcp_sendAck(void)
{
   addToTrace(MOD_TCP, "[TCP] sending ACK");
   tcpHeaderLen = 20; /* 20 bytes normal header, no options */
   tcpPayloadLen = 0;   /* only the TCP header, no data is in the first ACK message. */
   tcp_prepareTcpHeader(TCP_FLAG_ACK);
   tcp_packRequestIntoIp();
}

void tcp_transmit(void)
{
   if (tcpState == TCP_STATE_ESTABLISHED)
   {
      //addToTrace("[TCP] sending data");
      tcpHeaderLen = 20; /* 20 bytes normal header, no options */
      if (tcpPayloadLen+tcpHeaderLen<TCP_TRANSMIT_PACKET_LEN)
      {
          /* The packet fits into our transmit buffer. */
          addToTrace(MOD_TCPTRAFFIC, "TCP will transmit:", tcpPayload, tcpPayloadLen);
          tcp_prepareTcpHeader(TCP_FLAG_PSH + TCP_FLAG_ACK); /* data packets are always sent with flags PUSH and ACK. */
          tcp_packRequestIntoIp();
          lastUnackTransmissionTime = rtc_get_ms(); /* record the time of transmission, to be able to detect the timeout */
          retryCounter = TCP_MAX_NUMBER_OF_RETRANSMISSIONS; /* Allow n retries of the same packet */
      }
      else
      {
         addToTrace(MOD_TCP, "Error: tcpPayload and header do not fit into TcpTransmitPacket.");
      }
   }
}


void tcp_testSendData(void)
{
   if (tcpState == TCP_STATE_ESTABLISHED)
   {
      addToTrace(MOD_TCP, "[TCP] sending data");
      tcpHeaderLen = 20; /* 20 bytes normal header, no options */
      tcpPayloadLen = 3;   /* demo length */
      TcpTransmitPacket[tcpHeaderLen] = 0x55; /* demo data */
      TcpTransmitPacket[tcpHeaderLen+1] = 0xAA; /* demo data */
      TcpTransmitPacket[tcpHeaderLen+2] = 0xBB; /* demo data */
      tcp_prepareTcpHeader(TCP_FLAG_PSH + TCP_FLAG_ACK); /* data packets are always sent with flags PUSH and ACK. */
      tcp_packRequestIntoIp();
   }
}


static void tcp_prepareTcpHeader(uint8_t tcpFlag)
{
   uint16_t checksum;

   // # TCP header needs at least 24 bytes:
   // 2 bytes source port
   // 2 bytes destination port
   // 4 bytes sequence number
   // 4 bytes ack number
   // 4 bytes DO/RES/Flags/Windowsize
   // 2 bytes checksum
   // 2 bytes urgentPointer
   // n*4 bytes options/fill (empty for the ACK frame and payload frames)
   TcpTransmitPacket[0] = (uint8_t)(evccPort >> 8); /* source port */
   TcpTransmitPacket[1] = (uint8_t)(evccPort);
   TcpTransmitPacket[2] = (uint8_t)(seccTcpPort >> 8); /* destination port */
   TcpTransmitPacket[3] = (uint8_t)(seccTcpPort);

   TcpTransmitPacket[4] = (uint8_t)(TcpSeqNr>>24); /* sequence number */
   TcpTransmitPacket[5] = (uint8_t)(TcpSeqNr>>16);
   TcpTransmitPacket[6] = (uint8_t)(TcpSeqNr>>8);
   TcpTransmitPacket[7] = (uint8_t)(TcpSeqNr);

   TcpTransmitPacket[8] = (uint8_t)(TcpAckNr>>24); /* ack number */
   TcpTransmitPacket[9] = (uint8_t)(TcpAckNr>>16);
   TcpTransmitPacket[10] = (uint8_t)(TcpAckNr>>8);
   TcpTransmitPacket[11] = (uint8_t)(TcpAckNr);

   TcpTransmitPacketLen = tcpHeaderLen + tcpPayloadLen;
   TcpTransmitPacket[12] = (tcpHeaderLen/4) << 4; /* High-nibble: DataOffset in 4-byte-steps. Low-nibble: Reserved=0. */

   TcpTransmitPacket[13] = tcpFlag;
#define TCP_RECEIVE_WINDOW 1000 /* number of octetts we are able to receive */
   TcpTransmitPacket[14] = (uint8_t)(TCP_RECEIVE_WINDOW>>8);
   TcpTransmitPacket[15] = (uint8_t)(TCP_RECEIVE_WINDOW);

   // checksum will be calculated afterwards
   TcpTransmitPacket[16] = 0;
   TcpTransmitPacket[17] = 0;

   TcpTransmitPacket[18] = 0; /* 16 bit urgentPointer. Always zero in our case. */
   TcpTransmitPacket[19] = 0;

   checksum = calculateUdpAndTcpChecksumForIPv6(TcpTransmitPacket, TcpTransmitPacketLen, EvccIp, SeccIp, NEXT_TCP);
   TcpTransmitPacket[16] = (uint8_t)(checksum >> 8);
   TcpTransmitPacket[17] = (uint8_t)(checksum);
}


static void tcp_packRequestIntoIp(void)
{
   // # embeds the TCP into the lower-layer-protocol: IP, Ethernet
   uint8_t i;
   uint16_t plen;
   TcpIpRequestLen = TcpTransmitPacketLen + 8 + 16 + 16; // # IP6 header needs 40 bytes:
   //  #   4 bytes traffic class, flow
   //  #   2 bytes destination port
   //  #   2 bytes length (incl checksum)
   //  #   2 bytes checksum
   TcpIpRequest[0] = 0x60; // # traffic class, flow
   TcpIpRequest[1] = 0;
   TcpIpRequest[2] = 0;
   TcpIpRequest[3] = 0;
   plen = TcpTransmitPacketLen; // length of the payload. Without headers.
   TcpIpRequest[4] = plen >> 8;
   TcpIpRequest[5] = plen & 0xFF;
   TcpIpRequest[6] = NEXT_TCP; // next level protocol, 0x06 = TCP in this case
   TcpIpRequest[7] = 0x0A; // hop limit
   // We are the PEV. So the EvccIp is our own link-local IP address.
   //EvccIp = addressManager_getLinkLocalIpv6Address("bytearray");
   for (i=0; i<16; i++)
   {
      TcpIpRequest[8+i] = EvccIp[i]; // source IP address
   }
   for (i=0; i<16; i++)
   {
      TcpIpRequest[24+i] = SeccIp[i]; // destination IP address
   }
   //showAsHex(TcpIpRequest, TcpIpRequestLen, "TcpIpRequest");
   tcp_packRequestIntoEthernet();
}

static void tcp_packRequestIntoEthernet(void)
{
   //# packs the IP packet into an ethernet packet
   myethtransmitbufferLen = TcpIpRequestLen + 6 + 6 + 2; // # Ethernet header needs 14 bytes:
   // #  6 bytes destination MAC
   // #  6 bytes source MAC
   // #  2 bytes EtherType
   //# fill the destination MAC with the MAC of the charger
   fillDestinationMac(evseMac, 0);
   fillSourceMac(getOurMac(), 6); // bytes 6 to 11 are the source MAC
   myethtransmitbuffer[12] = 0x86; // # 86dd is IPv6
   myethtransmitbuffer[13] = 0xdd;
   myEthTransmit();
}



//void tcp_Disconnect(void)
//{
//   /* we should normally use the FIN handshake, to tell the charger that we closed the connection.
//   But for the moment, just go away silently, and use an other port for the next connection. The
//   server will detect our absense sooner or later by timeout, this should be good enough. */
//   tcpState = TCP_STATE_CLOSED;
//   /* use a new port */
//   /* But: This causes multiple open connections and the Win10 messes-up them. */
//   //evccPort++;
//   //if (evccPort>65000) evccPort=60000;
//}

bool tcp_isClosed(void)
{
   return (tcpState == TCP_STATE_CLOSED);
}

bool tcp_isConnected(void)
{
   return (tcpState == TCP_STATE_ESTABLISHED);
}

static void tcp_sendReset(void)
{
    if (tcpState != TCP_STATE_CLOSED)
    {
        printf("[TCP] sending RESET port %d\r\n", evccPort);
        tcpHeaderLen = 20;  // no options
        tcpPayloadLen = 0;  // no payload
        TcpAckNr = 0; // not acknowledging any data
        tcp_prepareTcpHeader(TCP_FLAG_RST);
        tcp_packRequestIntoIp();
        tcpState = TCP_STATE_CLOSED;  // close connection state
    }
}

void tcp_disconnect(void)
{
    tcp_sendReset();
    //if (tcpState == TCP_STATE_ESTABLISHED)
    //{
    //    addToTrace(MOD_TCP, "[TCP] sending FIN");
    //    tcpHeaderLen = 20;
    //    tcpPayloadLen = 0;
    //    tcp_prepareTcpHeader(TCP_FLAG_FIN | TCP_FLAG_ACK);
    //    tcp_packRequestIntoIp();
    //    TcpSeqNr++; // FIN consumes one sequence number
    //    tcpState = TCP_STATE_CLOSED;
    //}
}

void tcp_Mainfunction(void)
{
   if ((lastUnackTransmissionTime > 0) && ((rtc_get_ms() - lastUnackTransmissionTime) > TCP_ACK_TIMEOUT_MS))
   {
      if (retryCounter>0) {
        /* The maximum number of retransmissions are not yet over. We still are allowed to retransmit. */
        //Retransmit
        tcp_packRequestIntoEthernet();
        tcp_debug_totalRetryCounter++;
        lastUnackTransmissionTime = rtc_get_ms(); /* record the time of transmission, to be able to detect the timeout */
        retryCounter--;
        addToTrace(MOD_TCP, "[TCP] Last packet wasn't ACKed for 500 ms, retransmitting");
      } else {
        addToTrace(MOD_TCP, "[TCP] Giving up the retry");
        //tcp_sendReset();
      }
   }
   if (connMgr_getConnectionLevel()<50)
   {
      /* No SDP done. Means: It does not make sense to start or continue TCP. */
      lastUnackTransmissionTime = 0;
      //tcpState = TCP_STATE_CLOSED;
      tcp_sendReset();
      return;
   }
   if ((connMgr_getConnectionLevel()==50) && (tcpState == TCP_STATE_CLOSED))
   {
      /* SDP is finished, but no TCP connected yet. */
      /* use a new port */

      //if (evccPort < highestWrongPort)
      //  evccPort = highestWrongPort + 10; // try to get some distance (+10) from the wrongness
      //else

      evccPort++;

      if (evccPort>65000) evccPort=60000;
      printf("[TCP] Connect with port %d\r\n", evccPort);
      tcp_connect();
   }
}

// need to swap ips etc. for this to work..
//static void tcp_sendReset_reply(void)
//{
//    // Swap ports
//    TcpTransmitPacket[0] = myethreceivebuffer[36]; // dest port -> src
//    TcpTransmitPacket[1] = myethreceivebuffer[37];
//    TcpTransmitPacket[2] = myethreceivebuffer[34]; // src port -> dest
//    TcpTransmitPacket[3] = myethreceivebuffer[35];
//
//    // Sequence number = incoming packet's ACK number
//    TcpTransmitPacket[4] = myethreceivebuffer[42];
//    TcpTransmitPacket[5] = myethreceivebuffer[43];
//    TcpTransmitPacket[6] = myethreceivebuffer[44];
//    TcpTransmitPacket[7] = myethreceivebuffer[45];
//
//    // No ACK number (zeros)
//    TcpTransmitPacket[8] = 0x00;
//    TcpTransmitPacket[9] = 0x00;
//    TcpTransmitPacket[10] = 0x00;
//    TcpTransmitPacket[11] = 0x00;
//
//    TcpTransmitPacket[12] = 0x50; // Data offset = 5 (20 bytes), no options
//    TcpTransmitPacket[13] = TCP_FLAG_RST; // RST flag only, no ACK
//
//    TcpTransmitPacket[14] = 0x00; // Window size
//    TcpTransmitPacket[15] = 0x00;
//
//    TcpTransmitPacket[16] = 0x00; // checksum (calculated later)
//    TcpTransmitPacket[17] = 0x00;
//
//    TcpTransmitPacket[18] = 0x00; // urgent pointer
//    TcpTransmitPacket[19] = 0x00;
//
//    checksum = calculateUdpAndTcpChecksumForIPv6(TcpTransmitPacket, TcpTransmitPacketLen, EvccIp, SeccIp, NEXT_TCP);
//    TcpTransmitPacket[16] = (uint8_t)(checksum >> 8);
//    TcpTransmitPacket[17] = (uint8_t)(checksum);
//
//    tcpHeaderLen = 20;
//    tcpPayloadLen = 0;
//    tcp_packRequestIntoIp();
//}
//
