#include "ccs32_globals.h"

#define NEXT_TCP 0x06 /* the next protocol is TCP */

#define TCP_FLAG_FIN 0x01 // "I am done sending data, but can still recieve"
#define TCP_FLAG_SYN 0x02
#define TCP_FLAG_RST 0x04
#define TCP_FLAG_PSH 0x08
#define TCP_FLAG_ACK 0x10

#define TCP_TRANSMIT_PACKET_LEN 200

#define TCP_STATE_CLOSED 0
#define TCP_STATE_SYN_SENT 1
#define TCP_STATE_ESTABLISHED 2

#define TCP_ACK_INITIAL_TIMEOUT_MS 100     // Start with 100ms
#define TCP_ACK_MAX_TIMEOUT_MS     800     // Max per-retry delay
#define TCP_MAX_TOTAL_RETRY_TIME_MS 4000   // Retry attempts within 4s

#define CLIENT_MIN_PORT 49152
#define CLIENT_MAX_PORT 65535

static uint32_t nextRetryTime = 0;
static uint32_t retryDelay = TCP_ACK_INITIAL_TIMEOUT_MS;
static uint32_t retryTotalElapsed = 0;

static bool lastTransmitAckPending = false;

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

static bool finPending;
static bool peerFinPending;

/*** local function prototypes ****************************************************/

static void tcp_packRequestIntoEthernet(void);
static void tcp_packRequestIntoIp(void);
static void tcp_prepareTcpHeader(uint8_t tcpFlag);
static void tcp_sendAck(void);

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
   pLen = (((uint16_t)myethreceivebuffer[18]) << 8) + myethreceivebuffer[19]; /* length of the IP payload */
   hdrLen = (myethreceivebuffer[66] >> 4) * 4; /* header length in byte */
   //log_v("pLen=%d, hdrLen=%d", pLen, hdrLen);
   if (pLen >= hdrLen)
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
      addToTrace(MOD_TCP, "[TCP] wrong port: %d!=%d||%d!=%d", sourcePort, seccTcpPort, destinationPort, evccPort);
      return; /* wrong port */
   }

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

   if (flags & TCP_FLAG_RST)
   {
       addToTrace(MOD_TCP, "[TCP] RST received, closing connection");
       tcpState = TCP_STATE_CLOSED;
       finPending = peerFinPending = false;
       return;
   }

   /* It is no connection setup. We can have the following situations here: */
   if (tcpState == TCP_STATE_CLOSED && not peerFinPending)
   {
       /* received something while the connection is closed. Just ignore it. */
       addToTrace(MOD_TCP, "[TCP] ignore, not connected.");
       return;
   }

   if (tcpState == TCP_STATE_SYN_SENT)
   {
      if ((flags & (TCP_FLAG_SYN | TCP_FLAG_ACK)) == (TCP_FLAG_SYN | TCP_FLAG_ACK))   /* This is the connection setup response from the server. */
      {
         TcpSeqNr = remoteAckNr; /* The sequence number of our next transmit packet is given by the received ACK number. */
         TcpAckNr = remoteSeqNr + 1; /* The ACK number of our next transmit packet is one more than the received seq number. */
         setCheckpoint(303);
         tcpState = TCP_STATE_ESTABLISHED;
         tcp_sendAck();

         addToTrace(MOD_TCP, "[TCP] connected.");
         finPending = peerFinPending = true;
         connMgr_setLevel(CONNLEVEL_80_TCP_RUNNING);
      }
      // Ignore everything else
      return;
   }

   if (tcpState == TCP_STATE_ESTABLISHED)
   {
       /* It can be an ACK, or a data package, or a combination of both. We treat the ACK and the data independent from each other,
         to treat each combination. */
         //log_v("L=%d", tmpPayloadLen);
       if ((tmpPayloadLen > 0) && (tmpPayloadLen < TCP_RX_DATA_LEN))
       {
           /* This is a data transfer packet. */
           tcp_rxdataLen = tmpPayloadLen;
           /* myethreceivebuffer[74] is the first payload byte. */
           /* Fix for https://github.com/uhi22/ccs32clara/issues/15. We explicitely need to copy the data here,
           because the application will look asynchronously on the tcp_rxdata, and if in between some other received
           data would end up in the myethreceivebuffer (e.g. neighbour solicitation, or TCP traffic with other ports),
           the overwritten myethreceivebuffer would lead to not seeing the tcp_rxdata anymore, if this would be just
           a pointer to myethreceivebuffer[74]. */
           memcpy(tcp_rxdata, &myethreceivebuffer[54 + hdrLen], tcp_rxdataLen);  /* provide the received data to the application */
//           connMgr_TcpOk();
           TcpAckNr = remoteSeqNr + tcp_rxdataLen; /* The ACK number of our next transmit packet is tcp_rxdataLen more than the received seq number. */
           tcp_sendAck();

           addToTrace_bytes(MOD_TCPTRAFFIC, "Data received: ", tcp_rxdata, tcp_rxdataLen);
       }
   }

   if (flags & TCP_FLAG_ACK)
   {
       TcpSeqNr = remoteAckNr; /* The sequence number of our next transmit packet is given by the received ACK number. */
       lastTransmitAckPending = false;
   }

   if (flags & TCP_FLAG_FIN)
   {
       addToTrace(MOD_TCP, "[TCP] FIN received, sending ACK");
       TcpAckNr = remoteSeqNr + 1;
       tcp_sendAck();
       // tcpState = TCP_STATE_CLOSED;
       peerFinPending = false;
   }
}

void tcp_connect(void)
{
   addToTrace(MOD_TCP, "[TCP] Checkpoint301: connecting from %d", evccPort);
   setCheckpoint(301);

   tcpHeaderLen = 20; /* 20 bytes normal header, no options */
   tcpPayloadLen = 0;   /* only the TCP header, no data is in the connect message. */
   tcp_prepareTcpHeader(TCP_FLAG_SYN);
   tcp_packRequestIntoIp();
   tcpState = TCP_STATE_SYN_SENT;
   finPending = peerFinPending = false;
}

static void tcp_sendAck(void)
{
//   addToTrace(MOD_TCP, "[TCP] sending ACK");
   tcpHeaderLen = 20; /* 20 bytes normal header, no options */
   tcpPayloadLen = 0;   /* only the TCP header, no data is in the first ACK message. */
   tcp_prepareTcpHeader(TCP_FLAG_ACK);
   tcp_packRequestIntoIp();
}

static void tcp_setRetry()
{
    lastTransmitAckPending = true;
    retryDelay = TCP_ACK_INITIAL_TIMEOUT_MS;
    retryTotalElapsed = 0;
    nextRetryTime = rtc_get_ms() + retryDelay;
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
          addToTrace_bytes(MOD_TCPTRAFFIC, "TCP will transmit:", tcpPayload, tcpPayloadLen);
          tcp_prepareTcpHeader(TCP_FLAG_PSH | TCP_FLAG_ACK); /* data packets are always sent with flags PUSH and ACK. */
          tcp_packRequestIntoIp();
          tcp_setRetry();
      }
      else
      {
         addToTrace(MOD_TCP, "Error: tcpPayload and header do not fit into TcpTransmitPacket.");
      }
   }
}

static void tcp_prepareTcpHeader(uint8_t tcpFlag)
{
   uint16_t checksum;

   // # TCP header needs at least 20 bytes:
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

static void tcp_sendFin()
{
    addToTrace(MOD_TCP, "[TCP] sending FIN");
    tcpHeaderLen = 20;  // no options
    tcpPayloadLen = 0;  // no payload
    tcp_prepareTcpHeader(TCP_FLAG_FIN | TCP_FLAG_ACK);
    tcp_packRequestIntoIp();
}

void tcp_sendRst()
{
    addToTrace(MOD_TCP, "[TCP] sending RST");
    tcpHeaderLen = 20;  // no options
    tcpPayloadLen = 0;  // no payload
    tcp_prepareTcpHeader(TCP_FLAG_RST);
    tcp_packRequestIntoIp();
}

void tcp_disconnect(void)
{
    if (finPending) {
        tcp_sendFin();
        finPending = false;
    }
    else if (tcpState == TCP_STATE_SYN_SENT) {
        tcp_sendRst();
    }

    tcpState = TCP_STATE_CLOSED;
    lastTransmitAckPending = false;
}

bool tcp_isClosed(void)
{
   return (tcpState == TCP_STATE_CLOSED);
}

bool tcp_isConnected(void)
{
   return (tcpState == TCP_STATE_ESTABLISHED);
}

void tcp_checkRetry(void)
{
    if (lastTransmitAckPending && tcpState == TCP_STATE_ESTABLISHED)
    {
        uint32_t now = rtc_get_ms();
        if (now >= nextRetryTime)
        {
            if (retryTotalElapsed >= TCP_MAX_TOTAL_RETRY_TIME_MS)
            {
                addToTrace(MOD_TCP, "[TCP] Giving up the retry");
                tcp_disconnect();
                return;
            }

            addToTrace(MOD_TCP, "[TCP] Last packet wasn't ACKed, retransmitting"); // Resend the last packet
            tcp_packRequestIntoEthernet(); // retransmit the same content

            tcp_debug_totalRetryCounter++;

            retryTotalElapsed += retryDelay;

            // Update delay for next retry (exponential backoff, capped)
            retryDelay *= 2;
            if (retryDelay > TCP_ACK_MAX_TIMEOUT_MS)
                retryDelay = TCP_ACK_MAX_TIMEOUT_MS;

            nextRetryTime = now + retryDelay;
        }
    }
}

void tcp_Mainfunction(void)
{
   tcp_checkRetry();

   if (connMgr_getLevel() < CONNLEVEL_50_SDP_DONE_TCP_NEXT)
   {
      /* No SDP done. Means: It does not make sense to start or continue TCP. */
      tcp_disconnect();
      return;
   }

   if ((connMgr_getLevel() == CONNLEVEL_50_SDP_DONE_TCP_NEXT) && (tcpState == TCP_STATE_CLOSED))
   {
      /* SDP is finished, but no TCP connected yet. */
      /* use a new port */
      if (evccPort == CLIENT_MAX_PORT)
         evccPort = CLIENT_MIN_PORT;
      else
         evccPort++;

      tcp_connect();
   }
}

uint16_t setStartPort(uint32_t rand_num)
{
    evccPort = (rand_num % (CLIENT_MAX_PORT - CLIENT_MIN_PORT + 1)) + CLIENT_MIN_PORT;
    return evccPort;
}
