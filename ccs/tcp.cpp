
#include "ccs32_globals.h"

#define NEXT_TCP 0x06 /* the next protocol is TCP */

#define TCP_FLAG_FIN 0x01
#define TCP_FLAG_SYN 0x02
#define TCP_FLAG_RST 0x04
#define TCP_FLAG_PSH 0x08
#define TCP_FLAG_ACK 0x10

#define TCP_TRANSMIT_PACKET_LEN 200

#define TCP_STATE_CLOSED 0
#define TCP_STATE_SYN_SENT 1
#define TCP_STATE_ESTABLISHED 2
#define TCP_STATE_FIN_WAIT_1 3
#define TCP_STATE_FIN_WAIT_2 4
//#define TCP_STATE_CLOSE_WAIT 5
#define TCP_STATE_CLOSING 6
#define TCP_STATE_LAST_ACK 7
#define TCP_STATE_TIME_WAIT 8

#define TCP_ACK_INITIAL_TIMEOUT_MS 100     // Start with 100ms
#define TCP_ACK_MAX_TIMEOUT_MS     800     // Max per-retry delay
#define TCP_MAX_TOTAL_RETRY_TIME_MS 4000   // Retry attempts within 4s
#define TCP_DISCONNECT_TIMEOUT_MS 2000     // 2sec

#define CLIENT_MIN_PORT 49152
#define CLIENT_MAX_PORT 65535

static uint32_t nextRetryTime = 0;
static uint32_t retryDelay = TCP_ACK_INITIAL_TIMEOUT_MS;
static uint32_t retryTotalElapsed = 0;
static uint32_t disconnectStartTime = 0;

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

/*** local function prototypes ****************************************************/

static void tcp_packRequestIntoEthernet(void);
static void tcp_packRequestIntoIp(void);
static void tcp_prepareTcpHeader(uint8_t tcpFlag);
static void tcp_sendAck(void);
static void tcp_sendFirstAck(void);
static void tcp_sendFin(void);
static void tcp_checkDisconnectTimeout(void);
static const char* tcp_getStateString(uint8_t state);

/*** functions *********************************************************************/
uint32_t tcp_getTotalNumberOfRetries(void) {
  return tcp_debug_totalRetryCounter;
}

static void tcp_setStateClosed(void)
{
    tcpState = TCP_STATE_CLOSED;
    lastTransmitAckPending = false;
}

static const char* tcp_getStateString(uint8_t state) {
    switch(state) {
        case TCP_STATE_CLOSED: return "CLOSED";
        case TCP_STATE_SYN_SENT: return "SYN_SENT";
        case TCP_STATE_ESTABLISHED: return "ESTABLISHED";
        case TCP_STATE_FIN_WAIT_1: return "FIN_WAIT_1";
        case TCP_STATE_FIN_WAIT_2: return "FIN_WAIT_2";
        //case TCP_STATE_CLOSE_WAIT: return "CLOSE_WAIT";
        case TCP_STATE_CLOSING: return "CLOSING";
        case TCP_STATE_LAST_ACK: return "LAST_ACK";
        case TCP_STATE_TIME_WAIT: return "TIME_WAIT";
        default: return "UNKNOWN";
    }
}

void evaluateTcpPacket(void)
{
   uint8_t flags;
   uint32_t remoteSeqNr;
   uint32_t remoteAckNr;
   uint16_t sourcePort, destinationPort, pLen, hdrLen, tmpPayloadLen;

   /* todo: check the IP addresses, checksum etc */
   pLen =  (((uint16_t)myethreceivebuffer[18])<<8) + myethreceivebuffer[19]; /* length of the IP payload */
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
      addToTrace(MOD_TCP, "[TCP] wrong port.");
      log_v("%d %d %d %d",sourcePort,seccTcpPort,destinationPort, evccPort);
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

   // Handle RST - always close connection (except when already closed)
   if (flags & TCP_FLAG_RST)
   {
       if (tcpState != TCP_STATE_CLOSED)
       {
           addToTrace(MOD_TCP, "[TCP] RST received, closing connection");
           tcp_setStateClosed();
       }
       return;
   }

   switch (tcpState) {
       case TCP_STATE_CLOSED:
           addToTrace(MOD_TCP, "[TCP] ignore, connection closed.");
           break;

       case TCP_STATE_SYN_SENT:
           // Waiting for SYN+ACK
           if ((flags & (TCP_FLAG_SYN | TCP_FLAG_ACK)) == (TCP_FLAG_SYN | TCP_FLAG_ACK)) {
               TcpSeqNr = remoteAckNr;
               TcpAckNr = remoteSeqNr+1;
               setCheckpoint(303);
               tcpState = TCP_STATE_ESTABLISHED;
               tcp_sendFirstAck();
               connMgr_TcpOk();
               addToTrace(MOD_TCP, "[TCP] connected.");
           }
           // Ignore other packets in SYN_SENT state
           break;

       case TCP_STATE_ESTABLISHED:
           // Handle data packets
           if ((tmpPayloadLen>0) && (tmpPayloadLen<TCP_RX_DATA_LEN))
           {
              tcp_rxdataLen = tmpPayloadLen;
              memcpy(tcp_rxdata, &myethreceivebuffer[74], tcp_rxdataLen);
              connMgr_TcpOk();
              TcpAckNr = remoteSeqNr + tcp_rxdataLen;
              tcp_sendAck();
              addToTrace(MOD_TCPTRAFFIC, "Data received: ", tcp_rxdata, tcp_rxdataLen);
           }

           // Handle ACK for our data
           if (flags & TCP_FLAG_ACK)
           {
               if (remoteAckNr == TcpSeqNr)
               {
                   lastTransmitAckPending = false;
               }
               TcpSeqNr = remoteAckNr;  // Update sequence number here
           }

           // Handle FIN - peer wants to close (passive close)
           if (flags & TCP_FLAG_FIN)
           {
               addToTrace(MOD_TCP, "[TCP] FIN received, sending ACK+FIN");
               TcpAckNr = remoteSeqNr + 1;
               tcp_sendFin(); // Send combined ACK+FIN response (skip CLOSE_WAIT state)
               tcpState = TCP_STATE_LAST_ACK;
               disconnectStartTime = rtc_get_ms();
           }
           break;

       case TCP_STATE_FIN_WAIT_1:
           // Waiting for ACK of our FIN or FIN from peer
           if (flags & TCP_FLAG_ACK)
           {
               if (remoteAckNr == TcpSeqNr)
               {
                   lastTransmitAckPending = false;
                   tcpState = TCP_STATE_FIN_WAIT_2;
                   addToTrace(MOD_TCP, "[TCP] FIN ACKed, entering FIN_WAIT_2");
               }
               TcpSeqNr = remoteAckNr;  // Update sequence number
           }

           if (flags & TCP_FLAG_FIN)
           {
               TcpAckNr = remoteSeqNr + 1;
               tcp_sendAck();
               if (flags & TCP_FLAG_ACK && remoteAckNr == TcpSeqNr)
               {
                   // Simultaneous close
                   tcpState = TCP_STATE_TIME_WAIT;
                   addToTrace(MOD_TCP, "[TCP] Simultaneous close, entering TIME_WAIT");
               }
               else
               {
                   tcpState = TCP_STATE_CLOSING;
                   addToTrace(MOD_TCP, "[TCP] FIN received in FIN_WAIT_1, entering CLOSING");
               }
           }
           break;

       case TCP_STATE_FIN_WAIT_2:
           // Waiting for FIN from peer
           if (flags & TCP_FLAG_FIN)
           {
               addToTrace(MOD_TCP, "[TCP] FIN received in FIN_WAIT_2, entering TIME_WAIT");
               TcpAckNr = remoteSeqNr + 1;
               tcp_sendAck();
               tcpState = TCP_STATE_TIME_WAIT;
           }
           break;

       case TCP_STATE_CLOSING:
           // Waiting for ACK of our FIN (simultaneous close)
           if (flags & TCP_FLAG_ACK)
           {
               if (remoteAckNr == TcpSeqNr)
               {
                   lastTransmitAckPending = false;
                   tcpState = TCP_STATE_TIME_WAIT;
                   addToTrace(MOD_TCP, "[TCP] Final ACK received, entering TIME_WAIT");
               }
               TcpSeqNr = remoteAckNr;  // Update sequence number
           }
           break;

       case TCP_STATE_LAST_ACK:
           // Waiting for ACK of our FIN
           if (flags & TCP_FLAG_ACK)
           {
               if (remoteAckNr == TcpSeqNr)
               {
                   tcp_setStateClosed();
                   addToTrace(MOD_TCP, "[TCP] Connection closed gracefully");
               }
               TcpSeqNr = remoteAckNr;  // Update sequence number
           }
           break;

       case TCP_STATE_TIME_WAIT:
           // Respond to retransmitted FINs (in case our final ACK was lost)
           if (flags & TCP_FLAG_FIN)
           {
               addToTrace(MOD_TCP, "[TCP] Retransmitted FIN in TIME_WAIT, sending ACK");
               tcp_sendAck();
           }
           break;
   }
}

void tcp_connect(void)
{
    addToTrace(MOD_TCP, "[TCP] Checkpoint301: connecting");
    setCheckpoint(301);
    printf("evccPort:%d\r\n", evccPort);

    // options
    TcpTransmitPacket[20] = 0x02; // Kind: 2 = Maximum Segment Size (MSS)
    TcpTransmitPacket[21] = 0x04; // Length: 4
    TcpTransmitPacket[22] = 0x05; // MSS = 0x05A0 = 1440 bytes
    TcpTransmitPacket[23] = 0xA0;

    TcpTransmitPacket[24] = 0x01; // Kind: 1 = NOP
    TcpTransmitPacket[25] = 0x03; // Kind: 3 = Window Scale
    TcpTransmitPacket[26] = 0x03; // Length: 3
    TcpTransmitPacket[27] = 0x08; // Shift count = 8 (2^8 = 256x window)

    tcpHeaderLen = 28; /* 20 bytes normal header, plus 8 bytes options */
    tcpPayloadLen = 0;   /* only the TCP header, no data is in the connect message. */
    tcp_prepareTcpHeader(TCP_FLAG_SYN);
    tcp_packRequestIntoIp();
    tcpState = TCP_STATE_SYN_SENT;
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
//   addToTrace(MOD_TCP, "[TCP] sending ACK");
   tcpHeaderLen = 20; /* 20 bytes normal header, no options */
   tcpPayloadLen = 0;   /* only the TCP header, no data is in the first ACK message. */
   tcp_prepareTcpHeader(TCP_FLAG_ACK);
   tcp_packRequestIntoIp();
}

static void setRetry()
{
    lastTransmitAckPending = true;
    retryDelay = TCP_ACK_INITIAL_TIMEOUT_MS;
    retryTotalElapsed = 0;
    nextRetryTime = rtc_get_ms() + retryDelay;
}

static void tcp_sendFin(void)
{
   addToTrace(MOD_TCP, "[TCP] sending FIN");
   tcpHeaderLen = 20;
   tcpPayloadLen = 0;
   tcp_prepareTcpHeader(TCP_FLAG_FIN | TCP_FLAG_ACK);
   tcp_packRequestIntoIp();
   TcpSeqNr++; // FIN consumes one sequence number
   setRetry();
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
          TcpSeqNr += tcpPayloadLen; // Update sequence number by payload length
          setRetry();
      }
      else
      {
         addToTrace(MOD_TCP, "Error: tcpPayload and header do not fit into TcpTransmitPacket.");
      }
   }
}

#if false
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

      // Update sequence number by payload length
      TcpSeqNr += tcpPayloadLen;
   }
}
#endif

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

void tcp_disconnect(void)
{
   /* Graceful connection termination (active close) */
   if (tcpState == TCP_STATE_ESTABLISHED)
   {
      addToTrace(MOD_TCP, "[TCP] Initiating graceful disconnect");
      tcp_sendFin();
      tcpState = TCP_STATE_FIN_WAIT_1;
      disconnectStartTime = rtc_get_ms();
   }
}

#if false
void tcp_reset()
{
    /* Brutal connection termination */
    if (tcpState != TCP_STATE_CLOSED)
    {
        addToTrace(MOD_TCP, "[TCP] sending RST");
        tcpHeaderLen = 20;  // no options
        tcpPayloadLen = 0;  // no payload
        TcpAckNr = 0; // not acknowledging any data
        tcp_prepareTcpHeader(TCP_FLAG_RST);
        tcp_packRequestIntoIp();
        tcp_setStateClosed();
    }

    lastTransmitAckPending = false;
}
#endif

bool tcp_isClosed(void)
{
   return (tcpState == TCP_STATE_CLOSED);
}

bool tcp_isConnected(void)
{
   return (tcpState == TCP_STATE_ESTABLISHED);
}

bool tcp_isClosing(void)
{
   return (tcpState >= TCP_STATE_FIN_WAIT_1);
}

void tcp_checkRetry(void)
{
    if (lastTransmitAckPending == false)
        return;

    // Don't retry our own transmissions in TIME_WAIT state
    // (but we still respond to peer's retransmitted packets in evaluateTcpPacket)
    if (tcpState == TCP_STATE_TIME_WAIT)
        return;

    uint32_t now = rtc_get_ms();
    if (now >= nextRetryTime)
    {
        if (retryTotalElapsed >= TCP_MAX_TOTAL_RETRY_TIME_MS)
        {
            addToTrace(MOD_TCP, "[TCP] Giving up the retry");
            tcp_setStateClosed();
            return;
        }

        addToTrace(MOD_TCP, "[TCP] Last packet wasn't ACKed, retransmitting");
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

static void tcp_checkDisconnectTimeout(void)
{
    /*
    TCP_STATE_FIN_WAIT_1 // No special timeout: governed by normal FIN retransmit timer.
    TCP_STATE_FIN_WAIT_2 // waiting for peer's FIN
    TCP_STATE_CLOSING // waiting for ACK of our FIN (simultaneous close).
    TCP_STATE_LAST_ACK // waiting for ACK of our FIN (passive close).
    TCP_STATE_TIME_WAIT // standard TIME_WAIT timeout
    */

    if (tcpState >= TCP_STATE_FIN_WAIT_2) // everything after we've sent/acked a FIN
    {
        if (rtc_get_ms() - disconnectStartTime >= TCP_DISCONNECT_TIMEOUT_MS)
        {
            printf("[TCP] %s timeout, forcing close\r\n", tcp_getStateString(tcpState));
            tcp_setStateClosed();
        }
    }
}

void tcp_Mainfunction(void)
{
   tcp_checkRetry();
   tcp_checkDisconnectTimeout();

   if (connMgr_getConnectionLevel()<50)
   {
      /* No SDP done. Means: It does not make sense to start or continue TCP. */
      tcp_disconnect();
      return;
   }

   if ((connMgr_getConnectionLevel()==50) && (tcpState == TCP_STATE_CLOSED))
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

uint8_t tcp_getState(void)
{
    return tcpState;
}

const char* tcp_getStateString_public(void)
{
    return tcp_getStateString(tcpState);
}

