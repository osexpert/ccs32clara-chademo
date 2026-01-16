#include "ccs32_globals.h"

#define NEXT_UDP 0x11 /* next protocol is UDP */
#define NEXT_ICMPv6 0x3a /* next protocol is ICMPv6 */
#define UDP_PAYLOAD_LEN 100


const uint8_t broadcastIPv6[16] = { 0xff, 0x02, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1};
/* our link-local IPv6 address. Todo: do we need to calculate this from the MAC? Or just use a "random"? */
/* For the moment, just use the address from the Win10 notebook, and change the last byte from 0x0e to 0x1e. */
const uint8_t EvccIp[16] = {0xfe, 0x80, 0, 0, 0, 0, 0, 0, 0xc6, 0x90, 0x83, 0xf3, 0xfb, 0xcb, 0x98, 0x1e};
uint8_t SeccIp[16]; /* the IP address of the charger */
uint16_t seccTcpPort; /* the port number of the charger */
uint16_t evccPort=59219; /* some "random port" */

static uint8_t sourceIp[16];
static uint16_t sourceport;
static uint16_t destinationport;
static uint16_t udplen;
static uint16_t udpsum;
static uint8_t NeighborsMac[6];
static uint8_t NeighborsIp[16];
static uint8_t* udpPayload = &myethreceivebuffer[62];
static uint16_t udpPayloadLen;
static uint8_t IpRequestLen;
static uint8_t* IpRequest = &myethtransmitbuffer[14];
static uint8_t UdpRequestLen;
static uint8_t* UdpRequest = &IpRequest[40];
static uint8_t v2gtpFrameLen;
static uint8_t* v2gtpFrame = &UdpRequest[8];

/*** local function prototypes ******************************************/
void ipv6_packRequestIntoEthernet(void);
void ipv6_packRequestIntoIp(void);
void ipv6_packRequestIntoUdp(void);
void evaluateNeighborSolicitation(void);


/*** functions **********************************************************/

void evaluateUdpPayload(void) {
   uint16_t v2gptPayloadType;
   uint32_t v2gptPayloadLen;
   uint8_t i;
   if ((destinationport == 15118) || (sourceport == 15118)) { // port for the SECC
      if ((udpPayload[0]==0x01) && (udpPayload[1]==0xFE)) { //# protocol version 1 and inverted
         //# it is a V2GTP message
         //showAsHex(udpPayload, "V2GTP ")
         v2gptPayloadType = udpPayload[2] * 256 + udpPayload[3];
         //# 0x8001 EXI encoded V2G message (Will NOT come with UDP. Will come with TCP.)
         //# 0x9000 SDP request message (SECC Discovery)
         //# 0x9001 SDP response message (SECC response to the EVCC)
         if (v2gptPayloadType == 0x9001) {
            //# it is a SDP response from the charger to the car
            //addToTrace("it is a SDP response from the charger to the car");
            v2gptPayloadLen = (((uint32_t)udpPayload[4])<<24)  +
                              (((uint32_t)udpPayload[5])<<16) +
                              (((uint32_t)udpPayload[6])<<8) +
                              udpPayload[7];
            if (v2gptPayloadLen == 20) {
               //# 20 is the only valid length for a SDP response.
               if (connMgr_getLevel() >= CONNLEVEL_80_TCP_CONNECTED) {
                    /* we have TCP traffic running, so we ignore additional SDP messages. This
                    makes us robust against cross-talk from other charging cables.
                    Discussion here: https://github.com/uhi22/ccs32clara/issues/24 */
                    addToTrace(MOD_SDP, "[SDP] Ignoring SDP response, because high level communication is ongoing.");
                    return;
               }
               addToTrace(MOD_SDP, "[SDP] Checkpoint203: Received SDP response");
               setCheckpoint(203);
               //# at byte 8 of the UDP payload starts the IPv6 address of the charger.
               for (i=0; i<16; i++) {
                  SeccIp[i] = udpPayload[8+i]; // 16 bytes IP address of the charger
               }
               //# Extract the TCP port, on which the charger will listen:
               seccTcpPort = (((uint16_t)(udpPayload[8+16]))<<8) + udpPayload[8+16+1];
               /* in case we did not yet found out the chargers MAC, because we jumped-over the SLAC,
                  we extract the chargers MAC from the SDP response. */
               if ((evseMac[0]==0) && (evseMac[1]==0)) {
                  addToTrace(MOD_SDP, "[SDP] Taking evseMac from SDP response because not yet known.");
                  for (i=0; i<6; i++) {
                     evseMac[i] = myethreceivebuffer[6+i]; // source MAC starts at offset 6
                  }
               }
               addToTrace(MOD_SDP, "[SDP] Now we know the chargers IP.");
               readModemVersions(); /* read the software versions from our modem and from the chargers modem. Just for information/logging. */
               connMgr_setLevel(CONNLEVEL_50_SDP_DONE_TCP_NEXT);
            }
         }
         else {
            addToTrace(MOD_SDP, "v2gptPayloadType 0x%x not supported", v2gptPayloadType);
         }
      }
   }
}

void ipv6_evaluateReceivedPacket(void) {
   //# The evaluation function for received ipv6 packages.
   uint16_t nextheader;
   uint8_t icmpv6type;

   if (myethreceivebufferLen>60) {
      //# extract the source ipv6 address
      memcpy(sourceIp, &myethreceivebuffer[22], 16);
      nextheader = myethreceivebuffer[20];
      if (nextheader == 0x11) { //  it is an UDP frame
         addToTrace(MOD_IPV6, "Its a UDP.");
         sourceport = myethreceivebuffer[54] * 256 + myethreceivebuffer[55];
         destinationport = myethreceivebuffer[56] * 256 + myethreceivebuffer[57];
         udplen = myethreceivebuffer[58] * 256 + myethreceivebuffer[59];
         udpsum = myethreceivebuffer[60] * 256 + myethreceivebuffer[61];
         //# udplen is including 8 bytes header at the begin
         if (udplen>UDP_PAYLOAD_LEN) {
            /* ignore long UDP */
            addToTrace(MOD_IPV6, "Ignoring too long UDP");
            return;
         }
         if (udplen>8) {
            udpPayloadLen = udplen-8;
            sanityCheck("before evaluateUdpPayload");
            evaluateUdpPayload();
            sanityCheck("after evaluateUdpPayload");
         }
      }
      if (nextheader == 0x06) { // # it is an TCP frame
         addToTrace(MOD_IPV6, "TCP received");
         sanityCheck("before evaluateTcpPacket");
         evaluateTcpPacket();
         sanityCheck("before evaluateTcpPacket");
      }
      if (nextheader == NEXT_ICMPv6) { // it is an ICMPv6 (NeighborSolicitation etc) frame
         addToTrace(MOD_IPV6, "ICMPv6 received");
         icmpv6type = myethreceivebuffer[54];
         if (icmpv6type == 0x87) { /* Neighbor Solicitation */
            //addToTrace("[PEV] Neighbor Solicitation received");
            sanityCheck("before evaluateNeighborSolicitation");
            evaluateNeighborSolicitation();
            sanityCheck("after evaluateNeighborSolicitation");
         }
      }
   }
}

void ipv6_initiateSdpRequest(void) {
   //# We are the car. We want to find out the IPv6 address of the charger. We
   //# send a SECC Discovery Request.
   //# The payload is just two bytes: 10 00.
   //# First step is, to pack this payload into a V2GTP frame.
   addToTrace(MOD_SDP, "[SDP] initiating SDP request");
   v2gtpFrameLen = 8 + 2; // # 8 byte header plus 2 bytes payload
   v2gtpFrame[0] = 0x01; // # version
   v2gtpFrame[1] = 0xFE; // # version inverted
   v2gtpFrame[2] = 0x90; // # 9000 means SDP request message
   v2gtpFrame[3] = 0x00;
   v2gtpFrame[4] = 0x00;
   v2gtpFrame[5] = 0x00;
   v2gtpFrame[6] = 0x00;
   v2gtpFrame[7] = 0x02; // # payload size
   v2gtpFrame[8] = 0x10; // # payload
   v2gtpFrame[9] = 0x00; // # payload
   //# Second step: pack this into an UDP frame.
   ipv6_packRequestIntoUdp();
}

void ipv6_packRequestIntoUdp(void) {
   //# embeds the (SDP) request into the lower-layer-protocol: UDP
   //# Reference: wireshark trace of the ioniq car
   uint16_t lenInclChecksum;
   uint16_t checksum;
   UdpRequestLen = v2gtpFrameLen + 8; // # UDP header needs 8 bytes:
   //           #   2 bytes source port
   //           #   2 bytes destination port
   //           #   2 bytes length (incl checksum)
   //           #   2 bytes checksum
   UdpRequest[0] = evccPort >> 8;
   UdpRequest[1] = evccPort  & 0xFF;
   UdpRequest[2] = 15118 >> 8;
   UdpRequest[3] = 15118 & 0xFF;

   lenInclChecksum = UdpRequestLen;
   UdpRequest[4] = lenInclChecksum >> 8;
   UdpRequest[5] = lenInclChecksum & 0xFF;
   // checksum will be calculated afterwards
   UdpRequest[6] = 0;
   UdpRequest[7] = 0;
   // The content of buffer is ready. We can calculate the checksum. see https://en.wikipedia.org/wiki/User_Datagram_Protocol
   checksum = calculateUdpAndTcpChecksumForIPv6(UdpRequest, UdpRequestLen, EvccIp, broadcastIPv6, NEXT_UDP);
   UdpRequest[6] = checksum >> 8;
   UdpRequest[7] = checksum & 0xFF;
#ifdef VERBOSE_UDP
   showAsHex(UdpRequest, UdpRequestLen, "UDP request ");
#endif
   ipv6_packRequestIntoIp();
}

void ipv6_packRequestIntoIp(void) {
   // # embeds the (SDP) request into the lower-layer-protocol: IP, Ethernet
   uint8_t i;
   uint16_t plen;
   IpRequestLen = UdpRequestLen + 8 + 16 + 16; // # IP6 header needs 40 bytes:
   //  #   4 bytes traffic class, flow
   //  #   2 bytes destination port
   //  #   2 bytes length (incl checksum)
   //  #   2 bytes checksum
   IpRequest[0] = 0x60; // # traffic class, flow
   IpRequest[1] = 0;
   IpRequest[2] = 0;
   IpRequest[3] = 0;
   plen = UdpRequestLen; // length of the payload. Without headers.
   IpRequest[4] = plen >> 8;
   IpRequest[5] = plen & 0xFF;
   IpRequest[6] = 0x11; // next level protocol, 0x11 = UDP in this case
   IpRequest[7] = 0x0A; // hop limit
   // We are the PEV. So the EvccIp is our own link-local IP address.
   for (i=0; i<16; i++) {
      IpRequest[8+i] = EvccIp[i]; // source IP address
   }
   for (i=0; i<16; i++) {
      IpRequest[24+i] = broadcastIPv6[i]; // destination IP address
   }
   ipv6_packRequestIntoEthernet();
}

void ipv6_packRequestIntoEthernet(void) {
   //# packs the IP packet into an ethernet packet
   myethtransmitbufferLen = IpRequestLen + 6 + 6 + 2; // # Ethernet header needs 14 bytes:
   // #  6 bytes destination MAC
   // #  6 bytes source MAC
   // #  2 bytes EtherType
   //# fill the destination MAC with the IPv6 multicast
   myethtransmitbuffer[0] = 0x33;
   myethtransmitbuffer[1] = 0x33;
   myethtransmitbuffer[2] = 0x00;
   myethtransmitbuffer[3] = 0x00;
   myethtransmitbuffer[4] = 0x00;
   myethtransmitbuffer[5] = 0x01;
   fillSourceMac(getOurMac(), 6); // bytes 6 to 11 are the source MAC
   myethtransmitbuffer[12] = 0x86; // # 86dd is IPv6
   myethtransmitbuffer[13] = 0xdd;
   myEthTransmit();
}

void evaluateNeighborSolicitation(void) {
   uint16_t checksum;
   uint8_t i;
   /* The neighbor discovery protocol is used by the charger to find out the
      relation between MAC and IP. */

   /* We could extract the necessary information from the NeighborSolicitation,
      means the chargers IP and MAC address. But this is not fully necessary:
      - The chargers MAC was already discovered in the SLAC. So we do not need to extract
        it here again. But if we have not done the SLAC, because the modems are already paired,
        then it makes sense to extract the chargers MAC from the Neighbor Solicitation message.
      - For the chargers IPv6, there are two possible cases:
          (A) The charger made the SDP without NeighborDiscovery. This works, if
              we use the pyPlc.py as charger. It does not care for NeighborDiscovery,
              because the SDP is implemented independent of the address resolution of
              the operating system.
              In this case, we know the chargers IP already from the SDP.
          (B) The charger insists of doing NeighborSolitcitation in the middle of
              SDP. This behavior was observed on Alpitronics. Means, we have the
              following sequence:
              1. car sends SDP request
              2. charger sends NeighborSolicitation
              3. car sends NeighborAdvertisement
              4. charger sends SDP response
              In this case, we need to extract the chargers IP from the NeighborSolicitation,
              otherwise we have to chance to send the correct NeighborAdvertisement.
              We can do this always, because this does not hurt for case A, address
              is (hopefully) not changing. */
   /* More general approach: In the network there may be more participants than only the charger,
      e.g. a notebook for sniffing. Eeach of it may send a NeighborSolicitation, and we should NOT use the addresses from the
      NeighborSolicitation as addresses of the charger. The chargers address is only determined
      by the SDP. */

   /* save the requesters IP. The requesters IP is the source IP on IPv6 level, at byte 22. */
   memcpy(NeighborsIp, &myethreceivebuffer[22], 16);
   /* save the requesters MAC. The requesters MAC is the source MAC on Eth level, at byte 6. */
   memcpy(NeighborsMac, &myethreceivebuffer[6], 6);

   /* send a NeighborAdvertisement as response. */
   // destination MAC = neighbors MAC
   fillDestinationMac(NeighborsMac, 0); // bytes 0 to 5 are the destination MAC
   // source MAC = my MAC
   fillSourceMac(getOurMac(), 6); // bytes 6 to 11 are the source MAC
   // Ethertype 86DD
   myethtransmitbuffer[12] = 0x86; // # 86dd is IPv6
   myethtransmitbuffer[13] = 0xdd;
   myethtransmitbuffer[14] = 0x60; // # traffic class, flow
   myethtransmitbuffer[15] = 0;
   myethtransmitbuffer[16] = 0;
   myethtransmitbuffer[17] = 0;
   // plen
#define ICMP_LEN 32 /* bytes in the ICMPv6 */
   myethtransmitbuffer[18] = 0;
   myethtransmitbuffer[19] = ICMP_LEN;
   myethtransmitbuffer[20] = NEXT_ICMPv6;
   myethtransmitbuffer[21] = 0xff;
   // We are the PEV. So the EvccIp is our own link-local IP address.
   for (i=0; i<16; i++) {
      myethtransmitbuffer[22+i] = EvccIp[i]; // source IP address
   }
   for (i=0; i<16; i++) {
      myethtransmitbuffer[38+i] = NeighborsIp[i]; // destination IP address
   }
   /* here starts the ICMPv6 */
   myethtransmitbuffer[54] = 0x88; /* Neighbor Advertisement */
   myethtransmitbuffer[55] = 0;
   myethtransmitbuffer[56] = 0; /* checksum (filled later) */
   myethtransmitbuffer[57] = 0;

   /* Flags */
   myethtransmitbuffer[58] = 0x60; /* Solicited, override */
   myethtransmitbuffer[59] = 0;
   myethtransmitbuffer[60] = 0;
   myethtransmitbuffer[61] = 0;

   memcpy(&myethtransmitbuffer[62], EvccIp, 16); /* The own IP address */
   myethtransmitbuffer[78] = 2; /* Type 2, Link Layer Address */
   myethtransmitbuffer[79] = 1; /* Length 1, means 8 byte (?) */
   memcpy(&myethtransmitbuffer[80], getOurMac(), 6); /* The own Link Layer (MAC) address */

   checksum = calculateUdpAndTcpChecksumForIPv6(&myethtransmitbuffer[54], ICMP_LEN, EvccIp, NeighborsIp, NEXT_ICMPv6);
   myethtransmitbuffer[56] = checksum >> 8;
   myethtransmitbuffer[57] = checksum & 0xFF;
   myethtransmitbufferLen = 86; /* Length of the NeighborAdvertisement */
   addToTrace(MOD_IPV6, "transmitting Neighbor Advertisement");
   myEthTransmit();
}

