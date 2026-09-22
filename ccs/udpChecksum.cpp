
#include "ccs32_globals.h"

/* checksum calculation.
   For UDP and TCP
*/

#define PSEUDO_HEADER_LEN 40
uint8_t pseudoHeader[PSEUDO_HEADER_LEN];

uint16_t calculateUdpAndTcpChecksumForIPv6(uint8_t *UdpOrTcpframe, uint16_t UdpOrTcpframeLen, const uint8_t *ipv6source, const uint8_t *ipv6dest, uint8_t nxt) {
	uint16_t evenFrameLen, i, value16, checksum;
	uint32_t totalSum;
    // Parameters:
    // UdpOrTcpframe: the udp frame or tcp frame, including udp/tcp header and udp/tcp payload
    // ipv6source: the 16 byte IPv6 source address. Must be the same, which is used later for the transmission.
    // ipv6source: the 16 byte IPv6 destination address. Must be the same, which is used later for the transmission.
	// nxt: The next-protocol. 0x11 for UDP, ... for TCP.
	//
    // Goal: construct an array, consisting of a 40-byte-pseudo-ipv6-header, and the udp frame (consisting of udp header and udppayload).
	// For memory efficienty reason, we do NOT copy the pseudoheader and the udp frame together into one new array. Instead, we are using
	// a dedicated pseudo-header-array, and the original udp buffer.
	evenFrameLen = UdpOrTcpframeLen;
	if ((evenFrameLen & 1)!=0) {
        /* if we have an odd buffer length, we need to add a padding byte in the end, because the sum calculation
           will need 16-bit-aligned data. */
		evenFrameLen++;
		UdpOrTcpframe[evenFrameLen-1] = 0; /* Fill the padding byte with zero. */
	}
    memset(pseudoHeader, 0, PSEUDO_HEADER_LEN);
    /* fill the pseudo-ipv6-header */
    /* copy 16 bytes IPv6 addresses */
    memcpy(pseudoHeader, ipv6source, 16); /* IPv6 source address */
    memcpy(&pseudoHeader[16], ipv6dest, 16); /* IPv6 destination address */
    pseudoHeader[32] = 0; // # high byte of the FOUR byte length is always 0
    pseudoHeader[33] = 0; // # 2nd byte of the FOUR byte length is always 0
    pseudoHeader[34] = UdpOrTcpframeLen >> 8; // # 3rd
    pseudoHeader[35] = UdpOrTcpframeLen & 0xFF; // # low byte of the FOUR byte length
    pseudoHeader[36] = 0; // # 3 padding bytes with 0x00
    pseudoHeader[37] = 0;
    pseudoHeader[38] = 0;
    pseudoHeader[39] = nxt; // # the nxt is at the end of the pseudo header
    // pseudo-ipv6-header finished.
    // Run the checksum over the concatenation of the pseudoheader and the buffer.
    totalSum = 0;
	for (i=0; i<PSEUDO_HEADER_LEN/2; i++) { // running through the pseudo header, in 2-byte-steps
        value16 = pseudoHeader[2*i] << 8 | pseudoHeader[2*i+1]; // take the current 16-bit-word
        totalSum += value16; // we start with a normal addition of the value to the totalSum
        // But we do not want normal addition, we want a 16 bit one's complement sum,
        // see https://en.wikipedia.org/wiki/User_Datagram_Protocol
        if (totalSum>=65536) { // On each addition, if a carry-out (17th bit) is produced, 
            totalSum-=65536; // swing that 17th carry bit around 
            totalSum+=1; // and add it to the least significant bit of the running total.
		}
	}
	for (i=0; i<evenFrameLen/2; i++) { // running through the udp buffer, in 2-byte-steps
        value16 = UdpOrTcpframe[2*i] << 8 | UdpOrTcpframe[2*i+1]; // take the current 16-bit-word
        totalSum += value16; // we start with a normal addition of the value to the totalSum
        // But we do not want normal addition, we want a 16 bit one's complement sum,
        // see https://en.wikipedia.org/wiki/User_Datagram_Protocol
        if (totalSum>=65536) { // On each addition, if a carry-out (17th bit) is produced, 
            totalSum-=65536; // swing that 17th carry bit around 
            totalSum+=1; // and add it to the least significant bit of the running total.
		}
	}
    // Finally, the sum is then one's complemented to yield the value of the UDP checksum field.
    checksum = (uint16_t) (totalSum ^ 0xffff);
    return checksum;
}
