#pragma once
/* Interface header for udpChecksum.c */

/* Global Defines */

/* Global Variables */

/* Global Functions */

extern uint16_t calculateUdpAndTcpChecksumForIPv6(uint8_t *UdpOrTcpframe, uint16_t UdpOrTcpframeLen, const uint8_t *ipv6source, const uint8_t *ipv6dest, uint8_t nxt);
