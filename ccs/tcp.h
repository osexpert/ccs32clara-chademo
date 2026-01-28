/* Interface header for tcp.c */

/* Global Defines */

#define TCP_RX_DATA_LEN 200
#define TCP_PAYLOAD_LEN 200

/* Global Variables */

extern uint8_t tcp_rxdataLen;
extern uint8_t tcp_rxdata[TCP_RX_DATA_LEN];
extern uint8_t tcpPayloadLen;
extern uint8_t* tcpPayload;

/* Global Functions */
#ifdef __cplusplus
extern "C" {
#endif

extern void tcp_Mainfunction(void);
extern void evaluateTcpPacket(void);
extern uint32_t tcp_getTotalNumberOfRetries(void);
extern void tcp_disconnect(void);
extern bool tcp_isConnected(void);
extern void tcp_transmit(void);
extern uint16_t setStartPort(uint32_t rand_num);

#ifdef __cplusplus
}
#endif
