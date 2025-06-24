/* Interface header for qca7000.c */

/* Global Defines */

#define MY_SPI_TX_RX_BUFFER_SIZE 1100
#define MY_ETH_TRANSMIT_BUFFER_LEN 250
#define MY_ETH_RECEIVE_BUFFER_LEN 250

#ifdef __cplusplus
extern "C" {
#endif

/* Global Variables */

extern uint8_t myethtransmitbuffer[MY_ETH_TRANSMIT_BUFFER_LEN];
extern uint16_t myethtransmitbufferLen; /* The number of used bytes in the ethernet transmit buffer */
extern uint8_t myethreceivebuffer[MY_ETH_RECEIVE_BUFFER_LEN];
extern uint16_t myethreceivebufferLen;
extern uint8_t nMaxInMyEthernetReceiveCallback, nInMyEthernetReceiveCallback;

/* Global Functions */

void qca7000setup();
extern void demoQCA7000(void);
extern bool spiQCA7000checkForReceivedData(void);
extern void myEthTransmit(void);

#ifdef __cplusplus
}
#endif
