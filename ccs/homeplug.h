#pragma once
/* Interface header for homeplug.c */

extern uint8_t evseMac[6];

extern void composeGetSwReq(void);
extern uint16_t getEtherType(uint8_t *messagebufferbytearray);
extern void evaluateReceivedHomeplugPacket(void);
extern void runSlacSequencer(void);
extern void runSdpStateMachine(void);
extern void runSdpRecoveryStateMachine();
extern void readModemVersions(void);
extern void setOurMac(uint8_t* newMac);
extern const uint8_t* getOurMac();
