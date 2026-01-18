/* Interface header for homeplug.c */

/* Global Defines */

/* Global Variables */

extern uint8_t evseMac[6];
extern uint8_t numberOfSoftwareVersionResponses;

/* Global Functions */
#ifdef __cplusplus
extern "C" {
#endif

extern void fillDestinationMac(const uint8_t *mac, uint8_t offset);
extern void fillSourceMac(const uint8_t *mac, uint8_t offset);
extern void composeGetSwReq(void);
extern uint16_t getEtherType(uint8_t *messagebufferbytearray);
extern void evaluateReceivedHomeplugPacket(void);
extern void runSlacSequencer(void);
extern void runSdpStateMachine(void);
extern void runSdpRecoveryStateMachine();
extern void readModemVersions(void);
extern void setOurMac(uint8_t* newMac);
extern const uint8_t* getOurMac();

#ifdef __cplusplus
}
#endif
