/* Interface header for hardwareInterface.c */

/* Global Functions */
#ifdef __cplusplus
extern "C" {
#endif

extern void hardwareInterface_setStateB(void);
extern void hardwareInterface_setStateC(void);
extern void hardwareInterface_setPowerRelayOff(void);
extern void hardwareInterface_setPowerRelayOn(void);
extern void hardwareInterface_lockConnector(void);
extern void hardwareInterface_unlockConnector(void);
extern bool hardwareInterface_isConnectorLocked(void);
extern bool hardwareInterface_stopChargeRequested();
extern uint8_t hardwareInterface_getIsAccuFull(void);
extern uint8_t hardwareInterface_getSoc(void);
extern int16_t hardwareInterface_getAccuVoltage(void);
extern int16_t hardwareInterface_getInletVoltage(void);
extern int16_t hardwareInterface_getChargingTargetVoltage(void);
extern int16_t hardwareInterface_getChargingTargetCurrent(void);


#ifdef __cplusplus
}
#endif
