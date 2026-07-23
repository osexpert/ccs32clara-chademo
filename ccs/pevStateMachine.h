#pragma once
/* Interface header for pevStateMachine.c */

/* Global Defines */

#define V2GTP_HEADER_SIZE 8 /* The V2GTP header has 8 bytes */

/* Global Variables */
extern const char* const pevSttLabels[];

/* pev state machine */
extern void pevStateMachine_Mainfunction(void);

extern void pevStateMachine_reset();

// naming: _ccsXxx if the method is impemented in ccs. Else in chademo.
extern bool chademoInterface_preChargeCompleted();
extern bool chademoInterface_adapterContactorOpened();
extern bool chademoInterface_ccsInStateEnd();
extern bool chademoInterface_ccsPresentVoltageMirrorsTarget();
extern bool chademoInterface_ccsPresentCurrentMirrorsTarget();
extern bool chademoInterface_ccsInStateWaitForPreChargeStart();
extern int chademoInterface_chargingLoopPos();
extern int chademoInterface_ccsCurrentDemandPos();
