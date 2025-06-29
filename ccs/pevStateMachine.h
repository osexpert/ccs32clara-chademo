/* Interface header for pevStateMachine.c */

/* Global Defines */

#define V2GTP_HEADER_SIZE 8 /* The V2GTP header has 8 bytes */
#define MAX_LABEL_LEN     25

/* Global Variables */
extern const char pevSttLabels[][MAX_LABEL_LEN];

/* Global Functions */
#ifdef __cplusplus
extern "C" {
#endif
/* pev state machine */
extern void pevStateMachine_ReInit(void);
extern void pevStateMachine_Mainfunction(void);

extern bool chademoInterface_preChargeCompleted();
extern bool chademoInterface_continueWeldingDetection();

#ifdef __cplusplus
}
#endif
