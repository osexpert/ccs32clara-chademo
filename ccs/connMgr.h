/* Interface header for Connection Manager */
#include <stdint.h>
/* Global Defines */

#define CONNLEVEL_100_APPL_RUNNING 100
#define CONNLEVEL_80_TCP_RUNNING 80
#define CONNLEVEL_50_SDP_DONE_TCP_NEXT 50
#define CONNLEVEL_15_SLAC_DONE_SDP_NEXT 15
//#define CONNLEVEL_10_ONE_MODEM_FOUND 10
#define CONNLEVEL_0_START 0

/* Global Variables */

/* Global Functions */

/* ConnectionManager */
#ifdef __cplusplus
extern "C" {
#endif

extern void connMgr_Mainfunction(void);
extern uint8_t connMgr_getLevel(void);
extern void connMgr_restart();
extern void connMgr_setLevel(uint16_t level);
extern bool connMgr_sdpDoneTrigger();

#ifdef __cplusplus
}
#endif
