

/* Logging verbosity settings */
//#define VERBOSE_INIT_ETH
//#define VERBOSE_QCA7000
//#define VERBOSE_UDP
//#define VERBOSE_EXI_DECODER

/* Charging behavior */
// IEC 61851-23 says +-5%
// pyplc uses +-10V
// WHAT IS CORRECT?
#define PARAM_U_DELTA_MAX_FOR_END_OF_PRECHARGE 20 /* [volts] The maximum voltage difference during PreCharge, to close the relay. */


