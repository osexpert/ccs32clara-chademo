
/* definitions for logging */

#ifndef MY_LOGGING_H
#define MY_LOGGING_H

enum Module
{
   MOD_CONNMGR = 1,
   MOD_HWIF = 2,
   MOD_HOMEPLUG = 4,
   MOD_PEV = 8,
   MOD_QCA = 16,
   MOD_TCP = 32,
   MOD_TCPTRAFFIC = 64,
   MOD_IPV6 = 128,
   MOD_MODEMFINDER = 256,
   MOD_SDP = 512,
   MOD_ETHTRAFFIC = 1024
};

/* Here we define, which logging data is produced after power-on */

//the orginal
//#define DEFAULT_LOGGINGMASK (MOD_TCPTRAFFIC | MOD_HOMEPLUG | MOD_PEV | MOD_SDP | MOD_TCP | MOD_CONNMGR | MOD_ETHTRAFFIC | MOD_HWIF)

// slimmer
#define DEFAULT_LOGGINGMASK ( MOD_MODEMFINDER | MOD_HOMEPLUG | MOD_PEV | MOD_SDP | MOD_TCP | MOD_CONNMGR | MOD_HWIF)

#endif