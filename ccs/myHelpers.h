#pragma once
/* Interface header for myHelpers.c */

/* Global Defines */

//#define STR_TMP_SIZE 400
//#define MY_SERIAL_PRINTBUFFERLEN 400

extern uint16_t checkpointNumber;

/* Global Functions */

extern void sanityCheck(const char *hint);
//extern void mySerialPrint(void);
extern void setCheckpoint(uint16_t newcheckpoint);
extern void showAsHex(uint8_t* arr, uint16_t len, const char* info);

