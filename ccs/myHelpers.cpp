

#include "ccs32_globals.h"

//#include <cstdarg>


uint16_t checkpointNumber;

/* Helper functions */

void setCheckpoint(uint16_t newcheckpoint) {
    checkpointNumber = newcheckpoint;
    Param::SetInt(Param::checkpoint, newcheckpoint);
}

void addToTrace(enum Module module, const char* s) {
       if (Param::GetInt(Param::logging) & module)
           printf("[%u] %s\r\n", rtc_get_ms(), s);
}
//void addToTrace(enum Module module, const char* format, ...) {
//   if (Param::GetInt(Param::logging) & module)
//       printf("[%u] ", rtc_get_ms());
//
//   va_list args;
//   va_start(args, format);
//   printf(format, args);
//   va_end(args);
//
//       printf("\r\n");
//   // canbus_addStringToTextTransmitBuffer(mySerialPrintOutputBuffer); /* print to the CAN */
//}

void addToTrace(enum Module module, const char * s, uint8_t* data, uint16_t len) {
   if (Param::GetInt(Param::logging) & module) {
      printf("[%u] %s ", rtc_get_ms(), s);
      for (uint16_t i = 0; i < len; i++)
         printf("%02x", data[i]);
      printf("\r\n");
   }
}

void addToTrace(enum Module module, const char * s, int16_t value) {
   if (Param::GetInt(Param::logging) & module) {
      printf("[%u] %s ", rtc_get_ms(), s);
      printf("%d\r\n", value);
   }
}


void sanityCheck(const char*) {
    /* todo: check the canaries, config registers, maybe stack, ... */
}

extern "C" void* memcpy(void* __restrict target, const void* __restrict source, size_t length)
{
    uint8_t* dst = (uint8_t*)target;
    const uint8_t* src = (const uint8_t*)source;

    while (length--) {
        *dst++ = *src++;
    }

    return dst;
};
