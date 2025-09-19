

#include "ccs32_globals.h"

uint16_t checkpointNumber;

/* Helper functions */

void setCheckpoint(uint16_t newcheckpoint) {
    checkpointNumber = newcheckpoint;
    Param::SetInt(Param::checkpoint, newcheckpoint);
}

void addToTrace(enum Module module, const char* format, ...) {
    if (Param::GetInt(Param::logging) & module)
    {
        va_list args;
        va_start(args, format);
        vprintf(format, args);
        println();
    }
}

void addToTrace_bytes(enum Module module, const char * s, uint8_t* data, uint16_t len) {
   if (Param::GetInt(Param::logging) & module) {
      printf("%s ", s);
      for (uint16_t i = 0; i < len; i++)
         printf("%02x", data[i]);
      println();
   }
}

void showAsHex(uint8_t* arr, uint16_t len, const char* info)
{
    printf("%s has %d bytes: ", info, len);
    for (uint16_t i = 0; i < len; ++i) {
        printf("%02X", arr[i]);
    }
    println();
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

