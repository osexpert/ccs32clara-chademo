#include <stdint.h>
#include "logging.h"
#include "printf.h"
#include "ccs_params.h"

void log(enum Module module, const char* format, ...) {
    if (_ccs_params.logging & module)
    {
        printf("%s ", ModuleName(module));

        va_list args;
        va_start(args, format);
        vprintf(format, args);
        println();
    }
}

void log_bytes(enum Module module, const char* s, uint8_t* data, uint16_t len) {
    if (_ccs_params.logging & module) {

        printf("%s ", ModuleName(module));
        printf("%s ", s);
        for (uint16_t i = 0; i < len; i++)
            printf("%02x", data[i]);
        println();
    }
}