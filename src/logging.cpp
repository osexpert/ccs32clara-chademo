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
