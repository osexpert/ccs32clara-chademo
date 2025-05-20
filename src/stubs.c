#include <stddef.h>
#include <stdint.h>
#include <libopencm3/cm3/common.h>
#include <libopencm3/cm3/memorymap.h>
#include <libopencm3/cm3/itm.h>
#include <libopencm3/cm3/scs.h>
#include <libopencm3/stm32/dbgmcu.h>

//double __aeabi_f2d(float f) {
//    return (double)f;
//}
// 


void memcpy(void* target, void* source, size_t length)
{
    uint8_t* dst = (uint8_t*)target;
    const uint8_t* src = (const uint8_t*)source;

    while (length--) {
        *dst++ = *src++;
    }
};

