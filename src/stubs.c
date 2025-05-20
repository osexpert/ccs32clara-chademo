#include <stddef.h>
#include <stdint.h>
#include <libopencm3/cm3/common.h>
#include <libopencm3/cm3/memorymap.h>
#include <libopencm3/cm3/itm.h>
#include <libopencm3/cm3/scs.h>
#include <libopencm3/stm32/dbgmcu.h>

//#include <sys/stat.h>
//#include <sys/types.h>
//#include <errno.h>
//#include <unistd.h>
//
//extern char _end; // Provided by the linker script
//static char* heap_end = &_end;
//
//caddr_t _sbrk(int incr) {
//    extern char _estack; // top of the stack
//    char* prev_heap_end = heap_end;
//    if (heap_end + incr > &_estack) {
//        errno = ENOMEM;
//        return (caddr_t)-1;
//    }
//    heap_end += incr;
//    return (caddr_t)prev_heap_end;
//}
//
//int _write(int file, char* ptr, int len) {
//    // Implement with UART or ignore if not needed
//    return len;
//}
//
//int _read(int file, char* ptr, int len) {
//    // Implement with UART or return 0
//    return 0;
//}
//
//int _close(int file) {
//    return -1;
//}
//
//int _fstat(int file, struct stat* st) {
//    st->st_mode = S_IFCHR;
//    return 0;
//}
//
//int _lseek(int file, int ptr, int dir) {
//    return 0;
//}



//void trace_send_blocking8(int stimulus_port, char c)
//{
//    // If setup hasn't run, SCS_DEMCR.TRCENA might be 0.
//    // But debug_trace_setup() sets it, so you can comment this out *if you're sure* setup is always called.
//    // if (!(SCS_DEMCR & SCS_DEMCR_TRCENA)) {
//    //     return;
//    // }
//
//    // Already enabled in debug_trace_setup()
//    // if (!(DBGMCU_CR & DBGMCU_CR_TRACE_IOEN)) {
//    //     return;
//    // }
//
//    // Already enabled in debug_trace_setup()
//    // if (!(ITM_TCR & ITM_TCR_ITMENA)) {
//    //     return;
//    // }
//
//    // Only needed if you dynamically enable/disable stimulus ports elsewhere
//    // but since setup enables port 31 and this always uses port 31, you can skip this too:
//    // if (!(ITM_TER[0] & (1 << stimulus_port))) {
//    //     return;
//    // }
//
//
//  //  If there's a chance the function might be called before debug_trace_setup(), and you want to avoid risk of a fault, you could also check:
////    if (!(SCS_DEMCR & SCS_DEMCR_TRCENA)) return;
//
//    // debugger attached?
//    if ((SCS_DHCSR & SCS_DHCSR_C_DEBUGEN))
//    {
//
//
//
//        // Wait until ITM FIFO is ready (this is still necessary)
//        while (!(ITM_STIM8(stimulus_port) & ITM_STIM_FIFOREADY));
//
//        // Send the character
//        ITM_STIM8(stimulus_port) = c;
//    }
//}
//



//double __aeabi_f2d(float f) {
//    return (double)f;
//}
// 

//void* memcpy(void* __restrict, const void* __restrict, size_t);


void memcpy(void* target, void* source, size_t length)
{
    uint8_t* dst = (uint8_t*)target;
    const uint8_t* src = (const uint8_t*)source;

    while (length--) {
        *dst++ = *src++;
    }
};

