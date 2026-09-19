#ifndef SENSORLESS_AB_MAIN_H
#define SENSORLESS_AB_MAIN_H

#include <stdint.h>

/* Host-only IRQ shims. The replay runs a single-threaded ISR sequence. */
static inline uint32_t __get_PRIMASK(void) { return 0U; }
static inline void __disable_irq(void) { }
static inline void __enable_irq(void) { }

#endif
