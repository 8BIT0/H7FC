#ifndef __TRACE_ANALYSISER_H
#define __TRACE_ANALYSISER_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>
#include <string.h>

void trace_analysiser(uint32_t lr, uint32_t sp);

#ifdef __cplusplus
}
#endif

#endif
