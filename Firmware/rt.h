#ifndef LIBRT_INTERN_RT_H_
#define LIBRT_INTERN_RT_H_

#include <time.h>

#include "FreeRTOS.h"

#ifndef CLOCK_REALTIME
#define CLOCK_REALTIME 0
#endif

#ifndef CLOCK_MONOTONIC
#define CLOCK_MONOTONIC 1
#endif

// Offset used to calculate CLOCK_REALTIME from CLOCK_MONOTONIC
extern struct timespec __librt_realtime_offset;

#endif