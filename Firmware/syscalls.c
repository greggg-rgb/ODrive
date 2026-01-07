/**
 ******************************************************************************
 * @file           : syscalls.c
 * @brief          : This file implements printf functionality
 ******************************************************************************
 */

#include <board.h>
#include <rt.h>
#include <sys/stat.h>
#include <sys/time.h>
#include <sys/times.h>
#include <sys/unistd.h>
#include <time.h>

// int _read(int file, char *data, int len) {}
// int _close(int file) {}
// int _lseek(int file, int ptr, int dir) {}
// int _fstat(int file, struct stat *st) {}
// int _isatty(int file) {}

extern char _end;           // provided by the linker script: it's end of statically allocated section, which is where the heap starts.
extern char _heap_end_max;  // provided by the linker script
void* _end_ptr = &_end;
void* _heap_end_max_ptr = &_heap_end_max;
void* heap_end_ptr = 0;

/* @brief Increments the program break (aka heap end)
 *
 * This is called internally by malloc once it runs out
 * of heap space. Malloc might expect a contiguous heap,
 * so we don't call the FreeRTOS pvPortMalloc here.
 * If this function returns -1, malloc will return NULL.
 * Note that if this function returns NULL, malloc does not
 * consider this as an error and will return the pointer 0x8.
 *
 * You should still be careful with using malloc though,
 * as it does not guarantee thread safety.
 *
 * @return A pointer to the newly allocated block on success
 *         or -1 otherwise.
 */
intptr_t _sbrk(size_t size) {
    intptr_t ptr;
    {
        uint32_t mask = cpu_enter_critical();
        if (!heap_end_ptr)
            heap_end_ptr = _end_ptr;
        if (heap_end_ptr + size > _heap_end_max_ptr) {
            ptr = -1;
        } else {
            ptr = (intptr_t)heap_end_ptr;
            heap_end_ptr += size;
        }
        cpu_exit_critical(mask);
    }
    return ptr;
}

void _exit(int status) {
    (void)status;
    NVIC_SystemReset();
    while (1);
}

// Required by lely-core for time functions
int _gettimeofday(struct timeval* tv, void* tz) {
    (void)tz;
    if (tv) {
        // Convert FreeRTOS tick count to timeval
        // Assuming 1000 Hz tick rate (1ms per tick)
        uint32_t ticks = xTaskGetTickCount();
        tv->tv_sec = ticks / 1000;
        tv->tv_usec = (ticks % 1000) * 1000;
    }
    return 0;
}

// Required by lely-core
int _times(struct tms* buf) {
    if (buf) {
        clock_t ticks = xTaskGetTickCount();
        buf->tms_utime = ticks;
        buf->tms_stime = 0;
        buf->tms_cutime = 0;
        buf->tms_cstime = 0;
    }
    return xTaskGetTickCount();
}

// For lely-core's timespec_get implementation
int clock_gettime(clockid_t clock_id, struct timespec* tp) {
    if (clock_id != CLOCK_REALTIME && clock_id != CLOCK_MONOTONIC) {
        return -1;
    }

    TickType_t ticks = xTaskGetTickCount();

    // Convert FreeRTOS ticks to seconds/nanoseconds
    tp->tv_sec = ticks / configTICK_RATE_HZ;
    tp->tv_nsec = (ticks % configTICK_RATE_HZ) * (1000000000L / configTICK_RATE_HZ);

    if (clock_id == CLOCK_REALTIME) {
        // Apply the offset saved during settime
        tp->tv_sec += __librt_realtime_offset.tv_sec;
        tp->tv_nsec += __librt_realtime_offset.tv_nsec;

        // Final normalization
        if (tp->tv_nsec >= 1000000000L) {
            tp->tv_sec++;
            tp->tv_nsec -= 1000000000L;
        } else if (tp->tv_nsec < 0) {
            tp->tv_sec--;
            tp->tv_nsec += 1000000000L;
        }
    }

    return 0;
}

int clock_getres(clockid_t clock_id, struct timespec* tp) {
    if (clock_id != CLOCK_REALTIME) {
        return -1;
    }
    if (tp) {
        tp->tv_sec = 0;
        tp->tv_nsec = 1000000000L / configTICK_RATE_HZ;
    }
    return 0;
}

int clock_settime(clockid_t clock_id, const struct timespec* tp) {
    // Also validate nanosecond range [0, 999,999,999]
    if (tp->tv_nsec < 0 || tp->tv_nsec >= 1000000000L) {
        return -1;
    }

    struct timespec mono;
    // Get current monotonic time to calculate the new offset
    clock_gettime(CLOCK_MONOTONIC, &mono);

    /* * Critical Section: Prevents any other task (like your CANopen stack)
     * from calling clock_gettime() and reading a partially updated offset.
     */
    vPortEnterCritical();

    __librt_realtime_offset.tv_sec = tp->tv_sec - mono.tv_sec;
    __librt_realtime_offset.tv_nsec = tp->tv_nsec - mono.tv_nsec;

    // Normalize the offset so tv_nsec is always positive [0, 999,999,999]
    if (__librt_realtime_offset.tv_nsec < 0) {
        __librt_realtime_offset.tv_sec--;
        __librt_realtime_offset.tv_nsec += 1000000000L;
    } else if (__librt_realtime_offset.tv_nsec >= 1000000000L) {
        __librt_realtime_offset.tv_sec++;
        __librt_realtime_offset.tv_nsec -= 1000000000L;
    }

    vPortExitCritical();

    return 0;
}

int _getpid(void) {
    return 1;
}

int _kill(int pid, int sig) {
    (void)pid;
    (void)sig;
    return -1;
}

int _close(int file) { return -1; }
int _fstat(int file, struct stat* st) {
    st->st_mode = S_IFCHR;
    return 0;
}
int _isatty(int file) { return 1; }
int _lseek(int file, int ptr, int dir) { return 0; }
int _read(int file, char* ptr, int len) { return 0; }

// _write is defined in communication.cpp
