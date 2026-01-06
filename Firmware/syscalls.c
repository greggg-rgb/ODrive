/**
 ******************************************************************************
 * @file           : syscalls.c
 * @brief          : This file implements printf functionality
 ******************************************************************************
 */

#include <board.h>
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
int clock_gettime(clockid_t clk_id, struct timespec* tp) {
    (void)clk_id;
    if (tp) {
        uint32_t ticks = xTaskGetTickCount();
        tp->tv_sec = ticks / 1000;
        tp->tv_nsec = (ticks % 1000) * 1000000;
    }
    return 0;
}

int clock_getres(clockid_t clk_id, struct timespec* res) {
    (void)clk_id;
    if (res) {
        res->tv_sec = 0;
        res->tv_nsec = 1000000;  // 1ms resolution
    }
    return 0;
}

int clock_settime(clockid_t clk_id, const struct timespec* tp) {
    (void)clk_id;
    (void)tp;

    return -1;
}

int _getpid(void) {
    return 1;
}

int _kill(int pid, int sig) {
    (void)pid;
    (void)sig;
    return -1;
}

// _write is defined in communication.cpp
