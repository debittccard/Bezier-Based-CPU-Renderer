#ifndef TIMER_UTILS_H
#define TIMER_UTILS_H
#ifdef _WIN32
#include <windows.h>
typedef struct { double ms; } TimerStamp;
static inline TimerStamp get_timestamp(void) {
    LARGE_INTEGER f, c; QueryPerformanceFrequency(&f); QueryPerformanceCounter(&c);
    TimerStamp ts; ts.ms = (double)c.QuadPart * 1000.0 / f.QuadPart; return ts;
}
static inline double timespec_to_ms(const TimerStamp *t) { return t->ms; }
static inline double elapsed_ms(TimerStamp a, TimerStamp b) { return b.ms - a.ms; }
#else
#include <time.h>
typedef struct timespec TimerStamp;
static inline TimerStamp get_timestamp(void) {
    struct timespec ts; clock_gettime(CLOCK_MONOTONIC, &ts); return ts;
}
static inline double timespec_to_ms(const TimerStamp *t) {
    return t->tv_sec * 1000.0 + t->tv_nsec / 1e6;
}
static inline double elapsed_ms(TimerStamp a, TimerStamp b) {
    return timespec_to_ms(&b) - timespec_to_ms(&a);
}
#endif
#endif