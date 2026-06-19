#ifndef ATOMIC_UTILS_H
#define ATOMIC_UTILS_H
#ifdef _WIN32
#include <windows.h>
#define ATOMIC_ADD_AND_FETCH(ptr,val)   (InterlockedAdd((LONG volatile*)ptr,val))
#define ATOMIC_FETCH_AND_ADD(ptr,val)   (InterlockedExchangeAdd((LONG volatile*)ptr,val)+val)
#else
#define ATOMIC_ADD_AND_FETCH(ptr,val)   __sync_add_and_fetch(ptr,val)
#define ATOMIC_FETCH_AND_ADD(ptr,val)   __sync_fetch_and_add(ptr,val)
#endif
#endif