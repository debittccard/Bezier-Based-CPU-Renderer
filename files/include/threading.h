#pragma once

#ifdef _WIN32
    #include <windows.h>

    typedef HANDLE thread_t;

    typedef DWORD (WINAPI *thread_func_t)(void*);

    static inline thread_t thread_create(thread_func_t func, void* arg) {
        return CreateThread(NULL, 0, func, arg, 0, NULL);
    }

    static inline void thread_join(thread_t t) {
        WaitForSingleObject(t, INFINITE);
        CloseHandle(t);
    }

#else
    #include <pthread.h>

    typedef pthread_t thread_t;
    typedef void* (*thread_func_t)(void*);

    static inline thread_t thread_create(thread_func_t func, void* arg) {
        thread_t t;
        pthread_create(&t, NULL, func, arg);
        return t;
    }

    static inline void thread_join(thread_t t) {
        pthread_join(t, NULL);
    }

#endif