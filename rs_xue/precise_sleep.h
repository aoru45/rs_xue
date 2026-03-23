#ifndef PRECISE_SLEEP_H_
#define PRECISE_SLEEP_H_

#include <cmath>
#include <thread>
#include <chrono>

#ifdef _WIN32
#include <windows.h>
#include <mmsystem.h>
#pragma comment(lib, "winmm.lib")

struct WinTimeHelper {
    WinTimeHelper() {
        TIMECAPS tc;
        if (timeGetDevCaps(&tc, sizeof(tc)) == TIMERR_NOERROR) {
            resolution_ = tc.wPeriodMin;
            timeBeginPeriod(resolution_);
        } else resolution_ = 1;
    }
    ~WinTimeHelper() { timeEndPeriod(resolution_); }
private:
    UINT resolution_;
};
static WinTimeHelper g_time_helper;   // 全局单例，main 前自动调
#endif

// 高精度睡眠，单位：秒，可带小数
inline void precise_sleep(double sec) {
    if (sec <= 0.0) return;

#ifdef _WIN32
    // --- Windows 分支 ---
    static LARGE_INTEGER freq;
    static BOOL inited = QueryPerformanceFrequency(&freq);
    (void)inited;                       // 仅第一次初始化
    const LONGLONG ticks_total = static_cast<LONGLONG>(sec * freq.QuadPart);

    LARGE_INTEGER t0, t1;
    QueryPerformanceCounter(&t0);

    // 1. 先 Sleep 整数毫秒（让出 CPU）
    DWORD whole_ms = static_cast<DWORD>(sec * 1000.0);
    if (whole_ms >= 1) ::Sleep(whole_ms - 1);   // 留 1 ms 给自旋

    // 2. 自旋补齐剩余时间
    for (;;) {
        QueryPerformanceCounter(&t1);
        if ((t1.QuadPart - t0.QuadPart) >= ticks_total) break;
        std::this_thread::yield();      // 或者 _mm_pause() / YieldProcessor()
    }

#else
    // --- Linux / POSIX 分支 ---
    using namespace std::chrono;
    // 转成纳秒绝对时间
    auto expire = steady_clock::now() + duration<double>(sec);
    timespec ts{};
    auto nsec = duration_cast<nanoseconds>(expire.time_since_epoch()).count();
    ts.tv_sec  = nsec / 1'000'000'000;
    ts.tv_nsec = nsec % 1'000'000'000;
    int ret;
    do {
        ret = clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &ts, nullptr);
    } while (ret == EINTR);   // 被信号打断则重试
#endif
}

#endif // PRECISE_SLEEP_H_