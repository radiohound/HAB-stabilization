#pragma once
// Minimal Arduino stub for native unit tests.
// Provides just enough for heading_control.h to compile on the host PC.

#include <math.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>

#ifndef PI
#define PI 3.14159265358979323846f
#endif
#define TWO_PI      (2.0f * PI)
#define RAD_TO_DEG  (180.0f / PI)
#define DEG_TO_RAD  (PI / 180.0f)

// Minimal Serial stub — heading_control.h prints debug output when
// DEBUG_LEVEL >= 2.  We just swallow it silently in tests.
struct SerialStub {
    void begin(int) {}
    template<typename T>         void print(T)      {}
    template<typename T>         void println(T)    {}
    template<typename T, int N>  void print(T, int) {}
    void println() {}
};
extern SerialStub Serial;
