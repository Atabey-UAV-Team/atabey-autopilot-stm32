#ifndef APP_UTILS_MATH_UTILS_H
#define APP_UTILS_MATH_UTILS_H

#include <stdint.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

#define DEG2RAD(x)  ((x) * (M_PI / 180.0f))
#define RAD2DEG(x)  ((x) * (180.0f / M_PI))

static inline float constrainf(float x, float lo, float hi)
{
    if (x < lo) return lo;
    if (x > hi) return hi;
    return x;
}

static inline int32_t constraini(int32_t x, int32_t lo, int32_t hi)
{
    if (x < lo) return lo;
    if (x > hi) return hi;
    return x;
}

static inline float mapf(float x, float in_lo, float in_hi,
                                  float out_lo, float out_hi)
{
    return (x - in_lo) * (out_hi - out_lo) / (in_hi - in_lo) + out_lo;
}

#endif /* APP_UTILS_MATH_UTILS_H */
