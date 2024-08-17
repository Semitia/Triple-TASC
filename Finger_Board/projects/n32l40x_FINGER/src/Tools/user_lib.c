#include "user_lib.h"

#include <math.h>
#include <string.h>

/*  us */
void delay_us_f(float us) {
    int i;
    int total_count = (int)(us * 37);
    for (i = 0; i < total_count; i++);
}

// 快速开方
fp32 invSqrt(fp32 num) {
    fp32 halfnum = 0.5f * num;
    fp32 y       = num;
    long i       = *(long *)&y;
    i            = 0x5f3759df - (i >> 1);
    y            = *(fp32 *)&i;
    y            = y * (1.5f - (halfnum * y * y));
    return y;
}
float Sqrt(float x) {
    float y;
    float delta;
    float maxError;

    if (x <= 0) {
        return 0;
    }

    // initial guess
    y = x / 2;

    // refine
    maxError = x * 0.001f;

    do {
        delta = (y * y) - x;
        y -= delta / (2 * y);
    } while (delta > maxError || delta < -maxError);

    return y;
}

// 判断符号位
fp32 sign(fp32 value) {
    if (value >= 0.0f) {
        return 1.0f;
    } else {
        return -1.0f;
    }
}

float absf(float x) { return x > 0 ? x : -x; }
