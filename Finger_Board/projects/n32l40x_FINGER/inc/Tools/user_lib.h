#ifndef USER_LIB_H
#define USER_LIB_H

#include <stdint.h>

#ifndef TRUE
#define TRUE 1
#endif  // !
#ifndef FALSE
#define FALSE 0
#endif  // !
#define LIMIT_MAX_MIN(x, max, min) (((x) <= (min)) ? (min) : (((x) >= (max)) ? (max) : (x)))
#ifndef PI
#define PI 3.14159265358979f
#endif
#ifndef PIx2
#define PIx2 6.2831853072f
#endif

#ifndef user_malloc
#define user_malloc malloc
#endif

typedef float fp32;

typedef enum { FAILED = 0, PASSED = !FAILED } Status;

float Sqrt(float x);
void delay_us_f(float us);
float getSign(float x);
float absf(float x);

#endif
