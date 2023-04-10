#ifndef FASTMATH_H
#define FASTMATH_H

#include "stdint.h"

extern uint8_t sine_table_8[256];

extern uint8_t fast_sin8(uint8_t theta);
extern int fast_sin16(int theta);

// https://www.geeksforgeeks.org/fast-inverse-square-root/
// function to find the inverse square root
extern float inverse_rsqrt( float number );

#endif  // FASTMATH_H
