#ifndef STANDARD_FUNCTION_H_
#define STANDARD_FUNCTION_H_

#include "mbed.h"
#include <string>
#include <vector>
#include <array>
#include <algorithm>


constexpr double PI = 3.141592653589793;
constexpr double G = 9.80665;

template<typename T>
static inline T DegreeToRadian(T degree) {
    return degree * 0.017453292519943;
}
template<typename T>
static inline T RadianToDegree(T radian) {
    return radian * 57.295779513082320;
}

template<typename T>
static inline T Restrain(T x, T limit) {
    return ((x)<(-limit)?(-limit):((x)>(limit)?(limit):(x)));
}

template<typename T>
static inline T Convert(T x, T in_min, T in_max, T out_min, T out_max){
    return static_cast<T>((x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min);
}

#endif
