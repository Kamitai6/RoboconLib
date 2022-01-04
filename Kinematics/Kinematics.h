#ifndef KINEMATICS_H_
#define KINEMATICS_H_

#include "StandardFunctions.h"

static constexpr int ELEMENT = 3;


template<class NAME, int NUMBER>
class Kinematics {
    public:
        Kinematics(double max) : theta{}, length(), max_velocity(max), value{} {
            static_assert(is_same<decltype(declval<NAME>().Calculate(declval<array<double, ELEMENT>>())), void>::value);
            static_assert(is_same<decltype(declval<NAME>().GetValue()), array<double, ELEMENT>>::value);
        }
        Kinematics(array<double, NUMBER> &theta_, double length_, double max) : theta(theta_), length(length_), max_velocity(max), value{} {
            static_assert(is_same<decltype(declval<NAME>().Calculate(declval<array<double, ELEMENT>>())), void>::value);
            static_assert(is_same<decltype(declval<NAME>().GetValue()), array<double, ELEMENT>>::value);
        }
    protected:
        array<double, NUMBER> theta;
        double length;
        double max_velocity;
        array<double, NUMBER> value;
};

#endif