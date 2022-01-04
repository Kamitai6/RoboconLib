#include "RotaryEncoder.h"


RotaryEncoder::RotaryEncoder(PinName a, PinName b, int pulses_per_rotation, double dt_)
 : pinA(a, PullUp), pinB(b, PullUp),
   rotations(), pre_rotations(), delta_t(dt_)
{
    rotations_per_interrupt = 1.0 / (pulses_per_rotation * 4);
    pinA.rise(this, &RotaryEncoder::ARaise_);
    pinA.fall(this, &RotaryEncoder::AFall_);
    pinB.rise(this, &RotaryEncoder::BRaise_);
    pinB.fall(this, &RotaryEncoder::BFall_);
}


void RotaryEncoder::ChangeDirection()
{
    rotations_per_interrupt *= -1;
}


void RotaryEncoder::DefineNowRotations(double n)
{
    rotations = n;
}

double RotaryEncoder::GetRotations() const
{
    return rotations;
}

double RotaryEncoder::GetRps()
{
    double rps = (rotations - pre_rotations) / delta_t;
    pre_rotations = rotations;
    return rps;
}


void RotaryEncoder::ARaise_()
{
    if(!pinB)
        rotations += rotations_per_interrupt;
    else rotations -= rotations_per_interrupt;
}


void RotaryEncoder::AFall_()
{
    if(pinB)
        rotations += rotations_per_interrupt;
    else rotations -= rotations_per_interrupt;
}


void RotaryEncoder::BRaise_()
{
    if(pinA)
        rotations += rotations_per_interrupt;
    else rotations -= rotations_per_interrupt;
}


void RotaryEncoder::BFall_()
{
    if(!pinA)
        rotations += rotations_per_interrupt;
    else rotations -= rotations_per_interrupt;
}
