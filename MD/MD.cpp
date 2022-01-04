#include "MD.h"


MD::MD(PinName pwm_, PinName dir_, int frequency_, double max_output_, int pwm_mode_)
 : Pwm(pwm_), Dire(dir_),
   min_output(0.01), max_output(max_output_)
{
    Pwm.period(1.0 / frequency_);
    if(pwm_mode_ == 0) pwm_mode = DriverType::SM;
    else pwm_mode = DriverType::LAP;
}

void MD::drive(double output)
{
    switch(pwm_mode)
    {
        case  DriverType::SM: driveSM(output); break;
        case DriverType::LAP: driveLAP(output); break;
        default: break;
    }
}

void MD::driveSM(double output)
{
    double abs_output = abs(output);
    if(abs_output <= max_output && abs_output >= min_output)
    {
        Pwm = abs_output;
        if(output > 0) Dire = 0;
        else Dire = 1;
    }
    else
    {
        Pwm = 0;
        Dire = 0;
    }
}

void MD::driveLAP(double output)
{
    double abs_output = abs(output);
    if(abs_output <= max_output && abs_output >= min_output)
    {
        if(output > 0) Pwm = 0.5 + (output * 0.5);
        else Pwm = 0.5 - (output * 0.5);
        Dire = 1;
    }
    else
    {
        Pwm = 0;
        Dire = 0;
    }
}
