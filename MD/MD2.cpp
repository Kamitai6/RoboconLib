#include "MD2.h"


MD2::MD2(PinName pwm_1_, PinName pwm_2_, int frequency_, double max_output_)
 : Pwm1(pwm_1_),Pwm2(pwm_2_),
   min_output(0.01), max_output(max_output_)
{
    Pwm1.period(1.0 / frequency_);
    Pwm2.period(1.0 / frequency_);
}

void MD2::drive(double output)
{
    if(abs(output) > max_output)
    {
        Pwm1 = 0;
        Pwm2 = 0;
    }
    else if(output >= min_output)
    {
        Pwm1 = output;
        Pwm2 = 0;
    }
    else if(output <= min_output)
    {
        Pwm1 = 0;
        Pwm2 = -output;
    }
    else
    {
        Pwm1 = 0;
        Pwm2 = 0;
    }
}
