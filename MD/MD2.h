#ifndef MD2_H
#define MD2_H

#include "StandardFunctions.h"


class MD2
{
    public:
        MD2(PinName pwm_1_, PinName pwm_2_, int frequency_ = 10000, double max_output_ = 0.95);
        void drive(double output);
        
    private:
        PwmOut Pwm1;
        PwmOut Pwm2;
        void operator=(double output) {
            drive(output);
            return;
        }
        
        double min_output;
        double max_output;
};

#endif
