#ifndef MD_H
#define MD_H

#include "StandardFunctions.h"


enum class DriverType : uint8_t {
    SM = 0,
    LAP
};

class MD
{
    public:
        MD(PinName pwm_, PinName dir_, int frequency_ = 10000, double max_output_ = 0.95, int pwm_mode_ = 0);
        void drive(double output);
        void operator=(double output) {
            drive(output);
            return;
        }
        
    private:
        PwmOut Pwm;
        DigitalOut Dire;
        double min_output;
        double max_output;
        DriverType pwm_mode;
        
        void driveSM(double output);
        void driveLAP(double output);
};

#endif
