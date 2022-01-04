#ifndef PID_H
#define PID_H

#include "StandardFunctions.h"

constexpr int FIX_COMMAND = 0;
constexpr int FOLLOW_UP = 1;


//PID calculator
class PID
{
    public:
        PID(double p, double i, double d, double t, double max, int type, double allow_e, double range);
        
        int Compute();
        void SetParameter(double p, double i, double d, double t, double max, int type, double allow_e, double range);
        void SetInput(double* sensor_, double* target_);
        double GetOutput()const;
    
    private:
        double kp, ki, kd, delta_t, abs_max_output, time_range;
        int pid_type;
        
        double output; //result
        double *sensor, *target; //sensor value pointer & target value pointer
        double timer;
        double output_;
        double integral;
        double error, pre_error;
        double pre_proportion;
        double last_target;
        double start_time;
        double allowable_error;
        
        int IsConvergence();
        void Reset();
};

#endif
