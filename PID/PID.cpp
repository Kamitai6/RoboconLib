#include "PID.h"


/*  p,i,d:gain  t:control cycle  max:max output  type:mode setting  */
PID::PID (double p, double i, double d, double t, double max, int type, double allow_e, double range)
 : kp(p), ki(i), kd(d), delta_t(t), abs_max_output(max), pid_type(type), allowable_error(allow_e), time_range(range){}

void PID::SetParameter (double p, double i, double d, double t, double max, int type, double allow_e, double range)
{
    kp = p; ki = i; kd = d; delta_t = t; abs_max_output = max; pid_type = type; allowable_error = allow_e; time_range = range;
}

void PID::SetInput(double *sensor_, double *target_) {
    sensor = sensor_;
    target = target_;
}

double PID::GetOutput() const {
    return output;
}

void PID::Reset()
{
    integral = 0;
    pre_error = 0;
}

int PID::Compute()
{
    double proportion, differential;
    
    error = *target - *sensor;
    
    if(pid_type == FIX_COMMAND) {
        if(last_target != *target) Reset();
        
        proportion   = kp * error;
        integral    += ki * error * delta_t;
        differential = kd * (error - pre_error) / delta_t;
        
        integral = Restrain(integral, abs_max_output);
        
        output_ = proportion + integral + differential;
        
    } else if(pid_type == FOLLOW_UP) {
        
        proportion   = kp * (error - pre_error);
        integral     = ki * error * delta_t;
        differential = kd * (proportion - pre_proportion) / kp / delta_t;
        
        output_ += proportion + integral + differential;
    }
    output = Restrain(output_, abs_max_output);
    
    last_target = *target;
    pre_error = error;
    pre_proportion = proportion;
    
    return IsConvergence();
}

int PID::IsConvergence()
{
    if(abs(error) <= allowable_error) {
        timer += delta_t;
        if(timer > time_range) return 1;
        else return 0;
    } else {
        timer = 0;
        return 0;
    }
}
