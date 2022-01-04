#include "PSMC.h"


void PSMC::SetInput(double *target_value_, double *status_value_) {
    target_value = target_value_;
    status_value = status_value_;
}

void PSMC::Calculate() {
    if(b_constant[2] == 0 && c_constant[2] == 0.0) {
        CalcPD();
    }
    else {
        CalcPID();
    }
}

double PSMC::GetOutput() const {
    return output;
}

void PSMC::CalcPD() {
    double delta_target_value = *target_value - pre_target_value;
    double delta_status_value = *status_value - pre_status_value;
    
    double error = (*target_value - *status_value) + a_constant*(delta_target_value - delta_status_value);
    double result = b_constant[0]*error + b_constant[1]*proxy;
    output = Restrain<double>(result, max_output);
    
    proxy = c_constant[0]*proxy + c_constant[1]*output;
    
    pre_target_value = *target_value;
    pre_status_value = *status_value;
}

void PSMC::CalcPID() {
    double delta_target_value = *target_value - pre_target_value;
    double delta_status_value = *status_value - pre_status_value;
    
    double error = (*target_value - *status_value) + a_constant*(delta_target_value - delta_status_value);
    double result = b_constant[0]*error + b_constant[1]*proxy - b_constant[2]*pre_proxy;
    output = Restrain<double>(result, max_output);
    
    double patch = proxy;
    proxy = c_constant[0]*proxy - c_constant[1]*pre_proxy + c_constant[2]*output;
    pre_proxy = patch;
    
    pre_target_value = *target_value;
    pre_status_value = *status_value;
}