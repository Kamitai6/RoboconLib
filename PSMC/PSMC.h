#ifndef PSMC_H_
#define PSMC_H_

#include "StandardFunctions.h"


class PSMC
{
    public:
        constexpr PSMC(double p_gain_, double i_gain_, double d_gain_, double h_constant_, double max_output_, double delta_t_)
         : max_output(max_output_), a_constant(), b_constant(), c_constant(), target_value(nullptr), status_value(nullptr), pre_target_value(), pre_status_value(), proxy(), pre_proxy(), output()
        {
            if(i_gain_ == 0) {
                a_constant = h_constant_ / delta_t_;
                b_constant[0] = (d_gain_ + p_gain_*delta_t_) / (h_constant_ + delta_t_);
                b_constant[1] = (p_gain_*h_constant_ - d_gain_) / (h_constant_ + delta_t_);
                c_constant[0] = d_gain_ / (d_gain_ + p_gain_*delta_t_);
                c_constant[1] = delta_t_ / (d_gain_ + p_gain_*delta_t_);
            }
            else {
                a_constant = h_constant_ / delta_t_;
                b_constant[0] = (d_gain_ + p_gain_*delta_t_ + i_gain_*delta_t_*delta_t_) / (h_constant_ + delta_t_);
                b_constant[1] = (p_gain_*h_constant_ - d_gain_ + i_gain_*delta_t_*(2*h_constant_ + delta_t_)) / ((h_constant_ + delta_t_)*delta_t_);
                b_constant[2] = (p_gain_*h_constant_ - d_gain_ + i_gain_*delta_t_*h_constant_) / ((h_constant_ + delta_t_)*delta_t_);
                c_constant[0] = (2*d_gain_ + p_gain_*delta_t_) / (d_gain_ + p_gain_*delta_t_ + i_gain_*delta_t_*delta_t_);
                c_constant[1] = d_gain_ / (d_gain_ + p_gain_*delta_t_ + i_gain_*delta_t_*delta_t_);
                c_constant[2] = delta_t_*delta_t_ / (d_gain_ + p_gain_*delta_t_ + i_gain_*delta_t_*delta_t_);
            }
        }
        void SetInput(double *target_value_, double *status_value_);
        void Calculate();
        double GetOutput()const;
        
    private:
        void CalcPD();
        void CalcPID();
        double max_output;
        double a_constant, b_constant[3], c_constant[3];
        double *target_value, *status_value;
        double pre_target_value, pre_status_value;
        double proxy, pre_proxy;
        double output;
};

#endif
