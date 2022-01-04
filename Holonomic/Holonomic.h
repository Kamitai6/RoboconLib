#ifndef Holonomic_H_
#define Holonomic_H_

#include "Kinematics.h"


template<int NUMBER>
class Holonomic : public Kinematics<Holonomic<NUMBER>, NUMBER> {
    using Kinematics<Holonomic<NUMBER>, NUMBER>::Kinematics;
    public:
        Holonomic(array<array<double, ELEMENT>, NUMBER> &matrix_, double max) : Kinematics<Holonomic<NUMBER>, NUMBER>(max), matrix(matrix_) {}
        
        void Calculate(array<double, ELEMENT> &velocity) {
            if(this->length == 0.0) {
                for(int i{}; i < NUMBER; ++i) {
                    for(int j{}; j < ELEMENT; ++j) {
                        this->value.at(i) += matrix.at(i).at(j) * velocity.at(j);
                    }
                }
            }
            else {
                for(int i{}; i < NUMBER; ++i) {
                    this->value.at(i) = -sin(this->theta.at(i)) * velocity.at(0) + cos(this->theta.at(i)) * velocity.at(1) + this->length * velocity.at(2);
                }
            }
            
            double max_value = *max_element(this->value.begin(), this->value.end(), [](double a, double b){return (abs(a) < abs(b));});
            if(max_value > this->max_velocity) {
                for(auto &x : this->value) {
                    x *= this->max_velocity / max_value;
                }
            }
        }
        
        auto GetValue() const {
            return this->value;
        }
        
    private:
        array<array<double, ELEMENT>, NUMBER> matrix;
};

#endif