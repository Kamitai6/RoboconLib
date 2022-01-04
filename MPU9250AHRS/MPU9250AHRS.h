#ifndef MPU9250AHRS_H
#define MPU9250AHRS_H

#include "StandardFunctions.h"
#include "MPU9250I2C.h"
#include "MadgwickFilter.h"


class MPU9250AHRS
{
    public:
        MPU9250AHRS(PinName sda, PinName scl, double first_mag_offset[4], double t = 0.01);
        void CalcEulerHorns();
        void CalibrateMag();
        double roll, pitch, yaw, roll_, pitch_, yaw_;

    private:
        double acc_val[3];
        double gyro_val[3];
        double mag_val[3];
        double mag_offset[4];
        double delta_t;
        
        void Calculate(); //MadgwickFilterを使ってレート積分
        
        MPU9250I2C mpu9250;
        MadgwickFilter madgwick;
};

#endif