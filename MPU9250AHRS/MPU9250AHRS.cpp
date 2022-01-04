#include "MPU9250AHRS.h"


MPU9250AHRS::MPU9250AHRS(PinName sda, PinName scl, double first_mag_offset[4], double t)
 : mpu9250(sda, scl), madgwick(delta_t), 
   roll(), pitch(), yaw(), roll_(), pitch_(), yaw_(), acc_val(), gyro_val(), mag_val(), delta_t(t)
{
    double raw_value[6]{};
    double offset[9]{};
    double sample_angle[3]{};
    
    for(int i{}; i < 4; ++i) {
        mag_offset[i] = first_mag_offset[i];
    }
    wait(1.0); //Consider vibration
    
    for (int i = 0; i < 1000; i++) {
        mpu9250.GetAcc(&raw_value[0], &raw_value[1], &raw_value[2]);
        mpu9250.GetGyro(&raw_value[3], &raw_value[4], &raw_value[5]);
        for(int j = 0; j < 6; j++) {
            offset[j] += raw_value[j];
        }
    }
    
    for(int i = 0; i < 6; i++) {
        offset[i] /= 1000;
    }
    offset[2] -= 1; //gravity
    
    mpu9250.SetOffset(offset);
}

void MPU9250AHRS::CalcEulerHorns() {
    mpu9250.GetAcc(acc_val);
    mpu9250.GetGyro(gyro_val);
    mpu9250.GetMag(mag_val);
    
    Calculate();
}

void MPU9250AHRS::Calculate() {
    double val[3]{};
    
    for (int i = 0; i < 3; i++) gyro_val[i] *= DegreeToRadian(gyro_val[i]);
    madgwick.MadgwickAhrsUpdate(gyro_val[0], gyro_val[1], gyro_val[2], 
                                acc_val[0],  acc_val[1],  acc_val[2], 
                                0.0,  0.0,  0.0);
    madgwick.getEulerAngle(val);
    
    roll =  RadianToDegree(val[0]);
    pitch = RadianToDegree(val[1]);
    yaw =   RadianToDegree(val[2]);
    roll_ += RadianToDegree(gyro_val[0] * delta_t);
    pitch_ += RadianToDegree(gyro_val[1] * delta_t);
    yaw_ += RadianToDegree(gyro_val[2] * delta_t);
}

void MPU9250AHRS::CalibrateMag() {}
