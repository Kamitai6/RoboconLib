#ifndef MADGWICKFILTER_H
#define MADGWICKFILTER_H

#include "StandardFunctions.h"


class MadgwickFilter
{
    public:
        MadgwickFilter(double t = 0.01) : beta(0.01), q0(1.0), q1(), q2(), q3(), delta_t(t) {};
        void MadgwickAhrsUpdate(double gx, double gy, double gz, double ax, double ay, double az, double mx, double my, double mz);
        void MadgwickAhrsUpdate(double *gyro, double *acc, double *mag);
        void MadgwickAhrsUpdateIMU(double gx, double gy, double gz, double ax, double ay, double az);
        void getAttitude(double *q0_, double *q1_, double *q2_, double *q3_);
        void getEulerAngle(double *val);
        
    private:
        double beta;
        double q0, q1, q2, q3;
        double delta_t;
};


inline void MadgwickFilter::getAttitude(double *q0_, double *q1_, double *q2_, double *q3_) {
    *q0_ = q0;
    *q1_ = q1;
    *q2_ = q2;
    *q3_ = q3;
}

inline void MadgwickFilter::getEulerAngle(double *val) {
    double q0q0 = q0 * q0, q1q1q2q2 = q1 * q1 - q2 * q2, q3q3 = q3 * q3;
    val[0] = (atan2(2.0 * (q0 * q1 + q2 * q3), q0q0 - q1q1q2q2 + q3q3));
    val[1] = (-asin(2.0 * (q1 * q3 - q0 * q2)));
    val[2] = (atan2(2.0 * (q1 * q2 + q0 * q3), q0q0 + q1q1q2q2 - q3q3));
}

inline void MadgwickFilter::MadgwickAhrsUpdate(double gx, double gy, double gz, double ax, double ay, double az, double mx, double my, double mz) {
    static double acc_norm{};
    static double recip_norm{};
    static double s0{}, s1{}, s2{}, s3{};
    static double q_dot1{}, q_dot2{}, q_dot3{}, q_dot4{};
    static double hx{}, hy{};
    static double _2q0mx{}, _2q0my{}, _2q0mz{}, _2q1mx{}, _2bx{}, _2bz{}, _4bx{}, _4bz{}, _2q0{}, _2q1{}, _2q2{}, _2q3{}, _2q0q2{}, _2q_2q3{};
    static double q0q0{}, q0q1{}, q0q2{}, q0q3{}, q1q1{}, q1q2{}, q1q3{}, q2q2{}, q2q3{}, q3q3{};
 
    // Use IMU algorithm if magnetometer measurement invalid (avoids NaN in magnetometer normalisation)
    if((mx == 0.0) && (my == 0.0) && (mz == 0.0)) {
        MadgwickAhrsUpdateIMU(gx, gy, gz, ax, ay, az);
        return;
    }
 
    // Rate of change of quaternion from gyroscope
    q_dot1 = 0.5 * (-q1 * gx - q2 * gy - q3 * gz);
    q_dot2 = 0.5 * ( q0 * gx + q2 * gz - q3 * gy);
    q_dot3 = 0.5 * ( q0 * gy - q1 * gz + q3 * gx);
    q_dot4 = 0.5 * ( q0 * gz + q1 * gy - q2 * gx);
 
    // Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
    if(!((ax == 0.0) && (ay == 0.0) && (az == 0.0))) {
 
        // Normalise accelerometer measurement
        acc_norm = sqrt(ax * ax + ay * ay + az * az);
        recip_norm = 1.0 / acc_norm;
        ax *= recip_norm;
        ay *= recip_norm;
        az *= recip_norm;
 
        // Normalise magnetometer measurement
        recip_norm = 1.0 / sqrt(mx * mx + my * my + mz * mz);
        mx *= recip_norm;
        my *= recip_norm;
        mz *= recip_norm;
 
        // Auxiliary variables to avoid repeated arithmetic
        _2q0mx = 2.0 * q0 * mx;
        _2q0my = 2.0 * q0 * my;
        _2q0mz = 2.0 * q0 * mz;
        _2q1mx = 2.0 * q1 * mx;
        _2q0 = 2.0 * q0;
        _2q1 = 2.0 * q1;
        _2q2 = 2.0 * q2;
        _2q3 = 2.0 * q3;
        _2q0q2 = 2.0 * q0 * q2;
        _2q_2q3 = 2.0 * q2 * q3;
        q0q0 = q0 * q0;
        q0q1 = q0 * q1;
        q0q2 = q0 * q2;
        q0q3 = q0 * q3;
        q1q1 = q1 * q1;
        q1q2 = q1 * q2;
        q1q3 = q1 * q3;
        q2q2 = q2 * q2;
        q2q3 = q2 * q3;
        q3q3 = q3 * q3;
 
        // Reference direction of Earth's magnetic field
        hx = mx * q0q0 - _2q0my * q3 + _2q0mz * q2 + mx * q1q1 + _2q1 * my * q2 + _2q1 * mz * q3 - mx * q2q2 - mx * q3q3;
        hy = _2q0mx * q3 + my * q0q0 - _2q0mz * q1 + _2q1mx * q2 - my * q1q1 + my * q2q2 + _2q2 * mz * q3 - my * q3q3;
        _2bx = sqrt(hx * hx + hy * hy);
        _2bz = -_2q0mx * q2 + _2q0my * q1 + mz * q0q0 + _2q1mx * q3 - mz * q1q1 + _2q2 * my * q3 - mz * q2q2 + mz * q3q3;
        _4bx = 2.0 * _2bx;
        _4bz = 2.0 * _2bz;
 
        // Gradient decent algorithm corrective step
        s0 = -_2q2 * (2.0 * q1q3 - _2q0q2 - ax) + _2q1 * (2.0 * q0q1 + _2q_2q3 - ay) - _2bz * q2 * (_2bx * (0.5 - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (-_2bx * q3 + _2bz * q1) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + _2bx * q2 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5 - q1q1 - q2q2) - mz);
        s1 = _2q3 * (2.0 * q1q3 - _2q0q2 - ax) + _2q0 * (2.0 * q0q1 + _2q_2q3 - ay) - 4.0 * q1 * (1 - 2.0 * q1q1 - 2.0 * q2q2 - az) + _2bz * q3 * (_2bx * (0.5 - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (_2bx * q2 + _2bz * q0) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + (_2bx * q3 - _4bz * q1) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5 - q1q1 - q2q2) - mz);
        s2 = -_2q0 * (2.0 * q1q3 - _2q0q2 - ax) + _2q3 * (2.0 * q0q1 + _2q_2q3 - ay) - 4.0 * q2 * (1 - 2.0 * q1q1 - 2.0 * q2q2 - az) + (-_4bx * q2 - _2bz * q0) * (_2bx * (0.5 - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (_2bx * q1 + _2bz * q3) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + (_2bx * q0 - _4bz * q2) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5 - q1q1 - q2q2) - mz);
        s3 = _2q1 * (2.0 * q1q3 - _2q0q2 - ax) + _2q2 * (2.0 * q0q1 + _2q_2q3 - ay) + (-_4bx * q3 + _2bz * q1) * (_2bx * (0.5 - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (-_2bx * q0 + _2bz * q2) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + _2bx * q1 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5 - q1q1 - q2q2) - mz);
        recip_norm = 1.0 / sqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3); // normalise step magnitude
        s0 *= recip_norm;
        s1 *= recip_norm;
        s2 *= recip_norm;
        s3 *= recip_norm;
 
        //double deltaA = abs(acc_norm - 1.00);
        //beta = 0.1*exp(-1.0*deltaA*deltaA);
        //beta = 0.3*exp(-20.0*deltaA*deltaA);
        //beta = beta*exp(-30.0f*deltaA*deltaA);
        //beta = 1.0;
        //if(deltaA > 0.3)    beta = 0.0;
        
        // Apply feedback step
        q_dot1 -= beta * s0;
        q_dot2 -= beta * s1;
        q_dot3 -= beta * s2;
        q_dot4 -= beta * s3;
    }
    
    q0 += q_dot1 * delta_t;
    q1 += q_dot2 * delta_t;
    q2 += q_dot3 * delta_t;
    q3 += q_dot4 * delta_t;
 
    // Normalise quaternion
    recip_norm = 1.0 / sqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    q0 *= recip_norm;
    q1 *= recip_norm;
    q2 *= recip_norm;
    q3 *= recip_norm;
}

inline void MadgwickFilter::MadgwickAhrsUpdate(double *gyro, double *acc, double *mag) {
    double gx = gyro[0];
    double gy = gyro[1];
    double gz = gyro[2];
    double ax = acc[0];
    double ay = acc[1];
    double az = acc[2];
    double mx = mag[0];
    double my = mag[1];
    double mz = mag[2];
 
    // Use IMU algorithm if magnetometer measurement invalid (avoids NaN in magnetometer normalisation)
    if((mx == 0.0) && (my == 0.0) && (mz == 0.0)) {
        MadgwickAhrsUpdateIMU(gx, gy, gz, ax, ay, az);
        return;
    }
    else MadgwickAhrsUpdate(gx, gy, gz, ax, ay, az, mx, my, mz);
}

inline void MadgwickFilter::MadgwickAhrsUpdateIMU(double gx, double gy, double gz, double ax, double ay, double az) {
    static double recip_norm{};
    static double s0{}, s1{}, s2{}, s3{};
    static double q_dot1{}, q_dot2{}, q_dot3{}, q_dot4{};
    static double _2q0{}, _2q1{}, _2q2{}, _2q3{}, _4q0{}, _4q1{}, _4q2{}, _8q1{}, _8q2{}, q0q0{}, q1q1{}, q2q2{}, q3q3{};
    static double acc_norm{};
 
    // Rate of change of quaternion from gyroscope
    q_dot1 = 0.5 * (-q1 * gx - q2 * gy - q3 * gz);
    q_dot2 = 0.5 * (q0 * gx + q2 * gz - q3 * gy);
    q_dot3 = 0.5 * (q0 * gy - q1 * gz + q3 * gx);
    q_dot4 = 0.5 * (q0 * gz + q1 * gy - q2 * gx);
 
    // Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
    if(!((ax == 0.0) && (ay == 0.0) && (az == 0.0))) {
 
        // Normalise accelerometer measurement
        acc_norm = sqrt(ax * ax + ay * ay + az * az);
        recip_norm = 1.0 / acc_norm;
        ax *= recip_norm;
        ay *= recip_norm;
        az *= recip_norm;
        
        // Auxiliary variables to avoid repeated arithmetic
        _2q0 = 2.0 * q0;
        _2q1 = 2.0 * q1;
        _2q2 = 2.0 * q2;
        _2q3 = 2.0 * q3;
        _4q0 = 4.0 * q0;
        _4q1 = 4.0 * q1;
        _4q2 = 4.0 * q2;
        _8q1 = 8.0 * q1;
        _8q2 = 8.0 * q2;
        q0q0 = q0 * q0;
        q1q1 = q1 * q1;
        q2q2 = q2 * q2;
        q3q3 = q3 * q3;
 
        // Gradient decent algorithm corrective step
        s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay;
        s1 = _4q1 * q3q3 - _2q3 * ax + 4.0 * q0q0 * q1 - _2q0 * ay - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * az;
        s2 = 4.0 * q0q0 * q2 + _2q0 * ax + _4q2 * q3q3 - _2q3 * ay - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * az;
        s3 = 4.0 * q1q1 * q3 - _2q1 * ax + 4.0 * q2q2 * q3 - _2q2 * ay;
        recip_norm = 1.0 / sqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3); // normalise step magnitude
        s0 *= recip_norm;
        s1 *= recip_norm;
        s2 *= recip_norm;
        s3 *= recip_norm;
        
        //double deltaA = abs(acc_norm - 1.0);
        //beta = 0.1*exp(-1.0*deltaA*deltaA);
        //beta = 0.3*exp(-20.0*deltaA*deltaA);
        //beta = beta*exp(-30.0f*deltaA*deltaA);
        //beta = 1.0;
        //if(deltaA > 0.3) beta = 0.0;
        
        // Apply feedback step
        q_dot1 -= beta * s0;
        q_dot2 -= beta * s1;
        q_dot3 -= beta * s2;
        q_dot4 -= beta * s3;
    }
    
    q0 += q_dot1 * delta_t;
    q1 += q_dot2 * delta_t;
    q2 += q_dot3 * delta_t;
    q3 += q_dot4 * delta_t;
 
    // Normalise quaternion
    recip_norm = 1.0 / sqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    q0 *= recip_norm;
    q1 *= recip_norm;
    q2 *= recip_norm;
    q3 *= recip_norm;
}

#endif
