#include "MPU9250I2C.h"


MPU9250I2C::MPU9250I2C(PinName sda, PinName scl, MPU9250_AD0 celect)
 : i2c(sda, scl),
   acc_coef(MPU9250_ACC_LSB),
   gyro_coef(MPU9250_GYRO_LSB),
   mag_coef(MPU9250_MAG_LSB),
   acc_offset(),
   gyro_offset(),
   mag_offset(),
   mag_sensitivity_adjust()
{
    if(celect == AD0_HIGH) address = static_cast<char>(MPU6500_ADDR_HIGH);
    else address = static_cast<char>(MPU6500_ADDR_LOW);
    
    i2c.frequency(400000); //Hz
    Init();
}

void MPU9250I2C::Init() {
    WriteReg(address, MPU6500_PWR_MGMT_1, 0x80); //reset
    wait_ms(100);
    WriteReg(address, MPU6500_PWR_MGMT_1, 0x00); //startup
    SetBypass(true);
    SetGyro();
    SetAcc();
    SetLPF();
    SetRate();
    SetMag();
    SetSensors();
}

bool MPU9250I2C::SensorCheck() {
    if((ReadReg(address, AM_I_MPU6500)   ==  I_AM_MPU6500)
    && (ReadReg(AK8963_ADDR, AM_I_AK8963) == I_AM_AK8963)) {
        return true;
    }
    else return false;
}

void MPU9250I2C::SetAcc(MPU9250_ACC_RANGE acc_range) {
    char send_message{};
    switch(acc_range) {
        case MPU6500_ACC_2G:
            send_message = static_cast<char>(MPU6500_ACC_2G << 3);
            acc_coef = MPU9250_ACC_LSB;
            break;
        case MPU6500_ACC_4G:
            send_message = static_cast<char>(MPU6500_ACC_4G << 3);
            acc_coef = MPU9250_ACC_LSB * 2.0;
            break;
        case MPU6500_ACC_8G:
            send_message = static_cast<char>(MPU6500_ACC_8G << 3);
            acc_coef = MPU9250_ACC_LSB * 4.0;
            break;
        case MPU6500_ACC_16G:
            send_message = static_cast<char>(MPU6500_ACC_16G << 3);
            acc_coef = MPU9250_ACC_LSB * 8.0;
            break;
    }
    WriteReg(address, MPU6500_ACCEL_CONFIG, send_message);
}

void MPU9250I2C::SetGyro(MPU9250_GYRO_RANGE gyro_range) {
    char fchoice = ReadReg(address, MPU6500_GYRO_CONFIG) & 0x03;
    char send_message{};
    switch(gyro_range) {
        case MPU6500_GYRO_250DPS:
            send_message = static_cast<char>(MPU6500_GYRO_250DPS << 3) | fchoice;
            gyro_coef = MPU9250_GYRO_LSB;
            break;
        case MPU6500_GYRO_500DPS:
            send_message = static_cast<char>(MPU6500_GYRO_500DPS << 3) | fchoice;
            gyro_coef = MPU9250_GYRO_LSB * 2.0;
            break;
        case MPU6500_GYRO_1000DPS:
            send_message = static_cast<char>(MPU6500_GYRO_1000DPS << 3) | fchoice;
            gyro_coef = MPU9250_GYRO_LSB * 4.0;
            break;
        case MPU6500_GYRO_2000DPS:
            send_message = static_cast<char>(MPU6500_GYRO_2000DPS << 3) | fchoice;
            gyro_coef = MPU9250_GYRO_LSB * 8.0;
            break;
    }
    WriteReg(address, MPU6500_GYRO_CONFIG, send_message);
}

void MPU9250I2C::SetMag() {
    char data[3]{};
    SetBypass(true);
    WriteReg(AK8963_ADDR, AK8963_CNTL1, 0x10);
    WriteReg(AK8963_ADDR, AK8963_CNTL1, 0x1F);
    ReadReg (AK8963_ADDR, AK8963_ASAX, data, 3);
    for(int i = 0; i < 3; i++) {
        mag_sensitivity_adjust[i] = static_cast<long>(data[i]) + 128;
    }
    WriteReg(AK8963_ADDR, AK8963_CNTL1, 0x10);
    SetBypass(false);
    WriteReg(address, MPU6500_I2C_MST_CTRLI2C_MST_CTRL, 0x40);
    WriteReg(address, MPU6500_I2C_SLV0_ADDR, 0x8C);
    WriteReg(address, MPU6500_I2C_SLV0_REG, 0x02);
    WriteReg(address, MPU6500_I2C_SLV0_CTRL, 0x88);
    WriteReg(address, MPU6500_I2C_SLV1_ADDR, 0x0C);
    WriteReg(address, MPU6500_I2C_SLV1_REG, 0x0A);
    WriteReg(address, MPU6500_I2C_SLV1_CTRL, 0x81);
    WriteReg(address, MPU6500_I2C_SLV1_DO, 0x11);
    WriteReg(address, MPU6500_I2C_I2C_MST_DELAY_CTRL, 0x03);
}

void MPU9250I2C::SetLPF (MPU9250_GYRO_LPF g_lpf, MPU9250_ACC_LPF a_lpf) {
    WriteReg(address, MPU6500_CONFIG, g_lpf);
    WriteReg(address, MPU6500_ACCEL_CONFIG2, a_lpf);
}

void MPU9250I2C::SetRate(int rate, int m_rate) {
    if (rate < 4) rate = 4;
    else if(rate > 1000) rate = 1000;
    char send_message = 1000 / rate - 1;
    WriteReg(address, MPU6500_SMPLRT_DIV, send_message);
    
    if (m_rate > 100) m_rate = 100;
    send_message = rate / m_rate - 1;
    WriteReg(address, MPU6500_I2C_SLV4_CTRL, send_message);
}

void MPU9250I2C::SetSensors() {
    char send_message[4] = {};
    send_message[0] = 1;
    send_message[1] = 0;
    send_message[2] = 0x11;
    WriteReg(address, MPU6500_PWR_MGMT_1, send_message[0]);
    WriteReg(address, MPU6500_PWR_MGMT_2, send_message[1]);
    send_message[3] = ReadReg(address, MPU6500_USER_CTRL) | 0x20; //BIT_AUX_IF_EN
    WriteReg(address, MPU6500_I2C_SLV1_DO, send_message[2]);
    WriteReg(address, MPU6500_USER_CTRL, send_message[3]);
}

void MPU9250I2C::SetOffset(double ax, double ay, double az, 
                        double gx, double gy, double gz, 
                        double mx, double my, double mz)
{
    if (gx != 0.0 || gy != 0.0 || gz != 0.0) gyro_offset[0] = gx; gyro_offset[1] = gy; gyro_offset[2] = gz;
    if (ax != 0.0 || ay != 0.0 || az != 0.0) acc_offset [0] = ax; acc_offset [1] = ay; acc_offset [2] = az;
    if (mx != 0.0 || my != 0.0 || mz != 0.0) mag_offset [0] = mx; mag_offset [1] = my; mag_offset [2] = mz;
}

void MPU9250I2C::SetOffset(double offset[9]) {
    for(int i{}; i < 3; ++i) {
        if(offset[0] != 0.0 || offset[1] != 0.0 || offset[2] != 0.0)  {
            acc_offset [i] = offset[i];
        }
        if(offset[3] != 0.0 || offset[4] != 0.0 || offset[5] != 0.0) {
            gyro_offset[i] = offset[i+3];
        }
        if(offset[6] != 0.0 || offset[7] != 0.0 || offset[8] != 0.0) {
            mag_offset [i] = offset[i+6];
        }
    }
}

void MPU9250I2C::SetBypass(bool bypass) {
    char tmp{};
    if(bypass) {
        tmp = ReadReg(address, MPU6500_USER_CTRL) & 0xDF; //~BIT_AUX_IF_EN
        WriteReg(address, MPU6500_USER_CTRL, tmp);
        tmp = 0x02; //bit_bypass_enable
        WriteReg(address, MPU6500_INT_PIN_CFG, tmp);
    }
    else {
        tmp = ReadReg(address, MPU6500_USER_CTRL) | 0x20; //BIT_AUX_IF_EN
        WriteReg(address, MPU6500_USER_CTRL, tmp);
        tmp = 0x00; //bit_bypass_disable
        WriteReg(address, MPU6500_INT_PIN_CFG, tmp);
    }
}

template<typename T>void MPU9250I2C::GetAcc(T acc[3]) {
    char data[6]{};
    short sign{};
    ReadReg(address, MPU6500_ACCEL_XOUT_H, data, 6);
    for (int i{}; i < 3; ++i) {
        sign = (static_cast<short>(data[2*i]) << 8) | static_cast<short>(data[2*i+1]);
        acc[i] = static_cast<double>(sign) * acc_coef - acc_offset[i];
    }
}
template<typename T>void MPU9250I2C::GetAcc(T *ax, T *ay, T *az) {
    T acc[3]{};
    GetAcc(acc);
    *ax = acc[0];
    *ay = acc[1];
    *az = acc[2];
}
template<typename T>void MPU9250I2C::GetGyro(T gyro[3]) {
    char data[6]{};
    short sign{};
    ReadReg(address, MPU6500_GYRO_XOUT_H, data, 6);
    for(int i{}; i < 3; ++i) {
        sign = (static_cast<short>(data[2*i]) << 8) | static_cast<short>(data[2*i+1]);
        gyro[i] = static_cast<double>(sign) * gyro_coef - gyro_offset[i];
    }
}
template<typename T>void MPU9250I2C::GetGyro(T *gx, T *gy, T *gz) {
    T gyro[3]{};
    GetGyro(gyro);
    *gx = gyro[0];
    *gy = gyro[1];
    *gz = gyro[2];
}
template<typename T>void MPU9250I2C::GetMag(T mag[3]) {
    char data[8]{};
    short sign{};
    ReadReg(address, 0x49, data, 8);
    for(int i{}; i < 3; ++i) {
        sign = (static_cast<short>(data[2*i+2]) << 8) | static_cast<short>(data[2*i+1]);
        sign = static_cast<short>((static_cast<long>(sign) * mag_sensitivity_adjust[i]) >> 8);
        mag[i] = static_cast<double>(sign) * mag_coef - mag_offset[i];
    }
}
template<typename T>void MPU9250I2C::GetMag(T *mx, T *my, T *mz) {
    T mag[3]{};
    GetMag(mag);
    *mx = mag[0];
    *my = mag[1];
    *mz = mag[2];
}
template<typename T>void MPU9250I2C::GetALL(T imu[9]) {
    char data[14]{};
    short sign{};
    ReadReg(address, MPU6500_ACCEL_XOUT_H, data, 14);
    sign = (static_cast<short>(data[0]) << 8) | static_cast<short>(data[1]);
    imu[3] = static_cast<double>(sign) * acc_coef - acc_offset[0];
    sign = (static_cast<short>(data[2]) << 8) | static_cast<short>(data[3]);
    imu[4] = static_cast<double>(sign) * acc_coef - acc_offset[1];
    sign = (static_cast<short>(data[4]) << 8) | static_cast<short>(data[5]);
    imu[5] = static_cast<double>(sign) * acc_coef - acc_offset[2]; 
    
    sign = (static_cast<short>(data[8]) << 8) | static_cast<short>(data[9]);
    imu[0] = static_cast<double>(sign) * gyro_coef - gyro_offset[0];
    sign = (static_cast<short>(data[10]) << 8) | static_cast<short>(data[11]);
    imu[1] = static_cast<double>(sign) * gyro_coef - gyro_offset[1];
    sign = (static_cast<short>(data[12]) << 8) | static_cast<short>(data[13]);
    imu[2] = static_cast<double>(sign) * gyro_coef - gyro_offset[2];   
    
    char data2[8]{};
    short sign2{};
    ReadReg(address, 0x49, data2, 8);
    sign2 = (static_cast<short>(data2[2]) << 8) | static_cast<short>(data2[1]);
    sign2 = (static_cast<long>(sign2) * mag_sensitivity_adjust[0]) >> 8;
    imu[6] = static_cast<double>(sign2) * mag_coef - mag_offset[0];
    sign2 = (static_cast<short>(data2[4]) << 8) | static_cast<short>(data2[3]);
    sign2 = (static_cast<long>(sign2) * mag_sensitivity_adjust[1]) >> 8;
    imu[7] = static_cast<double>(sign2) * mag_coef - mag_offset[1];
    sign2 = (static_cast<short>(data2[6]) << 8) | static_cast<short>(data2[5]);
    sign2 = (static_cast<long>(sign2) * mag_sensitivity_adjust[2]) >> 8;
    imu[8] = static_cast<double>(sign2) * mag_coef - mag_offset[2];
}

template void MPU9250I2C::GetAcc<double>(double *ax, double *ay, double *az);
template void MPU9250I2C::GetAcc<float>(float *ax, float *ay, float *az);
template void MPU9250I2C::GetAcc<double>(double *acc);
template void MPU9250I2C::GetAcc<float>(float *acc);
    
template void MPU9250I2C::GetGyro<double>(double *gx, double *gy, double *gz);
template void MPU9250I2C::GetGyro<float>(float *gx, float *gy, float *gz);
template void MPU9250I2C::GetGyro<double>(double *gyro);
template void MPU9250I2C::GetGyro<float>(float *gyro);
    
template void MPU9250I2C::GetMag<double>(double *mx, double *my, double *mz);
template void MPU9250I2C::GetMag<float>(float *mx, float *my, float *mz);
template void MPU9250I2C::GetMag<double>(double *mag);    
template void MPU9250I2C::GetMag<float>(float *mag);  

template void MPU9250I2C::GetALL<double>(double *imu);//gx,gy,gz,ax,ay,az
template void MPU9250I2C::GetALL<float>(float *imu);//gx,gy,gz,ax,ay,az