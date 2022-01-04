#ifndef MPU9250_I2C_H
#define MPU9250_I2C_H

#include "StandardFunctions.h"


static constexpr double MPU9250_ACC_LSB =  0.00006103515; //[G / LSB] (2.0/32768.0)
static constexpr double MPU9250_GYRO_LSB = 0.00762939453; //[(degree / s) / LSB] (250.0/32768.0)
static constexpr double MPU9250_MAG_LSB =  0.15003052503; //[uT / LSB] (4915.0/32760.0)

static const bool MPU9250_WRITE_FLAG = 0;
static const bool MPU9250_READ_FLAG  = 1;

//! you can switch addresses of mpu9250 to use two
enum MPU9250_addresses {
    MPU6500_ADDR_LOW  = (0b1101000 << 1),
    MPU6500_ADDR_HIGH = (0b1101001 << 1),
    AK8963_ADDR       = (0b0001100 << 1)
};

enum MPU9250_register {
    MPU6500_SMPLRT_DIV =               0x19,
    MPU6500_CONFIG =                   0x1A,
    MPU6500_GYRO_CONFIG =              0x1B,
    MPU6500_ACCEL_CONFIG =             0x1C,
    MPU6500_ACCEL_CONFIG2 =            0x1D,
    MPU6500_LP_ACCEL_ODR =             0x1E,
    MPU6500_I2C_MST_CTRLI2C_MST_CTRL = 0x24,
    MPU6500_I2C_SLV0_ADDR =            0x25,
    MPU6500_I2C_SLV0_REG =             0x26,
    MPU6500_I2C_SLV0_CTRL =            0x27,
    MPU6500_I2C_SLV1_ADDR =            0x28,
    MPU6500_I2C_SLV1_REG =             0x29,
    MPU6500_I2C_SLV1_CTRL =            0x2A,
    MPU6500_I2C_SLV4_CTRL =            0x34,
    MPU6500_INT_PIN_CFG =              0x37,
    MPU6500_ACCEL_XOUT_H =             0x3B, //from lower to higher
    MPU6500_ACCEL_XOUT_L =             0x3C,
    MPU6500_ACCEL_YOUT_H =             0x3D,
    MPU6500_ACCEL_YOUT_L =             0x3E,
    MPU6500_ACCLE_ZOUT_H =             0x3F,
    MPU6500_ACCEL_ZOUT_L =             0x40,
    MPU6500_TEMP_OUT_H =               0x41,
    MPU6500_TEMP_OUT_L =               0x42,
    MPU6500_GYRO_XOUT_H =              0x43,
    MPU6500_GYRO_XOUT_L =              0x44,
    MPU6500_GYRO_YOUT_H =              0x45,
    MPU6500_GYRO_YOUT_L =              0x46,
    MPU6500_GYRO_ZOUT_H =              0x47,
    MPU6500_GYRO_ZOUT_L =              0x48,
    MPU6500_I2C_SLV1_DO =              0x64,
    MPU6500_I2C_I2C_MST_DELAY_CTRL =   0x67,
    MPU6500_USER_CTRL =                0x6A,
    MPU6500_PWR_MGMT_1 =               0x6B,
    MPU6500_PWR_MGMT_2 =               0x6C,
    I_AM_MPU6500 =                     0x71,
    AM_I_MPU6500 =                     0x75,
    AM_I_AK8963 =                      0x00,
    AK8963_INFO =                      0x01,
    AK8963_ST1 =                       0x02,
    AK8963_HXL =                       0x03, //from higher to lower
    AK8963_HXH =                       0x04,
    AK8963_HYL =                       0x05,
    AK8963_HYH =                       0x06,
    AK8963_HZL =                       0x07,
    AK8963_HZH =                       0x08,
    AK8963_ST2 =                       0x09,
    AK8963_ASAX =                      0x10,
    AK8963_CNTL1 =                     0x0A,
    AK8963_CNTL2 =                     0x0B,
    I_AM_AK8963 =                      0x48
};

enum MPU9250_AD0 {
    AD0_HIGH = 1,
    AD0_LOW  = 0
};

enum MPU9250_ACC_RANGE {
    MPU6500_ACC_2G =  0,
    MPU6500_ACC_4G,
    MPU6500_ACC_8G,
    MPU6500_ACC_16G
};

enum MPU9250_GYRO_RANGE {
    MPU6500_GYRO_250DPS =  0,
    MPU6500_GYRO_500DPS,
    MPU6500_GYRO_1000DPS,
    MPU6500_GYRO_2000DPS
};

enum MPU9250_GYRO_LPF {
    MPU6500_GYRO_LPF_188HZ = 1,
    MPU6500_GYRO_LPF_98HZ,
    MPU6500_GYRO_LPF_42HZ,
    MPU6500_GYRO_LPF_20HZ,
    MPU6500_GYRO_LPF_10HZ,
    MPU6500_GYRO_LPF_5HZ
};

enum MPU9250_ACC_LPF {
    MPU6500_ACC_LPF_460HZ = 0,
    MPU6500_ACC_LPF_184HZ,
    MPU6500_ACC_LPF_92HZ,
    MPU6500_ACC_LPF_41HZ,
    MPU6500_ACC_LPF_20HZ,
    MPU6500_ACC_LPF_10HZ,
    MPU6500_ACC_LPF_5HZ
};


class MPU9250I2C
{
    public:
        MPU9250I2C(PinName sda, PinName scl, MPU9250_AD0 celect = AD0_LOW);
        
        bool SensorCheck();
        
        void SetAcc(MPU9250_ACC_RANGE acc_range = MPU6500_ACC_8G);
        void SetGyro(MPU9250_GYRO_RANGE gyro_range = MPU6500_GYRO_2000DPS);
        void SetMag();
        void SetSensors();
        void SetBypass(bool bypass);
        void SetLPF (MPU9250_GYRO_LPF g_lpf = MPU6500_GYRO_LPF_42HZ, MPU9250_ACC_LPF a_lpf = MPU6500_ACC_LPF_41HZ);
        void SetRate(int rate = 1000, int m_rate = 100);
        void SetOffset(double ax, double ay, double az,
                       double gx, double gy, double gz,
                       double mx, double my, double mz);
        void SetOffset(double offset[9]);
        
        template<typename T>void GetAcc(T *ax, T *ay, T *az);
        template<typename T>void GetAcc(T acc[3]);
        template<typename T>void GetGyro(T *gx, T *gy, T *gz);
        template<typename T>void GetGyro(T gyro[3]);
        template<typename T>void GetMag(T *mx, T *my, T *mz);
        template<typename T>void GetMag(T mag[3]);
        template<typename T>void GetALL(T imu[9]); //gyro, acc, magの順
        
    private:
        I2C i2c;
        void Init();
        void WriteReg(char addr, char data);
        void WriteReg(char addr, char reg, char data);
        char ReadReg (char addr, char reg);
        void ReadReg (char addr, char start_reg, char* buff, char num);
        
        char address;
        double acc_coef; //coefficient
        double gyro_coef;
        double mag_coef;
        double acc_offset[3];
        double gyro_offset[3];
        double mag_offset[3];
        long mag_sensitivity_adjust[3];
};

inline void MPU9250I2C::WriteReg(char addr, char data)
{
    i2c.write(addr | MPU9250_WRITE_FLAG, &data, 1);
    wait_us(100);
}

inline void MPU9250I2C::WriteReg(char addr, char reg, char data)
{
    char temp[2] = { reg, data};
    i2c.write(addr | MPU9250_WRITE_FLAG, temp, 2);
    wait_us(100);
}

inline char MPU9250I2C::ReadReg(char addr, char reg)
{
    char buff{};
    WriteReg(addr, reg);
    i2c.read(addr | MPU9250_READ_FLAG, &buff, 1, true);
    return buff;
}

inline void MPU9250I2C::ReadReg (char addr, char start_reg, char* buff, char num)
{
    WriteReg(addr, start_reg);
    i2c.read(addr | MPU9250_READ_FLAG, buff, num, true);
}


#endif