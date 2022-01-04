#include "SpiEncoder.h"


template<int T>
SpiEncoder<T>::SpiEncoder(PinName mosi_, PinName miso_, PinName sclk_, PinName* cs_, double delta_t_)
 : encoder(mosi_, miso_, sclk_),
   delta_t(delta_t_), direction(), angle(), angular_velocity()
{
    encoder.format(8,1);
    encoder.frequency(4000000);
    cs = cs_;
    for(int i = 0; i < T; ++i) {
        cs[i] = 1;
    }
    wait(0.1); //encoder init
};

template<int T>
double SpiEncoder<T>::GetAngle(int num) const {
    return angle[num];
}

template<int T>
double SpiEncoder<T>::GetVelocity(int num) const {
    return angular_velocity[num];
}

template<int T>
void SpiEncoder<T>::Inverse(int num)
{
    direction[num] = !direction[num];
}

template<int T>
int SpiEncoder<T>::GetPosition(int num)
{
    uint8_t temp[2]{};
    double pre_angle = angle[num];
    double angle_{};
    int error_stack{};
    bool error = false;
    
    uint8_t received = SpiWrite(num, 0x10);   //read command
    while (received != 0x10) {              //loop while encoder is not ready to send
        if(error_stack > 100) {
            error = true;
            error_stack = 0;
            break;
        }
        received = SpiWrite(num, 0x00);
        error_stack++;
    }
    if(!error) {
        temp[0] = SpiWrite(num, 0x00);       //Recieve MSB
        temp[1] = SpiWrite(num, 0x00);       //recieve LSB
        temp[0] &=~ 0xF0;                 //mask out the first 4 bits
        uint16_t encoder_byte_data  = temp[0] << 8;
        encoder_byte_data += temp[1];
        
        if (!direction[num]) angle_ = encoder_byte_data / 4096.0f * 2.0f * PI;  //normal
        else    angle_ = 2.0f * PI - (encoder_byte_data / 4096.0f * 2.0f * PI); //inverse
        
        if(angle_ > PI) angle[num] = angle_ - 2 * PI;                         //0~2π → -π~π
        else angle[num] = angle_;
        
        angular_velocity[num] = (angle[num] - pre_angle) / delta_t;
        
        return 0;
    }
    else {
        angle[num] = pre_angle;
        angular_velocity[num] = 0;
        
        return 1;
    }
}

template<int T>
bool SpiEncoder<T>::SetZero(int num)
{
    uint8_t received = SpiWrite(num, 0x70);   //read command
    while (received != 0x80) {              //loop while encoder is not ready to send
        received = SpiWrite(num, 0x00);
    }
    return true;
}

/* switch MSB or LSB */
template<int T>
uint8_t SpiEncoder<T>::SpiWrite (int num, uint8_t spi_transmit)
{
    Switching(num, 0);
    uint8_t spi_temp = encoder.write(spi_transmit);
    Switching(num, 1);
    wait_us(20);            //Timmig is critical
    return (spi_temp);
}

/* switch read encoder */
template<int T>
void SpiEncoder<T>::Switching(int num, int value)
{
    cs[num] = value;
}
