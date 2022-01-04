#ifndef SPI_ENCODER_H
#define SPI_ENCODER_H
 
#include "StandardFunctions.h"


/* cui amt203 */
template<int T>
class SpiEncoder
{
    public:
        SpiEncoder(PinName mosi_, PinName miso_, PinName sclk_, PinName* cs_, double delta_t_);
        int GetPosition(int num);
        double GetAngle(int num)const;
        double GetVelocity(int num)const;
        bool SetZero(int num);
        void Inverse(int num);
        
    private:
        SPI encoder;
        DigitalOut cs[T];
        double delta_t;
        bool direction[T];
        double angle[T];
        double angular_velocity[T];
        
        uint8_t SpiWrite (int num, uint8_t spi_transmit);
        void Switching(int num, int value);
};

#endif
