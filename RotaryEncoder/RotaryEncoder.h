#ifndef ROTENC_H
#define ROTENC_H

#include "StandardFunctions.h"


class RotaryEncoder
{
    public:
        //A相, B相, パルス数
        RotaryEncoder(PinName a, PinName b, int pulses_per_rotation = 100, double dt_ = 0.01);
        
        //回転数を取得する
        double GetRotations()const;
        
        //回転速度を取得する
        double GetRps();
        
        //回転数を加算する方向を逆にする
        void ChangeDirection();
        
        //現在の回転数を再定義する
        void DefineNowRotations(double n);
    
    private:
        double rotations; //エンコーダーの総回転数[rotations]
        double pre_rotations;
        double delta_t;
        double rotations_per_interrupt;
        InterruptIn pinA;
        InterruptIn pinB;
        
        void ARaise_();
        void AFall_();
        void BRaise_();
        void BFall_();
};

#endif
