#ifndef ODOMETRIKRAI_H
#define ODOMETRIKRAI_H

#include "mbed.h"
#include "encoderHAL/encoderHAL.h"
#include "CMPS12_KRAI/CMPS12_KRAI.h"
#include "Path/Path.h"

class odometriKRAI{

public:
    Coordinate position;
    odometriKRAI(TIM_TypeDef *_TIMEncX, TIM_TypeDef *_TIMEncY, PinName SDA, PinName SCL);
    void resetOdom(void);
    void updatePosition(void);
    
private:
    encoderHAL encX;
    encoderHAL encY;
    CMPS12_KRAI kompass;
};


#endif