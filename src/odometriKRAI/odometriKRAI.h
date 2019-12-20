#ifndef ODOMETRIKRAI_H
#define ODOMETRIKRAI_H

#include "mbed.h"
#include "encoderHAL.h"
#include "CMPS12_KRAI.h"

class odometriKRAI{

public:
    float x ,y ,theta;
    odometriKRAI(TIM_TypeDef *_TIMEncX, TIM_TypeDef *_TIMEncY, PinName SDA, PinName SCL);
    void resetOdom(void);
    void updatePosition(void);
    
private:
    encoderHAL encX;
    encoderHAL encY;
    CMPS12_KRAI kompass;
};


#endif