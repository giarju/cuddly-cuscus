/***************************************************************************
 * Title      : Library untuk odometri base
 * Name       : odometriKrai.cpp
 * Version    : 1.0
 * Author     : Oktavianus Irvan Sitanggang EL 18
 * Date       : 17 Desember 2019
 * Description:
 *
 * Library ini digunakan untuk melakukan perhitungan posisi robot
 * 
 * 
 ***************************************************************************/

/******************************* library ***********************************/

#include "odometriKRAI.h"
#include "mbed.h"

#ifndef PHI
#define PHI  3.14159265359
#endif

#ifndef D_RODA
#define D_RODA 0.06
#endif

odometriKRAI::odometriKRAI(TIM_TypeDef *_TIMEncX, TIM_TypeDef *_TIMEncY, PinName SDA, PinName SCL)
: encX(_TIMEncX), encY(_TIMEncY), kompass(SDA, SCL, 0xC0) {
    x = 0;      // initiate all Value
    y = 0;      
    theta = 0;  
    //kompas.compass_reset((float)kompass.getAngle()/10);
    kompass.compassResetOffsetValue();

    }
/* update position from base */
void odometriKRAI::updatePosition(void){
    float xTemp = encX.getPulses(1);
    float yTemp = encY.getPulses(1);

    //kompas.compass_update((float)kompass.getAngle()/10);
    kompass.compassUpdateValue();
    theta = kompass.compassValue();;
    x +=  (xTemp*PHI*D_RODA/4000)*cos(theta) + (yTemp*PHI*D_RODA/4000)*-sin(theta);
    y +=  (xTemp*PHI*D_RODA/4000)*sin(theta) + (yTemp*PHI*D_RODA/4000)*cos(theta);

}

/* to reset all the position */
void odometriKRAI::resetOdom(void){
    x = 0;      // initiate all Value
    y = 0;      
    theta = 0;
    
    //kompas.compass_reset((float)kompass.getAngle()/10);
    kompass.compassResetOffsetValue();
}



