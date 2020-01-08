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
/*************************** inisiasi class *******************************/
odometriKRAI::odometriKRAI(TIM_TypeDef *_TIMEncX, TIM_TypeDef *_TIMEncY, PinName SDA, PinName SCL)
: encX(_TIMEncX), encY(_TIMEncY), kompass(SDA, SCL, 0xC0) {
    position.x = 0;      // initiate all Value
    position.y = 0;      
    position.teta = 0;  
    kompass.compassResetOffsetValue();
    }


/*************************** definisi fungsi ******************************/

/* update position from base */
void odometriKRAI::updatePosition(void){
    float xTemp = encX.getPulses(1);                            /* butuh 1.5us */
    float yTemp = encY.getPulses(1);                            /* butuh 1.5us */

    kompass.compassUpdateValue();                               /* ??? */
    position.teta = kompass.compassValue();;
    position.x +=  (xTemp*PHI*D_RODA/4000)*cos(position.teta) + (yTemp*PHI*D_RODA/4000)*-sin(position.teta);        /* butuh 4.5 us */
    position.y +=  (xTemp*PHI*D_RODA/4000)*sin(position.teta) + (yTemp*PHI*D_RODA/4000)*cos(position.teta);         /* butuh 4.5 us */

}

/* to reset all the position */
void odometriKRAI::resetOdom(void){
    position.x = 0;      // initiate all Value
    position.y = 0;      
    position.teta = 0;
    
    kompass.compassResetOffsetValue();
}



