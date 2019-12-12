#include "mbed.h"
#include "pneumatikKRAI.h"

pneumatikKRAI ::pneumatikKRAI(PinName pinPneu,
                              int posisi,
                              PneumatikPos pneumatikPos) : pinPneu_(pinPneu)
{
    pneuMaju   = 1;
    pneuMundur = 0;
    if ((pneumatikPos == MAJU && posisi == 0) || (pneumatikPos == MUNDUR  && posisi == 1) ){
        pneuMaju = 0;
        pneuMundur = 1;
    }
}                              

void pneumatikKRAI::Contract(void){
    pinPneu_ = pneuMundur;
}
void pneumatikKRAI::Extend(void){
    pinPneu_ = pneuMaju;
}

