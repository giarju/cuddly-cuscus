/**
 * 
 * 
 * 
 * 
 **/

#ifndef PNEUMATIKKRAI_H
#define PNEUMATIKKRAI_H

#include "mbed.h"

class pneumatikKRAI {

public:

    typedef enum PneumatikPos {

        MAJU,
        MUNDUR

    } PneumatikPos;

    pneumatikKRAI(PinName pinPneu, int posisi ,PneumatikPos pneumatikPos);  // posisi pneumatik 1

    void Extend(void); 
    /**
     * Extend digunakan untuk memanjangkan Peumatik 
     * 
     **/
    void Contract(void);
    /**
     * Contaract digunakan untuk memendekkan Pneumatik 
     * 
     **/
private:

    PneumatikPos pneumatikPos_ ;

    DigitalOut pinPneu_ ;

    int pneuMaju;
    int pneuMundur;
};

#endif