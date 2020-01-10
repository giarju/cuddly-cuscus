#ifndef PROCEDURE_GARUDAGO_H
#define PROCEDURE_GARUDAGO_H

/********************** Library ******************************/

#include "Configuration/constant.h"
#include "Configuration/variable.h"
#include "Configuration/robotpin.h"

/**************** function declaration ***********************/


/* 
 * prosedur untuk melakukan sampling odometri base
 * 
 * */
void odometrySamp();

/* 
 * prosedur untuk melakukan sampling encoder motor base
 * 
 * */
void encoderMotorSamp();

/* 
 * prosedur untuk melakukan sampling pwm motor base
 * 
 * */
void motorSamp();

/* 
 * prosedur untuk menghitung pid motor base
 * 
 * */
void pidMotorSamp();

/* 
 * prosedur untuk melakukan tracking path
 * 
 * */
void trackingSamp();


/* 
 * prosedur untuk print string dengan uart setiap sampling time
 * 
 * */
void pcSerialSamp();

/* 
 * prosedur untuk mengirimkan data melalui uart dengan double buffer
 * 
 * */
void sendUart(char *buffer);

/* 
 * prosedur untuk menulis data ke hardware uart
 * 
 * */
void writeUart();


#endif