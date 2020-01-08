/*Garudago ITB 2019 Attributes*/

#ifndef ROBOTPIN_H
#define ROBOTPIN_H

/*************************** Library  **********************************/
#include "CMPS12_KRAI.h"
#include "encoderHAL.h"
#include "encoderKRAI.h"
#include "mbed.h"
#include "millis.h"
#include "Motor.h"
#include <Configuration/pinList.h>

/*************************** Deklarasi PIN  **********************************/
/*Deklarasi Serial*/
Serial pc(USBTX, USBRX, 115200);

//Deklarasi I2C CMPS12
CMPS12_KRAI compass(CMPS_SDA, CMPS_SCL, 0xC0);

//Deklarasi Encoder Motor
encoderKRAI enc_A(PIN_A_CHA, PIN_A_CHB, 538, encoderKRAI::X4_ENCODING);
encoderKRAI enc_B(PIN_B_CHA, PIN_B_CHB, 538, encoderKRAI::X4_ENCODING);
encoderKRAI enc_C(PIN_C_CHA, PIN_C_CHB, 538, encoderKRAI::X4_ENCODING);
encoderKRAI enc_D(PIN_D_CHA, PIN_D_CHB, 538, encoderKRAI::X4_ENCODING);

//Deklarasi Encoder Motor Shagai
encoderKRAI encoderPelontar(PIN_P_CHB, PIN_P_CHA, 538, encoderKRAI::X4_ENCODING);

//Deklarasi Encoder Eksternal
encoderHAL enc2(TIM4);
encoderHAL enc3(TIM2);
encoderHAL enc1(TIM3);
   
//Deklarasi Motor
Motor motorA(PIN_PWM_A, PIN_FWD_A, PIN_REV_A);
Motor motorB(PIN_PWM_B, PIN_FWD_B, PIN_REV_B);
Motor motorC(PIN_PWM_C, PIN_FWD_C, PIN_REV_C);
Motor motorD(PIN_PWM_D, PIN_FWD_D, PIN_REV_D);
Motor motorPelontar(PIN_PWM_P, PIN_FWD_P, PIN_REV_P);

//Deklarasi Pneumatic
DigitalOut pneu[6] = {(PIN_PNEU_0),(PIN_PNEU_1),(PIN_PNEU_2),(PIN_PNEU_3),(PIN_PNEU_4),(PIN_PNEU_5)};
//pneu[0]   opto 7   = pneu buat mengambil shagai           1->Pengambil tertutup
//pneu[1]   opto 8   = pneu untuk extention                 1->Ekstension memendek
//pneu[2]   opto 9   = pneu untuk pelontar                  1->Pelontar tidak memanjang
//pneu[3]   opto 2   = pneu untuk pencapit gerege           1->Pencapit tertutup
//pneu[4]   opto 5   = pneu untuk menaikkan gerege          1->Posisi naik
//pneu[5]   opto 10  = pneu untuk membelokan tangan pencapit gerege

#endif