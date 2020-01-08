/*Garudago ITB 2019 Attributes*/

#ifndef PINLIST_H
#define PINLIST_H

//////////////////////
//    UART SERIAL   //
//       TX-RX      //
//////////////////////
// Arduino
#define PIN_TX          PA_0
#define PIN_RX          PA_1

//////////////////////
//        I2C       //
//      SDA-SCL     //
//////////////////////
//COMPASS
#define CMPS_SDA        PC_9
#define CMPS_SCL        PA_8

//////////////////////
//      ENCODER     //
//      CHA-CHB     //
//////////////////////

// Pin Encoder Motor Roda
//Encoder A
#define PIN_A_CHA       PC_12
#define PIN_A_CHB       PC_11
//Encoder B
#define PIN_B_CHA       PC_3
#define PIN_B_CHB       PC_2
//Encoder C
#define PIN_C_CHA       PC_15
#define PIN_C_CHB       PC_14
//Encoder D
#define PIN_D_CHA       PC_10
#define PIN_D_CHB       PC_13
//Encoder Pelontar
#define PIN_P_CHA       PC_6
#define PIN_P_CHB       PC_7

//////////////////////
//       MOTOR      //
//    PWM-FWD-REV   //
//////////////////////

//Motor A
#define PIN_PWM_A       PA_7
#define PIN_FWD_A       PA_5
#define PIN_REV_A       PA_6
//Motor B
#define PIN_PWM_B       PB_0
#define PIN_FWD_B       PC_1
#define PIN_REV_B       PC_0
//Motor C
#define PIN_PWM_C       PB_1
#define PIN_FWD_C       PB_14
#define PIN_REV_C       PB_15
//Motor D
#define PIN_PWM_D       PA_11
#define PIN_FWD_D       PA_12
#define PIN_REV_D       PB_12
//Motor Pelontar
#define PIN_PWM_P       PA_15
#define PIN_FWD_P       PA_14
#define PIN_REV_P       PA_13

//////////////////////
//     PNEUMATIC    //
//    DIGITAL OUT   //
//////////////////////
#define PIN_PNEU_0      PB_9//PB_2 //pengambil shagai
#define PIN_PNEU_1      PB_8// PB_9 //extension shagai
#define PIN_PNEU_2      PB_4 //pelontar shagai
#define PIN_PNEU_3      PC_8 //pencapit gerege
#define PIN_PNEU_4      PB_2 //penaik gerege
#define PIN_PNEU_5      PC_5 //pembelok gerege

/*
        pneumatic_extension = 1;         //Pendek
        pneumatic_penaik    = 1;         //Naik
        pneumatic_pelontar  = 0;         //Pendek
        pneumatic_pencapit  = 0;         //Tutup
        pneumatic_pengambil = 1;         //Tutup

        pneu[0]=1;      //Pneu pengambil shagai
        pneu[1]=1;      //Pneu untuk ekstension pelotar
        pneu[2]=0;      //Pneu untuk pelontar
        pneu[3]=1;      //Pneu untuk pencapit gerege
        pneu[4]=1;      //Pneu untuk penaik gerege
        pneu[5] // pembelok gerege
*/

#endif