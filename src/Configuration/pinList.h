/*Garudago ITB 2020 Attributes*/

#ifndef PINLIST_H
#define PINLIST_H

/************ Serial UART ************/

/* Arduino transmit*/
#define PIN_TX          PA_0
/* Arduino receive*/
#define PIN_RX          PA_1

/************ Serial I2C ************/
/* CMPS12 data */
#define CMPS_SDA        PC_9
/* CMPS12 clock*/
#define CMPS_SCL        PA_8


/************ Encoder motor ************/
/* Encoder Motor Base */

/* Encoder A */
#define PIN_A_CHA       PC_12
/* Encoder A */
#define PIN_A_CHB       PC_11

/* Encoder B */
#define PIN_B_CHA       PC_3
/* Encoder B */
#define PIN_B_CHB       PC_2

/* Encoder C */
#define PIN_C_CHA       PC_15
/* Encoder C */
#define PIN_C_CHB       PC_14

/* Encoder D */
#define PIN_D_CHA       PC_10
/* Encoder D */
#define PIN_D_CHB       PC_13

/************ pwm motor ************/


/* Motor A */
#define PIN_PWM_A       PA_7
/* Motor A */
#define PIN_FWD_A       PA_5
/* Motor A */
#define PIN_REV_A       PA_6

/* Motor B */
#define PIN_PWM_B       PB_0
/* Motor B */
#define PIN_FWD_B       PC_1
/* Motor B */
#define PIN_REV_B       PC_0

/* Motor C */
#define PIN_PWM_C       PB_1
/* Motor C */
#define PIN_FWD_C       PB_14
/* Motor C */
#define PIN_REV_C       PB_15

/* Motor D */
#define PIN_PWM_D       PA_11
/* Motor D */
#define PIN_FWD_D       PA_12
/* Motor D */
#define PIN_REV_D       PB_12

#endif