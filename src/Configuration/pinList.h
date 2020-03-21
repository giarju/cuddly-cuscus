/*Garudago ITB 2020 Attributes*/

#ifndef PINLIST_H
#define PINLIST_H
 
/************ Serial UART ************/

/* Joystick transmit*/
#define PIN_JOY_TX          PA_0
/* Joystick receive*/
#define PIN_JOY_RX          PA_1

/************ Serial I2C ************/
/* CMPS12 data */
#define CMPS_SDA        PC_9
/* CMPS12 clock*/
#define CMPS_SCL        PA_8


/************ Pneumatic ************/
#define PIN_ARM_KIRI    PA_9
#define PIN_ARM_KANAN   PC_7
#define PIN_TEMBAK      PH_1



/*********Encoder Eksternal*********/

#define TIMENCX TIM2

#define TIMENCY TIM3

//Encoder 1
#define PIN_1_CHA       PB_8 // TIM 2
#define PIN_1_CHB       PB_9
//Encoder 2
#define PIN_2_CHA       PB_4 //TIM 3
#define PIN_2_CHB       PB_5
//Encoder 3
#define PIN_3_CHA       PB_6 // TIM 4
#define PIN_3_CHB       PB_7

/************ Encoder motor ************/
/* Encoder Motor Base */

/* Encoder A */
#define PIN_A_CHA       PC_11
/* Encoder A */
#define PIN_A_CHB       PC_12

/* Encoder B */
#define PIN_B_CHA       PC_3
/* Encoder B */
#define PIN_B_CHB       PC_2

/* Encoder C */
#define PIN_C_CHA       PC_14
/* Encoder C */
#define PIN_C_CHB       PC_15

/* Encoder D */
#define PIN_D_CHA       PC_13
/* Encoder D */
#define PIN_D_CHB       PC_10


/************ pwm motor ************/


/* Motor A */
#define PIN_PWM_A       PA_7
/* Motor A */
#define PIN_FWD_A       PA_6
/* Motor A */
#define PIN_REV_A       PA_5

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
#define PIN_PWM_D       PB_10
/* Motor D */
#define PIN_FWD_D       PA_10
/* Motor D */
#define PIN_REV_D       PB_3




#endif