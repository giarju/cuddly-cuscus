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


/************ Encoder motor ************/
/* Encoder Motor Base */

/* Encoder A */
#define PIN_A_CHA       PC_12
/* Encoder A */
#define PIN_A_CHB       PC_11

/* Encoder B */
#define PIN_B_CHA       PC_2
/* Encoder B */
#define PIN_B_CHB       PC_3

/* Encoder C */
#define PIN_C_CHA       PC_15
/* Encoder C */
#define PIN_C_CHB       PC_14

/* Encoder D */
#define PIN_D_CHA       PC_10
/* Encoder D */
#define PIN_D_CHB       PC_13

/* Encoder arm kanan */
#define PIN_ARM_RIGHT_CHA       PB_7
/* Encoder arm kanan */
#define PIN_ARM_RIGHT_CHB       PB_6

/* Encoder arm kiri */
#define PIN_ARM_LEFT_CHA       PB_2
/* Encoder arm_kiri */
#define PIN_ARM_LEFT_CHB       PD_2

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
#define PIN_FWD_B       PC_0
/* Motor B */
#define PIN_REV_B       PC_1

/* Motor C */
#define PIN_PWM_C       PB_1
/* Motor C */
#define PIN_FWD_C       PB_14
/* Motor C */
#define PIN_REV_C       PB_15

/* Motor D */
#define PIN_PWM_D       PB_10
/* Motor D */
#define PIN_FWD_D       PB_3
/* Motor D */
#define PIN_REV_D       PA_10

/* Motor D */
#define PIN_PWM_ARM_RIGHT       PA_15
/* Motor D */
#define PIN_FWD_ARM_RIGHT       PA_14
/* Motor D */
#define PIN_REV_ARM_RIGHT       PA_13

/* Motor D */
#define PIN_PWM_ARM_LEFT       PC_8
/* Motor D */
#define PIN_FWD_ARM_LEFT       PC_6
/* Motor D */
#define PIN_REV_ARM_LEFT       PC_5




#endif