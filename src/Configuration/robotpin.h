/*Garudago ITB 2019 Attributes*/

#ifndef ROBOTPIN_H
#define ROBOTPIN_H

/*************************** Library  **********************************/
#include "CMPS12_KRAI.h"
#include "encoderKRAI.h"
#include "Motor.h"
#include "Configuration/pinList.h" 
#include "odometriKRAI/odometriKRAI.h"
 
/*************************** Deklarasi PIN  **********************************/

/************ Deklarasi Serial UART ************/
 
/*komunikasi serial dengan PC */
#if defined(SERIAL_DEBUG) && !defined(PCSERIAL_DEBUG)
RawSerial pc(USBTX, USBRX, 115200);
#endif
#if !defined(SERIAL_DEBUG) && defined(PCSERIAL_DEBUG)
Serial pc(USBTX, USBRX, 115200);
#endif
#ifdef JOYSTICK_DEBUG
joysticknucleo stick(PIN_JOY_TX, PIN_JOY_RX);
#endif

/************ Deklarasi objek I2C **************/
 

/************ Deklarasi objek Pneumatic **************/
DigitalOut armKiri(PIN_ARM_KIRI);
DigitalOut armKanan(PIN_ARM_KANAN);
DigitalOut tembak(PIN_TEMBAK);

 
/************ Deklarasi objek odometri ************/
#ifdef ODOMETRY_DEBUG
/* Odometry            : melakukan perhitungan odometry
 * TIM2 & TIM3         : Register timer untuk encoder eksternal
 * CMPS_SDA & CMPS_SCL : PIN I2C untuk compass */
odometriKRAI Odometry(TIMENCX, TIMENCY, CMPS_SDA, CMPS_SCL);
#endif


/************ Deklarasi objek encoder ************/
#ifdef ENCMOTOR_DEBUG
/* encoder untuk motor A base */ 
encoderKRAI A_enc(PIN_A_CHA, PIN_A_CHB, 538, encoderKRAI::X4_ENCODING);
/* encoder untuk motor B base */ 
encoderKRAI B_enc(PIN_B_CHA, PIN_B_CHB, 538, encoderKRAI::X4_ENCODING);
/* encoder untuk motor C base */ 
encoderKRAI C_enc(PIN_C_CHA, PIN_C_CHB, 538, encoderKRAI::X4_ENCODING);
/* encoder untuk motor D base */ 
encoderKRAI D_enc(PIN_D_CHA, PIN_D_CHB, 538, encoderKRAI::X4_ENCODING);
/* encoder untuk motor kanan arm */ 
// encoderKRAI right_arm_enc(PIN_ARM_RIGHT_CHA, PIN_ARM_RIGHT_CHB, 538, encoderKRAI::X4_ENCODING);
/* encoder untuk motor kanan arm */ 
//encoderKRAI left_arm_enc(PIN_ARM_LEFT_CHA, PIN_ARM_LEFT_CHB, 538, encoderKRAI::X4_ENCODING);
#endif

   
/************ Deklarasi objek motor ************/ 
#ifdef MOTOR_DEBUG
/* motor A base */ 
Motor A_motor(PIN_PWM_A, PIN_FWD_A,PIN_REV_A); 
/* motor B base */
Motor B_motor(PIN_PWM_B, PIN_FWD_B,PIN_REV_B); 
/* motor C base */
Motor C_motor(PIN_PWM_C, PIN_FWD_C,PIN_REV_C);
/* motor D base */ 
Motor D_motor(PIN_PWM_D, PIN_FWD_D,PIN_REV_D); 
#endif

/************ Deklarasi objek digital button ************/ 

/* button builtin Nucleo */
DigitalIn mybutton(USER_BUTTON);

#endif