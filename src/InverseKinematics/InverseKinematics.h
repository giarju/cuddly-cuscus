/***************************************************************************
 * Title      : Library inverse kinematics robot 
 * Name       : InverseKinematics.h
 * Version    : 1.0
 * Author     : Gian Arjuna EL 16
 * Date       : 13 Desember 2019
 * Description:
 *
 * Library ini digunakan untuk melakukan perhitungan kecepatan motor base
 * dengan persamaan inverse kinematics robot
 * 
 ***************************************************************************/

#ifndef KINEMATICS_H
#define KINEMATICS_H



/******************************* defines ***********************************/
#define SQRT2 1.41421356237

/* radius base */
#define R_BASE 1.0


/************************* deklarasi fungsi  ***********************************/

/*
 * melakukan perhitungan inverse kinematics untuk base omniwheel 4 roda 45 derajat
 * 
 * @param v_target : target kecepatan robot
 * @param motor1,2,3,4: kecepatan untuk 4 motor
 */
void base4Omni(Coordinate v_target, float *motor1, float *motor2, float *motor3, float *motor4);

#endif