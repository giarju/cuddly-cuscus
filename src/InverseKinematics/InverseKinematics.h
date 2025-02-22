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

#include "mbed.h"
#include "Path.h"


/******************************* defines ***********************************/
#define SQRT2 1.41421356237f

/* radius base */
#define R_BASE 0.32f


/************************* deklarasi fungsi  ***********************************/

/*
 * melakukan perhitungan inverse kinematics untuk base omniwheel 4 roda 45 derajat
 * 
 * @param v_target : target kecepatan robot
 * @param motor1,2,3,4: kecepatan untuk 4 motor
 */
void base4Omni(Coordinate v_target, float *motor1, float *motor2, float *motor3, float *motor4);

void trapeziodProfile(float *v_now, float *v_last, float t_s_in_ms);

void baseTrapezoidProfile(Coordinate *base, Coordinate *base_last, float ax_max, float ay_max, float alfa_max, float t_s);

float thetaFeedback(float omega_now ,float theta_now ,float theta_destination, float *last_theta, float *total_theta, float t_s_in_ms);

#endif