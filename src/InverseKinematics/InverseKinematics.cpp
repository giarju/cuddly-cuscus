/***************************************************************************
 * Title      : Library inverse kinematics robot 
 * Name       : InverseKinematics.cpp
 * Version    : 1.0
 * Author     : Gian Arjuna EL 16
 * Date       : 13 Desember 2019
 * Description:
 *
 * Library ini digunakan untuk melakukan perhitungan kecepatan motor base
 * dengan persamaan inverse kinematics robot
 * 
 ***************************************************************************/

/******************************* library ***********************************/
#include "mbed.h"
#include "Path.h"
#include "InverseKinematics.h"


/*************************** definisi fungsi *******************************/

/* melakukan perhitungan inverse kinematics untuk base omniwheel 4 roda 45 derajat */
void base4Omni(Coordinate v_target, float *motor1, float *motor2, float *motor3, float *motor4)
{
    motor1 = (v_target.velocity.y + v_target.velocity.x)*SQRT2  + v_target.velocity.teta*R_BASE;  
    motor2 = (v_target.velocity.y - v_target.velocity.x)*SQRT2  + v_target.velocity.teta*R_BASE;
    motor3 = (-v_target.velocity.y - v_target.velocity.x)*SQRT2 + v_target.velocity.teta*R_BASE; 
    motor4 = (-v_target.velocity.y + v_target.velocity.x)*SQRT2 + v_target.velocity.teta*R_BASE; 
}

