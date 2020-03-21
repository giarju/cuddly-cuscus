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
    *motor3 = (+v_target.y + v_target.x)*SQRT2  + v_target.teta*R_BASE;  
    *motor2 = (+v_target.y - v_target.x)*SQRT2  + v_target.teta*R_BASE;
    *motor1 = (-v_target.y - v_target.x)*SQRT2 + v_target.teta*R_BASE; 
    *motor4 = (-v_target.y + v_target.x)*SQRT2 + v_target.teta*R_BASE; 
}


/* 
    Melakukan Trapezoidal Kinematics pembatasan kinematics
    Input : alamat v_target, alamat v sebelumnya, percepatan max
            time sampling
    Output: Trapezoid profile
*/
void trapeziodProfile(float *v_target, float *v_last, float a_max, float t_s_in_ms){
    float a_now = (*v_target - *v_last)*1000/t_s_in_ms;

    if (fabs(a_now) > a_max){
        *v_target = *v_last + (a_now*a_max*t_s_in_ms/(1000*fabs(a_now)));
    }
}

/* 
    Melakukan Trapezoidal Kinematics dengan struct base 
    Input : adress base, adress base last, ax max, ay max
            alfa max, time sampling
    Output: Trapezoid profile
*/
void baseTrapezoidProfile(Coordinate *base, Coordinate *base_last, float ax_max, float ay_max, float alfa_max, float t_s){
    trapeziodProfile(&(base->x), &(base_last->x),ax_max,t_s);
    trapeziodProfile(&(base->y), &(base_last->y),ay_max,t_s);
    trapeziodProfile(&(base->teta), &(base_last->teta),alfa_max, t_s);
}

/* 
    Melakukan theta feedback 
    Input : kondisi active, theta sekarang, alamat last_theta, alamat total_theta,theta destination
            time sampling dalam ms
    Output: omega untuk perputaran robot
*/
float thetaFeedback(int active ,float theta_now , float *last_theta, float *total_theta,float theta_destination, float t_s_in_ms){
    float kp = 0;
    float kd = 0;
    float ki = 0;

    float om  = 0;
    float theta_bound = 1;

    if (active){
        om = kp*(theta_destination - theta_now) + (kd*(theta_now - last_theta)/t_s_in_ms) + ki*(*total_theta);
    }
    else{
        om = 0;
    }

    *last_theta = theta_now;
    if (fabs(theta_now - theta_destination) < theta_bound && active){
        *total_theta += (*last_theta + theta_now)*t_s_in_ms/2;  // trapezoid
    }
    else{
        *total_theta = 0;
    }    

    return om;
}
