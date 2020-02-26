/*
 *  Author      : Dagozilla ITB
 *  Developer   : Dagozilla ITB
 *  Reference   : https://www.scilab.org/discrete-time-pid-controller-implementation
 */
 
#include "PID.h"
 
 
PID :: PID(float p , float i , float d , float _N , float _Ts, float ka, float kb, Mode _mode)
{
 
    N = _N ; 
    Ts = _Ts ;
    mode = _mode;

    setTunings(p, i, d, ka, kb);
 
    ku1 = a1/a0;
    ku2 = a2/a0;
    ke0 = b0/a0;
    ke1 = b1/a0;
    ke2 = b2/a0;
}

float PID::createpwm( float setpoint , float feedback, float saturate)
{
    e2 = e1 ;
    e1 = e0 ;
    u2 = u1 ;
    u1 = u0 ;
    e0 = setpoint-feedback;
    u0 = - (ku1 * u1 )  - ( ku2*u2 )  + ke0*e0 + ke1*e1 + ke2*e2;

    if (u0 >= saturate)
    {
        u0 = saturate;
    }
    else if (u0 <= -saturate)
    {
        u0 = -saturate;
    }

    float u0_feed_forward = FF*setpoint - FFs*prev_setpoint;
    float u0_total = u0 + u0_feed_forward;
  
    if (u0_total >= saturate)
    {
        u0_total = saturate;
    }
    else if (u0_total <= -saturate)
    {
        u0_total = -saturate;
    }

    prev_setpoint = setpoint;

    return u0_total ;   
}
 
void PID::setTunings(float p, float i, float d, float ka, float kb){
    
    Kp = p ; Kd = d ; Ki = i ;
    FF = ka/Ts + kb; FFs = ka/Ts;
    
    if(mode == PID_MODE){
        a0 = (1+N*Ts);
        a1 = -(2 + N*Ts);
        a2 = 1;
        b0 = Kp*(1+N*Ts) + Ki*Ts*(1+N*Ts) + Kd*N;
        b1 = -(Kp*(2+N*Ts) + Ki*Ts + 2*Kd*N);
        b2 = Kp + Kd*N;
 
    }
    else if(mode == PI_MODE){
        a0 = 1;
        a1 = -1;
        a2 = 0;
        b0 = Kp + Ki*Ts;
        b1 = -Kp;
        b2 = 0;
    }
}