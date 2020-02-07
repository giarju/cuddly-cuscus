/*
 *  Author      : Dagozilla ITB
 *  Developer   : Dagozilla ITB
 *  Reference   : https://www.scilab.org/discrete-time-pid-controller-implementation
 */
 
#ifndef PID_H
#define PID_H
 
#include "mbed.h"
 
class PID{
    public :
            typedef enum Mode{
                PI_MODE,
                PID_MODE
            }Mode;
 
            PID(float p , float i , float d , float _N , float _Ts, float FF, Mode _mode) ;
 
            void setTunings(float p, float i, float d);
 
            float createpwm( float setpoint , float feedback ) ;
            float createOutput( float setpoint , float feedback , float saturate, float feed);
        
    private :
            float Kp ;
            float Kd ;
            float Ki ;
            float N ;
            float Ts ;
            float a0;
            float a1;
            float a2;
            float b0;
            float b1;
            float b2;
            float ku1;
            float ku2;
            float ke0;
            float ke1;
            float ke2;
            float e2;
            float e1;
            float e0;
            float u2;
            float u1;
            float u0; 
            float FF;
            Mode mode;
};
#endif