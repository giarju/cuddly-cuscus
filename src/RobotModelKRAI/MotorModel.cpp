/***************************************************************************
 * Title      : Model motor 
 * Name       : MotorModel.cpp
 * Version    : 1.0
 * Author     : Gian Arjuna EL 16
 * Date       : 20 Mei 2020
 * Description:
 *
 * Class ini berfungsi untuk mensimulasikan motor secara diskrit
 * 
 ***************************************************************************/

#include "MotorModel.h"

MotorModel::MotorModel(double k1, double k2, double k3, double k4, double k5, double k6, double base_volt)
{
    /* koefision k1-k6 harus dicari terlebih dahulu dari matlab
     * koefisien ini adalah koefisien persamaan difference dari 
     * theta motor terhadap tegangan
     */
    
    _k1 = k1;
    _k2 = k2;
    _k3 = k3;
    _k4 = k4;
    _k5 = k5;
    _k6 = k6;
    _base_volt = base_volt;
}

void MotorModel::motorSim(double pwm)
{
    if (pwm > 1)
    {
        pwm =  1;
    }
    else if (pwm < -1)
    {
        pwm = -1;
    }
    
    curr_theta = (-1)*(_k2/_k1)*prev_theta - (_k3/_k1)*prev2_theta + (_k4/_k1)*prev_volt + (_k5/_k1)*prev2_volt + (_k6/_k1)*prev3_volt;
   
    prev2_theta = prev_theta;
    prev_theta = curr_theta;
    prev3_volt = prev2_volt;
    prev2_volt = prev_volt;
    prev_volt = pwm*_base_volt;   
}



