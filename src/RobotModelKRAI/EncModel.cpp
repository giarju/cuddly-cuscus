/***************************************************************************
 * Title      : Model Encoder
 * Name       : EncModel.cpp
 * Version    : 1.0
 * Author     : Gian Arjuna EL 16
 * Date       : 20 Mei 2020
 * Description:
 *
 * Class ini berfungsi untuk mensimulasikan encoder secara diskrit
 * 
 ***************************************************************************/

#include "EncModel.h"

EncModel::EncModel(int total_pulse, float wheel_rad)
{
    _total_pulse = total_pulse;
    _wheel_rad = wheel_rad;
}

void EncModel::encThetaSim(float value)
{
    curr_value = value;
    pulses += (curr_value - prev_value)/2/3.14159265358979*_total_pulse;
    prev_value = curr_value;
}

void EncModel::encDistanceSim(float value)
{
    curr_value = value;
    pulses += (curr_value - prev_value)/2/3.14159265358979/_wheel_rad*_total_pulse;
    prev_value = curr_value;
}

float EncModel::pulsesTheta()
{
    return pulses;
}


float EncModel::pulsesOmega()
{
    float pulses_omega = pulses;
    pulses = 0;
    return pulses_omega;
}
