/* Bismillahirrahmaanirrahiim
 *
 * Author       : KRAI ITB
 * Developer    : KRAI ITB
 * Maintainer   : Moh. Tamamul Firdaus
 * Version      : 1.0.0
 * Notes        :
 * 
*/

#include "mbed.h"
#include "sensor.h"
#include "constant.h"


Compass::Compass(){
    // Constructor
}

void Compass::compass_reset(float _value){
//procedure that update offset value of compass
    _offset_compass_value = _value;
}

void Compass::compass_update(float _value){
//procedure that update compass value
    _theta_origin = _value;
    _theta_offset = _value - _offset_compass_value;
}

float Compass::compass_value(){
//function that return compass value after transform
    float theta_transformed;

    if(_theta_offset > 180.0f && _theta_offset <= 360.0f)
        theta_transformed = (_theta_origin - 360.0f - _offset_compass_value)/-radian_to_degree;
    else if(_theta_offset < -180.0f && _theta_offset >= -360.0f)
        theta_transformed = (_theta_origin  + 360.0f - _offset_compass_value)/-radian_to_degree;
    else
        theta_transformed = (_theta_origin - _offset_compass_value)/-radian_to_degree;

    return theta_transformed; 
}
