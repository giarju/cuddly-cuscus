#ifndef CONSTANT_H
#define CONSTANT_H

const float  PI                                     = 3.141592;
const float  radian_to_degree                       = 57.295779;
const float  wheel_radius                           = 0.050;
const float  wheel_distance_from_center             = 0.180;

const float manual_linear_velocity                  = 2.0;
const float manual_rotation_velocity                = 3.0;

const float right_kp = 0.3; 
const float right_ki = 0.1;         
const float right_kd = 0.0;
const float right_N  = 0.0;
const float right_TS = 0.01;        
const float right_FF = 0.0;

const float left_kp = right_kp; 
const float left_ki = right_kp;         
const float left_kd = 0.0;
const float left_N  = 0.0;
const float left_TS = 0.01;        
const float left_FF = 0.0;

const float back_kp = right_kp; 
const float back_ki = right_kp;         
const float back_kd = 0.0;
const float back_N  = 0.0;
const float back_TS = 0.01;        
const float back_FF = 0.0;


const float  position_tolerance = 0.005;        
const float  orientation_tolerance = 0.06;

const float  max_velocity_magnitude = 1.5;      
const float  max_rotation_magnitude = 2.0;      
const float  cutoff_velocity_magnitude = 0.8;  
const float  cutoff_rotation_magnitude = 1.7;   

const float  max_linear_acceleration_magnitude  = 2.5; 
const float  max_angular_acceleration_magnitude = 2.5;
const float  max_linear_deceleration_magnitude  = 3.0;
const float  max_angular_deceleration_magnitude = 3.0;

const float  linear_acceleration_feedforward_factor  = 1.7;
const float  angular_acceleration_feedforward_factor = 1.7;
const float  linear_deceleration_feedforward_factor  = 1.7;
const float  angular_deceleration_feedforward_factor = 1.7;


#endif