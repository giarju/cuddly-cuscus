#ifndef CONSTANT_H
#define CONSTANT_H


#define  PI                                     3.14159265358979f
#define  ENC_MOTOR_PULSE                        538
#define  MS_TO_S                                1000
#define  US_TO_S                                1000000
#define  radian_to_degree                       57.295779
#define  wheel_radius                           0.050
#define  wheel_distance_from_center             0.180

#define manual_linear_velocity                   2.0
#define manual_rotation_velocity                 3.0

#define right_kp  0.3 
#define right_ki  0.1         
#define right_kd  0.0
#define right_N   0.0
#define right_TS  0.01        
#define right_FF  0.0

#define left_kp  right_kp 
#define left_ki  right_kp         
#define left_kd  0.0
#define left_N   0.0
#define left_TS  0.01        
#define left_FF  0.0

#define back_kp  right_kp 
#define back_ki  right_kp         
#define back_kd  0.0
#define back_N   0.0
#define back_TS  0.01        
#define back_FF  0.0


#define  position_tolerance  0.005        
#define  orientation_tolerance  0.06

#define  max_velocity_magnitude  1.5      
#define  max_rotation_magnitude  2.0      
#define  cutoff_velocity_magnitude  0.8  
#define  cutoff_rotation_magnitude  1.7   

#define  max_linear_acceleration_magnitude   2.5 
#define  max_angular_acceleration_magnitude  2.5
#define  max_linear_deceleration_magnitude   3.0
#define  max_angular_deceleration_magnitude  3.0

#define  linear_acceleration_feedforward_factor   1.7
#define  angular_acceleration_feedforward_factor  1.7
#define  linear_deceleration_feedforward_factor   1.7
#define  angular_deceleration_feedforward_factor  1.7


#endif