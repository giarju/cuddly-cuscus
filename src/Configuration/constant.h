#ifndef CONSTANT_H
#define CONSTANT_H

/************* Konversi *************/

#define  PI                                     3.14159265358979f
#define  ENC_MOTOR_PULSE                        538
#define  MS_TO_S                                1000
#define  US_TO_S                                1000000
#define  radian_to_degree                       57.295779
#define  wheel_radius                           0.050
#define  wheel_distance_from_center             0.180

/************* PID Motor *************/
#define A_kp  0.3 
#define A_ki  0.1         
#define A_kd  0.0
#define A_N   0.0
#define A_TS  0.01        
#define A_FF  0.0

#define B_kp  0.3 
#define B_ki  0.1         
#define B_kd  0.0
#define B_N   0.0
#define B_TS  0.01        
#define B_FF  0.0

#define C_kp  0.3 
#define C_ki  0.1         
#define C_kd  0.0
#define C_N   0.0
#define C_TS  0.01        
#define C_FF  0.0

#define D_kp  0.3 
#define D_ki  0.1         
#define D_kd  0.0
#define D_N   0.0
#define D_TS  0.01        
#define D_FF  0.0

/************* waktu sampling dalam microsecond *************/

/* sampling odometry */
#define ODOMETRY_SAMP 60173
/* sampling encoder base */
#define ENC_MOTOR_SAMP 7173
/* sampling komunikasi UART */
#define SERIAL_SAMP 500173
/* sampling gerak motor base */
#define MOTOR_SAMP 9173
/* sampling tracking path */
#define TRACKING_SAMP 3173
/* sampling pid motor base */
#define PID_MOTOR_SAMP 5173


#endif