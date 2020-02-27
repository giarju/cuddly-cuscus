#ifndef CONSTANT_H
#define CONSTANT_H

/************* Konversi *************/

#define  PI                                     3.14159265358979f
#define  ENC_MOTOR_PULSE                        538
#define  MS_TO_S                                1000
#define  US_TO_S                                1000000
#define  RAD_TO_DEG                             57.295779
#define  WHEEL_RAD                              0.075
#define  wheel_distance_from_center             0.180

/************* PID Motor *************/
/*rise 0.3 settling 1.06 ov 2.5%   rt .3183  tb 0.85*/
#define A_kp  0.280702765920569
#define A_ki  0.530678194814341      
#define A_kd  0.0
#define A_N   0.0
#define A_TS  0.004173        
#define A_ka  0.0337 
#define A_kb  0.0382   


#define B_kp  0.266840134135711 
#define B_ki  0.544568637099078         
#define B_kd  0.0
#define B_N   0.0
#define B_TS  0.004173        
#define B_ka  0.0337
#define B_kb  0.0382

#define C_kp  0.275515003804776 
#define C_ki  0.488895341886798         
#define C_kd  0.0
#define C_N   0.0
#define C_TS  0.004173        
#define C_ka  0.0337
#define C_kb  0.0382

#define D_kp  0.350824541013637
#define D_ki  0.528542810598886         
#define D_kd  0.0
#define D_N   0.0
#define D_TS  0.004173        
#define D_ka  0.0437
#define D_kb  0.0482

/************* waktu sampling dalam microsecond *************/

/* sampling odometry */
#define ODOMETRY_SAMP 60173
/* sampling encoder base */
#define ENC_MOTOR_SAMP 7173
/* sampling komunikasi UART */
#define SERIAL_SAMP 7173
/* sampling gerak motor base */
#define MOTOR_SAMP 9173
/* sampling tracking path */
#define TRACKING_SAMP 5173
/* sampling pid motor base */
#define PID_MOTOR_SAMP 4173

#define STICK_SAMP 5173


#endif