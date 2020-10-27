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
#define  PURSUIT_RADIUS_BASE                    0.05f

/************* PID Motor *************/
/*rise 0.194s | settling 0.409s | os 0% | ts 7.173 ms |rt 0.1727| tb 0.9*/
#define A_kp  4.743614260950874
#define A_ki  65.890575711334310      
#define A_kd  -0.037613522928484
#define A_N   0.052720515154962
#define A_TS  0.007173        
#define A_kf1  51.048895929452830 
#define A_kf2  -44.946519262615790 
#define A_kf3  0.0  
#define A_kf4  0.002632881758475 

#define B_kp  5.047071647758642
#define B_ki  72.204761698429760   
#define B_kd  -0.023466811057628
#define B_N   0.078577677156486
#define B_TS  0.007173        
#define B_kf1  68.211306680393850
#define B_kf2  -60.230000278996150 
#define B_kf3  0.189793189037091 
#define B_kf4  0.195165374556462 

#define C_kp  5.047954619068664 
#define C_ki  75.365882022074090         
#define C_kd  -0.044234611330264
#define C_N   0.064265069807276
#define C_TS  0.007173        
#define C_kf1  55.077543529709274
#define C_kf2  -47.979533420370200 
#define C_kf3  0.0  
#define C_kf4 0.008470776414568

#define D_kp  4.836138846898381
#define D_ki  71.063369454015560          
#define D_kd  -0.039707939756869
#define D_N   0.064251253452039
#define D_TS  0.007173        
#define D_kf1  52.853975255550250
#define D_kf2  -46.193650762220870 
#define D_kf3  0.0  
#define D_kf4  0.005697044473534
/************* waktu sampling dalam microsecond *************/

/* sampling odometry */
#define ODOMETRY_SAMP 60173
/* sampling encoder base */
#define ENC_MOTOR_SAMP 7173
/* sampling komunikasi UART */
#define SERIAL_SAMP 7173
/* sampling gerak motor base */
#define MOTOR_SAMP 1000 //5173 aslinya
/* sampling tracking path */
#define TRACKING_SAMP 5173
/* sampling pid motor base */
#define PID_MOTOR_SAMP 7173

#define STICK_SAMP 20173


#endif