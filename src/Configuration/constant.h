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
/*rise 10.7 s | settling 19.1s | os 0% | ts 7.173 ms |rt 9.767| tb 0.9*/
#define A_kp  0.165753520204336*150
#define A_ki  0.0965661691095015*0.01    
#define A_kd  0 
#define A_N   0
#define A_TS  0.007173        
#define A_kf1  3.016524409519466e+03
#define A_kf2  -3.004088350637785e+03 
#define A_kf3  0.148749100767772  
#define A_kf4  0.111924135987302 

#define B_kp  0.245561697353188*100
#define B_ki  0.1069500992917*0.01    
#define B_kd  0
#define B_N   0
#define B_TS  0.007173        
#define B_kf1  4.475854469047804e+04
#define B_kf2  -8.171484391983945e+04
#define B_kf3  3.698068453630511e+04 
#define B_kf4  0.938368096989886 

#define C_kp  0.130725641191755*170
#define C_ki  0.101390115536248*0.01        
#define C_kd  0
#define C_N   0
#define C_TS  0.007173        
#define C_kf1  3.283735750224793e+04
#define C_kf2  -6.147384225844021e+04
#define C_kf3 2.865979074603082e+04
#define C_kf4 0.955660877593407

#define D_kp  0.132034365929487*100
#define D_ki  0.0966865949171276*0.01        
#define D_kd  0
#define D_N   0
#define D_TS  0.007173        
#define D_kf1  4.475854469047804e+04
#define D_kf2  -8.171484391983945e+04 
#define D_kf3  3.698068453630511e+04 
#define D_kf4  0.938368096989886
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
/* Sampling joystick */
#define STICK_SAMP 20173
/* Sampling fsm */
#define FSM_SAMP 10173

typedef struct stick_message_t{
        bool stick_kanan; bool stick_kiri; bool stick_atas; bool stick_bawah;
        bool stick_segitiga; bool stick_lingkaran; bool stick_silang; bool stick_kotak;

        bool stick_kanan_click; bool stick_kiri_click; bool stick_atas_click; bool stick_bawah_click;
        bool stick_segitiga_click; bool stick_lingkaran_click; bool stick_silang_click; bool stick_kotak_click;

        bool stick_R1; bool stick_R2; bool stick_L1; bool stick_L2;
        bool stick_select; bool stick_start; 

        bool stick_R1_click; bool stick_R2_click; bool stick_L1_click; bool stick_L2_click;
        bool stick_select_click; bool stick_start_click;    
} stick_message;


#endif