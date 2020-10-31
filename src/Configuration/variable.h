#ifndef VARIABLE_H
#define VARIABLE_H

/*************************** Library  **********************************/
#include "string.h"
#include "Path/Path.h"
#include "Tracking/Tracking.h"
#include "InverseKinematics/InverseKinematics.h"
#include "JoystickPS3/JoystickPS3.h"
#include "PID/PID.h"
#include "Configuration/constant.h"
#include "Statoo/state.h"

 
/*************************** Timer  **********************************/

/* timer  dan ticker untuk mengukur waktu */

/* profiler         : mengukur lama waktu suatu potongan program dijalankan */
Timer profiler;
/* interupsi sampling odometri*/
Ticker odometry_ticker;  
/* interupsi sampling encoder motor*/    
Ticker encoder_motor_ticker;
/* interupsi sampling serial print*/
Ticker serial_ticker;
/* interupsi sampling motor base*/
Ticker motor_ticker;
/* interupsi sampling tracking*/
Ticker tracking_ticker;
/* interupsi sampling pid motor base*/
Ticker pid_motor_ticker;

Ticker stick_ticker;

Ticker simul_ticker;

Ticker fsm_ticker;

/* variable untuk menyimpan waktu 
 * prof_start1 : menyimpan waktu awal 
 * prof_end1   : menyimpan waktu akhir
 * diff1       : perbedaan awal dan akhir*/
uint32_t prof_start1, prof_end1, diff1;

/* variable untuk menyimpan waktu terakhir sampling
 *
 * @param lastTimeTangan : sampling tangan 
 * @param last_time_joystick   : sampling terakhir joystick
 * @param joysamptime       : sampling time joystick
 */
uint32_t lastTimeTangan, last_time_joystick;
float joysamptime;



/********************** Pergerakan Base ******************************/

/* variable untuk menyimpan kecepatan motor base 
 * */

/* x_motor_speed : kecepatan aktual motor */
float a_motor_speed,b_motor_speed,c_motor_speed,d_motor_speed,right_arm_speed,left_arm_speed;

/* x_target_speed: kecepatan target motor 
 * x_next_speed : kecepatan target berikutnya 
 * */
float a_target_speed,b_target_speed,c_target_speed,d_target_speed;
float a_next_speed,b_next_speed,c_next_speed,d_next_speed;

/* X_pwm         : pwm yang diberikan ke motor*/
float A_pwm,B_pwm,C_pwm,D_pwm;

/* variable untuk menyimpan `trajectory */

/* i-th point dari map trajectory*/
int index_curr_pos;
/* i-th point dari map trajectory*/
int index_next_pos;

/* kecapatan base, berisi nilai vx,vy,omega*/
Coordinate base_speed;
Coordinate base_prev_speed;

/*pembacaan kecepatan base_motor*/
float v_resultan;

/*pembacaan sudut dari current position ke titik target */
float alpha;

/* variable Trajectory Tracking
 * */
int curr_clst_point; int tp;
Trajectory_vr next_point;
int target_point;
int is_intersect;
float alphass;
float cos_alpha;
float vr_sim;
int map_state;
int map_check;
uint32_t button_debounce;
bool near_last;

/* variable robot manual */
float lastThetaRobot = 0;
float totalThetaRobot = 0;
float theta_destination = 0;

/********************* State Machine ****************************/
FSM Statoo(0);

/********************** PID Motor ******************************/

/* Objek untuk melakukan PID motor */ 

/* PID untuk motor A base (mode PID) */
PID A_pid_motor(A_kp, A_ki, A_kd, A_N, A_TS, A_kf1, A_kf2, A_kf3, A_kf4, PID::PID_CONT_MODE);
/* PID untuk motor B base (mode PID) */
PID B_pid_motor(B_kp, B_ki, B_kd, B_N, B_TS, B_kf1, B_kf2, B_kf3, B_kf4, PID::PID_CONT_MODE);
/* PID untuk motor C base (mode PID) */
PID C_pid_motor(C_kp, C_ki, C_kd, C_N, C_TS, C_kf1, C_kf2, C_kf3, C_kf4, PID::PID_CONT_MODE);
/* PID untuk motor D base (mode PID) */
PID D_pid_motor(D_kp, D_ki, D_kd, D_N, D_TS, D_kf1, D_kf2, D_kf3, D_kf4, PID::PID_CONT_MODE);


/********************** Komunikasi ******************************/

/* variable untuk melakukan komunikasi UART dengan double buffer */

/* buffer yang dibaca hardware dan dikirimkan melalui UART
 * maks ukuran string 64 karakter*/ 
char uart_buffer[64];
/* buffer yang ditulis oleh user
 * maks ukuran string 64 karakter*/ 
char str_buffer[64];
/* pointer untuk menunjuk address uart_buffer*/ 
char* uart_pointer;
/* panjang string yang dikirimkan user*/ 
int8_t uart_buff_len;

/* mutex untuk critical section UART*/
PlatformMutex uart_mutex; 

/********************** Tangan ******************************/
bool state_kiri;
float posisi_tangan;
int count_print = 0;
int count_select = 0;

/********************** Joystick Press ******************************/
stick_message joy;

/********************** Simulasi ******************************/
#ifdef SIMUL_DEBUG
RobotModel robot_sim1(RobotModel::OMNI_4);
int simulcnt = 0;
float simulspeed,simv2;
float simv;
float simv3;
#endif

/********************** Akuisisi Data Motor *************************/

#if defined(GET_MOTOR) || defined(PID_TUNE)
float data1[600], data2[600], data3[600], data4[600];
uint32_t data_t[600];
int data_i = 0;
#endif

/********************** Other ******************************/
uint32_t time_s = 0;
int statePrint;


#endif