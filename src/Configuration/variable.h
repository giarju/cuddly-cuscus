#ifndef VARIABLE_H
#define VARIABLE_H

/*************************** Library  **********************************/
#include "string.h"
#include "Path/Path.h"
#include "Tracking/Tracking.h"
#include "InverseKinematics/InverseKinematics.h"
#include "JoystickPS3/JoystickPS3.h"
#include "PID/PID.h"
#include "millis/millis.h"

 
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

/* variable untuk menyimpan waktu 
 * prof_start1 : menyimpan waktu awal 
 * prof_end1   : menyimpan waktu akhir
 * diff1       : perbedaan awal dan akhir*/
uint32_t prof_start1, prof_end1, diff1;

uint32_t lastTimeTangan;


/********************** Pergerakan Base ******************************/

/* variable untuk menyimpan kecepatan motor base 
 *  */

/* x_motor_speed : kecepatan aktual motor */
float a_motor_speed,b_motor_speed,c_motor_speed,d_motor_speed,right_arm_speed,left_arm_speed;
/* x_target_speed: kecepatan target motor */
float a_target_speed,b_target_speed,c_target_speed,d_target_speed;
/* X_pwm         : pwm yang diberikan ke motor*/
float A_pwm,B_pwm,C_pwm,D_pwm;
float A_pwm_dzcompensated,B_pwm_dzcompensated,C_pwm_dzcompensated,D_pwm_dzcompensated;

/* variable untuk menyimpan trajectory */

/* i-th point dari map trajectory*/
int index_traject;
/* Map trajectory, berisi nilai x,y,theta dan vx,vy,omega*/
Trajectory map[100];
/* kecapatan base, berisi nilai vx,vy,omega*/
Coordinate base_speed;


/********************** PID Motor ******************************/

/* Objek untuk melakukan PID motor */ 

/* PID untuk motor A base (mode PI) */
PID A_pid_motor(A_kp, A_ki, A_kd, A_N, A_TS, A_ka, A_kb, PID::PI_MODE);
/* PID untuk motor B base (mode PI) */
PID B_pid_motor(B_kp, B_ki, B_kd, B_N, B_TS, B_ka, B_kb, PID::PI_MODE);
/* PID untuk motor C base (mode PI) */
PID C_pid_motor(C_kp, C_ki, C_kd, C_N, C_TS, C_ka, C_kb, PID::PI_MODE);
/* PID untuk motor D base (mode PI) */
PID D_pid_motor(D_kp, D_ki, D_kd, D_N, D_TS, D_ka, D_kb, PID::PI_MODE);


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

/********************** Pneumatic ******************************/
bool state_kiri;


#endif