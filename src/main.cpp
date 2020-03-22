/***************************************************************************
 * Title      : Program Debug Motor dan Encoder
 * Name       : debug_motor.cpp
 * Version    : 1.0
 * Author     : Gian Arjuna EL 16
 * Date       : 12 Desember 2019
 * Description:
 *
 * Program ini dapat digunakan untuk mengambil data pulse encoder, menggerakan
 * motor, mengambil data kecepatan, dan mengambil data PID. Untuk menggunakan
 * program ini UBAH DEKLARASI PIN PADA BAGIAN PIN LIST DIBAWAH agar sesuai
 * dengan pin yang sedang digunakan. Kemudian pilih mode debug yang ingin 
 * dilakukan dengan comment/uncomment "#define" pada bagian mode debug
 *
 ***************************************************************************/

 
/* LIBRARY */
#include "mbed.h"
#include "encoderKRAI.h"
#include "Motor.h"
#include "pid_dagoz/PID.h"
 
/* MODE DEBUG YANG DIINGINKAN */ 
//#define AMBIL_ENCODER
#define TES_PID_TANGAN_KANAN
//#define TES_PID_TANGAN_KIRI
 
/***** PIN LIST *****/
/* pin assignment untuk encoder */
encoderKRAI enc_arm1(PB_7, PB_6, 538, encoderKRAI::X4_ENCODING);
//encoderKRAI enc_arm2(PB_2, PD_2, 538, encoderKRAI::X4_ENCODING);

/* pin assignment untuk motor */

Motor arm1(PA_15, PA_13, PA_14);
//Motor arm2(PC_8, PC_6, PC_5);

/* pin assignment untuk pneumatik */
DigitalOut pneuKiri(PA_9);
DigitalOut pneuKanan(PC_7);

float kp1 = 0.01718422,kp2 = 0.0102622;
float kd1 = 0.065,kd2 = 0.065;

/* pin assignment lainnya */
DigitalIn mybutton(USER_BUTTON, PullUp); 

/* komunikasi serial dengan UART */
Serial pc(USBTX, USBRX, 115200); 

/* timer untuk mendapatkan waktu */
Timer timer1;


/* deklarasi variable global */
/* array untuk menyimpan data kecepatan */
float theta1[400];
float theta2[400];
float input[400];
float time_array[400];

/* variable kecepatan dan posisi*/
float curr_theta, prev_theta;
float en1,en2;

/* variable sampling time */
int samp, samp_pid, TS;
int i;
int counttt;

/* variable pid */
float curr_theta1,curr_theta2;
float prev_error, lowpass_error, prev_lowpass_error;
float pwm1,pwm2;

/* variable penyimpan waktu */
uint32_t last_baca, last_pid, last_motor;

void tangankanan(float target,float feedback);
void tangankiri(float target,float feedback);

DigitalIn kanan(PA_7);
DigitalIn kiri(PA_6);

int main() {
    /* ================================================================== */
        /* setup and initialization*/
        timer1.start();
        arm1.period(0.005);
//        arm2.period(0.005);
    
        /* command move motor and sample data*/ 
        while(1)
        {      
            if (timer1.read_us()-samp >= 7123){
                curr_theta2 += (float)enc_arm1.getPulses()*360/538;
                curr_theta1 += (float)enc_arm1.getPulses()*360/538;

                enc_arm1.reset();
                        
                samp = timer1.read_us(); 
                }
                
            if (timer1.read_us() - samp_pid > 9127){   
                if(kiri==1 && kanan==0){
                    tangankiri(150,curr_theta2);
                    }
                if(kiri==0 && kanan==1){
                    tangankiri(0,curr_theta2);
                    }
                if(kanan==1 && kiri==0){
                    tangankanan(110,curr_theta1);
                    }
                if(kanan==0 && kiri==1){
                    tangankanan(10,curr_theta1);
                    }  
                samp_pid = timer1.read_us();     
                }
                  
            if(timer1.read_us() - last_baca > 100000){
                if (curr_theta2>=10 && curr_theta1<=10){
                    pc.printf("%.2f %.2f\n", curr_theta2, pwm2);
                    }
                if (curr_theta1>=10 && curr_theta2<=10){
                    pc.printf("%.2f %.2f\n", curr_theta1, pwm1);
                    }                
                }         
        }
    }
    /* ================================================================== */

                     
/* fungsi untuk menggerakan tangan kanan */
void tangankanan(float target,float feedback){
    float error = target - feedback;

    lowpass_error = 0.1*error + 0.9*prev_error;
    
    pwm1 = kp1*(lowpass_error) + kd1*(lowpass_error - prev_lowpass_error);
    pwm1 = fabs(pwm1) > 0.85 ? 0.85*fabs(pwm1)/pwm1 : pwm1;
    
    prev_lowpass_error = lowpass_error;
    
    prev_error = error;
                    
    if (error>0){
        if (feedback>90 && feedback < 130 && target >= 90){
            arm1.speed(0);
            arm1.brake();
        }
        else if (feedback> 85 && feedback <= 90){
            arm1.speed(0.075*pwm1);
        }
        else if (feedback>70 && feedback <= 85){
            arm1.speed(0.65*pwm1);
        }
        else {
            arm1.speed(pwm1); 
        }
    }
    else if (error<0){
        //kode turun
        if (feedback>0 && feedback< 60 && target <= 30){
            arm1.speed(0);
            arm1.brake();
        }
        else if (feedback>50 && feedback <= 90){
            arm1.speed(0.3*pwm1);
        }
        else {
            arm1.speed(0.7*pwm1); 
        }
    }
}

/* fungsi untuk menggerakan tangan kiri */
void tangankiri(float target,float feedback){
    float error = target - feedback;

    lowpass_error = 0.1*error + 0.9*prev_error;
    
    pwm2 = kp2*(lowpass_error) + kd2*(lowpass_error - prev_lowpass_error);
    pwm2 = fabs(pwm2) > 0.85 ? 0.85*fabs(pwm2)/pwm2 : pwm2;
    
    prev_lowpass_error = lowpass_error;
    
    prev_error = error;

    if (error>0){
        if (feedback>140 && feedback < 170 && target >= 145){
            arm1.speed(0);
            arm1.brake();
        }
        else if (feedback>130 && feedback <= 140){
            arm1.speed(0.075*pwm2);
        }
        else if (feedback>115 && feedback <= 130){
            arm1.speed(0.65*pwm2);
        }
        else {
            arm1.speed(pwm2); 
        }
    }
    else if (error<0){
        //kode turun
        if (feedback>0 && feedback< 100){
            arm1.speed(0);
            arm1.brake();
        }
        else if (feedback>100 && feedback <= 140){
            arm1.speed(0.6*pwm2);
        }
        else {
            arm1.speed(0.8*pwm2); 
        }
    }
}
