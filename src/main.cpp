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
#include "PID/PID.h"
 
/* MODE DEBUG YANG DIINGINKAN */ 
// #define AMBIL_ENCODER
// #define GERAK_MOTOR
// #define AMBIL_KECEPATAN
#define TES_PID
 
 
/***** PIN LIST *****/
/* pin assignment untuk encoder */
encoderKRAI enc(PC_10 , PC_11 ,  538, encoderKRAI::X4_ENCODING);
encoderKRAI enc2(PC_13 , PC_12 ,  538, encoderKRAI::X4_ENCODING);
encoderKRAI enc3(PC_14 , PC_3 ,  538, encoderKRAI::X4_ENCODING);
encoderKRAI enc4(PC_15 , PC_2 ,  538, encoderKRAI::X4_ENCODING);

/* pin assignment untuk motor */
Motor motor1(PA_6, PA_5, PA_7); 
Motor motor2(PB_2, PB_15,PB_1); 
Motor motor3(PB_2, PB_15,PB_1); 
Motor motor4(PB_2, PB_15,PB_1);

float kp1,kp2,kp3,kp4;
float kd1 = 0,kd2 = 0,kd3 = 0,kd4 = 0;
float ki1 = 0,ki2 = 0,ki3 = 0,ki4 = 0;
float ff1 = 0,ff2 = 0,ff3 = 0,ff4 = 0;
float n1 = 0,n2 = 0,n3 = 0,n4 = 0;
float ts1 = 0.007;

PID pid1(kp1, ki1, kd1, n1, ts1, ff1, PID::PI_MODE);
PID pid2(kp2, ki2, kd2, n2, ts1, ff2, PID::PI_MODE);
PID pid3(kp3, ki3, kd3, n3, ts1, ff3, PID::PI_MODE);
PID pid4(kp4, ki4, kd4, n4, ts1, ff4, PID::PI_MODE);

/* pin assignment lainnya */
DigitalIn mybutton(USER_BUTTON); 

/* komunikasi serial dengan UART */
Serial pc(USBTX, USBRX, 115200); 

/* timer untuk mendapatkan waktu */
Timer timer1;


/* deklarasi variable global */
/* array untuk menyimpan data kecepatan */
float speed[400];
float speed2[400];
float speed3[400];
float speed4[400];

/* variable kecepatan dan posisi*/
float curr_speed, prev_speed;
float en1,en2,en3,en4;

/* variable sampling time */
int samp, samp_pid, TS;
int i;
int counttt;


/* variable pid */
float curr_speed2,curr_speed3,curr_speed4;
float pwm1,pwm2,pwm3,pwm4;

float teta_ref; 
float teta_act;
float kp_teta = 0.005;
float ki_teta = 0;
float kd_teta = 0;
float TS_pid;
float w_ref;
float prev_teta_ref;
float w_act;
float kp_w = 0.005;
float ki_w = 0;
float kd_w = 0.005;
float pwm;

/* variable penyimpan waktu */
uint32_t last_baca, last_pid, last_motor;

/* prototipe fungsi */
//void pid (float ref, float curr_feed, float prev_feed, float feedforward, float actual, float kp, float ki, float kd, float TS, float* output);

int main() {
    /* ================================================================== */
    #ifdef AMBIL_ENCODER 
        while(1)
        {          
            en1= (float)enc.getPulses();
            en2= (float)enc2.getPulses();
            en3= (float)enc3.getPulses(); 
            en4= (float)enc4.getPulses();

            pc.printf("%f\t %f\t %f\t %f\n", en1, en2, en3, en4);
        }
    #endif

    /* ================================================================== */

    #ifdef GERAK_MOTOR
        while(1)
        {
            float pwm = 0.5;
            motor1.speed(pwm);

            // pc.printf("%f", pwm);
            // motor2.speed(0.5);
            // motor3.speed(0.5);
            // motor4.speed(0.5);
        }
    #endif    

    /* ================================================================== */
    #ifdef AMBIL_KECEPATAN

        /* setup and initialization*/
        timer1.start();
        motor.period(0.00004);
    
        /* command move motor and sample data*/ 
        while(counttt <= 400)
        {           
            if (t.read_us()-samp >= 5000)
            {
                TS = t.read_us()-samp;
                curr_speed = (float)enc.getPulses()*360/538/TS*1000000;
                speed[counttt] = curr_speed;
                enc.reset();
                motor.speed(0.3);
                counttt ++;
                samp = t.read_us(); 
            }    
        }
        motor.speed(0); /* turn off motor after sampling done */

        /* print data */
        for(i = 0; i < 400; i++)
        {
            pc.printf("%f\n", speed[i]);          
        }   
    #endif
    /* ================================================================== */

    #ifdef TES_PID
        /* setup and initialization*/
        timer1.start();
        motor1.period(0.00004);
        motor2.period(0.00004);
        motor3.period(0.00004);
        motor4.period(0.00004);
    
        /* command move motor and sample data*/ 
        while(counttt <= 400)
        {           
            if (timer1.read_us()-samp >= 7000)
            {
                TS = timer1.read_us()-samp;
                curr_speed = (float)enc.getPulses()*360/538/TS*1000000;
                curr_speed2 = (float)enc2.getPulses()*360/538/TS*1000000;
                curr_speed3 = (float)enc3.getPulses()*360/538/TS*1000000;
                curr_speed4 = (float)enc4.getPulses()*360/538/TS*1000000;
                speed[counttt] = curr_speed;
                speed2[counttt] = curr_speed2;
                speed3[counttt] = curr_speed3;
                speed4[counttt] = curr_speed4;
                enc.reset();
                enc2.reset();
                enc3.reset();
                enc4.reset();
                
                counttt ++;
                samp = timer1.read_us(); 
            } 
            if (timer1.read_us() - samp_pid > 9000)
            {
                float setpoint = 1;
                // pwm1 = pid1.createpwm(setpoint,curr_speed);
                // pwm2 = pid2.createpwm(setpoint,curr_speed2);
                // pwm3 = pid3.createpwm(setpoint,curr_speed3);
                // pwm4 = pid4.createpwm(setpoint,curr_speed4);
                
                motor1.speed(0.5);                
                motor2.speed(0.5);
                motor3.speed(0.5);
                motor4.speed(0.5);

            }   
            
        }

        
        motor1.speed(0);
        motor2.speed(0);
        motor3.speed(0);
        motor4.speed(0);
         /* turn off motor after sampling done */

        /* print data */
        for(i = 0; i < 400; i++)
        {
            pc.printf("%f   %f  %f  %f\n", speed[i], speed2[i], speed3[i], speed4[i]);          
        } 
    #endif
    /* ================================================================== */

}


/* definisi fungsi */
// void pid (float ref, float curr_feed, float prev_feed, float feedforward, float actual, float kp, float ki, float kd, float TS, float* output)
// {
//     float error = ref - actual;
//     float ref_diff = feedforward*(curr_feed - prev_feed)/TS;

//     float input = error + ref_diff;
//     float total_input += input;


//     output = kp*input + kd*(input-last_input) + ki*(total_input);  

//     float last_input =  input;

// }

// /*fungsi untuk mendapatkan kecepatan dan posisi encoder*/
// void baca_enc()
// {
//     teta_actual += enc.getPulses()*2*3.14/538;
//     w_actual = enc.getPulses()*2*3.14/538/TS;
//     enc.reset();
// }














