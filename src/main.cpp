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
 
/* MODE DEBUG YANG DIINGINKAN */ 
#define AMBIL_ENCODER
// #define GERAK_MOTOR
// #define AMBIL_KECEPATAN
// #define TES_PID
 
 
/***** PIN LIST *****/
/* pin assignment untuk encoder */
encoderKRAI enc(PC_10 , PC_11 ,  538, encoderKRAI::X4_ENCODING);
encoderKRAI enc2(PC_13 , PC_12 ,  538, encoderKRAI::X4_ENCODING);
encoderKRAI enc3(PC_14 , PC_3 ,  538, encoderKRAI::X4_ENCODING);
encoderKRAI enc4(PC_15 , PC_2 ,  538, encoderKRAI::X4_ENCODING);

/* pin assignment untuk motor */
Motor motor1(PB_2, PB_15,PB_1); 
Motor motor2(PB_2, PB_15,PB_1); 
Motor motor3(PB_2, PB_15,PB_1); 
Motor motor4(PB_2, PB_15,PB_1);

/* pin assignment lainnya */
DigitalIn mybutton(USER_BUTTON); 

/* komunikasi serial dengan UART */
Serial pc(USBTX, USBRX, 115200); 

/* timer untuk mendapatkan waktu */
Timer timer1;


/* deklarasi variable global */
/* array untuk menyimpan data kecepatan */
float speed[20000];

/* variable kecepatan dan posisi*/
float curr_speed, prev_speed;
float en1,en2,en3,en4;

/* variable sampling time */
int samp=1, TS;
int i;
int counttt;


/* variable pid */
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
            motor1.speed(0.5);
            motor2.speed(0.5);
            motor3.speed(0.5);
            motor4.speed(0.5);
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
        while(1)
        {
            teta_ref=3.14/6;
            t.start();
            motor.period(0.00004);

            if (t.read_us() - last_baca > 7000)
            {
                baca_enc();
                last_baca = t.read.us();
            }

            if(t.read_us() - last_pid > 8000)
            {
                TS_pid = t.read_us() - last_pid;
                /*pid theta*/
                // pid(teta_ref,0, 0, 0, teta_act, kp_teta, ki_teta, kd_teta, TS_pid, w_ref);
                // pid(w_ref, teta_ref, prev_teta_ref, 1, w_act, kp_w, ki_w, kd_w, TS_pid, pwm);

                prev_teta_ref = teta_ref;
                last_pid = t.read.us();
                
            }
            if(t.read_us() - last_motor > 9000)
            {
                motor.speed(pwm);

                last_motor = t.read_us();
            }
            
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














