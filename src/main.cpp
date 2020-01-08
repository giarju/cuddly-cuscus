#include "mbed.h"
#include "Configuration/pinList.h"
#include "Path/Path.h"
#include "odometriKRAI/odometriKRAI.h"

#define ODOMETRY_DEBUG 
// #define SERIAL_DEBUG 
// #define MOTOR_DEBUG
// #define SERIAL_DEBUG


Serial pc(USBTX, USBRX, 115200);
DigitalIn mybutton(USER_BUTTON);

odometriKRAI Odometry(TIM2, TIM3, CMPS_SDA, CMPS_SCL);

// encoderKRAI A_enc(PIN_A_CHA, PIN_A_CHB, 538, encoderKRAI::X4_ENCODING);
// encoderKRAI B_enc(PIN_B_CHA, PIN_B_CHB, 538, encoderKRAI::X4_ENCODING);
// encoderKRAI C_enc(PIN_C_CHA, PIN_C_CHB, 538, encoderKRAI::X4_ENCODING);
// encoderKRAI D_enc(PIN_D_CHA, PIN_D_CHB, 538, encoderKRAI::X4_ENCODING);

// Motor A_motor(PB_2, PB_15,PB_1); 
// Motor B_motor(PB_2, PB_15,PB_1); 
// Motor C_motor(PB_2, PB_15,PB_1); 
// Motor D_motor(PB_2, PB_15,PB_1); 

uint32_t odometry_last, serial_last;
uint32_t prof_start1, prof_end1, diff1;


/* dalam microsecond */
#define ODOMETRY_SAMP 5000
#define SERIAL_SAMP 500000

/* timer untuk mendapatkan waktu */
Timer timer_samp;
Timer profiler;
Ticker ticker_samp;

/* function declaration */
void odometry_samp();

int main ()
{
    /* initial setup */
    // Odometry.resetOdom();

    while (1)
    {
        /* sampling untuk setiap proses*/
        #ifdef ODOMETRY_DEBUG      
        ticker_samp.attach_us(odometry_samp, ODOMETRY_SAMP);
        // if (timer_samp.read_us()- odometry_last >= ODOMETRY_SAMP)
        // {
        //     Odometry.updatePosition();  
        //     odometry_last = timer_samp.read_us(); 
        // } 
        
        #endif 

        #ifdef ENCMOTOR_DEBUG
        if (timer_samp.read_us()- enc_motor_last >= ENCMOTOR_SAMP)
        {
            uint32_t enc_motor_ts = timer_samp.read_us() - enc_motor_last;
            a_motor_speed = (float)A_enc.getPulses()*2*PI/ENC_MOTOR_PULSE/enc_motor_ts*1000000;
            b_motor_speed = (float)B_enc.getPulses()*2*PI/ENC_MOTOR_PULSE/enc_motor_ts*1000000;
            c_motor_speed = (float)C_enc.getPulses()*2*PI/ENC_MOTOR_PULSE/enc_motor_ts*1000000;
            d_motor_speed = (float)D_enc.getPulses()*2*PI/ENC_MOTOR_PULSE/enc_motor_ts*1000000;
            
            A_enc.reset();
            B_enc.reset();
            C_enc.reset();
            D_enc.reset();

            enc_motor_last = timer_samp.read_us(); 
        } 
        #endif 

        #ifdef MOTOR_DEBUG
        if (timer_samp.read_us()- motor_last >= MOTOR_SAMP)
        {

            A_motor.speed(A_pwm);
            B_motor.speed(B_pwm);
            C_motor.speed(C_pwm); 
            D_motor.speed(D_pwm);

            motor_last = timer_samp.read_us(); 
        } 
        #endif 


        #ifdef SERIAL_DEBUG
        if (timer_samp.read_us()- serial_last >= SERIAL_SAMP)
        {
            /* debug odometry */
            pc.printf("x : %.2f   y: %.2f   teta :%.2f", Odometry.position.x,Odometry.position.y,Odometry.position.teta)  
            serial_last = t.read_us(); 
        } 
        #endif        
    } 
}

void odometry_samp ()
{
    prof_start1 = profiler.read_us();
    Odometry.updatePosition();
    prof_end1 = profiler.read_us();
    diff1 = prof_end1 - prof_start1;

    pc.printf("%lu  \n", diff1);
}