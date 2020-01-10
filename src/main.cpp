/***************************************************************************
 * Title      : MAIN PROGRAM ROBOT PR
 * Author     : KRAI ITB 2020
 *
 * 
 ***************************************************************************/

/********************** Library ******************************/

#include "mbed.h"
#include "Configuration/procedure.h"

/******************** Aktivasi Debug ************************/

#define ODOMETRY_DEBUG 
// #define SERIAL_DEBUG 
// #define MOTOR_DEBUG
// #define ENCMOTOR_DEBUG
// #define PID_MOTOR_DEBUG 

/******************* Main Function **************************/
int main ()
{
    /* initial setup */
    Odometry.resetOdom();   
    profiler.start(); 

    /* inisialisasi sampling untuk setiap proses dengan callback*/
    #ifdef ODOMETRY_DEBUG
        /* sampling odometri base */
        odometry_ticker.attach_us(&odometrySamp, ODOMETRY_SAMP);
    #endif

    #ifdef ENCMOTOR_DEBUG
        /*sampling encoder motor base*/
        encoder_motor_ticker.attach_us(&encoderMotorSamp, ENC_MOTOR_SAMP);
    #endif 

    #ifdef MOTOR_DEBUG
        /* sampling pwm motor base*/
        motor_ticker.attach_us(&motorSamp, MOTOR_SAMP);
    #endif

    #ifdef PID_MOTOR_DEBUG
        /* sampling pid motor base */
        pid_motor_ticker.attach_us(&pidMotorSamp, PID_MOTOR_SAMP);
    #endif 

    #ifdef TRACKING_DEBUG
        /* sampling tracking base */
        tracking_ticker.attach_us(&trackingSamp, TRACKING_SAMP);
    #endif  

    #ifdef SERIAL_DEBUG
        /* sampling komunikasi serial */
        serial_ticker.attach_us(&pcSerialSamp, SERIAL_SAMP);
    #endif 

    while (1)
    {   
        wait(10.0);   
            // prof_start1 = profiler.read_us();
            // prof_end1 = profiler.read_us();
            // diff1 = prof_end1 - prof_start1;
            // pc.printf("%lu\n", diff1);       
    } 
}
