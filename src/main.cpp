/***************************************************************************
 * Title      : MAIN PROGRAM ROBOT PR
 * Author     : KRAI ITB 2020
 *
 * 
 ***************************************************************************/

/********************** Library ******************************/

#include "mbed.h"
#include "Configuration/constant.h"
#include "Configuration/variable.h"
#include "Configuration/robotpin.h"
#include "InverseKinematics/InverseKinematics.h"

/******************** Aktivasi Debug ************************/

#define ODOMETRY_DEBUG 
#define SERIAL_DEBUG 
#define MOTOR_DEBUG
#define ENCMOTOR_DEBUG
#define PID_MOTOR_DEBUG 
#define TRACKING_DEBUG
// #define PENGAMAN_PWM



float lastThetaRobot = 0;
float totalThetaRobot = 0;
float theta_destination = 0;


/**************** function declaration ***********************/
void encoderMotorSamp();

/* 
 * prosedur untuk melakukan sampling pwm motor base
 * 
 * */
void motorSamp();

/* 
 * prosedur untuk menghitung pid motor base
 * 
 * */
void pidMotorSamp();

/* 
 * prosedur untuk melakukan tracking path
 * 
 * */
void trackingSamp();

/* 
 * prosedur untuk print string dengan uart setiap sampling time
 * 
 * */
void pcSerialSamp();

/* 
 * prosedur untuk mengirimkan data melalui uart dengan double buffer
 * 
 * */
void sendUart(char *buffer);

/* 
 * prosedur untuk menulis data ke hardware uart
 * 
 * */
void writeUart();

/* 
 * prosedur untuk state joystick
 * 
 * */
void stickState();

/* 
 * prosedur untuk test kecepatan trapesium motor
 * 
 * */
float trapeziumProfile(float amax, float vmax, float smax, float TS,float prev_speed, uint32_t initial_time, uint32_t time);
float trapeziumTarget(float amax, float vmax, float prev_speed, float TS);

void odometrySamp();


/******************* Main Function **************************/
int main ()
{
    /* initial setup */
    // Odometry.resetOdom();   
    profiler.start(); 
    stick.setup();
    stick.idle();
    Odometry.resetOdom();


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
        #ifdef JOYSTICK_DEBUG
        if (stick.readable()){
            stick.baca_data();
            stick.olah_data();
        } 
        if(profiler.read_us() - last_time_joystick > STICK_SAMP ){

            stickState();
            // sprintf(str_buffer, "k\n");
            last_time_joystick = profiler.read_us();
        }
        #endif
            
        // wait(10.0);   
        // prof_start1 = profiler.read_us();
        // prof_end1 = profiler.read_us();
        // diff1 = prof_end1 - prof_start1;
        // pc.printf("");  
    }     
}

#ifdef ODOMETRY_DEBUG
void odometrySamp (){    /*butuh 48018 us */  
    /* update posisi robot berdasarkan odometri */
    Odometry.updatePosition();                                     
}
#endif

/* 
 * prosedur untuk melakukan sampling encoder motor base
 * 
 * */
#ifdef ENCMOTOR_DEBUG
void encoderMotorSamp()  /* butuh 8 us */
{
    /* ukur kecepatan motor base dalam m/s */
    a_motor_speed = (float)A_enc.getPulses()*2*PI*WHEEL_RAD/ENC_MOTOR_PULSE/ENC_MOTOR_SAMP*US_TO_S;
    b_motor_speed = (float)B_enc.getPulses()*2*PI*WHEEL_RAD/ENC_MOTOR_PULSE/ENC_MOTOR_SAMP*US_TO_S;
    c_motor_speed = (float)C_enc.getPulses()*2*PI*WHEEL_RAD/ENC_MOTOR_PULSE/ENC_MOTOR_SAMP*US_TO_S;
    d_motor_speed = (float)D_enc.getPulses()*2*PI*WHEEL_RAD/ENC_MOTOR_PULSE/ENC_MOTOR_SAMP*US_TO_S;

    /* reset nilai encoder */
    A_enc.reset();
    B_enc.reset();
    C_enc.reset();
    D_enc.reset();
}
#endif

/* 
 * prosedur untuk melakukan sampling pwm motor base
 * 
 * */
#ifdef MOTOR_DEBUG
void motorSamp()
{
    if (base_speed.x == 0 && base_speed.y == 0 && base_speed.teta == 0)
    {
        A_motor.forcebrake();
        B_motor.forcebrake();
        C_motor.forcebrake(); 
        D_motor.forcebrake();
    }
    else{      

        A_motor.speed(A_pwm);
        B_motor.speed(B_pwm);
        C_motor.speed(C_pwm); 
        D_motor.speed(D_pwm);
    }

    /* menggerakan motor base */
    A_motor.speed(A_pwm);
    B_motor.speed(B_pwm);
    C_motor.speed(C_pwm); 
    D_motor.speed(D_pwm);
}
#endif

/* 
 * prosedur untuk menghitung pid motor base
 * 
 * */
#ifdef PID_MOTOR_DEBUG
void pidMotorSamp()
{   
    
    /* menghitung pid motor base */
    float max_pwm = 1;
    A_pwm = A_pid_motor.createpwm(a_target_speed, a_motor_speed, max_pwm);
    B_pwm = B_pid_motor.createpwm(b_target_speed, b_motor_speed, max_pwm);
    C_pwm = C_pid_motor.createpwm(c_target_speed, c_motor_speed, max_pwm);
    D_pwm = D_pid_motor.createpwm(d_target_speed, d_motor_speed, max_pwm);   
}
#endif
 
/* 
 * prosedur untuk melakukan tracking path
 * 
 * */
#ifdef TRACKING_DEBUG
void trackingSamp()
{
    /* menghitung kecepatan robot berdasarkan map dan posisi aktual*/
    // base_speed = velocityTracker(map[index_traject], Odometry.position); /* index harusnya dari fsm (index bahaya, shared variable sama fsm)*/
    /* menghitung kecepatan masing2 motor base */
    
    base_speed.teta = thetaFeedback(base_speed.teta,Odometry.position.teta,theta_destination,&lastThetaRobot, &totalThetaRobot, TRACKING_SAMP/1000);
    baseTrapezoidProfile(&base_speed, &base_prev_speed,2, 2, 1, TRACKING_SAMP/1000);
    base4Omni(base_speed, &a_target_speed, &b_target_speed, &c_target_speed, &d_target_speed);
    
    base_prev_speed.x = base_speed.x;
    base_prev_speed.y = base_speed.y;
    base_prev_speed.teta = base_speed.teta;
}
#endif

/* 
 * prosedur untuk print string dengan uart setiap sampling time
 * 
 * */
void pcSerialSamp() /*1200 us untuk 16 karakter pc.printf*/ /* 20 us dgn attach*/
{
    /* write string ke buffer 1 (str_buffer) */     
    sprintf(str_buffer, "%.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f\n",  a_motor_speed, b_motor_speed, c_motor_speed, d_motor_speed, A_pwm, B_pwm, C_pwm, D_pwm);
    // sprintf(str_buffer, "%.2f\n",  a_target_speed);
    /* mengirimkan string melalui uart */
    sendUart(str_buffer);      
}

/* 
 * prosedur untuk mengirimkan data melalui uart dengan double buffer
 * 
 * */
void sendUart(char *buffer)
{
    /*lock critical section */
    uart_mutex.lock();
    /* copy buffer 1 ke buffer 2 untuk dikirimkan ke hardware uart */
    strcpy(uart_buffer,buffer);
    uart_pointer = uart_buffer;
    uart_buff_len = strlen((const char*) uart_buffer);
    /* mengirimkan karakter setiap hardware uart kosong */
    pc.attach(&writeUart, pc.TxIrq); 
}

/* 
 * prosedur untuk menulis data ke hardware uart
 * 
 * */
void writeUart() /* butuh 4 us */
{
    CriticalSectionLock::enable();
    /* selama masih ada karakter dalam buffer uart */
    if (uart_buff_len >= 0)
    {
        /* kirim karakter yang ditunjuk uart_pointer */
        pc.putc(*uart_pointer);
        /* merujuk ke karakter selanjutnya */
        uart_pointer++;
        /* panjang karakter yang belum dikirimkan berkuran 1 */
        uart_buff_len--;
    }
    /* buffer sudah kosong, seluruh karakter sudah dikirim */
    else 
    {
        /* lepas function writeUart dari callback */
        pc.attach(0, pc.TxIrq);
        /* mengembalikan state awal */
        uart_pointer = uart_buffer;
        uart_mutex.unlock();
    }
    CriticalSectionLock::disable();
}
    

/* state joystick */
void stickState(){
//Procedure to read command from stick and take action

    joysamptime = (profiler.read_us() - last_time_joystick) / 1000000;
    /* RESET STATE */ 
    if (stick.START){        
        sprintf(str_buffer, "start\n");
        sendUart(str_buffer);      
        // pc.printf("start\n");
        count_print = 0;
    } 
    else if (stick.SELECT){
        //pc.printf("select \n");
        while(count_select < 400){
            // pc.printf("%f %f %f %f %f %d\n", speed_array_a[count_select],speed_array_b[count_select],speed_array_c[count_select],speed_array_d[count_select],time_array[count_select], count_select);
            count_select++;
        }
    }

    /* STICK ROTATION STATE */ 
    if ((stick.R1)){     
        base_speed.teta = -2*PI;
        theta_destination = Odometry.position.teta;
    } 
    else if ((stick.L1)){
        base_speed.teta = 2*PI;
        theta_destination = Odometry.position.teta;
    }

    statePrint = 1;
    /* STICK ARROW STATE */
    if ((!stick.atas)&&(!stick.bawah)&&(!stick.kanan)&&(!stick.kiri)
            &&(!stick.R2)&&(!stick.R1)&&(!stick.L1)){
    //no input condition
        base_speed.x = 0;
        base_speed.y = 0;;
        base_speed.teta = 0;
        theta_destination = Odometry.position.teta;
        statePrint = 0;
        //pc.printf("diam\n");
    } 
    else if ((stick.atas)&&(!stick.bawah)&&(!stick.kanan)&&(!stick.kiri)&&(!stick.R2)){
    //stick up
        base_speed.x = 0;
        base_speed.y = 1;
        base_speed.teta = 0;
        //pc.printf("atas\n");
    } 
    else if ((!stick.atas)&&(stick.bawah)&&(!stick.kanan)&&(!stick.kiri)&&(!stick.R2)){
    //stick down
        base_speed.x = 0;
        base_speed.y = -1;
        base_speed.teta = 0;
        //pc.printf("bawah\n");
    } 
    else if ((!stick.atas)&&(!stick.bawah)&&(stick.kanan)&&(!stick.kiri)&&(!stick.R2)){
    //stick right
        base_speed.x = 1;
        base_speed.y = 0;
        base_speed.teta = 0;
        //pc.printf("kiri\n");
    } 
    else if ((!stick.atas)&&(!stick.bawah)&&(!stick.kanan)&&(stick.kiri)&&(!stick.R2)){
    //stick left
        base_speed.x = -1;
        base_speed.y = 0;
        base_speed.teta = 0;
        //pc.printf("kanan\n");
    } 
    else if ((stick.atas)&&(!stick.bawah)&&(stick.kanan)&&(!stick.kiri)&&(!stick.R2)){
    //stick right up
        base_speed.x = 1.5/2;
        base_speed.y = 1.5/2;
        base_speed.teta = 0;
    } 
    else if ((stick.atas)&&(!stick.bawah)&&(!stick.kanan)&&(stick.kiri)&&(!stick.R2)){
    //stick left up
        base_speed.x = -1.5/2;
        base_speed.y = 1.5/2;
        base_speed.teta = 0;
    } 
    else if ((!stick.atas)&&(stick.bawah)&&(stick.kanan)&&(!stick.kiri)&&(!stick.R2)){ 
    //stick right down
        base_speed.x = 1.5/2;
        base_speed.y = -1.5/2;
        base_speed.teta = 0;
    } 
    else if ((!stick.atas)&&(stick.bawah)&&(!stick.kanan)&&(stick.kiri)&&(!stick.R2)){
        //stick left down
        base_speed.x = -1.5/2;
        base_speed.y = -1.5/2;
        base_speed.teta = 0;
    }
    //mode lambat
    else if ((stick.atas)&&(!stick.bawah)&&(!stick.kanan)&&(!stick.kiri)&&(stick.R2)){
    //stick up
        base_speed.x = 0;
        base_speed.y = 0.75/2;
        base_speed.teta = 0;
        //pc.printf("atas\n");
    } 
    
}

// void gerakAuto(){
//     index_curr_pos = 0;
//     index_next_pos = nextIndex(distance[index_curr_pos+1], Odometry.position, index_curr_pos); // baca indeks berikutnya
//     v_resultan = vwGenerator(distance[index_next_pos],Odometry.position, distance[index_curr_pos-1], float accel, float decel, float saturation); //cari kecepatan target,input accel, decel, saturation
//     alpha= computeAlpha(distance[index_next_pos], Odometry.position); // 
//     velocity.x = v_resultan*cos(alpha);
//     velocity.y = v_resultan*sin(alpha);
//     velocity.theta = v_resultan;
// }