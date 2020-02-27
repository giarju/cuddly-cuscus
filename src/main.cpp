/***************************************************************************
 * Title      : MAIN PROGRAM ROBOT PR
 * Author     : KRAI ITB 2020
 *
 * 
 ***************************************************************************/

//to do : make critical section on function
//to do : 

/********************** Library ******************************/

#include "mbed.h"
#include "Configuration/constant.h"
#include "Configuration/variable.h"
#include "Configuration/robotpin.h"

/******************** Aktivasi Debug ************************/

// #define ODOMETRY_DEBUG 
// #define SERIAL_DEBUG 
#define MOTOR_DEBUG
#define ENCMOTOR_DEBUG
#define PID_MOTOR_DEBUG 
// #define JOYSTICK_DEBUG
#define TRACKING_DEBUG



int counttt = 0;
float joysamptime;
float testy, testprevy = 0.2;
/**************** function declaration ***********************/


/* 
 * prosedur untuk melakukan sampling odometri base
 * 
 * */
void odometrySamp();

/* 
 * prosedur untuk melakukan sampling encoder motor base
 * 
 * */
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
 * prosedur untuk sampling joystick
 * 
 * */
void stickSamp();

/* 
 * prosedur untuk kontrol joystick
 * 
 * */
float trapeziumProfile(float amax, float TS, float prev_speed, uint32_t time);

float trapeziumTarget(float amax, float vmax, float prev_speed, float TS);



/******************* Main Function **************************/
int main ()
{
    /* initial setup */
    // Odometry.resetOdom();   
    profiler.start(); 
    stick.setup();
    stick.idle();


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

    #ifdef JOYSTICK_DEBUG
        /* sampling komunikasi serial */
        stick_ticker.attach_us(&stickSamp, STICK_SAMP);
    #endif 

    while (1)
    {   
        if(profiler.read_us() - last_time_joystick > STICK_SAMP ){
            if (stick.readable()){
                stick.baca_data();
                stick.olah_data();
                // sprintf(str_buffer, "r\n");
            }
            stickState();
            last_time_joystick = profiler.read_us();
        }
            
        // wait(10.0);   
        // prof_start1 = profiler.read_us();
        // prof_end1 = profiler.read_us();
        // diff1 = prof_end1 - prof_start1;
        // pc.printf("");       
    } 
}


/* 
 * prosedur untuk melakukan sampling odometri base
 * 
 * */
#ifdef ODOMETRY_DEBUG
void odometrySamp ()  /*butuh 48018 us */  
{  
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
    right_arm_speed = (float)right_arm_enc.getPulses()*360*180/(124.46*ENC_MOTOR_PULSE);
    //left_arm_speed = (float)left_arm_enc.getPulses()*2*PI/ENC_MOTOR_PULSE/ENC_MOTOR_SAMP*US_TO_S;
    
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
    /* menggerakan motor base */
    // base_speed.x = 1;
    // base_speed.y = 1;
    // base_speed.teta = 1;

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
}
#endif

/* 
 * prosedur untuk menghitung pid motor base
 * 
 * */
#ifdef PID_MOTOR_DEBUG
void pidMotorSamp()
{

    // a_target_speed = trapeziumProfile(-4, 0.004713, a_target_speed, profiler.read_us());
    // b_target_speed = trapeziumProfile(4, 0.004713, b_target_speed, profiler.read_us());
    // c_target_speed = trapeziumProfile(4, 0.004713, c_target_speed, profiler.read_us());
    // d_target_speed = trapeziumProfile(-4, 0.004713, d_target_speed, profiler.read_us());

    /* menghitung pid motor base */
    A_pwm = A_pid_motor.createpwm(a_target_speed, a_motor_speed, 1);
    B_pwm = B_pid_motor.createpwm(b_target_speed, b_motor_speed, 1);
    C_pwm = C_pid_motor.createpwm(c_target_speed, c_motor_speed, 1);
    D_pwm = D_pid_motor.createpwm(d_target_speed, d_motor_speed, 1);
    
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
    base4Omni(base_speed, &a_target_speed, &b_target_speed, &c_target_speed, &d_target_speed);
}
#endif

#ifdef SERIAL_DEBUG
/* 
 * prosedur untuk print string dengan uart setiap sampling time
 * 
 * */
void pcSerialSamp() /*1200 us untuk 16 karakter pc.printf*/ /* 20 us dgn attach*/
{
    /* write string ke buffer 1 (str_buffer) */     
    sprintf(str_buffer, "%.2f %.2f %.2f %d\n",  a_target_speed, joysamptime, a_motor_speed, profiler.read_us());
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
#endif
    
float trapeziumProfile(float amax, float TS, float prev_speed, uint32_t time){
    if (time < 500000){
        return prev_speed + amax*TS;
    }
    else if ((500000 <= time) && (time < 4000000)){
        return prev_speed;
    }
    else if ((4000000 <= time) && (time < 4500000)){
        return prev_speed - amax*TS;
    }
    else {
        return 0;
    }
}

float trapeziumTarget(float amax, float vmax, float prev_speed, float TS){
    if(prev_speed < vmax && amax > 0){
        return prev_speed + amax*TS;
    }
    else if(prev_speed > vmax && amax < 0){
        return prev_speed + amax*TS;
    }
    else {
        return vmax;
    }
}

#ifdef JOYSTICK_DEBUG
void stickSamp(){
    // stick.baca_data();
    // stick.olah_data();
    // stickState();
}
#endif

/* state joystick */
void stickState(){
//Procedure to read command from stick and take action

    joysamptime = (profiler.read_us() - last_time_joystick) / 1000000;
    /* RESET STATE */ 
    if (stick.START){        
        // pc.printf("start \n");
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
        base_speed.teta = PI/2;
    } 
    else if ((stick.L1)){
        base_speed.teta = -PI/2;
    }

    statePrint = 1;
    /* STICK ARROW STATE */
    if ((!stick.atas)&&(!stick.bawah)&&(!stick.kanan)&&(!stick.kiri)
            &&(!stick.R2)&&(!stick.R1)&&(!stick.L1)){
    //no input condition
        base_speed.x = 0;
        base_speed.y = trapeziumTarget(-0.2, 0, base_prev_speed.y, joysamptime);
        base_speed.teta = 0;
        statePrint = 0;
        //pc.printf("diam\n");
    } 
    else if ((stick.atas)&&(!stick.bawah)&&(!stick.kanan)&&(!stick.kiri)&&(!stick.R2)){
    //stick up
        base_speed.x = 0;
        base_speed.y = trapeziumTarget(0.2, 2, base_prev_speed.y, joysamptime);
        base_speed.teta = 0;
        //pc.printf("atas\n");
    } 
    else if ((!stick.atas)&&(stick.bawah)&&(!stick.kanan)&&(!stick.kiri)&&(!stick.R2)){
    //stick down
        base_speed.x = 0;
        base_speed.y = -2;
        base_speed.teta = 0;
        //pc.printf("bawah\n");
    } 
    else if ((!stick.atas)&&(!stick.bawah)&&(stick.kanan)&&(!stick.kiri)&&(!stick.R2)){
    //stick right
        base_speed.x = 2;
        base_speed.y = 0;
        base_speed.teta = 0;
        //pc.printf("kiri\n");
    } 
    else if ((!stick.atas)&&(!stick.bawah)&&(!stick.kanan)&&(stick.kiri)&&(!stick.R2)){
    //stick left
        base_speed.x = -2;
        base_speed.y = 0;
        base_speed.teta = 0;
        //pc.printf("kanan\n");
    } 
    else if ((stick.atas)&&(!stick.bawah)&&(stick.kanan)&&(!stick.kiri)&&(!stick.R2)){
    //stick right up
        base_speed.x = 1.5;
        base_speed.y = 1.5;
        base_speed.teta = 0;
    } 
    else if ((stick.atas)&&(!stick.bawah)&&(!stick.kanan)&&(stick.kiri)&&(!stick.R2)){
    //stick left up
        base_speed.x = -1.5;
        base_speed.y = 1.5;
        base_speed.teta = 0;
    } 
    else if ((!stick.atas)&&(stick.bawah)&&(stick.kanan)&&(!stick.kiri)&&(!stick.R2)){ 
    //stick right down
        base_speed.x = 1.5;
        base_speed.y = -1.5;
        base_speed.teta = 0;
    } 
    else if ((!stick.atas)&&(stick.bawah)&&(!stick.kanan)&&(stick.kiri)&&(!stick.R2)){
        //stick left down
        base_speed.x = -1.5;
        base_speed.y = -1.5;
        base_speed.teta = 0;
    }
    //mode lambat
    else if ((stick.atas)&&(!stick.bawah)&&(!stick.kanan)&&(!stick.kiri)&&(stick.R2)){
    //stick up
        base_speed.x = 0;
        base_speed.y = 0.75;
        base_speed.teta = 0;
        //pc.printf("atas\n");
    } 
    else if ((!stick.atas)&&(stick.bawah)&&(!stick.kanan)&&(!stick.kiri)&&(stick.R2)){
    //stick down
        base_speed.x = 0;
        base_speed.y = -0.75;
        base_speed.teta = 0;
        //pc.printf("bawah\n");
    } 
    else if ((!stick.atas)&&(!stick.bawah)&&(stick.kanan)&&(!stick.kiri)&&(stick.R2)){
    //stick right
        base_speed.x = 0.75;
        base_speed.y = 0;
        base_speed.teta = 0;
        //pc.printf("kiri\n");
    } 
    else if ((!stick.atas)&&(!stick.bawah)&&(!stick.kanan)&&(stick.kiri)&&(stick.R2)){
    //stick left
        base_speed.x = -0.75;
        base_speed.y = 0;
        base_speed.teta = 0;
        //pc.printf("kanan\n");
    } 
    else if ((stick.atas)&&(!stick.bawah)&&(stick.kanan)&&(!stick.kiri)&&(stick.R2)){
    //stick right up
        base_speed.x = 1.5*3/5;
        base_speed.y = 1.5*3/5;
        base_speed.teta = 0;
    } 
    else if ((stick.atas)&&(!stick.bawah)&&(!stick.kanan)&&(stick.kiri)&&(stick.R2)){
    //stick left up
        base_speed.x = -1.5*3/5;
        base_speed.y = 1.5*3/5;
        base_speed.teta = 0;
    } 
    else if ((!stick.atas)&&(stick.bawah)&&(stick.kanan)&&(!stick.kiri)&&(stick.R2)){ 
    //stick right down
        base_speed.x = 1.5*3/5;
        base_speed.y = -1.5*3/5;
        base_speed.teta = 0;
    } 
    else if ((!stick.atas)&&(stick.bawah)&&(!stick.kanan)&&(stick.kiri)&&(stick.R2)){
        //stick left down
        base_speed.x = -1.5*3/5;
        base_speed.y = -1.5*3/5;
        base_speed.teta = 0;
    }
    
    
    //untuk pneumatik
    if(!stick.silang && !stick.lingkaran && !stick.kotak && !stick.segitiga){
        tembak = 1;
    }
    else if(!stick.silang && stick.lingkaran && !stick.kotak && !stick.segitiga){
        if(millis()-lastTimeTangan>500){
            if(state_kiri){
                armKiri=!armKiri;
            }
            else{
                armKanan=!armKanan;
            }
            lastTimeTangan=millis();
        }
    }
    else if(!stick.silang && !stick.lingkaran && !stick.kotak && stick.segitiga){
        tembak = 0;
        count_select = 0;
    }

    if(stick.R1){
        if(count_print < 400 && statePrint == 1 && millis() - time_s > 3){
            speed_array_a[count_print] = a_motor_speed;
            speed_array_b[count_print] = b_motor_speed;
            speed_array_c[count_print] = c_motor_speed;
            speed_array_d[count_print] = d_motor_speed;
            time_array[count_print] = millis();
            count_print++;
            time_s = millis();
        }
    }
    //Ganti lapangan
    if(stick.L3){
        state_kiri=1;
        armKiri=0;
        armKanan=1;
    }
    if(stick.R3){
        state_kiri=0;
        armKiri=1;
        armKanan=0;
    }

    
    base_prev_speed.x = base_speed.x;
    base_prev_speed.y = base_speed.y;
    base_prev_speed.teta = base_speed.teta;
}