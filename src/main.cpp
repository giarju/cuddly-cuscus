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

#define ODOMETRY_DEBUG 
#define SERIAL_DEBUG 
#define MOTOR_DEBUG
#define ENCMOTOR_DEBUG
#define PID_MOTOR_DEBUG 
#define JOYSTICK_DEBUG

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
 * prosedur untuk kontrol joystick
 * 
 * */
void stickState();


/******************* Main Function **************************/
int main ()
{
    /* initial setup */
    Odometry.resetOdom();   
    profiler.start(); 
    startMillis();
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

    while (1)
    {   
        wait(10.0);   
            // prof_start1 = profiler.read_us();
            // prof_end1 = profiler.read_us();
            // diff1 = prof_end1 - prof_start1;
            // pc.printf("%lu\n", diff1);       
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
    a_motor_speed = (float)A_enc.getPulses()*2*PI/ENC_MOTOR_PULSE/ENC_MOTOR_SAMP*US_TO_S;
    b_motor_speed = (float)B_enc.getPulses()*2*PI/ENC_MOTOR_PULSE/ENC_MOTOR_SAMP*US_TO_S;
    c_motor_speed = (float)C_enc.getPulses()*2*PI/ENC_MOTOR_PULSE/ENC_MOTOR_SAMP*US_TO_S;
    d_motor_speed = (float)D_enc.getPulses()*2*PI/ENC_MOTOR_PULSE/ENC_MOTOR_SAMP*US_TO_S;
    
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
    if (base_speed.x == 0 && base_speed.y == 0 && base_speed.teta == 0)
    {
        A_motor.forcebrake();
        B_motor.forcebrake();
        C_motor.forcebrake(); 
        D_motor.forcebrake();
    }
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
    /* menghitung pid motor base */
    A_pwm = A_pid_motor.createpwm(a_target_speed, a_motor_speed);
    B_pwm = B_pid_motor.createpwm(b_target_speed, b_motor_speed);
    C_pwm = C_pid_motor.createpwm(c_target_speed, c_motor_speed);
    D_pwm = D_pid_motor.createpwm(d_target_speed, d_motor_speed);
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
    /* write string ke buffer 1 (str_buffer)*/     
    sprintf(str_buffer, "bbbbbbbbbbbbbbbbbb\n");
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

/* state joystick */
void stickState(){
//Procedure to read command from stick and take action
    /* RESET STATE */ 
    if (stick.START){        
        pc.printf("start \n");
    } 
    else if (stick.SELECT){
        //pc.printf("select \n");
    }

    /* STICK ROTATION STATE */ 
    if ((stick.R1)){     
        base_speed.teta = PI/2;
    } 
    else if ((stick.L1)){
        base_speed.teta = -PI/2;
    }

    
    /* STICK ARROW STATE */
    if ((!stick.atas)&&(!stick.bawah)&&(!stick.kanan)&&(!stick.kiri)
            &&(!stick.R2)&&(!stick.R1)&&(!stick.L1)){
    //no input condition
        base_speed.x = 0;
        base_speed.y = 0;
        base_speed.teta = 0;
        
        //pc.printf("diam\n");
    } 
    else if ((stick.atas)&&(!stick.bawah)&&(!stick.kanan)&&(!stick.kiri)&&(!stick.R2)){
    //stick up
        base_speed.x = 0;
        base_speed.y = 2;
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
}