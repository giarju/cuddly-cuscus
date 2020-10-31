/***************************************************************************
 * Title      : MAIN PROGRAM ROBOT PR
 * Author     : KRAI ITB 2020
 *
 * 
 ***************************************************************************/

/******************** Aktivasi Debug ************************/

// #define GET_MOTOR
// #define PID_TUNE

// #define MANUAL_DEBUG
// #define AUTOMATIC_DEBUG
// #define ODOMETRY_DEBUG 
// #define SERIAL_DEBUG 
// #define PCSERIAL_DEBUG
#define MOTOR_DEBUG
#define ENCMOTOR_DEBUG
#define PID_MOTOR_DEBUG 
// #define TRACKING_DEBUG
#define FSM_DEBUG
#define JOYSTICK_DEBUG
// #define SIMUL_DEBUG

/********************** Library ******************************/
 
#include "mbed.h"
#include "Configuration/constant.h"
#include "Configuration/variable.h"
#include "Configuration/robotpin.h"
#include "Configuration/map.h"

#ifdef SIMUL_DEBUG
#include "RobotModelKRAI/MotorModel.h"
#include "RobotModelKRAI/EncModel.h"
#include "RobotModelKRAI/RobotModel.h"
#endif

/**************** function declaration ***********************/
/* 
 * prosedur untuk melakukan sampling odometry
 * 
 * */
void odometrySamp();

/* 
 * prosedur untuk melakukan sampling enc motor
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
 * prosedur sampling state machine
 * 
 * */
void stateSamp();

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
 * state untuk ganti map di simulasi
 * 
 * */
void mapState();

/* 
 * prosedur untuk menyimpan variable joystick
 * */
void simpanStick();

/* 
 * prosedur sampling joystick
 * */
void joystickSamp();

/* 
 * prosedur sampling simulasi
 * */
void simul_samp();


/******************* Main Function **************************/
int main ()
{
    /* initial setup */
    profiler.start(); 
    

    /* inisialisasi sampling untuk setiap proses dengan callback*/
    #ifdef ODOMETRY_DEBUG
        /* sampling odometri base */
        Odometry.resetOdom();
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

    #ifdef JOYSTICK_DEBUG
        /* sampling komunikasi serial */
        serial_ticker.attach_us(&joystickSamp, STICK_SAMP);
        stick.setup();
        stick.idle();
        stick.reset();
    #endif 

    #ifdef SERIAL_DEBUG
        /* sampling komunikasi serial */
        stick_ticker.attach_us(&pcSerialSamp, SERIAL_SAMP);
    #endif 

    #ifdef FSM_DEBUG
        fsm_ticker.attach_us(&stateSamp, FSM_SAMP);
    #endif

    #ifdef SIMUL_DEBUG
        /* sampling komunikasi serial */
        simul_ticker.attach_us(&simul_samp, 1000);
        /* create robot simulation instance */
        robot_sim1.createRobot();
    #endif 

    #ifdef PID_TUNING
    #endif


    while (1)
    {  

        /* print data acquisition */
        #ifdef GET_MOTOR
            if (data_i == 600 && stick_kotak){
                for (int q = 0; q < 600; q++){
                    pc.printf("%.2f %.2f %.2f %.2f %d\n", data1[q], data2[q], data3[q], data4[q], data_t[q]);
                }
                break;
            } 
        #endif

            
        // wait(10.0);   
        // prof_start1 = profiler.read_us();
        // prof_end1 = profiler.read_us();
        // diff1 = prof_end1 - prof_start1;
    }
}     

#ifdef ODOMETRY_DEBUG
void odometrySamp ()  /*butuh 48018 us */  
{  
    /* update posisi robot berdasarkan odometri */
    // Odometry.updatePosition();    

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
    #ifdef AUTOMATIC
    mapState();
    findClosestIntersection(map_pointer[map_state], robot_sim1.Odometry_position, map_size[map_state], curr_clst_point, PURSUIT_RADIUS_BASE,  &curr_clst_point, &tp, &is_intersect);
    next_point =  getLinearIntpPoint(map_pointer[map_state], robot_sim1.Odometry_position,is_intersect, tp, PURSUIT_RADIUS_BASE);
    alphass = computeAlpha(next_point.distance,robot_sim1.Odometry_position);
    
    base_speed.x= next_point.vr*cos(alphass);
    base_speed.y = next_point.vr*sin(alphass);
    base_speed.teta = next_point.distance.teta;
    #endif

    #ifdef MANUAL
    base_speed.teta = thetaFeedback(base_speed.teta,Odometry.position.teta,&lastThetaRobot, &totalThetaRobot, TRACKING_SAMP/1000);
    baseTrapezoidProfile(&base_speed, &base_prev_speed,2, 2, 1, TRACKING_SAMP/1000);
    #endif

    #ifdef SIMUL_DEBUG
    mapState();
    map_check = map_size[map_state];
    findClosestIntersection(map_pointer[map_state], robot_sim1.Odometry_position, map_size[map_state], curr_clst_point, PURSUIT_RADIUS_BASE,  &curr_clst_point, &tp, &is_intersect);
    next_point =  getLinearIntpPoint(map_pointer[map_state], robot_sim1.Odometry_position,is_intersect, tp, PURSUIT_RADIUS_BASE);
    alphass = computeAlpha(next_point.distance,robot_sim1.Odometry_position);
    
    base_speed.x= next_point.vr*cos(alphass);
    base_speed.y = next_point.vr*sin(alphass);
    base_speed.teta = next_point.distance.teta;
    #endif

    /* menghitung kecepatan masing2 motor base dengan inverse kinematic*/
    base4Omni(base_speed, &a_target_speed, &b_target_speed, &c_target_speed, &d_target_speed);
    
    base_prev_speed.x = base_speed.x;
    base_prev_speed.y = base_speed.y;
    base_prev_speed.teta = base_speed.teta; 
}
#endif

/* 
 * prosedur untuk melakukan sampling encoder motor base
 * 
 * */
#ifdef ENCMOTOR_DEBUG
void encoderMotorSamp()  /* butuh 8 us */
{
    #ifndef SIMUL_DEBUG
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
    #endif

    /* akuisisi data motor */
    #ifdef GET_MOTOR
    if (data_i < 600 && stick_silang){ 
        data1[data_i] = a_motor_speed;
        data2[data_i] = b_motor_speed;
        data3[data_i] = c_motor_speed;
        data4[data_i] = d_motor_speed;
        data_t[data_i] = profiler.read_us();
        data_i++;
    }
    #endif

    #ifdef SIMUL_DEBUG
    a_motor_speed = robot_sim1.encA.pulsesOmega()*2*PI*WHEEL_RAD/ENC_MOTOR_PULSE/ENC_MOTOR_SAMP*US_TO_S;
    b_motor_speed = robot_sim1.encB.pulsesOmega()*2*PI*WHEEL_RAD/ENC_MOTOR_PULSE/ENC_MOTOR_SAMP*US_TO_S;
    c_motor_speed = robot_sim1.encC.pulsesOmega()*2*PI*WHEEL_RAD/ENC_MOTOR_PULSE/ENC_MOTOR_SAMP*US_TO_S;
    d_motor_speed = robot_sim1.encD.pulsesOmega()*2*PI*WHEEL_RAD/ENC_MOTOR_PULSE/ENC_MOTOR_SAMP*US_TO_S;
    #endif
}
#endif

/* 
 * prosedur untuk menghitung pid motor base
 * 
 * */
#ifdef PID_MOTOR_DEBUG
void pidMotorSamp()
{   
    #ifdef PID_TUNE
    if (data_i < 677){
        a_target_speed = -fwd[data_i]*0.66;
        b_target_speed = fwd[data_i]*0.66;
        c_target_speed = fwd[data_i]*0.66;
        d_target_speed = -fwd[data_i]*0.66 ;
        data_i++;
    }
    #endif

    /* menghitung pid motor base */
    float max_pwm = 24;
    A_pwm = A_pid_motor.createpwm(a_target_speed, a_motor_speed, max_pwm);
    B_pwm = B_pid_motor.createpwm(b_target_speed, b_motor_speed, max_pwm);
    C_pwm = C_pid_motor.createpwm(c_target_speed, c_motor_speed, max_pwm);
    D_pwm = D_pid_motor.createpwm(d_target_speed, d_motor_speed, max_pwm);   
}
#endif

/* 
 * prosedur untuk melakukan sampling pwm motor base
 * 
 * */
#ifdef MOTOR_DEBUG
void motorSamp()
{
    #ifdef GET_MOTOR
    /* input akuisisi data */
    float pwm_test;
    if (data_i < 500 && stick_silang){
        pwm_test = 0.6;
    }
    else{
        pwm_test = 0;
    }
    A_motor.speed(-pwm_test);
    B_motor.speed(pwm_test);
    C_motor.speed(pwm_test); 
    D_motor.speed(-pwm_test);
    #endif

    #if !defined(GET_MOTOR) && !defined(SIMUL_DEBUG)
    /* menggerakan motor base */
    A_motor.speed(A_pwm);
    B_motor.speed(B_pwm);
    C_motor.speed(C_pwm); 
    D_motor.speed(D_pwm);
    #endif

    #ifdef SIMUL_DEBUG
    robot_sim1.motor1.motorSim(A_pwm);
    robot_sim1.motor2.motorSim(B_pwm);
    robot_sim1.motor3.motorSim(C_pwm);
    robot_sim1.motor4.motorSim(D_pwm);
    #endif
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
    // sprintf(str_buffer, "%d %d\n", stick_atas, stick_silang);
    sprintf(str_buffer, "%.2f %.2f %.2f %.2f\n", a_motor_speed, b_motor_speed, c_motor_speed, d_motor_speed);
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
    
#ifdef JOYSTICK_DEBUG
void simpanStick(){
    joy.stick_kanan = stick.kanan; 
    joy.stick_kiri = stick.kiri; 
    joy.stick_atas = stick.atas; 
    joy.stick_bawah = stick.bawah;

    joy.stick_segitiga = stick.segitiga; 
    joy.stick_lingkaran = stick.lingkaran; 
    joy.stick_silang = stick.silang; 
    joy.stick_kotak = stick.kotak;

    joy.stick_R1 = stick.R1; 
    joy.stick_R2 = stick.R2; 
    joy.stick_L1 = stick.L1; 
    joy.stick_L2 = stick.L2;

    joy.stick_select = stick.SELECT; 
    joy.stick_start = stick.START; 

    joy.stick_kanan_click = stick.kanan_click; 
    joy.stick_kiri_click = stick.kiri_click; 
    joy.stick_atas_click = stick.atas_click; 
    joy.stick_bawah_click = stick.bawah_click;

    joy.stick_segitiga_click = stick.segitiga_click; 
    joy.stick_lingkaran_click = stick.lingkaran_click; 
    joy.stick_silang_click = stick.silang_click; 
    joy.stick_kotak_click = stick.kotak_click;

    joy.stick_R1_click = stick.R1_click; 
    joy.stick_R2_click = stick.R2_click; 
    joy.stick_L1_click = stick.L1_click; 
    joy.stick_L2_click = stick.L2_click;
    
    joy.stick_select_click = stick.SELECT_click; 
    joy.stick_start_click= stick.START_click; 
}
void joystickSamp(){
    stick.olah_data();
    simpanStick();
    Statoo.putChangeStick(joy);
    stick.reset();
    stick.idle();
}

/* state joystick */
// void stickState(){
// //Procedure to read command from stick and take action

//     joysamptime = (profiler.read_us() - last_time_joystick) / 1000000;
//     /* RESET STATE */ 
//     if (stick.START){        
//         sprintf(str_buffer, "start\n");
//         sendUart(str_buffer);      
//         // pc.printf("start\n");
//         count_print = 0;
//     } 
//     else if (stick.SELECT){
//         //pc.printf("select \n");
//         while(count_select < 400){
//             // pc.printf("%f %f %f %f %f %d\n", speed_array_a[count_select],speed_array_b[count_select],speed_array_c[count_select],speed_array_d[count_select],time_array[count_select], count_select);
//             count_select++;
//         }
//     }

//     /* STICK ROTATION STATE */ 
//     if ((stick.R1)){     
//         base_speed.teta = -2*PI;
//         theta_destination = Odometry.position.teta;
//     } 
//     else if ((stick.L1)){
//         base_speed.teta = 2*PI;
//         theta_destination = Odometry.position.teta;
//     }

//     statePrint = 1;
//     /* STICK ARROW STATE */
//     if ((!stick.atas)&&(!stick.bawah)&&(!stick.kanan)&&(!stick.kiri)
//             &&(!stick.R2)&&(!stick.R1)&&(!stick.L1)){
//     //no input condition
//         base_speed.x = 0;
//         base_speed.y = 0;;
//         base_speed.teta = 0;
//         theta_destination = Odometry.position.teta;
//         statePrint = 0;
//         //pc.printf("diam\n");
//     } 
//     else if ((stick.atas)&&(!stick.bawah)&&(!stick.kanan)&&(!stick.kiri)&&(!stick.R2)){
//     //stick up
//         base_speed.x = 0;
//         base_speed.y = 1;
//         base_speed.teta = 0;
//         //pc.printf("atas\n");
//     } 
//     else if ((!stick.atas)&&(stick.bawah)&&(!stick.kanan)&&(!stick.kiri)&&(!stick.R2)){
//     //stick down
//         base_speed.x = 0;
//         base_speed.y = -1;
//         base_speed.teta = 0;
//         //pc.printf("bawah\n");
//     } 
//     else if ((!stick.atas)&&(!stick.bawah)&&(stick.kanan)&&(!stick.kiri)&&(!stick.R2)){
//     //stick right
//         base_speed.x = 1;
//         base_speed.y = 0;
//         base_speed.teta = 0;
//         //pc.printf("kiri\n");
//     } 
//     else if ((!stick.atas)&&(!stick.bawah)&&(!stick.kanan)&&(stick.kiri)&&(!stick.R2)){
//     //stick left
//         base_speed.x = -1;
//         base_speed.y = 0;
//         base_speed.teta = 0;
//         //pc.printf("kanan\n");
//     } 
//     else if ((stick.atas)&&(!stick.bawah)&&(stick.kanan)&&(!stick.kiri)&&(!stick.R2)){
//     //stick right up
//         base_speed.x = 1.5/2;
//         base_speed.y = 1.5/2;
//         base_speed.teta = 0;
//     } 
//     else if ((stick.atas)&&(!stick.bawah)&&(!stick.kanan)&&(stick.kiri)&&(!stick.R2)){
//     //stick left up
//         base_speed.x = -1.5/2;
//         base_speed.y = 1.5/2;
//         base_speed.teta = 0;
//     } 
//     else if ((!stick.atas)&&(stick.bawah)&&(stick.kanan)&&(!stick.kiri)&&(!stick.R2)){ 
//     //stick right down
//         base_speed.x = 1.5/2;
//         base_speed.y = -1.5/2;
//         base_speed.teta = 0;
//     } 
//     else if ((!stick.atas)&&(stick.bawah)&&(!stick.kanan)&&(stick.kiri)&&(!stick.R2)){
//         //stick left down
//         base_speed.x = -1.5/2;
//         base_speed.y = -1.5/2;
//         base_speed.teta = 0;
//     }
//     //mode lambat
//     else if ((stick.atas)&&(!stick.bawah)&&(!stick.kanan)&&(!stick.kiri)&&(stick.R2)){
//     //stick up
//         base_speed.x = 0;
//         base_speed.y = 0.75/2;
//         base_speed.teta = 0;
//         //pc.printf("atas\n");
//     } 
     
// }
#endif

#ifdef FSM_DEBUG
void stateSamp()
{
    Statoo.fsmAuto(near_last, posisi_tangan);
}
#endif 

/* state untuk simulasi */
void mapState()
{
    /* next map */
    if (!mybutton && profiler.read_us() - button_debounce > 500000)
    {
        map_state++;
        curr_clst_point = 0;
        button_debounce = profiler.read_us();
    }
    /* prev map*/
    // else if(){}
}

#ifdef SIMUL_DEBUG
void simul_samp()
{
    robot_sim1.omni4WheelModel();
    simv = robot_sim1.y_robot;
    simv2 = robot_sim1.x_robot;
    simv3 = robot_sim1.h_robot;
}
#endif
