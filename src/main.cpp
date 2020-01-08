#include "mbed.h"
#include "string.h"
#include "platform/mbed_critical.h"
#include "Configuration/pinList.h"
#include "Configuration/constant.h"
#include "odometriKRAI/odometriKRAI.h"
#include "encoderKRAI/encoderKRAI.h"
#include "Motor/Motor.h"
#include "Path/Path.h"
#include "Tracking/Tracking.h"
#include "InverseKinematics/InverseKinematics.h"
#include "PID/PID.h"

/* to do 
 * 1. make pid for motor 
 * 2. make path and load map
 * 3. make mock up fsm
 * 4. make comment
 * 5. tidy up code
 */

// #define ODOMETRY_DEBUG 
#define SERIAL_DEBUG 
#define MOTOR_DEBUG
#define ENCMOTOR_DEBUG


RawSerial pc(USBTX, USBRX, 115200);
DigitalIn mybutton(USER_BUTTON);

odometriKRAI Odometry(TIM2, TIM3, CMPS_SDA, CMPS_SCL);

encoderKRAI A_enc(PIN_A_CHA, PIN_A_CHB, 538, encoderKRAI::X4_ENCODING);
encoderKRAI B_enc(PIN_B_CHA, PIN_B_CHB, 538, encoderKRAI::X4_ENCODING);
encoderKRAI C_enc(PIN_C_CHA, PIN_C_CHB, 538, encoderKRAI::X4_ENCODING);
encoderKRAI D_enc(PIN_D_CHA, PIN_D_CHB, 538, encoderKRAI::X4_ENCODING);

PID(right_kp, right_ki, ri)

Motor A_motor(PIN_PWM_A, PIN_FWD_A,PIN_REV_A); 
Motor B_motor(PIN_PWM_B, PIN_FWD_B,PIN_REV_B); 
Motor C_motor(PIN_PWM_C, PIN_FWD_C,PIN_REV_C); 
Motor D_motor(PIN_PWM_D, PIN_FWD_D,PIN_REV_D); 

volatile float a_motor_speed,b_motor_speed,c_motor_speed,d_motor_speed;
volatile float A_pwm,B_pwm,C_pwm,D_pwm;

uint32_t prof_start1, prof_end1, diff1;

volatile uint8_t sending;
char uart_buffer[64];
char str_buffer[64];
volatile char* uart_pointer;
volatile int8_t uart_buff_len;

volatile int index;
Trajectory map[100];
volatile Coordinate base_speed;

/* dalam microsecond */
#define ODOMETRY_SAMP 60173
#define ENC_MOTOR_SAMP 7173
#define SERIAL_SAMP 500173
#define MOTOR_SAMP 9173
#define TRACKING_SAMP 3173
    
/* timer untuk mendapatkan waktu */
Timer profiler;
Ticker odometry_ticker;
Ticker encoder_motor_ticker;
Ticker serial_ticker;
Ticker motor_ticker;
Ticker tracking_ticker;

/* mutex untuk critical section*/
PlatformMutex uart_mutex; 


/* function declaration */
void odometrySamp();
void encoderMotorSamp();
void pcSerialSamp();
void motorSamp();
void sendUart(char *buffer);
void writeUart();


int main ()
{
    /* initial setup */
    Odometry.resetOdom();   
    profiler.start(); 

    /* sampling untuk setiap proses */
    #ifdef ODOMETRY_DEBUG
        odometry_ticker.attach_us(&odometrySamp, ODOMETRY_SAMP);
    #endif

    #ifdef ENCMOTOR_DEBUG
        encoder_motor_ticker.attach_us(&encoderMotorSamp, ENC_MOTOR_SAMP);
    #endif 

    #ifdef MOTOR_DEBUG
        motor_ticker.attach_us(&motorSamp, MOTOR_SAMP);
    #endif

    void pidMotorSamp();
    void trackingSamp();
    Ticker pid_motor_ticker;

    #define PID_MOTOR_SAMP 5173
    #define PID_MOTOR_DEBUG 
    #ifdef PID_MOTOR_DEBUG
        pid_motor_ticker.attach_us(&pidMotorSamp, PID_MOTOR_SAMP);
    #endif 

    #ifdef TRACKING_DEBUG
        tracking_ticker.attach_us(&trackingSamp, TRACKING_SAMP);
    #endif  

    #ifdef SERIAL_DEBUG
        serial_ticker.attach_us(&pcSerialSamp, SERIAL_SAMP);
    #endif 

    while (1)
    {   
        wait(10.0);          
    } 
}

void odometrySamp ()  /*butuh 48018 us */  
{  
    Odometry.updatePosition();                                     
}

void encoderMotorSamp()  /* butuh 8 us */
{
    a_motor_speed = (float)A_enc.getPulses()*2*PI/ENC_MOTOR_PULSE/ENC_MOTOR_SAMP*US_TO_S;
    b_motor_speed = (float)B_enc.getPulses()*2*PI/ENC_MOTOR_PULSE/ENC_MOTOR_SAMP*US_TO_S;
    c_motor_speed = (float)C_enc.getPulses()*2*PI/ENC_MOTOR_PULSE/ENC_MOTOR_SAMP*US_TO_S;
    d_motor_speed = (float)D_enc.getPulses()*2*PI/ENC_MOTOR_PULSE/ENC_MOTOR_SAMP*US_TO_S;
    
    A_enc.reset();
    B_enc.reset();
    C_enc.reset();
    D_enc.reset();
}

void motorSamp()
{
    A_motor.speed(A_pwm);
    B_motor.speed(B_pwm);
    C_motor.speed(C_pwm); 
    D_motor.speed(D_pwm);
}

void pidMotorSamp()
{

}

void trackingSamp()
{
    base_speed = velocityTracker(map[index], Odometry.position); /* index harusnya dari fsm (index bahaya, shared variable sama fsm)*/
    base4Omni(base_speed, &a_motor_speed, &b_motor_speed, &c_motor_speed, &d_motor_speed);
}

void pcSerialSamp() /*1200 us untuk 16 karakter pc.printf*/ /* 20 us dgn attach*/
{
    /* debug odometry */
    //pc.printf("x : %.2f   y: %.2f   teta :%.2f", Odometry.position.x,Odometry.position.y,Odometry.position.teta);

    /* debug profiler */
    
    
    sprintf(str_buffer, "bbbbbbbbbbbbbbbbbb\n");
    sendUart(str_buffer);
      
}

void sendUart(char *buffer)
{
    uart_mutex.lock();
    strcpy(uart_buffer,buffer);
    uart_pointer = uart_buffer;
    uart_buff_len = strlen((const char*) uart_buffer);
    pc.attach(&writeUart, pc.TxIrq); 
}

void writeUart() /* butuh 4 us */
{
    CriticalSectionLock::enable();
    if (uart_buff_len >= 0)
    {
        pc.putc(*uart_pointer);
        uart_pointer++;
        uart_buff_len--;
    }
    else 
    {
        pc.attach(0, pc.TxIrq);
        uart_pointer = uart_buffer;
        uart_mutex.unlock();
    }
    CriticalSectionLock::disable();
    
    // prof_start1 = profiler.read_us();
    // prof_end1 = profiler.read_us();
    // diff1 = prof_end1 - prof_start1;
    // pc.printf("%lu\n", diff1);
}