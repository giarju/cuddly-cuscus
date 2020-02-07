#include "mbed.h"
#include "odometriKRAI.h"
#include "encoderKRAI.h"
#include "Motor.h"
#include "control.h"
#include "PID.h"
#include "millis/millis.h"

Serial pc(USBTX, USBRX,115200);
CMPS12_KRAI kompas(PC_9,PA_8,0xC0);
joysticknucleo stick(PA_0, PA_1);
//Compass kompass(PC_9, PA_8, 0xC0);


DigitalOut armKiri(PA_9);
DigitalOut armKanan(PC_7);
DigitalOut tembak(PH_1);



PID PI_motorA(0.15, 0, 0 , 0, 0.004 , 0, PID::PI_MODE);
PID PI_motorB(0.15, 0, 0 , 0, 0.004 , 0, PID::PI_MODE);
PID PI_motorC(0.15, 0, 0 , 0, 0.004 , 0, PID::PI_MODE);
PID PI_motorD(0.15, 0, 0 , 0, 0.004 , 0, PID::PI_MODE);
//odometriKRAI odom(TIM2, TIM3, PC_9,PA_8);

encoderKRAI encA      (PC_12, PC_11, 538, encoderKRAI::X4_ENCODING);
encoderKRAI encB      (PC_2, PC_3, 538, encoderKRAI::X4_ENCODING);
encoderKRAI encC      (PC_15, PC_14, 538, encoderKRAI::X4_ENCODING);
encoderKRAI encD      (PC_10, PC_13, 538, encoderKRAI::X4_ENCODING);
encoderKRAI encRight  (PB_7, PB_6, 538, encoderKRAI::X4_ENCODING);
//encoderKRAI encLeft   (PB_2, PD_2, 538, encoderKRAI::X4_ENCODING);

Motor motorA          (PA_7, PA_5, PA_6);
Motor motorB          (PB_0, PC_0, PC_1);
Motor motorC          (PB_1, PB_14, PB_15);
Motor motorD          (PB_10, PB_3,PA_10);
//Motor armLeft         (PC_8,PC_6,PC_5);
//Motor armRight        (PA_15,PA_14,PA_13);

float vx,vy,om, pwmA, pwmB, pwmC, pwmD; //target yang harus dipenuhi oleh robot
//const float  PI = 3.141592;
const float  RAD_TO_DEG = 57.295779;
const float  WHEEL_RAD = 0.075;
uint32_t lastTimerDe, lastTimerStick,lastTimeTangan,lastSpeedArm,lastSpeedPID;
float v_feedback_A, v_feedback_B, v_feedback_C, v_feedback_D;
bool kiri;

float theta_left,theta_pulse_left, prev_theta_left;
float err = 0,
      setpoint = 0,
      sum_error = 0,
      diff_error = 0,
      prev_error = 30
      ,pwm;
double kp = 1;
double ki = 0.0005;
double kd = 0;


float theta_right,theta_pulse_right, prev_theta_right;
//float err = 0,
//      setpoint = 0,
//      sum_error = 0,
//      diff_error = 0,
//      prev_error = 30
//      ,pwm;
//double kp = 2.0;
//double ki = 0.0001;
//double kd = 50;


//Fungsi Penting
void findSpeed();
void stickState();
//void hitungThetaLeft();
//void hitungThetaRight();
//void pidLeft(float setpoint);
//void pidRight(float setpoint);


int main ()
{
    //odom.resetOdom();
    
    
    float vA,vB,vC,vD; //target yang harus dipenuhi tiap motor
    uint32_t lastMotorTimer = 0;
    uint32_t lastSpeedTimer = 0;
    
    startMillis();
    stick.setup();
    stick.idle();
    armKiri=1;
    //kompas.compass_reset((float)kompass.getAngle()/10);
    while(1){
        //hitungThetaRight();

//        pc.printf("a=%d,b=%d,c=%d,d=%d,theta=%f %\n",encA.getPulses(),encB.getPulses(),encC.getPulses(),encD.getPulses());
//        pc.printf("%d\n",millis());
        if(stick.readable()){
            stick.baca_data();
            stick.olah_data();
        }
        if (millis() - lastTimerStick > 19){
            stickState();
            lastTimerStick = millis();
        }
        

        if (millis()-lastMotorTimer >= 4){
            findSpeed();
            pwmA = PI_motorA.createpwm(vA, v_feedback_A);
            pwmB = PI_motorB.createpwm(vB, v_feedback_B);
            pwmC = PI_motorC.createpwm(vC, v_feedback_C);
            pwmD = PI_motorD.createpwm(vD, v_feedback_D);
            
            moveMotor(&vA,&vB,&vC,&vD,vx,vy,om);
            
            if(vx==0 && vy==0 && om == 0){
                motorA.forcebrake();
                motorB.forcebrake();
                motorC.forcebrake();
                motorD.forcebrake(); 
            }
            else{
                motorA.speed(pwmA);
                motorB.speed(pwmB);
                motorC.speed(pwmC);
                motorD.speed(pwmD);  
            }
            lastMotorTimer = millis();     
        }
        
        if (millis() - lastTimerDe >= 11){
            //pwmA = PI_motorA.createpwm(vA, v_feedback_A);
//            pwmB = PI_motorB.createpwm(vB, v_feedback_B);
//            pwmC = PI_motorC.createpwm(vC, v_feedback_C);
//            pwmD = PI_motorD.createpwm(vD, v_feedback_D);
//            findSpeed();
            lastTimerDe = millis(); 
        }
        
        
//        if(millis() - lastSpeedTimer >101){
//            pc.printf("%f,%f,%f,%f\n",v_feedback_A,v_feedback_B,v_feedback_C,v_feedback_D);
//            //pc.printf("theta: %f", theta_right);
//            lastSpeedTimer=millis();
//        }
        
//        if(millis() - lastSpeedPID >7){
//            pidRight(0);
//            lastSpeedPID = millis();
//        }
//        if(millis() - lastSpeedArm >5){
//            armRight.speed(pwm);
//            lastSpeedArm = millis();
//        }
    }
}

//void pidLeft(float setpoint){
//    err = (setpoint - theta_left)/360;
//    
//    sum_error += err;
// 
//    if(theta_left >= 90){
//        ki = 0;
//        }
//    diff_error = err - prev_error;
//    
//    pwm = kp*err + ki*sum_error + kd*diff_error;
// 
//    if (pwm >= 0.5) {
//        pwm = 0.5;
//    }
//    else if (pwm <= -0.5){
//        pwm =-0.5;
//    }
//    prev_error = err;
//}
//
//
//void pidRight(float setpoint){
//    err = (setpoint - theta_right)/360;
//    
//    sum_error += err;
// 
//    if(theta_right >= 90){
//        ki = 0;
//        }
//    diff_error = err - prev_error;
//    
//    pwm = kp*err + ki*sum_error + kd*diff_error;
// 
//    if (pwm >= 0.8) {
//        pwm = 0.8;
//    }
//    else if (pwm <= -0.8){
//        pwm =-0.8;
//    }
//    prev_error = err;
//}

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
        om = pi/2;
    } 
    else if ((stick.L1)){
        om = -pi/2;
    }

    
    /* STICK ARROW STATE */
    if ((!stick.atas)&&(!stick.bawah)&&(!stick.kanan)&&(!stick.kiri)
            &&(!stick.R2)&&(!stick.R1)&&(!stick.L1)){
    //no input condition
        vx = 0;
        vy = 0;
        om = 0;
        
        //pc.printf("diam\n");
    } 
    else if ((stick.atas)&&(!stick.bawah)&&(!stick.kanan)&&(!stick.kiri)&&(!stick.R2)){
    //stick up
        vx = 0;
        vy = 2;
        om = 0;
        //pc.printf("atas\n");
    } 
    else if ((!stick.atas)&&(stick.bawah)&&(!stick.kanan)&&(!stick.kiri)&&(!stick.R2)){
    //stick down
        vx = 0;
        vy = -2;
        om = 0;
        //pc.printf("bawah\n");
    } 
    else if ((!stick.atas)&&(!stick.bawah)&&(stick.kanan)&&(!stick.kiri)&&(!stick.R2)){
    //stick right
        vx = 2;
        vy = 0;
        om = 0;
        //pc.printf("kiri\n");
    } 
    else if ((!stick.atas)&&(!stick.bawah)&&(!stick.kanan)&&(stick.kiri)&&(!stick.R2)){
    //stick left
        vx = -2;
        vy = 0;
        om = 0;
        //pc.printf("kanan\n");
    } 
    else if ((stick.atas)&&(!stick.bawah)&&(stick.kanan)&&(!stick.kiri)&&(!stick.R2)){
    //stick right up
        vx = 1.5;
        vy = 1.5;
        om = 0;
    } 
    else if ((stick.atas)&&(!stick.bawah)&&(!stick.kanan)&&(stick.kiri)&&(!stick.R2)){
    //stick left up
        vx = -1.5;
        vy = 1.5;
        om = 0;
    } 
    else if ((!stick.atas)&&(stick.bawah)&&(stick.kanan)&&(!stick.kiri)&&(!stick.R2)){ 
    //stick right down
        vx = 1.5;
        vy = -1.5;
        om = 0;
    } 
    else if ((!stick.atas)&&(stick.bawah)&&(!stick.kanan)&&(stick.kiri)&&(!stick.R2)){
        //stick left down
        vx = -1.5;
        vy = -1.5;
        om = 0;
    }
    //mode lambat
    else if ((stick.atas)&&(!stick.bawah)&&(!stick.kanan)&&(!stick.kiri)&&(stick.R2)){
    //stick up
        vx = 0;
        vy = 0.75;
        om = 0;
        //pc.printf("atas\n");
    } 
    else if ((!stick.atas)&&(stick.bawah)&&(!stick.kanan)&&(!stick.kiri)&&(stick.R2)){
    //stick down
        vx = 0;
        vy = -0.75;
        om = 0;
        //pc.printf("bawah\n");
    } 
    else if ((!stick.atas)&&(!stick.bawah)&&(stick.kanan)&&(!stick.kiri)&&(stick.R2)){
    //stick right
        vx = 0.75;
        vy = 0;
        om = 0;
        //pc.printf("kiri\n");
    } 
    else if ((!stick.atas)&&(!stick.bawah)&&(!stick.kanan)&&(stick.kiri)&&(stick.R2)){
    //stick left
        vx = -0.75;
        vy = 0;
        om = 0;
        //pc.printf("kanan\n");
    } 
    else if ((stick.atas)&&(!stick.bawah)&&(stick.kanan)&&(!stick.kiri)&&(stick.R2)){
    //stick right up
        vx = 1.5*3/5;
        vy = 1.5*3/5;
        om = 0;
    } 
    else if ((stick.atas)&&(!stick.bawah)&&(!stick.kanan)&&(stick.kiri)&&(stick.R2)){
    //stick left up
        vx = -1.5*3/5;
        vy = 1.5*3/5;
        om = 0;
    } 
    else if ((!stick.atas)&&(stick.bawah)&&(stick.kanan)&&(!stick.kiri)&&(stick.R2)){ 
    //stick right down
        vx = 1.5*3/5;
        vy = -1.5*3/5;
        om = 0;
    } 
    else if ((!stick.atas)&&(stick.bawah)&&(!stick.kanan)&&(stick.kiri)&&(stick.R2)){
        //stick left down
        vx = -1.5*3/5;
        vy = -1.5*3/5;
        om = 0;
    }
    
    
    
    //untuk pneumatik
    if(!stick.silang && !stick.lingkaran && !stick.kotak && !stick.segitiga){
        tembak = 1;
    }
    else if(!stick.silang && stick.lingkaran && !stick.kotak && !stick.segitiga){
        if(millis()-lastTimeTangan>500){
            if(kiri){
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
        kiri=1;
        armKiri=0;
        armKanan=1;
    }
    if(stick.R3){
        kiri=0;
        armKiri=1;
        armKanan=0;
    }
}

//void hitungThetaLeft(){
//    theta_pulse_left += (float)encLeft.getPulses();
//    theta_left = theta_pulse_left*360/538;
//    theta_left = 0.5*theta_left + 0.5* prev_theta_left;
//    encLeft.reset();
//    prev_theta_left = theta_left;
//}


//void hitungThetaRight(){
//    theta_pulse_right += (float)encRight.getPulses();
//    theta_right = theta_pulse_right*360/538;
//    theta_right = 0.5*theta_right + 0.5* prev_theta_right;
//    encRight.reset();
//    prev_theta_right = theta_right;
//}

void findSpeed(){

    v_feedback_A = (encA.getPulses()*360/538)*1000*WHEEL_RAD/(RAD_TO_DEG*4);
    v_feedback_B = (encB.getPulses()*360/538)*1000*WHEEL_RAD/(RAD_TO_DEG*4);
    v_feedback_C = (encC.getPulses()*360/538)*1000*WHEEL_RAD/(RAD_TO_DEG*4);
    v_feedback_D = (encD.getPulses()*360/538)*1000*WHEEL_RAD/(RAD_TO_DEG*4);
    
    encA.reset();
    encB.reset();
    encC.reset();
    encD.reset();
        
}