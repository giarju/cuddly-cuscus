#ifndef ROBOT_MODEL_H
#define ROBOT_MODEL_H

#include "MotorModel.h"
#include "EncModel.h"
#include "Path/Path.h"
#include "math.h"
        

#define WHEEL_RAD_MOTOR  0.075
#define WHEEL_RAD_ODOM  0.06
#define ROBOT_RAD 0.32
#define TOTAL_PULSE_MOTOR  538
#define TOTAL_PULSE_ODOM  4000
#define BASE_VOLT  240
#define SQR2 1.41421356237

#define MOTOR1_K1 1.000000000000000     
#define MOTOR1_K2 -1.99861880768107    
#define MOTOR1_K3 0.998618807681070     
#define MOTOR1_K4 4.55434472841233e-07
#define MOTOR1_K5 4.55243089052985e-07
#define MOTOR1_K6 9.28554056001574e-17

#define MOTOR2_K1 1.000000000000000   
#define MOTOR2_K2 -1.99861880768107   
#define MOTOR2_K3 0.998618807681070   
#define MOTOR2_K4 4.55434472841233e-07
#define MOTOR2_K5 4.55243089052985e-07
#define MOTOR2_K6 9.28554056001574e-17

#define MOTOR3_K1 1.000000000000000   
#define MOTOR3_K2 -1.99861880768107   
#define MOTOR3_K3 0.998618807681070   
#define MOTOR3_K4 4.55434472841233e-07
#define MOTOR3_K5 4.55243089052985e-07
#define MOTOR3_K6 9.28554056001574e-17

#define MOTOR4_K1 1.000000000000000   
#define MOTOR4_K2 -1.99861880768107   
#define MOTOR4_K3 0.998618807681070   
#define MOTOR4_K4 4.55434472841233e-07
#define MOTOR4_K5 4.55243089052985e-07
#define MOTOR4_K6 9.28554056001574e-17

class RobotModel
{
    public : 
        typedef enum TYPE
        {
            OMNI_3,
            OMNI_4
        }TYPE;

        TYPE _type;
        Coordinate Odometry_position;
        float y_robot;
        float x_robot;
        float h_robot;
        float y_global;
        float x_global;
        
        MotorModel motor1;
        MotorModel motor2; 
        MotorModel motor3; 
        MotorModel motor4; 

        EncModel encA; 
        EncModel encB; 
        EncModel encC; 
        EncModel encD; 

        EncModel encYODOM;
        EncModel encXODOM;

        RobotModel(TYPE type);
        void createRobot();
        void omni4WheelModel();

    private : 



};

#endif