/***************************************************************************
 * Title      : Model robot KRAI
 * Name       : RobotModel.cpp
 * Version    : 1.0
 * Author     : Gian Arjuna EL 16
 * Date       : 20 Mei 2020
 * Description:
 *
 * Class ini berfungsi untuk mensimulasikan pergerakan robot. fungsi yang
 * berada di kelas ini akan menggantikan aktuator dan sensor di dunia nyata.
 * 
 ***************************************************************************/

#include "RobotModel.h"

RobotModel::RobotModel(TYPE type)
:motor1(MOTOR1_K1, MOTOR1_K2, MOTOR1_K3, MOTOR1_K4, MOTOR1_K5, MOTOR1_K6,BASE_VOLT)
,motor2(MOTOR2_K1, MOTOR2_K2, MOTOR2_K3, MOTOR2_K4, MOTOR2_K5, MOTOR2_K6,BASE_VOLT)
,motor3(MOTOR3_K1, MOTOR3_K2, MOTOR3_K3, MOTOR3_K4, MOTOR3_K5, MOTOR3_K6,BASE_VOLT)
,motor4(MOTOR4_K1, MOTOR4_K2, MOTOR4_K3, MOTOR4_K4, MOTOR4_K5, MOTOR4_K6,BASE_VOLT)
,encA(TOTAL_PULSE_MOTOR, WHEEL_RAD_MOTOR)
,encB(TOTAL_PULSE_MOTOR, WHEEL_RAD_MOTOR)
,encC(TOTAL_PULSE_MOTOR, WHEEL_RAD_MOTOR)
,encD(TOTAL_PULSE_MOTOR, WHEEL_RAD_MOTOR)
,encYODOM(TOTAL_PULSE_ODOM, WHEEL_RAD_ODOM)
,encXODOM(TOTAL_PULSE_ODOM, WHEEL_RAD_ODOM)
{
    /* untuk saat ini model yang dibuat hanya robot omniwheel 4 roda*/
    _type = type;
    
}

void RobotModel::createRobot()
{
    if (_type == OMNI_4)
    {
        /* construct object */     
    
 
    }
}

void RobotModel::omni4WheelModel()
{
    //forward kinematics omni 4 wheel 
    x_robot = SQR2/2*WHEEL_RAD_MOTOR*(-motor1.curr_theta - motor2.curr_theta + motor3.curr_theta + motor4.curr_theta);
    y_robot = SQR2/2*WHEEL_RAD_MOTOR*(-motor1.curr_theta + motor2.curr_theta + motor3.curr_theta - motor4.curr_theta);
    h_robot = WHEEL_RAD_MOTOR/ROBOT_RAD/4*(motor1.curr_theta + motor2.curr_theta + motor3.curr_theta + motor4.curr_theta);
    
    y_global = x_robot*sin(h_robot) + y_robot*cos(h_robot);
    x_global = x_robot*cos(h_robot) - y_robot*sin(h_robot);

    encA.encThetaSim(motor1.curr_theta);
    encB.encThetaSim(motor2.curr_theta);
    encC.encThetaSim(motor3.curr_theta);
    encD.encThetaSim(motor4.curr_theta);

    encYODOM.encDistanceSim(y_robot);
    encXODOM.encDistanceSim(x_robot);

    Odometry_position.x = x_robot;
    Odometry_position.y = y_robot;
    Odometry_position.teta = h_robot;
}




