#ifndef MOTOR_MODEL_H
#define MOTOR_MODEL_H


class MotorModel 
{
    public : 
        double curr_theta;

        MotorModel(double k1, double k2, double k3, double k4, double k5, double k6, double base_volt);
        void motorSim(double pwm);


    private : 
        //constant is tuned for zoh discrete transfer function of theta motor with time samp 1 ms
        double _k1;  //theta(n) coeff (z^0 den)
        double _k2;  //theta(n-1) (z^-1 den)
        double _k3;  //theta(n-2) (z^-2 den)
        double _k4;  //volt(n-1) (z^-1 num)
        double _k5;  //volt(n-2) (z^-2 num)
        double _k6;  //volt(n-3) (z^-3 num)
        double _base_volt;

        double prev_theta;
        double prev2_theta;
        double prev_volt;
        double prev2_volt;
        double prev3_volt;
};




#endif