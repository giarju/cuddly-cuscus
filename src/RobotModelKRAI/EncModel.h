#ifndef ENC_MODEL_H
#define ENC_MODEL_H

class EncModel
{
    public : 
        float curr_value; 
        float prev_value;   
        float pulses;    

        EncModel(int total_pulse, float wheel_rad);
        void encThetaSim(float value);
        void encDistanceSim(float value);
        float pulsesTheta();
        float pulsesOmega();

    private : 
        
        float _wheel_rad;
        int _total_pulse;

};

#endif