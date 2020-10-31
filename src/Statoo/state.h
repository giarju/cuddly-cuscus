#ifndef STATOO_H
#define STATOO_H 

#include "mbed/mbed.h"
#include "Configuration/constant.h"
class FSM
{
private:
    stick_message stick_press;
    stick_message queue_stick[1];
    bool queue_full_stick;
    bool lapangan_merah = true;
    
    
public:
    int map_state;
    int fungsional_state;
    int capit_pneu;
    int lempar_pneu;
    float tangan_target;

    FSM(int _initial_state);

    /*untuk mengatur lapangan*/
    void setLapanganMerah();
    void setLapanganBiru();

    /*state gerakan tangan dan pelempar*/
    void fsmFungsional(stick_message stick_press, float posisi_tangan);

    stick_message resetChangeStick();

    void putChangeStick(stick_message m_stick);

    stick_message getChangeStick();

    /*untuk mengganti map di trajectory tracking*/
    void fsmMap(stick_message stick_press, bool near_last);

    /*untuk menjalankan gerakan otomatis*/
    void fsmAuto(bool near_last, float posisi_tangan);

    /*untuk menjalankan gerakan manual*/
    void fsmManual();

    /*State joystick*/
    void stickArrow();


};
#endif