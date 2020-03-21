#include "Tracking/Tracking.h"
#include "Path/Path.h"
#include "JoystickPS3/JoystickPS3.h"

class FSM
{
private:
    enum {START,TO_POINT,WAIT,WALL_FOLLOW,STICK_ARROW} kondisi;
    joysticknucleo *stik;
    Coordinate *speedBase;
    Trajectory *passPosition;
    Trajectory rackTarget[5];
    Trajectory wfStart; //posisi awal untuk wall following
    //output pelempar
    DigitalOut lempar;
    //output capit
    DigitalOut gripLeft;
    DigitalOut gripRight;
    //output untuk sudut tangan {0,1}={0,180}
    DigitalOut armLeft;
    DigitalOut armRight;
    //input untuk IR tangan
    DigitalIn irLeft;
    DigitalIn irRight;
    bool lapanganMerah;
    //timer untuk time sampling
    Timer *profil;
    uint32_t debounceO,debounceX,debounceKotak;
    bool lastO,lastX,lastKotak;
    //bola yang udah diambil
    int bola;
public:
    FSM(joysticknucleo *joy //objek joystik
    , Coordinate *speedBase //output target kecepatan
    , Trajectory *pas //kumpulan posisi untuk mengumpan
    , PinName pelempar
    , PinName tanganKiri
    , PinName tanganKanan
    , PinName capitKiri
    , PinName capitKanan
    , PinName sensorKiri
    , PinName sensorKanan
    , Timer *t
    );
    ~FSM();

    //method

    //untuk mengatur lapangan
    void setLapanganMerah();
    void setLapanganBiru();

    //lempar bola
    void tembak();

    //untuk menjalankan gerakan otomatis
    void fsmAuto(Coordinate);

    //wallFollowing
    void wallFollow();

    //untuk menjalankan gerakan manual
    void fsmManual();

    //Prosedur untuk menggerakan dengan arrow
    void stickArrow();

    
};