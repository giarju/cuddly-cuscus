#include "state.h"

FSM::FSM(joysticknucleo *joy //objek joystik
    , Coordinate *speedBase //output target kecepatan
    , Trajectory *pas //kumpulan posisi untuk mengumpan
    , PinName pelempar
    , PinName tanganKiri
    , PinName tanganKanan
    , PinName capitKiri
    , PinName capitKanan
    , PinName sensorKiri
    , PinName sensorKanan
    , Timer *t) : lempar(pelempar),armLeft(tanganKiri),armRight(tanganKanan),
    gripLeft(capitKiri),gripRight(capitKanan),
    irLeft(sensorKiri),irRight(sensorKanan)
{
    kondisi = START;
    profil = t;
    stik = joy;
    lempar = 1;
    armLeft = 1;
    armRight = 1;
    lapanganMerah = true;
    passPosition=pas;
}

FSM::~FSM()
{
}

//untuk menjalankan gerakan otomatis
void FSM::fsmAuto(Coordinate currPos)
{
    bool irCondition;
    if (lapanganMerah)
    {
        irCondition = irRight;
    }
    else
    {
        irCondition = irLeft;
    }
    
    if (kondisi==START)
    {
        if (stik->silang && lastX!=stik->silang && profil->read_us()-debounceX>50000)
        {
            kondisi = WALL_FOLLOW;
        }
        lastX = stik->silang;
        debounceX = profil->read_us();
    }
    else if (kondisi==TO_POINT)
    {
        float alfa = computeAlpha(passPosition[bola],currPos);
        float kecepatan = vwGenerator(passPosition[bola],currPos,rackTarget[bola],6,7,3);
        speedBase->x=kecepatan*cos(alfa);
        speedBase->y=kecepatan*sin(alfa);
        //untuk pivot
        if (sqrt(pow((passPosition[bola].distance.x - currPos.x),2) + 
                               pow((passPosition[bola].distance.y - currPos.y),2))<R_ACCEPTANCE)
        {
            kondisi=WAIT;
        }
        
    }
    else if (kondisi==WALL_FOLLOW)
    {
        float alfa = computeAlpha(rackTarget[bola],currPos);
        float kecepatan = vwGenerator(rackTarget[bola],currPos,wfStart,6,7,3);
        speedBase->x=kecepatan*cos(alfa);
        speedBase->y=kecepatan*sin(alfa);
        wallFollow();

        if (sqrt(pow((rackTarget[bola].distance.x - currPos.x),2) + 
                               pow((rackTarget[bola].distance.y - currPos.y),2))<R_ACCEPTANCE
                                    && !irCondition)
        {
            kondisi=WAIT;
            bola++;
        }
    }
    else if (kondisi==WAIT)
    {
        if (stik->silang && lastX!=stik->silang && profil->read_us()-debounceX>50000)
        {
            if (sqrt(pow((rackTarget[bola].distance.x - currPos.x),2) + 
                               pow((rackTarget[bola].distance.y - currPos.y),2))<R_ACCEPTANCE)
            {
                kondisi=TO_POINT;
            }
            else
            {
                kondisi=WALL_FOLLOW;
            }
            debounceX = profil->read_us();
        }
        lastX = stik->silang;
        fsmManual();
        
    }
    
}

//wallFollowing
void FSM::wallFollow()
{
    speedBase->y = -0.5;
    if (lapanganMerah)
    {
        speedBase->x+=0.05;
    }
    else
    {
        speedBase->x-=0.05;
    }
    
}

//mengatur lapangan
void FSM::setLapanganBiru(){
    lapanganMerah=false;
}
void FSM::setLapanganMerah(){
    lapanganMerah=true;
}

//lempar bola
void FSM::tembak()
{
    lempar = 0;
    int lastShoot = profil->read_us();
    while (profil->read_us()-lastShoot>500000)
    {
        //tunggu proses nembak
    }
    lempar = 1;
}

//untuk menjalankan gerakan manual
void FSM::fsmManual()
{
    stickArrow();
    if (profil->read_us()-debounceO>50000 && stik->lingkaran && stik->lingkaran!=lastO)
    {
        armLeft = !armLeft;                 /*armRight = 1 naik*/
        debounceO = profil->read_us();
    }
    lastO = stik->lingkaran;
    if (profil->read_us()-debounceKotak>50000 && stik->kotak && stik->kotak!=lastKotak)
    {
        armLeft = !armLeft;                 /*armLeft = 1 naik*/
        debounceKotak = profil->read_us();
    }
    lastKotak = stik->kotak;
    if (stik->segitiga && !armLeft && !armRight)
    {
        tembak();
    }
    
}

//Prosedur untuk menggerakan dengan arrow dan pivot
void FSM::stickArrow()
{
    /* STICK ARROW STATE */
    if ((!stik->atas)&&(!stik->bawah)&&(!stik->kanan)&&(!stik->kiri)
            &&(!stik->R2)&&(!stik->R1)&&(!stik->L1)){
    //no input condition
        speedBase->x = 0;
        speedBase->y = 0;;
        speedBase->teta = 0;
        //pc.printf("diam\n");
    } 
    else if ((stik->atas)&&(!stik->bawah)&&(!stik->kanan)&&(!stik->kiri)&&(!stik->R2)){
    //stick up
        speedBase->x = 0;
        speedBase->y = 1;
        speedBase->teta = 0;
        //pc.printf("atas\n");
    } 
    else if ((!stik->atas)&&(stik->bawah)&&(!stik->kanan)&&(!stik->kiri)&&(!stik->R2)){
    //stick down
        speedBase->x = 0;
        speedBase->y = -1;
        speedBase->teta = 0;
        //pc.printf("bawah\n");
    } 
    else if ((!stik->atas)&&(!stik->bawah)&&(stik->kanan)&&(!stik->kiri)&&(!stik->R2)){
    //stick right
        speedBase->x = 1;
        speedBase->y = 0;
        speedBase->teta = 0;
        //pc.printf("kiri\n");
    } 
    else if ((!stik->atas)&&(!stik->bawah)&&(!stik->kanan)&&(stik->kiri)&&(!stik->R2)){
    //stick left
        speedBase->x = -1;
        speedBase->y = 0;
        speedBase->teta = 0;
        //pc.printf("kanan\n");
    } 
    else if ((stik->atas)&&(!stik->bawah)&&(stik->kanan)&&(!stik->kiri)&&(!stik->R2)){
    //stick right up
        speedBase->x = 1.5/2;
        speedBase->y = 1.5/2;
        speedBase->teta = 0;
    } 
    else if ((stik->atas)&&(!stik->bawah)&&(!stik->kanan)&&(stik->kiri)&&(!stik->R2)){
    //stick left up
        speedBase->x = -1.5/2;
        speedBase->y = 1.5/2;
        speedBase->teta = 0;
    } 
    else if ((!stik->atas)&&(stik->bawah)&&(stik->kanan)&&(!stik->kiri)&&(!stik->R2)){ 
    //stick right down
        speedBase->x = 1.5/2;
        speedBase->y = -1.5/2;
        speedBase->teta = 0;
    } 
    else if ((!stik->atas)&&(stik->bawah)&&(!stik->kanan)&&(stik->kiri)&&(!stik->R2)){
        //stick left down
        speedBase->x = -1.5/2;
        speedBase->y = -1.5/2;
        speedBase->teta = 0;
    }
    //mode lambat
    else if ((stik->atas)&&(!stik->bawah)&&(!stik->kanan)&&(!stik->kiri)&&(stik->R2)){
    //stick up
        speedBase->x = 0;
        speedBase->y = 0.75/2;
        speedBase->teta = 0;
        //pc.printf("atas\n");
    } 
    else if ((!stik->atas)&&(stik->bawah)&&(!stik->kanan)&&(!stik->kiri)&&(stik->R2)){
    //stick down
        speedBase->x = 0;
        speedBase->y = -0.75/2;
        speedBase->teta = 0;
        //pc.printf("bawah\n");
    } 
    else if ((!stik->atas)&&(!stik->bawah)&&(stik->kanan)&&(!stik->kiri)&&(stik->R2)){
    //stick right
        speedBase->x = 0.75/2;
        speedBase->y = 0;
        speedBase->teta = 0;
        //pc.printf("kiri\n");
    } 
    else if ((!stik->atas)&&(!stik->bawah)&&(!stik->kanan)&&(stik->kiri)&&(stik->R2)){
    //stick left
        speedBase->x = -0.75/2;
        speedBase->y = 0;
        speedBase->teta = 0;
        //pc.printf("kanan\n");
    } 
    else if ((stik->atas)&&(!stik->bawah)&&(stik->kanan)&&(!stik->kiri)&&(stik->R2)){
    //stick right up
        speedBase->x = 1.5*3/5;
        speedBase->y = 1.5*3/5;
        speedBase->teta = 0;
    } 
    else if ((stik->atas)&&(!stik->bawah)&&(!stik->kanan)&&(stik->kiri)&&(stik->R2)){
    //stick left up
        speedBase->x = -1.5*3/5;
        speedBase->y = 1.5*3/5;
        speedBase->teta = 0;
    } 
    else if ((!stik->atas)&&(stik->bawah)&&(stik->kanan)&&(!stik->kiri)&&(stik->R2)){ 
    //stick right down
        speedBase->x = 1.5*3/5;
        speedBase->y = -1.5*3/5;
        speedBase->teta = 0;
    } 
    else if ((!stik->atas)&&(stik->bawah)&&(!stik->kanan)&&(stik->kiri)&&(stik->R2)){
        //stick left down
        speedBase->x = -1.5*3/5;
        speedBase->y = -1.5*3/5;
        speedBase->teta = 0;
    }
    else if (stik->R1&&(stik->R2))
    {
        //pivot lambat
        speedBase->x = 0;
        speedBase->y = 0;
        speedBase->teta = 3;
    }
    else if (stik->L1&&(stik->R2))
    {
        //pivot lambat
        speedBase->x = 0;
        speedBase->y = 0;
        speedBase->teta = -3;
    }
    else if (stik->R1&&!(stik->R2))
    {
        //pivot cepat
        speedBase->x = 0;
        speedBase->y = 0;
        speedBase->teta = 6;
    }
    else if (stik->R1&&!(stik->R2))
    {
        //pivot cepat
        speedBase->x = 0;
        speedBase->y = 0;
        speedBase->teta = 6;
    }
}