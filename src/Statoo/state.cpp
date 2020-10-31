// #define FSMLAMA
// #define MANUALLAMA

#include "state.h"

FSM::FSM(int _initial_state)
{
    fungsional_state = _initial_state;
    map_state = _initial_state;
}

//mengatur lapangan
void FSM::setLapanganBiru(){
    lapangan_merah=false;
}
void FSM::setLapanganMerah(){
    lapangan_merah=true;
}

#ifdef FSMLAMA
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
#endif

#ifndef FSMLAMA
stick_message FSM::resetChangeStick()
{
    stick_message press;
    press.stick_segitiga = false;
    press.stick_lingkaran = false;
    press.stick_silang = false;
    press.stick_kotak = false;;
    press.stick_atas = false;
    press.stick_kanan = false;
    press.stick_bawah = false;
    press.stick_kiri = false;

    press.stick_segitiga_click = false;
    press.stick_lingkaran_click = false;
    press.stick_silang_click = false;
    press.stick_kotak_click = false;;
    press.stick_atas_click = false;
    press.stick_kanan_click = false;
    press.stick_bawah_click = false;
    press.stick_kiri_click = false;

    press.stick_R1_click = false;
    press.stick_R2_click = false;
    press.stick_L1_click = false;
    press.stick_L2_click = false;
    press.stick_start_click = false;
    press.stick_select_click = false;
    
    press.stick_R1 = false;
    press.stick_R2 = false;
    press.stick_L1 = false;
    press.stick_L2 = false;
    press.stick_start = false;
    press.stick_select = false;

    return(press);
}

void FSM::putChangeStick(stick_message m_stick)
{
    if (!queue_full_stick){
        queue_stick[0] = m_stick;
        queue_full_stick = true;
    }
}

stick_message FSM::getChangeStick()
{
    if (queue_full_stick){      
        queue_full_stick = false;
        return(queue_stick[0]);
    }
    else {
        return(resetChangeStick());
    }
}

void FSM::fsmMap(stick_message stick_press, bool near_last)
{
    if (stick_press.stick_silang_click && near_last)
    {
        map_state++;
    }
    else if (stick_press.stick_segitiga_click)
    {
        map_state--;
    }
}

void FSM::fsmFungsional(stick_message stick_press, float posisi_tangan)
{
    /* capit bawah buka, lempar turun*/
    if(fungsional_state == 0)
    {
        capit_pneu = 0;
        lempar_pneu = 0;
        tangan_target = 0;

        if (stick_press.stick_kotak_click)
        {
            fungsional_state++;
        }
        else if (stick_press.stick_lingkaran_click)
        {
            fungsional_state = 5;
        }
    }
    /* capit bawah tutup */
    else if (fungsional_state == 1)
    {
        capit_pneu = 1;
        lempar_pneu = 0;
        tangan_target = 0;

        if (stick_press.stick_kotak_click)
        {
            fungsional_state++;
        }
        else if (stick_press.stick_lingkaran_click)
        {
            fungsional_state--;
        }
    }
    /* capit naik */
    else if (fungsional_state == 2)
    {
        capit_pneu = 1;
        lempar_pneu = 0;
        tangan_target = 180;

        if (stick_press.stick_kotak_click)
        {
            fungsional_state++;
        }
        else if (stick_press.stick_lingkaran_click)
        {
            fungsional_state--;
        }
    }
    /* capit atas buka */
    else if (fungsional_state == 3)
    {
        capit_pneu = 0;
        lempar_pneu = 0;
        tangan_target = 180;
        
        if (stick_press.stick_kotak_click)
        {
            fungsional_state++;
        }
        else if (stick_press.stick_lingkaran_click)
        {
            fungsional_state--;
        }
    }
    /* capit turun */
    else if (fungsional_state == 4)
    {
        capit_pneu = 0;
        lempar_pneu = 0;
        tangan_target = 0;
        
        if (stick_press.stick_kotak_click && posisi_tangan < 25.0)
        {
            fungsional_state++;
        }
        else if (stick_press.stick_lingkaran_click)
        {
            fungsional_state--;
        }
    }
    /* lempar naik */
    else if (fungsional_state == 5)
    {
        capit_pneu = 0;
        lempar_pneu = 1;
        tangan_target = 0;
        
        if (stick_press.stick_kotak_click)
        {
            fungsional_state = 0;
        }
        else if (stick_press.stick_lingkaran_click)
        {
            fungsional_state--;
        }
    }

    /* reset to initial */
    if (stick_press.stick_select_click)
    {
        fungsional_state = 0;
    }
}

void FSM::fsmAuto(bool near_last, float posisi_tangan)
{
    stick_press = getChangeStick();
    fsmMap(stick_press, near_last);
    fsmFungsional(stick_press, posisi_tangan);
}
#endif

#ifdef MANUALLAMA
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
#endif