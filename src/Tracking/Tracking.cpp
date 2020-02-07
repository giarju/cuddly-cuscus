/***************************************************************************
 * Title      : Library untuk melakukan trajectory tracking 
 * Name       : Tracking.cpp
 * Version    : 1.0
 * Author     : Gian Arjuna EL 16
 * Date       : 12 Desember 2019
 * Description:
 *
 * Library ini digunakan untuk melakukan perhitungan kecepatan base robot
 * berdasarkan posisi robot dan posisi target
 * 
 ***************************************************************************/

/******************************* library ***********************************/
#include "Tracking.h"

/*************************** definisi fungsi ******************************/

/* Melakukan keputusan untuk pindah ke titik selanjutnya pada trajectory map */
int nextIndex(Trajectory trajectory_map, Coordinate current_pos, int index)
{
    /* menghitung jarak robot ke titik target */
    double dist_to_next = sqrt(pow((trajectory_map.distance.x - current_pos.x),2) + 
                               pow((trajectory_map.distance.y - current_pos.y),2));
    

    /* pindah ke target berikutnya jika sudah berada pada 
     * jarak R_ACCEPTANCE dari titik target*/
    if (dist_to_next < R_ACCEPTANCE)
    {
        return (index + 1);
    }
    else
    {
        return (index);
    }
}

/* Menghitung kecepatan target robot */
Coordinate velocityTracker(Trajectory next_pos, Coordinate  current_pos)
{
    Coordinate velocity;

    /* memberikan nilai target kecepatan dengan PIDF (FF target kecepatan dari map)*/
    // velocity.x = pid_x_base.createOutput(next_pos.distance.x, current_pos.x, BASE_SATURATE, next_pos.velocity.x);
    // velocity.y = pid_y_base.createOutput(next_pos.distance.y, current_pos.y, BASE_SATURATE, next_pos.velocity.y);
    // velocity.teta = pid_teta_base.createOutput(next_pos.distance.teta, current_pos.teta, BASE_SATURATE, next_pos.velocity.teta);

    return(velocity);
}

/* Menghitung arah gerak robot */
float computeAlpha(Coordinate next_pos, Coordinate current_pos)
{
    return(atan2(next_pos.y - current_pos.y, next_pos.x - current_pos.x));
}

/* Menghitung kecepatan target robot  */
float vwGenerator(Trajectory next, Trajectory current, Trajectory prev, float accel, float decel, float saturation)
{  
    /* mencari jarak robot ke titik asal dan target */
    float dist_to_prev = sqrt( pow(prev.distance.y - current.distance.y ,2)+pow(prev.distance.x - current.distance.x ,2));
    float dist_to_next = sqrt( pow(next.distance.y - current.distance.y ,2)+pow(next.distance.x - current.distance.x ,2));
    
    /* mencari kecepatan awal dan akhir robot */
    float initial_speed = sqrt( pow(prev.velocity.y - current.velocity.y ,2)+pow(prev.velocity.x - current.velocity.x ,2));
    float final_speed = sqrt( pow(next.velocity.y - current.velocity.y ,2)+pow(next.velocity.x - current.velocity.x ,2));

    /* kecepatan target base */
    float base_speed = sqrt(2*accel*dist_to_prev);
    if(dist_to_prev < dist_to_next){
        base_speed = sqrt(2*accel*dist_to_prev + initial_speed);
    }
    else{
        base_speed = sqrt(2*decel*dist_to_next + final_speed);
    }

    /* saturasi kecepatan base */
    if (base_speed > saturation){
        base_speed = saturation;
    }
    else if (base_speed < -saturation){
        base_speed = -saturation;
    }

    return(base_speed);
}



