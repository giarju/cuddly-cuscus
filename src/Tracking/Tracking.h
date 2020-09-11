/***************************************************************************
 * Title      : Library untuk melakukan trajectory tracking 
 * Name       : Tracking.h
 * Version    : 1.0
 * Author     : Gian Arjuna EL 16
 * Date       : 12 Desember 2019
 * Description:
 *
 * Library ini digunakan untuk melakukan perhitungan kecepatan base robot
 * berdasarkan posisi robot dan posisi target
 * 
 ***************************************************************************/
#ifndef TRACKING_H
#define TRACKING_H

#include "mbed.h"
#include "Path/Path.h"
#include "PID/PID.h"
#include "math.h"

/******************************* defines ***********************************/

/*lingkaran di titik, satuan dalam meter */
#define R_ACCEPTANCE 1  

/*kecepatan maksimal base, satuan dalam m/s*/
#define BASE_SATURATE 1


/************************* deklarasi variable ***********************************/

/************************* deklarasi fungsi  ***********************************/

/*
 * Melakukan keputusan untuk pindah ke titik selanjutnya pada trajectory map
 * 
 * @param trajectory_map : titik target yang dituju oleh robot
 * @param current_pos : titik koordinat robot saat ini 
 * @param index : index array dari titik target 
 */
int nextIndex(Trajectory trajectory_map, Coordinate current_pos, int index);

/*
 * Menghitung arah gerak robot berdasarkan posisi robot dan target
 * 
 * @param next_point : titik target yang dituju oleh robot
 * @param current_pos : titik koordinat robot saat ini 
 */
float computeAlpha(Coordinate next_pos, Coordinate current_pos);

void findClosestIntersection(Trajectory_vr* map, Coordinate current_pos, int map_size, int prev_closest, float pursuit_radius, int* closest_point, int* target_point, int* is_intersect);

Trajectory_vr getLinearIntpPoint(Trajectory_vr* map, Coordinate current_pos, int is_intersect, int target_point, float pursuit_radius);


/*
 * Menghitung kecepatan target robot berdasarkan posisi robot saat ini dan 
 * kecepatan dan posisi robot pada titik selanjutnya (implementasi dengan pid)
 * 
 * @param next_point : titik target yang dituju oleh robot
 * @param current_pos : titik koordinat robot saat ini 
 */
void velocityTracker(Trajectory_vr* map, Coordinate current_pos, int map_size, float pursuit_radius, Coordinate* velocity, int* closest_point, int* target_point);


/*
 * Menghitung kecepatan target robot berdasarkan posisi robot saat ini dan 
 * kecepatan dan posisi robot pada titik selanjutnya (implementasi dengan akar)
 * 
 * @param next : titik target trajectory yang dituju oleh robot
 * @param current : titik trajectory robot saat ini 
 * @param prev : titik trajectory robot sebelumnya
 * @param accel : konstanta akselerasi robot
 * @param decel : konstanta deselerasi robot
 * @param accel : kecepatan maksimum robot
 */
float vwGenerator(Trajectory next, Trajectory current, Trajectory prev, float accel, float decel, float saturation);


#endif