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

/******************************* defines ***********************************/

/*lingkaran di titik, satuan dalam meter */
#define R_ACCEPTANCE 1  

/*kecepatan maksimal base, satuan dalam m/s*/
#define BASE_SATURATE 10

/*konstanta pid untuk vx*/
#define KP_X_BASE 3
#define KI_X_BASE 0
#define KD_X_BASE 0
#define N_X_BASE 0
#define TS_X_BASE 0.3173
#define FF_X_BASE 1

/*konstanta pid untuk vy*/
#define KP_Y_BASE 3
#define KI_Y_BASE 0
#define KD_Y_BASE 0
#define N_Y_BASE 0
#define TS_Y_BASE 0.3173
#define FF_Y_BASE 1

/*konstanta pid untuk omega*/
#define KP_TETA_BASE 3
#define KI_TETA_BASE 0
#define KD_TETA_BASE 0
#define N_TETA_BASE 0
#define TS_TETA_BASE 0.3173
#define FF_TETA_BASE 1


/************************* deklarasi variable ***********************************/

/* deklarasi pid untuk base */
PID pid_x_base(KP_X_BASE, KI_X_BASE, KD_X_BASE, N_X_BASE, TS_X_BASE, FF_X_BASE, PID::PI_MODE);
PID pid_y_base(KP_Y_BASE, KI_Y_BASE, KD_Y_BASE, N_Y_BASE, TS_Y_BASE, FF_Y_BASE, PID::PI_MODE);
PID pid_teta_base(KP_TETA_BASE, KI_TETA_BASE, KD_TETA_BASE, N_TETA_BASE, TS_TETA_BASE, FF_TETA_BASE, PID::PI_MODE);


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
 * Menghitung kecepatan target robot berdasarkan posisi robot saat ini dan 
 * kecepatan dan posisi robot pada titik selanjutnya (implementasi dengan pid)
 * 
 * @param next_point : titik target yang dituju oleh robot
 * @param current_pos : titik koordinat robot saat ini 
 */
Coordinate velocityTracker(Trajectory next_point, Coordinate  current_pos);

/*
 * Menghitung arah gerak robot berdasarkan posisi robot dan target
 * 
 * @param next_point : titik target yang dituju oleh robot
 * @param current_pos : titik koordinat robot saat ini 
 */
float computeAlpha(Coordinate next_pos, Coordinate current_pos);

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
float vwGenerator(Trajectory next, Trajectory current, Trajectory prev, float accel, float decel, float saturation)


#endif