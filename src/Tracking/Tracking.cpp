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
int nextIndex(Coordinate trajectory_map, Coordinate current_pos, int index)
{
    /* menghitung jarak robot ke titik target */
    double dist_to_next = sqrt(pow(( trajectory_map.x - current_pos.x),2) + 
                               pow(( trajectory_map.y - current_pos.y),2));
    

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

/* Menghitung arah gerak robot */
float computeAlpha(Coordinate next_pos, Coordinate current_pos)
{
    return(atan2(next_pos.y - current_pos.y, next_pos.x - current_pos.x));
}

/* Mencari intersection circle and path terjauh dan point terdekat*/
void findClosestIntersection(Trajectory_vr* map, Coordinate current_pos, uint32_t map_size, uint32_t prev_closest, float pursuit_radius, int* closest_point, int* target_point, int* is_intersect)
{
    //brute force find closest distance of points to robot 
    float distance_2_robot = 1000000; //initialize to very big size
    *is_intersect = 0;
    float distance; 
    float intersection = 0;
    float prev_intersection = 0;
    for (uint32_t i = prev_closest; i < map_size && i <= prev_closest + 100; i++)
    {
        distance = sqrt((map[i].distance.x - current_pos.x)*(map[i].distance.x - current_pos.x) 
                        + (map[i].distance.y - current_pos.y)*(map[i].distance.y - current_pos.y));

        if (distance <= distance_2_robot)
        {
            distance_2_robot = distance;
            *closest_point = i;
        }

        if (distance_2_robot <= pursuit_radius) //inside circle
        {
            intersection = distance - pursuit_radius;
            if(intersection > 0 && prev_intersection <= 0)// farthest intersection
            {   
                *target_point = i;
                *is_intersect = 1;               
            }
            prev_intersection = intersection;
        }          
        else
        {
            *target_point = *closest_point; //no intersection
            *is_intersect = 0;
        }
    } 
}


Trajectory_vr getLinearIntpPoint(Trajectory_vr* map, Coordinate current_pos, int is_intersect, int target_point, float pursuit_radius)
{
    Trajectory_vr next_point;

    if (is_intersect)
    {
        /* intersection with linear interpolation between points */
        float a = (map[target_point].distance.x - map[target_point - 1].distance.x)*(map[target_point].distance.x - map[target_point - 1].distance.x)
            + (map[target_point].distance.y - map[target_point - 1].distance.y)*(map[target_point].distance.y - map[target_point - 1].distance.y);
        float b = 2*(map[target_point - 1].distance.x - current_pos.x)*(map[target_point].distance.x - map[target_point - 1].distance.x)
            + 2*(map[target_point - 1].distance.y - current_pos.y)*(map[target_point].distance.y - map[target_point - 1].distance.y);
        float c =(map[target_point - 1].distance.x - current_pos.x)*(map[target_point - 1].distance.x - current_pos.x)
            + (map[target_point - 1].distance.y - current_pos.y)*(map[target_point - 1].distance.y - current_pos.y) - pursuit_radius*pursuit_radius;
        
        float discriminant = b*b-4*a*c;
    
        float point1_x = map[target_point - 1].distance.x + (-b - sqrt(discriminant))/2/a*(map[target_point].distance.x - map[target_point - 1].distance.x);
        float point1_y = map[target_point - 1].distance.y + (-b - sqrt(discriminant))/2/a*(map[target_point].distance.y - map[target_point - 1].distance.y);
        float point2_x = map[target_point - 1].distance.x + (-b + sqrt(discriminant))/2/a*(map[target_point].distance.x - map[target_point - 1].distance.x);
        float point2_y = map[target_point - 1].distance.y + (-b + sqrt(discriminant))/2/a*(map[target_point].distance.y - map[target_point - 1].distance.y);
    
        float d_to_next1 = sqrt((map[target_point].distance.x - point1_x)*(map[target_point].distance.x - point1_x)
            + (map[target_point].distance.y - point1_y)*(map[target_point].distance.y - point1_y)); 
        float d_to_next2 = sqrt((map[target_point].distance.x - point2_x)*(map[target_point].distance.x - point2_x)
            + (map[target_point].distance.y - point2_y)*(map[target_point].distance.y - point2_y)); 

        if (d_to_next1 <=  d_to_next2)
        {
            next_point.distance.x = point1_x;
            next_point.distance.y = point1_y;
        }
        else
        {
            next_point.distance.x = point2_x;
            next_point.distance.y = point2_y;
        }   
        next_point.distance.teta = map[target_point].distance.teta; 
        next_point.vr = map[target_point].vr;   
    }
    else 
    {
        next_point.distance.x    = map[target_point].distance.x;
        next_point.distance.y    = map[target_point].distance.y;
        next_point.distance.teta = map[target_point].distance.teta;
        next_point.vr            = map[target_point].vr;   
    }


    return (next_point);
}
    

/* Menghitung kecepatan target robot */
void velocityTracker(Trajectory_vr* map, Coordinate current_pos, int map_size, float pursuit_radius, Coordinate* velocity, int* closest_point, int* target_point)
{
    Trajectory_vr next_point;
    // int target_point;
    int is_intersect;
    float alpha;

    findClosestIntersection(map, current_pos, map_size, *closest_point, pursuit_radius,  closest_point, target_point, &is_intersect);
    next_point =  getLinearIntpPoint(map, current_pos, is_intersect, *target_point, pursuit_radius);
    alpha = computeAlpha(next_point.distance,current_pos);

    velocity->x = next_point.vr*cos(alpha);
    velocity->y = next_point.vr*sin(alpha);
    velocity->teta = next_point.distance.teta;
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



