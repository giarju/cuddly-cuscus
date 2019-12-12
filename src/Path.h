/**
 *  Header Path KRAI
 *  Mendapatkan output berupa point pada titik tertentu
 **/
 
#ifndef PATH_H
#define PATH_H

//Library
#include "mbed.h"

 /******************************************
 Struct : defines a physically grouped list of variables under one name (IN THIS
 CASE "Coordinate") in a block of memory, allowing the different variables to be 
 accessed via a single pointer or by the struct declared name which returns 
 the same address. 
 In this case, we use 4 variables such as x,y,teta,and v
 *******************************************/

typedef struct Coordinate_t 
{
   float  x;        /* Get x position at current point */
   float  y;        /* Get y position at current point */
   float  teta;     /* Get measured angle from the initial position*/
}Coordinate;


typedef struct Trajectory_t
{
   Coordinate Velocity;
   Coordinate Distance;
}Trajectory;


//Get x,y,teta,v at current point
Coordinate getPoint (int index, Coordinate *path);
/*******************************************
 * Call our struct name ("Coordinate")
 * Give name for our function ("getPoint")
 * Our input ("int index")
 * Struct name + pointer to get the value of path
 * This function will return the value of position at certain index
 *******************************************/
 
 //Get x,y,teta,v at previous point
Coordinate getPrevPoint(int index, Coordinate *path);
/*******************************************
 * This function will return the value of position at previous index
 *******************************************/
 
 //Get x,y,teta,v at next point
Coordinate getNextPoint(int index, Coordinate *path);
/*******************************************
 * This function will return the value of position at next index
 *******************************************/
 
  //Get velocity and distance at current point
Trajectory getPoint(int index, Trajectory *path);
/*******************************************
 * This function will return the value of velocity and distance at certain index
 *******************************************/
 
  //Get velocity and distance at previous point
Trajectory getPrevPoint(int index, Trajectory *path);
/*******************************************
 * This function will return the value of velocity and distance at previous index
 *******************************************/
 
 //Get velocity and distance at next point
Trajectory getNextPoint(int index, Trajectory *path);
/*******************************************
 * This function will return the value of velocity and distance at next index
 *******************************************/
#endif