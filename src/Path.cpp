#include "mbed.h"
#include "Path.h"

//Get x,y,teta,v at current point
Coordinate getPoint (int index, Coordinate *path)
{
   return (*(path+index));
}

//Get x,y,teta,v at previous point
Coordinate getPrevPoint(int index, Coordinate *path)
{
   return (*(path+index-1));
}

//Get x,y,teta,v at next point
Coordinate getNextPoint(int index, Coordinate *path)
{
   return (*(path+index+1));
}

//Get velocity and distance at current point
Trajectory getPoint (int index, Trajectory *path)
{
   return (*(path+index));
}

//Get velocity and distance at prev point
Trajectory getPrevPoint (int index, Trajectory *path)
{
   return (*(path+index-1));
}

//Get velocity and distance at next point
Trajectory getNextPoint (int index, Trajectory *path)
{
   return (*(path+index+1));
}