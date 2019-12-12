#include "mbed.h"
#include "Path.h"
Serial pc(USBTX, USBRX,115200);

int main ()
{
    while (1){
      Coordinate A[10];
      Coordinate B[10];
      A[1].x = 8.9;
      A[1].y = 4.0;
      A[1].teta = 30.0;
      A[2].x = 10.1;
      A[2].y = 5.1;
      A[2].teta = 25.6;
      A[3].x = 3.1;
      A[3].y = 7.1;
      A[3].teta = 27.1;

      B[1] = getPoint (1, A);
      B[2] = getPrevPoint (3,A);
      B[3] = getNextPoint (2,A);
      pc.printf ("%f, %f, %f \n", B[1].x, B[1].y, B[1].teta);
      pc.printf ("%f, %f, %f \n", B[2].x, B[2].y, B[2].teta); 
      pc.printf ("%f, %f, %f \n", B[3].x, B[3].y, B[3].teta); 
    } 
}