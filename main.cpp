#include "mbed.h"
#include "millis.h"

//DigitalOut myled(LED1);
Serial pc(USBTX, USBRX, 115200);
AnalogIn IR1(PD_2);

void GP2A();

int i = 0;
int last_ir = 0;
double distance_IR1 =1;
double vDist = 0;

int main() {
    startMillis();
    while(1) {
        i++;
        pc.printf("%d %.2f %.2f\n",i, distance_IR1, vDist);
        if (millis() - last_ir > 25){
            GP2A();
            last_ir = millis();
        }
    }
}

void GP2A()
{
    double m_slope = 23.3;
    vDist = (double)IR1.read()*3.3;
    distance_IR1 = m_slope/vDist;
    if (distance_IR1 > 600) distance_IR1 = 600;
    else if (distance_IR1 < 100) distance_IR1 = 100;
}

a

































