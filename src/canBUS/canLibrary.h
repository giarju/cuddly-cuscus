#ifndef CANLIBRARY
#define CANLIBRARY

#include "mbed.h"
#define WAIT_SEND        0.005
#define WAIT_CAN         0.05
#define FREKUENSI_CAN  	200000

typedef void (*procType) (void);

void sendDataToSlave(int ID_DATA, float* msgAddres, procType proc);

void canSetup();

int receiveData(int ID_DATA, int* var);

void sendKpKiKd(float Kp, float Ki, float Kd);

void sendSetPoint()
#endif