#include "mbed.h"
#include "canLibrary.h"



void sendDataToSlave(int ID_DATA, float* msgAddres, procType proc){
    CANMessage msg(ID_DATA, reinterpret_cast<char*>(msgAddres), 4);
    if (msg.write(msg)){
        proc();
    }
    wait(WAIT_SEND);
}

int receiveData(int ID_DATA, int* var){
    if (msg.id == ID_DATA){
        *var = *reinterpret_cast<int*>(msg.data);
        wait(WAIT_CAN);
    }
}
