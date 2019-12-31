#include "mbed.h"
#include "canLibrary.h"

#define ID_DATA_Kp  
#define ID_DATA_Ki
#define ID_DATA_Kd
#define ID_DATA_4

void canSetup(){
    can.frequency(FREKUENSI_CAN);
}
void sendDataToSlave(int ID_DATA, float* _msg_addres, procType proc){   // procedure send data to Slave
    CANMessage msg(ID_DATA, reinterpret_cast<char*>(_msg_addres), 4);   // proc is procedure
    if (msg.write(msg)){
        proc();
    }
}
void doNothing(){

}
void receiveData(int ID_DATA, int* var){                               //procedure receive data
    if (msg.id == ID_DATA){
        *var = *reinterpret_cast<int*>(msg.data);
    }
}


void sendKpKiKd(float Kp, float Ki, float Kd){
    sendDataToSlave(ID_DATA_Kp, Kp, doNothing);
    wait(WAIT_SEND);
    sendDataToSlave(ID_DATA_Ki, Ki, doNothing);
    wait(WAIT_SEND);
    sendDataToSlave(ID_DATA_Kd, Kd, doNothing);
}
