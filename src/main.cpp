/***************************************************************************
 * Title      : Program Debug Motor dan Encoder
 * Name       : debug_motor.cpp
 * Version    : 1.0
 * Author     : Gian Arjuna EL 16
 * Date       : 12 Desember 2019
 * Description:
 *
 *
 ***************************************************************************/

 
/* LIBRARY */
#include "mbed.h"
#include "encoderKRAI.h"
#include "Motor.h"
 
/* MODE DEBUG YANG DIINGINKAN */ 
#define AMBIL_TOF

/* pin assignment lainnya */
DigitalIn mybutton(USER_BUTTON); 

/* komunikasi serial dengan UART */
Serial pc(USBTX, USBRX, 115200); 

/* komunikasi i2c */
I2C tfmini(PC_9, PA_8);

/* timer untuk mendapatkan waktu */
Timer timer1;
Ticker ticker1;

unsigned long samp;

uint8_t trigger_done, mode;
uint16_t dist_HL, strength_HL;;
uint8_t addr = 0x10;

char receive_buffer[7];


void readTF()
{
    tfmini.read(addr, receive_buffer, 7);  

    mode = receive_buffer[0];
    strength_HL = (receive_buffer[1]) << 8 | (receive_buffer[2]);
    dist_HL = (receive_buffer[3]) << 8 | (receive_buffer[4]);
    trigger_done = receive_buffer[6];
}


int main() {
    /* ================================================================== */
    #ifdef AMBIL_TOF 
        timer1.start();
        // ticker1.attach_us(&readTF, 100000);  // 10 hz
        while(1)
        {          
            // pc.printf("done : %u  dist : %u  str : %u  mode : %u\n", trigger_done, dist_HL, strength_HL, mode);
            if (timer1.read_us()-samp >= 10000)
            {
                readTF();    
                pc.printf("done : %u  dist : %u  str : %u  mode : %u\n", trigger_done, dist_HL, strength_HL, mode);
                samp = timer1.read_us(); 
            }     
        }
    #endif

} 














