/***************************************************************************
 * Title      : Program Debug TFMini
 * Name       : TFMini.cpp
 * Version    : 1.0
 * Author     : Gian Arjuna EL 16
 * Date       : 11 Januari 2020
 * Description:
 *
 *
 ***************************************************************************/

 
/* LIBRARY */
#include "mbed.h"
#include "TFMini/TFMini.h"
 
/* MODE DEBUG YANG DIINGINKAN */ 
#define AMBIL_TOF

/* pin assignment lainnya */
DigitalIn mybutton(USER_BUTTON); 

/* komunikasi serial dengan UART */
Serial pc(USBTX, USBRX, 115200); 

/* komunikasi i2c */
I2C tfmini(PB_3, PB_10);

/* timer untuk mendapatkan waktu */
Timer timer1;
Timer profiler1;
Ticker ticker1;

unsigned long samp, start, endtime, diff;

uint8_t trigger_done, mode_tf;
uint16_t dist_HL, strength_HL;;
char addr = 0x10 << 1;
char receive_buffer[7];

/* sampai dapat data butuh 1130 us */
void TFCallback(int event)
{
    trigger_done = receive_buffer[0];
    dist_HL = (receive_buffer[3]) << 8 | (receive_buffer[2]);
    strength_HL = (receive_buffer[5]) << 8 | (receive_buffer[4]);
    mode_tf = receive_buffer[6];
    
    endtime = profiler1.read_us();
    diff = endtime - start;

    pc.printf("done : %u  dist : %u  str : %u  mode : %u  t : %lu\n", trigger_done, dist_HL, strength_HL, mode_tf, diff);

}

/* butuh 9 us*/
void initTFAsync()
{  
    start = profiler1.read_us();
    event_callback_t dataTFEvent = TFCallback;
    tfmini.transfer(addr, READ_CMD, READ_CMD_LENGTH, receive_buffer, RECEIVE_BUFFER_LENGTH, dataTFEvent);  
}

int main() {
    /* ================================================================== */
    #ifdef AMBIL_TOF 
        timer1.start();
        profiler1.start();
        tfmini.frequency(100000);
        ticker1.attach_us(&initTFAsync, 100000);  // 10 hz

        while(1)
        {          
            wait(10);
        }
    #endif
} 