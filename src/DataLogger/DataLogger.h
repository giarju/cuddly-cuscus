/**
 *  Header DataLogger KRAI
 *  untuk memasukkan data ke dalam SD card
 **/
 
#ifndef DATALOGGER_H
#define DATALOGGER_H

//Library
#include "mbed.h"
#include "SDFileSystem.h"

/* Penggunaan Library 

 sd.mount()         untuk mengakses SD Card
 sd.unmount()       untuk menyelesaikan akses
 
 FILE *fp = fopen("/sd/namafile.csv", "ket");
    untuk mengakses file
    ket = r/w/a (read / write / append )
    read untuk membaca file
    write untuk menuliskan data ke dalam file
    append untuk menuliskan data ke dalam file ( melanjutkan yang sudah ada )
    
 fclose()
    untuk menyelesaikan akses ke file
    
 fprintf (fp, " print ");
    untuk meenuliskan string ke file
    
 c = fgetc(fp)
    untuk membaca file per karakter 

*/

/******************************
 * DATA LOGGER KRAI Interface *
 ******************************/
 
// class dataloggerKRAI {
    
// public:
//     void printLog (int x, y, v, teta);
    
// };

#endif /*DATALOGGER_H*/