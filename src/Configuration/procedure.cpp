/***************************************************************************
 * Title      : PROCEDURE ROBOT PR
 * Author     : KRAI ITB 2020
 * Description: 
 * 
 * prosedur dan fungsi yang digunakan pada main program
 * 
 ***************************************************************************/

/* 
 * prosedur untuk melakukan sampling odometri base
 * 
 * */
void odometrySamp ()  /*butuh 48018 us */  
{  
    /* update posisi robot berdasarkan odometri */
    Odometry.updatePosition();                                     
}

/* 
 * prosedur untuk melakukan sampling encoder motor base
 * 
 * */
void encoderMotorSamp()  /* butuh 8 us */
{
    /* ukur kecepatan motor base dalam m/s */
    a_motor_speed = (float)A_enc.getPulses()*2*PI/ENC_MOTOR_PULSE/ENC_MOTOR_SAMP*US_TO_S;
    b_motor_speed = (float)B_enc.getPulses()*2*PI/ENC_MOTOR_PULSE/ENC_MOTOR_SAMP*US_TO_S;
    c_motor_speed = (float)C_enc.getPulses()*2*PI/ENC_MOTOR_PULSE/ENC_MOTOR_SAMP*US_TO_S;
    d_motor_speed = (float)D_enc.getPulses()*2*PI/ENC_MOTOR_PULSE/ENC_MOTOR_SAMP*US_TO_S;
    
    /* reset nilai encoder */
    A_enc.reset();
    B_enc.reset();
    C_enc.reset();
    D_enc.reset();
}

/* 
 * prosedur untuk melakukan sampling pwm motor base
 * 
 * */
void motorSamp()
{
    /* menggerakan motor base */
    A_motor.speed(A_pwm);
    B_motor.speed(B_pwm);
    C_motor.speed(C_pwm); 
    D_motor.speed(D_pwm);
}

/* 
 * prosedur untuk menghitung pid motor base
 * 
 * */
void pidMotorSamp()
{
    /* menghitung pid motor base */
    A_pwm = A_pid_motor.createpwm(a_target_speed, a_motor_speed);
    B_pwm = B_pid_motor.createpwm(b_target_speed, b_motor_speed);
    C_pwm = C_pid_motor.createpwm(c_target_speed, c_motor_speed);
    D_pwm = D_pid_motor.createpwm(d_target_speed, d_motor_speed);
}

/* 
 * prosedur untuk melakukan tracking path
 * 
 * */
void trackingSamp()
{
    /* menghitung kecepatan robot berdasarkan map dan posisi aktual*/
    base_speed = velocityTracker(map[index_traject], Odometry.position); /* index harusnya dari fsm (index bahaya, shared variable sama fsm)*/
    /* menghitung kecepatan masing2 motor base */
    base4Omni(base_speed, &a_target_speed, &b_target_speed, &c_target_speed, &d_target_speed);
}

/* 
 * prosedur untuk print string dengan uart setiap sampling time
 * 
 * */
void pcSerialSamp() /*1200 us untuk 16 karakter pc.printf*/ /* 20 us dgn attach*/
{
    /* write string ke buffer 1 (str_buffer)*/     
    sprintf(str_buffer, "bbbbbbbbbbbbbbbbbb\n");
    /* mengirimkan string melalui uart */
    sendUart(str_buffer);      
}

/* 
 * prosedur untuk mengirimkan data melalui uart dengan double buffer
 * 
 * */
void sendUart(char *buffer)
{
    /*lock critical section */
    uart_mutex.lock();
    /* copy buffer 1 ke buffer 2 untuk dikirimkan ke hardware uart */
    strcpy(uart_buffer,buffer);
    uart_pointer = uart_buffer;
    uart_buff_len = strlen((const char*) uart_buffer);
    /* mengirimkan karakter setiap hardware uart kosong */
    pc.attach(&writeUart, pc.TxIrq); 
}

/* 
 * prosedur untuk menulis data ke hardware uart
 * 
 * */
void writeUart() /* butuh 4 us */
{
    CriticalSectionLock::enable();
    /* selama masih ada karakter dalam buffer uart */
    if (uart_buff_len >= 0)
    {
        /* kirim karakter yang ditunjuk uart_pointer */
        pc.putc(*uart_pointer);
        /* merujuk ke karakter selanjutnya */
        uart_pointer++;
        /* panjang karakter yang belum dikirimkan berkuran 1 */
        uart_buff_len--;
    }
    /* buffer sudah kosong, seluruh karakter sudah dikirim */
    else 
    {
        /* lepas function writeUart dari callback */
        pc.attach(0, pc.TxIrq);
        /* mengembalikan state awal */
        uart_pointer = uart_buffer;
        uart_mutex.unlock();
    }
    CriticalSectionLock::disable();
}