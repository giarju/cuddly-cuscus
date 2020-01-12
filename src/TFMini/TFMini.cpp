#include "mbed.h"
#include "TFMini.h"

/* butuh 1130 us*/
void getDataTF(I2C *tfmini, char addr, uint8_t *trigger_done, uint16_t *dist_HL, uint16_t *strength_HL, uint8_t *mode)
{
    char cmd_buffer[3] = {H_DATA_REG_TFMINI, L_DATA_REG_TFMINI, DATA_LENGTH_TFMINI};
    char receive_buffer[7];

    tfmini -> write(addr, cmd_buffer, 3, true);
    tfmini -> read(addr, receive_buffer, 7);  

    *trigger_done = receive_buffer[0];
    *dist_HL = (receive_buffer[3]) << 8 | (receive_buffer[2]);
    *strength_HL = (receive_buffer[5]) << 8 | (receive_buffer[4]);
    *mode = receive_buffer[6];
    
    /* to do 
     * 1. interrupt based communication
     * 2. reset if for certain time data has not change
     * 3. ignore data if done is 0
     * 3.2 check for strength if data is valid or not
     * 3.5 get accuracy precision
     */
}


void setGearMode(I2C *tfmini, char addr, char mode) //untested
{
    char cmd_buffer[3] = {H_GEAR_MODE_REG_TFMINI, L_GEAR_MODE_REG_TFMINI, BYTE_LENGTH_16};

    writeConfigTF(tfmini, addr, cmd_buffer, mode);
    wait_ms(500);
}

void setUnitTF(I2C *tfmini, char addr, char unit) 
{
    char cmd_buffer[3] = {H_UNIT_REG_TFMINI, L_UNIT_REG_TFMINI, BYTE_LENGTH_16};
    
    writeConfigTF(tfmini, addr, cmd_buffer, unit);
    wait_ms(500);
}


void resetConfigTF(I2C *tfmini, char addr)
{
    char cmd_buffer[3] = {H_RESET_REG_TFMINI, L_RESET_REG_TFMINI, BYTE_LENGTH_16};

    writeConfigTF(tfmini, addr, cmd_buffer, 0x02);
    wait_ms(500);
}

void setSlaveAddress(I2C *tfmini, char addr_old, char addr_new) //untested
{
    addr_new = addr_new << 1;
    char cmd_buffer[3] = {H_SLAVE_REG_TFMINI, L_SLAVE_REG_TFMINI, BYTE_LENGTH_16};

    writeConfigTF(tfmini, addr_old, cmd_buffer, addr_new);
    wait_ms(500);
}

void writeConfigTF(I2C *tfmini, char addr, char *cmd_buffer, char val)
{
    char data[2];

    tfmini -> write(addr, cmd_buffer, 3, true);
    tfmini -> read(addr, data, 2);
    data[0] = val;

    tfmini -> start();
    tfmini -> write(addr);
    tfmini -> write(cmd_buffer[0]);
    tfmini -> write(cmd_buffer[1]);
    tfmini -> write(cmd_buffer[2]);
    
    tfmini -> start();
    tfmini -> write(addr);
    tfmini -> write(data[0]);
    tfmini -> write(data[1]);
    tfmini -> stop();
}