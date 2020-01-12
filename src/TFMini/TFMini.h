#ifndef TFMINI_H
#define TFMINI_H

#define SHORT_MODE_TFMINI 0x00
#define MIDDLE_MODE_TFMINI 0x03
#define LONG_MODE_TFMINI 0x07

#define BYTE_LENGTH_8 0x01
#define BYTE_LENGTH_16 0x02
#define DATA_LENGTH_TFMINI 0x07

#define MM_UNIT_TFMINI 0x00
#define CM_UNIT_TFMINI 0x01

#define MAX_SAMPLING_MS_TFMINI 10

#define H_SLAVE_REG_TFMINI 0x00
#define L_SLAVE_REG_TFMINI 0x26

#define H_GEAR_MODE_REG_TFMINI 0x00
#define L_GEAR_MODE_REG_TFMINI 0x50

#define H_UNIT_REG_TFMINI 0x00
#define L_UNIT_REG_TFMINI 0x66

#define H_RESET_REG_TFMINI 0x00
#define L_RESET_REG_TFMINI 0x70

#define H_DATA_REG_TFMINI 0x01
#define L_DATA_REG_TFMINI 0x02
#define READ_CMD_LENGTH 3
#define RECEIVE_BUFFER_LENGTH 7

char READ_CMD[3] = {H_DATA_REG_TFMINI, L_DATA_REG_TFMINI, DATA_LENGTH_TFMINI};


void getDataTF(I2C *tfmini, char addr, uint8_t *trigger_done, uint16_t *dist_HL, uint16_t *strength_HL, uint8_t *mode);

void setGearMode(I2C *tfmini, char addr, char mode);

void setUnitTF(I2C *tfmini, char addr, char unit);

void resetConfigTF(I2C *tfmini, char addr);

void setSlaveAddress(I2C *tfmini, char addr_old, char addr_new); 

void writeConfigTF(I2C *tfmini, char addr, char *cmd_buffer, char val);

#endif