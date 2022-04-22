#ifndef BATTERY_H
#define BATTERY_H

#include <stdio.h> // printf()
#include <sys/types.h> // open()
#include <sys/stat.h> // open()
#include <fcntl.h> // open()
#include <sys/ioctl.h> // ioctl()
#include <errno.h> // errno
#include <string.h> // strerror()
#include <unistd.h> // close()
#include <linux/i2c-dev.h> // struct i2c_msg
#include <linux/i2c.h> // struct i2c_rdwr_ioctl_data

#include "I2C.h"

//device address
#define SLAVE_ADDR              0x55

//commands
#define CONTROL                 0x00
#define STATE_OF_CHARGE         0x02
#define MAX_ERROR               0x03
#define REMAINING_CAPACITY      0x04
#define FULL_CHARGE_CAPACITY    0x06
#define VOLTAGE                 0x08
#define AVERAGE_CURRENT         0x0A
#define TEMPERATURE             0x0C
#define CURRENT                 0x10

//extended data commands
#define AVAILABLE_ENERGY        0x24    //10mW/h
#define AVERAGE_POWER           0X26    //10mW
#define INTERNAL_TEMPERATURE    0x2A    //0.1k
#define PACK_CONFIGURATION      0x3A
#define DESIGN_CAPACITY         0x3C    //0X3C



//control subcommands
#define CONTROL_STATUS          0x0000 //return the status of key features
#define DEVICE_TYPE             0x0001 //return the device type of 0x100(indicate BQ34Z100-G1)
#define CHEM_ID                 0x0008 // return the chemID value
#define BOARD_OFFSET            0X0009 // force the device to measure and store board offset
#define CC_OFFSET               0x000A // force the device to measure the internal cc offset
#define CC_OFFSET_SAVE          0x000B // force the device to store the internal cc offset
#define SEALED                  0x0020 // places the device in sealed mode
#define IT_ENABLE               0x0021 // enable the impedance track algorith
#define CAL_ENABLE              0x002D // toggle calibration mode enable
#define RESET                   0x0041 // reset the fuel guage
#define ENTER_CAL               0x0081 // enter the calibration mode
#define EXIT_CAL                0x0080 // exit the calibration mode
#define OFFSET_CAL              0x0082 // report internal cc offset in calibration mode



typedef unsigned char uint8_t;
typedef unsigned short uint16_t;


void gauge_seal();

void gauge_unseal();

void gauge_full_access();

void reset_guage();

uint16_t board_offset();

uint16_t cc_offset();

uint16_t board_offset();

void offset_calibration();

void enable_block_data_control();

uint16_t read_control(uint16_t control_subcommand);

uint8_t checksum();

unsigned char * read_flash_block(uint8_t sub_class, uint8_t offset);

uint8_t write_flash_block(uint8_t sub_class, uint8_t offset);

uint16_t readVDivider();

uint16_t read_pack_configuration();

uint8_t read_voltsel()

uint16_t read_design_capacity();

uint16_t read_design_energy();

uint16_t read_flash_update_ok_voltage();

uint8_t read_series_cell();

uint8_t read_design_energy_scale();

void set_vdivider(uint16_t v_divider);

void set_series_cell(uint8_t series_cell);

void set_design_capacity(uint16_t design_capacity);

void set_design_energy_scale(uint8_t design_energy_scale);

void set_design_energy(uint16_t design_energy);

void set_voltsel();

void set_flash_update_ok_voltage(uint16_t flash_update_ok_voltage);

uint8_t soc();

uint16_t enable_calibration();

uint16_t enter_calibration();

uint16_t calibration_exit();

uint16_t it_enable();

uint16_t control_status();

uint16_t device_type();


//battery parameter fuctions

float internal_temperature();

float temperature();

uint16_t voltage();

int16_t current();

int16_t average_current();


#endif