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


#define RAM_FILE_PATH   "/tmp/battery_parameters"
#define FILE_MODE       "w"

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


int gauge_seal();

static int gauge_unseal();

static int gauge_full_access();

int gauge_unlock();

int reset_guage();



//autocalibration fuctions

static int board_offset();

static int cc_offset();

static int offset_calibration();

static int enable_calibration();

static int enter_calibration();

static int calibration_exit();

static int it_enable();

int autocalibrate();


static int enable_block_data_control();

static int read_control(uint16_t control_subcommand);

static uint8_t checksum(uint8_t * checksum_data);

static unsigned char * read_flash_block(uint8_t sub_class, uint8_t offset);

static int write_flash_block(uint8_t sub_class, uint8_t offset, uint8_t * data);

static int readVDivider();

static int read_pack_configuration();

static int read_voltsel();

static int read_design_capacity();

static int read_design_energy();

static int read_flash_update_ok_voltage();

static int read_series_cell();

static int read_design_energy_scale();

static int set_vdivider(uint16_t v_divider);

static int set_series_cell(uint16_t series_cell);

static int set_design_capacity(uint16_t design_capacity);

static int set_design_energy_scale(uint16_t design_energy_scale);

static int set_design_energy(uint16_t design_energy);

static int set_voltsel(uint16_t dummy_value);

static int set_flash_update_ok_voltage(uint16_t flash_update_ok_voltage);

int gauge_verify_and_calibrate();

static int verify_calibrate_func(int (*read_func)(), int (*set_func)(uint16_t),uint16_t value);

static void failed_to_calibrate(uint16_t value);

static int calibrated_data();

uint8_t soc();



int control_status();

int device_type();


//battery parameter fuctions

int internal_temperature();

int temperature();

int voltage();

int current();

int average_current();

//int gauge_parameters();

int write_to_file();



#endif