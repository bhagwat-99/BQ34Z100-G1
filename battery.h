#ifndef BATTERY_H
#define BATTERY_

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

#endif