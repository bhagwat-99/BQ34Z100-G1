#include <stdio.h>
#include "I2C.h"


// i2c bus file descriptor
int fd_i2c = -1;

//char array to store read value
unsigned char i2c_data[35];


// i2c initialize
//i2c_bus - i2c adapter
int i2c_init(char *i2c_bus)
{
    if ((fd_i2c = open(i2c_bus, O_RDWR)) < 0)
    {
        printf("Failed to open %s\n.",i2c_bus);
        return -1;
    }
    return 0;
}



// close the i2c bus
//i2c_bus - i2c adaper 
int i2c_close(char *i2c_bus) 
{
    if (close(fd_i2c) < 0)
    {
        printf("Failed to close %s\n.",i2c_bus);
        return -1;
    }
    return 0;
}





// Write to an I2C slave device's register:
int i2c_write(unsigned char slave_addr, unsigned char reg, unsigned char *data, unsigned char NBytes)
{

    struct i2c_msg msgs[1];
    struct i2c_rdwr_ioctl_data msgset[1];

    i2c_data[0] = reg;

    for(int i=1;i<=NBytes;i++)
    {
        i2c_data[i] = *(data + (i-1));
    }

    msgs[0].addr = slave_addr;
    msgs[0].flags = 0;// 0 for write 
    msgs[0].len = NBytes+1;
    msgs[0].buf = i2c_data;

    msgset[0].msgs = msgs;
    msgset[0].nmsgs = 1;

    if (ioctl(fd_i2c, I2C_RDWR, &msgset) < 0)
    {
        perror("ioctl(I2C_RDWR) in i2c_write");
        return -1;
    }

    return 0;
}


// Read the given I2C slave device's register and return the read value in `*result`:
unsigned char * i2c_read(unsigned char slave_addr, unsigned char reg, unsigned char NBytes) 
{
    
    i2c_data[0]=reg;


    struct i2c_msg msgs[2];
    struct i2c_rdwr_ioctl_data msgset[1];

    msgs[0].addr = slave_addr;
    msgs[0].flags = 0;
    msgs[0].len = 1;
    msgs[0].buf = i2c_data;

    msgs[1].addr = slave_addr;
    msgs[1].flags = I2C_M_RD;
    msgs[1].len = NBytes;
    msgs[1].buf = i2c_data;

    msgset[0].msgs = msgs;
    msgset[0].nmsgs = 2;

    if (ioctl(fd_i2c, I2C_RDWR, &msgset) < 0)
    {
        perror("ioctl(I2C_RDWR) in i2c_read");
        return NULL;
    }
    
    return i2c_data;
}