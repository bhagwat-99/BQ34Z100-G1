#include "i2c_custom.h"


unsigned char slave_address = 0x55;

//register address to read and write data
unsigned char reg;

//array to save data to be written
unsigned char data_to_write[2];

//character pointer for access data read
unsigned char *data_to_read;


int main()
{
        if(i2c_init()<0)
        {
                return -1;
        }

        //reading voltage
        reg = 0x08; 
        data_to_read = i2c_read(slave_address,reg,2);
        printf("%x\n",*data_to_read);
        printf("%x\n",*(data_to_read+1));

        i2c_close();
        return 0;
}
