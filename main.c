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

        //reading voltage mV
        reg = 0x08; //0x09
        data_to_read = i2c_read(slave_address,reg,2);
        printf("%x\n",*data_to_read);
        printf("%x\n",*(data_to_read+1));

        //remaining capacity
        
        //full charge capacity

        //average current 0x0A/0x0B mA

        //temperature measurement 0x0C/0x0D 0.1k

        //control 0x00/0x01

        //state of charge 0x02/0x03 %

        //RemainingCapacity()  0x04/0x05 mAh

        //FullChargeCapacity()  0x06/0x07 mah

        






        i2c_close();
        return 0;
}
