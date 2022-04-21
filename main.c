#include "battery.h"
#include "I2C.h"

char *i2c_bus = "/dev/i2c-0";

unsigned char data[32];
unsigned char * p_data;
unsigned char ret_value;


int main()
{
        if(i2c_init(i2c_bus)<0)
        {
                return -1;
        }

        p_data = read_flash_block(0x30,0x00);
        for(uint8_t i=0 ; i< 32; i++)
        {
                printf("i= %d : %x\n",i,*(p_data+i));
        }

        i2c_close(i2c_bus);
        return 0;
}
