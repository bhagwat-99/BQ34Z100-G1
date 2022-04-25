#include "battery.h"
#include "I2C.h"


// for apalis imx8 
char *i2c_bus = "/dev/apalis-i2c1";

//for colibri imx6uul
//char *i2c_bus = "/dev/i2c-0";

uint16_t delay = 2;

uint8_t * p_ret_value;
uint8_t data[32];

int main()
{
        if (i2c_init(i2c_bus) < 0)
        {
                return -1;
        }

        gauge_unseal();
        gauge_full_access();

       p_ret_value = read_flash_block(0x40,0x07);

       for(uint8_t i=0 ; i<32; i++)
       {
               printf("%d : %x \n",i,*(p_ret_value+i));
       }

        i2c_close(i2c_bus);
        return 0;
}
