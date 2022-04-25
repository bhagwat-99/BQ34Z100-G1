#include "battery.h"
#include "I2C.h"


// for apalis imx8 
char *i2c_bus = "/dev/apalis-i2c1";

//for colibri imx6uul
//char *i2c_bus = "/dev/i2c-0";

uint8_t * p_return_value;


int main()
{
        if (i2c_init(i2c_bus) < 0)
        {
                return -1;
        }

        // unseal the guage
        gauge_unseal();
        printf("unseal done\n");
        printf("%x\n", control_status());

        // full access of guage
        gauge_full_access();
        printf("full access done\n");
        printf("%x\n", control_status());

        p_return_value = read_flash_block(0x68, 0x0e);

        for(uint8_t i=0; i<32;i++)
        {
                printf("%d : %x \n",i,*(p_return_value+i));
        }


        i2c_close(i2c_bus);

}
