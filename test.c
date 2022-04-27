#include "battery.h"
#include "I2C.h"


// for apalis imx8 
char *i2c_bus = "/dev/apalis-i2c1";

//for colibri imx6uul
//char *i2c_bus = "/dev/i2c-0";

uint8_t * p_ret_val;
uint8_t data[32];

int main()
{
        // start the i2c bus
        if(i2c_init(i2c_bus)<0)
        {
                return -1;
        }

        p_ret_val = read_flash_block(0x68, 0x00);

        for(uint8_t i=0; i<32;i++)
        {
                //data[i] = *(p_ret_val+i);
                printf("%d : %x \n",i,*(p_ret_val+i));
        }
        // data[0] = 0x7f;
        // data[1] = 0x71;
        // data[2] = 0x20;
        // data[3] = 0x5c;
        // data[4] = 0x94;
        // data[5] = 0x08;
        // data[6] = 0x98;
        // data[7] = 0xc0;
        // data[8] = 0xfa;
        // data[9] = 0x4a;
        // data[10] = 0x00;
        // data[11] = 0x00;
        // data[12] = 0x00;
        // data[13] = 0x00;
        // data[14] = 0x91;
        // data[15] = 0xf4;
        // data[16] = 0x00;
        // data[17] = 0x00;
        // data[18] = 0x00;
        // data[19] = 0x00;
        // data[20] = 0x00;
        // data[21] = 0x00;
        // data[22] = 0x00;
        // data[23] = 0x00;
        // data[24] = 0x00;
        // data[25] = 0x00;
        // data[26] = 0x00;
        // data[27] = 0x00;
        // data[28] = 0x00;
        // data[29] = 0x00;
        // data[30] = 0x00;
        // data[31] = 0x00;

        // write_flash_block(0x68, 0x00,data);

        // close the i2c bus
        i2c_close(i2c_bus);
        return 0;
}
