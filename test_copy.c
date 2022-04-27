#include "I2C.h"

typedef unsigned short uint16_t;
typedef unsigned char uint8_t;

#define wait_time 0.2
#define SLAVE_ADDR 0x55

// for apalis imx8 
char *i2c_bus = "/dev/apalis-i2c1";

//for colibri imx6uul
//char *i2c_bus = "/dev/i2c-0";

uint16_t delay = 2;

uint8_t * p_ret_value;
uint8_t data[32];


uint8_t checksum()
{
         //checksum
        uint8_t checksum_value = 0x00;

        for (uint8_t n = 0; n < 32; n++)
        {
            checksum_value += data[n];
        }

        checksum_value = 0xFF - checksum_value;
        return checksum_value;
        
}

int main()
{
        if (i2c_init(i2c_bus) < 0)
        {
                return -1;
        }

        uint8_t reg_addr = 0x40;
        uint8_t n_bytes = 32;

        p_ret_value = i2c_read(SLAVE_ADDR, reg_addr, n_bytes);

        for(uint8_t i=0 ; i<32; i++)
        {
                printf("%d : %x \n",i,*(p_ret_value+i));
                data[i]= *(p_ret_value+i);
        }

        printf("checksum value : %x\n",checksum());

        // data[11] = 0x4e;
        // data[12] = 0x20;
        // printf("checksum value new : %x\n",checksum());

        // reg_addr = 0x40;
        // n_bytes = 32;
        // i2c_write(SLAVE_ADDR,reg_addr,data, n_bytes);

        i2c_close(i2c_bus);
        return 0;
}
