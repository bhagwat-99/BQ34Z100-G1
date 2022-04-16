#include "battery.h"


unsigned char slave_address = 0x55;

//register address to read and write data
unsigned char reg;

//array to save data to be written
unsigned char data_to_write[32];

//character pointer for access data read
unsigned char *data_to_read;


int main()
{
        if(i2c_init()<0)
        {
                return -1;
        }

       

        

        
        
        // //read 32 byte data from 0x40 to 0x5F
        // // 0x20 = 32 in decimal - read 32 bytes
        // reg = 0x40;
        // data_to_read = i2c_read(slave_address, reg, 0x20);
        // sleep(0.5);
        
        // i = 0;
        // for(i=0;i<32;i++)
        // {
        //         data_to_write[i] = *(data_to_read+i);
        // }

        // unsigned char j = 0;
        // for(j=0;j<32;j++)
        // {
        //         printf("%x\n",data_to_write[j]);
        // }
        // printf("modified data\n");
        // data_to_write[14]=0x91;//msb
        // data_to_write[15]=0xf4;//lsb
        // // data_to_write[13]=0x62;//msb
        // // data_to_write[14]=0x70;//lsb
        // // data_to_write[30]=0x0a;//lsb
        // j = 0;
        // for(j=0;j<32;j++)
        // {
        //         printf("%x\n",data_to_write[j]);
        // }

        // //calculate checksum of new data
        // unsigned char checksum_value;
        // checksum_value = checksum(data_to_write,0x20);
        // printf("checksum_value : %x\n",checksum_value);

        // // write 32 byte data from 0x40 to 0x5F
        // // 0x20 = 32 in decimal - write 32 bytes
        // reg = 0x40;
        // i2c_write(slave_address, reg,data_to_write ,0x20);
        // sleep(0.5);

        // // //write checksum to 0x60
        // reg = 0x60;
        // data_to_write[0] = checksum_value;
        // i2c_write(slave_address,reg, data_to_write,0x01);
        // sleep(0.5);

        // // // printf("reseting the guage");
        // // // // //reset the guage
        // // // reg = 0x00;
        // // // data_to_write[0] =0x41;
        // // // data_to_write[1] =0x00;
        // // // i2c_write(slave_address,reg, data_to_write,0x01);
        // // // sleep(0.5);


        //read current value;
        reg = 0x10;
        data_to_read = i2c_read(slave_address, reg, 0x02);
        int16_t current ;
        current = *(data_to_read + 1) << 8 | *data_to_read;
        printf("current : %d mA\n",current);


        i2c_close();
        return 0;
}
