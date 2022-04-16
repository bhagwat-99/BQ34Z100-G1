#include "battery.h"


unsigned char slave_address = 0x55;

//register address to read and write data
unsigned char reg;

//array to save data to be written
unsigned char data_to_write[32];

//character pointer for access data read
unsigned char *data_to_read;

/*
Check sum calculation
The check sum is the sum of all 32 data bytes within the current data block, truncated to 8-bits and
complemented. Example code to calculate the check sum follows:
pData = a pointer to the data that was changed in the data block.
nLength = length of the data block.

*/
unsigned char checksum(unsigned char *pData, unsigned char nLength)
{
unsigned char nSum = 0x00;
unsigned char n;
for (n = 0; n <nLength; n++)
nSum += pData[n];
nSum = 0xFF - nSum;
return nSum;
}
int main()
{
        if(i2c_init()<0)
        {
                return -1;
        }

        // //unseal the gauge
        // reg =0x00;
        // data_to_write[0]=0x14;//lsb
        // data_to_write[1]=0x04;//msb
        // i2c_write(slave_address,reg, data_to_write,0x02);
        // sleep(0.5);

        // reg =0x00;
        // data_to_write[0]=0x72;//lsb
        // data_to_write[1]=0x36;//msb
        // i2c_write(slave_address,reg, data_to_write,0x02);
        // sleep(0.5);

        // //full access the gauge
        // unsigned char i=0;
        // for(i=0;i<3;i++)
        // {
        //         reg =0x00;
        //         data_to_write[0]=0xFF;//lsb
        //         data_to_write[1]=0xFF;//msb
        //         i2c_write(slave_address,reg, data_to_write,0x02);
        //         sleep(0.2);
        // }


        // //enable block data control - 0x00 to 0x61
        // reg =0x61;
        // data_to_write[0]=0x00;
        // i2c_write(slave_address,reg, data_to_write,0x01);
        // sleep(0.2);

        // //write subclass to 0x3E
        // reg = 0x3E;
        // data_to_write[0] = 0x68;
        // i2c_write(slave_address,reg, data_to_write,0x01);
        // sleep(0.2);

        // //write offset to 0x3F
        // reg = 0x3F;
        // data_to_write[0] = 0x00;
        // i2c_write(slave_address,reg, data_to_write,0x01);
        // sleep(0.2);

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
