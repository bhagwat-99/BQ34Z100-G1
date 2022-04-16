#include "battery.h"
#include "I2C.h"



//array to store the read and write data
uint8_t reg_addr;
uint8_t data_read[32];
uint8_t data_write[32];
uint8_t return_value;
uint8_t * p_return_value;
uint8_t n_bytes;//number of bytes to read and write


void gauge_seal()
{

}

//unseal the gauge before read/write
void gauge_unseal()
{
        reg_addr = 0x00;
        n_bytes = 0x02;
        data_write[0]=0x14;//lsb
        data_write[1]=0x04;//msb
        return_value = i2c_write(SLAVE_ADDR,reg_addr, data_write,n_bytes);
        if (return_value < 0 )
        {
            perror("gauge seal failed");
        }
        sleep(0.2);

        reg_addr =0x00;
        data_write[0]=0x72;//lsb
        data_write[1]=0x36;//msb
        return_value = i2c_write(SLAVE_ADDR,reg_addr, data_write,0x02);
        if (return_value < 0 )
        {
            perror("gauge seal failed");
        }
        sleep(0.2);
}

//full access mode of gauge. gauge must be in unsealed mode to full access mode
void gauge_full_access()
{
    unsigned char i=0;
    for(i=0;i<3;i++)
    {
        unsigned char reg_addr =0x00;
        data_write[0]=0x72;//lsb
        data_write[1]=0x36;//msb
        return_value = i2c_write(SLAVE_ADDR,reg_addr, data_write,0x02);
        if (return_value < 0 )
        {
            perror("full access failed");
        }
        sleep(0.2);
    }

}


//enable accessing block data
void enable_block_data_control()
{
    unsigned char reg_addr =0x61;
    data_write[0]=0x00;
 
    return_value = i2c_write(SLAVE_ADDR,reg_addr, data_write,0x01);
    if (return_value < 0 )
    {
        perror("block data enable failed");
    }
    sleep(0.2);
}

//read control command
uint8_t * read_control(uint16_t control_subcommand)
{
    n_bytes = 2;
    data_write[0] =  control_subcommand & 0x00FF; //lsb
    data_write[1] = control_subcommand >> 8; //msb
    return_value = i2c_write(SLAVE_ADDR, CONTROL, data_write, n_bytes);
    if (return_value < 0 )
    {
        perror("read control write command failed");
        return -1;
    }
    sleep(0.001);
    p_return_value = i2c_read(SLAVE_ADDR, CONTROL, n_bytes);
    if (p_return_value < 0 )
    {
        perror("read control failed");
        return -1;
    }
    return p_return_value;
    
}


//write control command
uint8_t write_control(uint16_t control_subcommand)
{
    n_bytes = 2;
    data_write[0] =  control_subcommand & 0x00FF; //lsb
    data_write[1] = control_subcommand >> 8; //msb
    return_value = i2c_write(SLAVE_ADDR, CONTROL, data_write, n_bytes);
    if (return_value < 0 )
    {
        perror("write control failed");
        return -1;
    }
    sleep(0.001);
    return 0;
       
}

/*
Check sum calculation
The check sum is the sum of all 32 data bytes within the current data block, truncated to 8-bits and complemented. 
pData = a pointer to the data that was changed in the data block.
nLength = length of the data block.

*/
//unsigned char checksum(unsigned char *pData, unsigned char nLength)
uint8_t checksum()
{
        unsigned char checksum_value = 0x00;
        unsigned char n;
        for (n = 0; n < 32; n++)
        checksum_value += data_write[n];
        checksum_value = 0xFF - checksum_value;

        //write checksum to address 0x60
        reg_addr = 0x60;
        n_bytes = 0x01;
        data_write[0]=checksum_value;
        return_value = i2c_write(SLAVE_ADDR,reg_addr, data_write,n_bytes);
        if (return_value < 0 )
        {
            perror("failed to write checksum");
            return -1;
        }
        sleep(0.2);
        return 0;

}


//read flash block
uint8_t * read_flash_block(uint8_t sub_class, uint8_t offset)
{
    //enable block data
    enable_block_data_control();
    
    //write subclass id and offset to 0x3e and 0x3f register
    reg_addr = 0x3e;
    n_bytes = 2;
    data_write[0] =  sub_class ; //lsb
    data_write[1] =  offset/32 ; //msb
    i2c_write(SLAVE_ADDR, CONTROL, data_write, n_bytes);
    sleep(0.001);
    
    //read data from 0x40 address
    reg_addr = 0x40;
    n_bytes = 32;
    return i2c_read(SLAVE_ADDR, reg_addr, n_bytes);
}

uint8_t write_flash_block(uint8_t sub_class, uint8_t offset) {


    //write_reg(0x61, 0x00); // Block control
    enable_block_data_control();

    //write subclass id and offset to 0x3e and 0x3f register
    reg_addr = 0x3e;
    n_bytes = 2;
    data_write[0] =  sub_class ; //lsb
    data_write[1] =  offset/32 ; //msb
    i2c_write(SLAVE_ADDR, CONTROL, data_write, n_bytes);
    sleep(0.001);


    //write data to 0x40 address
    reg_addr = 0x40;
    n_bytes = 32;
    i2c_write(SLAVE_ADDR, reg_addr, data_write, n_bytes);
    sleep(0.001);

    //write checksum value to 0x60
    checksum();


}


