#include "battery.h"
#include "I2C.h"



//array to store the read and write data
uint8_t reg_addr;
uint8_t battery_data_read[32];
uint8_t battery_data_write[32];
uint16_t return_value;
uint8_t * p_return_value;
uint8_t n_bytes;//number of bytes to read and write


void gauge_seal()
{
    read_control(SEALED);
}

//unseal the gauge before read/write
void gauge_unseal()
{
        reg_addr = 0x00;
        n_bytes = 0x02;
        battery_data_write[0]=0x14;//lsb
        battery_data_write[1]=0x04;//msb
        return_value = i2c_write(SLAVE_ADDR,reg_addr, battery_data_write,n_bytes);
        if (return_value < 0 )
        {
            perror("gauge seal failed");
        }
        sleep(0.2);

        reg_addr =0x00;
        battery_data_write[0]=0x72;//lsb
        battery_data_write[1]=0x36;//msb
        return_value = i2c_write(SLAVE_ADDR,reg_addr, battery_data_write,0x02);
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
        battery_data_write[0]=0x72;//lsb
        battery_data_write[1]=0x36;//msb
        return_value = i2c_write(SLAVE_ADDR,reg_addr, battery_data_write,0x02);
        if (return_value < 0 )
        {
            perror("full access failed");
        }
        sleep(0.2);
    }

}

//reset the guage
void reset_guage()
{
    read_control(RESET);
}

// force the device to measure and store board offset
uint16_t board_offset()
{
    p_return_value = read_control(BOARD_OFFSET);
    return (uint16_t)(*(p_return_value+1)) << 8 | *p_return_value ;
}

 // force the device to measure the internal cc offset
uint16_t cc_offset()
{
    p_return_value = read_control(CC_OFFSET);
    return (uint16_t)(*(p_return_value+1)) << 8 | *p_return_value ;
}

//get device type 0x0100 for bq34z100-g1
uint16_t board_offset()
{
    p_return_value = read_control(DEVICE_TYPE);
    return (uint16_t)(*(p_return_value+1)) << 8 | *p_return_value ;
}

//report internal cc offset in calibration mode
void offset_calibration()
{
    read_control(OFFSET_CAL);
}

//enable accessing block data
void enable_block_data_control()
{
    unsigned char reg_addr =0x61;
    battery_data_write[0]=0x00;
 
    return_value = i2c_write(SLAVE_ADDR,reg_addr, battery_data_write,0x01);
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
    battery_data_write[0] =  control_subcommand & 0x00FF; //lsb
    battery_data_write[1] = control_subcommand >> 8; //msb
    return_value = i2c_write(SLAVE_ADDR, CONTROL, battery_data_write, n_bytes);
    if (return_value < 0 )
    {
        perror("read control write command failed");
        return -1;
    }
    sleep(0.2);
    return i2c_read(SLAVE_ADDR, CONTROL, n_bytes);
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
        checksum_value += battery_data_write[n];
        checksum_value = 0xFF - checksum_value;

        //write checksum to address 0x60
        reg_addr = 0x60;
        n_bytes = 0x01;
        battery_data_write[0]=checksum_value;
        return_value = i2c_write(SLAVE_ADDR,reg_addr, battery_data_write,n_bytes);
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
    battery_data_write[0] =  sub_class ; //lsb
    battery_data_write[1] =  offset/32 ; //msb
    i2c_write(SLAVE_ADDR, CONTROL, battery_data_write, n_bytes);
    sleep(0.001);
    
    //read data from 0x40 address
    reg_addr = 0x40;
    n_bytes = 32;
    return i2c_read(SLAVE_ADDR, reg_addr, n_bytes);
}

uint8_t write_flash_block(uint8_t sub_class, uint8_t offset)
{


    //write_reg(0x61, 0x00); // Block control
    enable_block_data_control();

    //write subclass id and offset to 0x3e and 0x3f register
    reg_addr = 0x3e;
    n_bytes = 2;
    battery_data_write[0] =  sub_class ; //lsb
    battery_data_write[1] =  offset/32 ; //msb
    i2c_write(SLAVE_ADDR, CONTROL, battery_data_write, n_bytes);
    sleep(0.001);


    //write data to 0x40 address
    reg_addr = 0x40;
    n_bytes = 32;
    i2c_write(SLAVE_ADDR, reg_addr, battery_data_write, n_bytes);
    sleep(0.001);

    //write checksum value to 0x60
    checksum();

}


//read voltage divider
uint16_t readVDivider()
{
    uint16_t voltage_divider;
    p_return_value = read_flash_block(0x68, 0x0e);
    voltage_divider = (uint16_t)(*(p_return_value + 14)) << 8 | *(p_return_value + 15);
    return voltage_divider;

}

//set voltage divider value
void set_vdivider(uint16_t v_divider)
{
    uint8_t msb = v_divider >> 8 ;
    uint8_t lsb = v_divider & 0x00FF;

    p_return_value = read_flash_block(0x68,0x0e);
    
    for(uint8_t i = 0; i < 32; i++)
    {
        battery_data_write[i] = *(p_return_value+i);
    }

    

}


//set series cell

//set cell capacity


//get soc
uint8_t soc()
{
    return *(i2c_read(SLAVE_ADDR,STATE_OF_CHARGE,0x01));
}

//enable calibration
uint16_t enable_calibration()
{
    p_return_value = read_control(CAL_ENABLE);
    return (uint16_t)(*(p_return_value+1)) << 8 | *p_return_value ;
}

//enter calibration
uint16_t enter_calibration()
{
    p_return_value = read_control(ENTER_CAL);
    return (uint16_t)(*(p_return_value+1)) << 8 | *p_return_value ;
}

//exit calibration
uint16_t calibration_exit()
{
    p_return_value = read_control(EXIT_CAL);
    return (uint16_t)(*(p_return_value+1)) << 8 | *p_return_value ;
}

//enable IT
uint16_t it_enable()
{
    p_return_value = read_control(IT_ENABLE);
    return (uint16_t)(*(p_return_value+1)) << 8 | *p_return_value ;
}

//control status
uint16_t control_status()
{
    p_return_value = read_control(CONTROL_STATUS);
    return (uint16_t)(*(p_return_value+1)) << 8 | *p_return_value ;
}



//get internal temperature 0.1K
uint16_t internal_temperature()
{
    p_return_value = (i2c_read(SLAVE_ADDR,INTERNAL_TEMPERATURE,0x02));
    return (uint16_t)(*(p_return_value+1)) << 8 | *p_return_value ;
}

//get temperature 0.1K
uint16_t temperature()
{
    p_return_value = (i2c_read(SLAVE_ADDR,TEMPERATURE,0x01));
    return (uint16_t)(*(p_return_value+1)) << 8 | *p_return_value ;
}

//get voltage mV
uint16_t voltage()
{
    p_return_value = (i2c_read(SLAVE_ADDR,VOLTAGE,0x02));
    return (uint16_t)(*(p_return_value+1)) << 8 | *p_return_value ;
}

//get current in mA
uint16_t current()
{
    p_return_value = (i2c_read(SLAVE_ADDR,CURRENT,0x02));
    return (uint16_t)(*(p_return_value+1)) << 8 | *p_return_value ;
}


//get average current mA
uint16_t average_current()
{
    p_return_value = (i2c_read(SLAVE_ADDR,AVERAGE_CURRENT,0x02));
    return (uint16_t)(*(p_return_value+1)) << 8 | *p_return_value ;
}
