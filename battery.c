#include "battery.h"
#include "I2C.h"

//array to store the read and write data
uint8_t reg_addr;
uint8_t battery_data[32];

uint8_t reg_data[4];
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
        reg_data[0]=0x14;//lsb
        reg_data[1]=0x04;//msb
        return_value = i2c_write(SLAVE_ADDR,reg_addr, reg_data,n_bytes);
        if (return_value < 0 )
        {
            perror("gauge seal failed");
        }
        sleep(0.2);

        reg_addr =0x00;
        reg_data[0]=0x72;//lsb
        reg_data[1]=0x36;//msb
        return_value = i2c_write(SLAVE_ADDR,reg_addr, reg_data,n_bytes);
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
    n_bytes = 2;
    for(i=0;i<3;i++)
    {
        reg_addr =0x00;
        reg_data[0]=0xFF;//lsb
        reg_data[1]=0xFF;//msb
        return_value = i2c_write(SLAVE_ADDR,reg_addr, reg_data,n_bytes);
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
    return read_control(BOARD_OFFSET);
}

 // force the device to measure the internal cc offset
uint16_t cc_offset()
{
    return read_control(CC_OFFSET);
}

//get device type 0x0100 for bq34z100-g1
uint16_t device_type()
{
    return read_control(DEVICE_TYPE);
}

//report internal cc offset in calibration mode
void offset_calibration()
{
    read_control(OFFSET_CAL);
}

//enable accessing block data
void enable_block_data_control()
{
    reg_addr =0x61;
    reg_data[0]=0x00;
    n_bytes = 1;
 
    return_value = i2c_write(SLAVE_ADDR,reg_addr, reg_data,n_bytes);
    if (return_value < 0 )
    {
        perror("block data enable failed");
    }
    sleep(0.2);
}

//read control command
uint16_t  read_control(uint16_t control_subcommand)
{
    n_bytes = 2;
    reg_data[0] = control_subcommand & 0x00FF; //lsb
    reg_data[1] = control_subcommand >> 8; //msb
    return_value = i2c_write(SLAVE_ADDR, CONTROL, reg_data, n_bytes);
    sleep(0.2);
    p_return_value = i2c_read(SLAVE_ADDR, CONTROL, n_bytes);
    sleep(0.2);

    return (uint16_t)((uint16_t)(*(p_return_value+1)) << 8 | *p_return_value) ;
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
        {
            checksum_value += battery_data[n];
        }

        checksum_value = 0xFF - checksum_value;

        //write checksum to address 0x60
        reg_addr = 0x60;
        n_bytes = 0x01;
        reg_data[0]=checksum_value;
        //printf("checksum : %x",checksum_value);
        return_value = i2c_write(SLAVE_ADDR,reg_addr, reg_data,n_bytes);
        if (return_value < 0 )
        {
            perror("failed to write checksum");
            return -1;
        }
        sleep(0.2);
        return 0;

}


//read flash block
unsigned  char * read_flash_block(uint8_t sub_class, uint8_t offset)
{
    //enable block data
    enable_block_data_control();
    
    //write subclass id and offset to 0x3e and 0x3f register
    reg_addr = 0x3e;
    n_bytes = 2;
    reg_data[0] =  sub_class ; //lsb
    reg_data[1] =  offset/32 ; //msb
    i2c_write(SLAVE_ADDR, reg_addr, reg_data, n_bytes);
    sleep(0.2);
    
    //read data from 0x40 address
    reg_addr = 0x40;
    n_bytes = 32;
    return i2c_read(SLAVE_ADDR, reg_addr, n_bytes);
    sleep(0.2);
}

uint8_t write_flash_block(uint8_t sub_class, uint8_t offset)
{
    //write_reg(0x61, 0x00); // Block control
    enable_block_data_control();

    //write subclass id and offset to 0x3e and 0x3f register
    reg_addr = 0x3e;
    n_bytes = 2;
    reg_data[0] =  sub_class ; //lsb
    reg_data[1] =  offset/32 ; //msb
    i2c_write(SLAVE_ADDR, reg_addr,reg_data, n_bytes);
    sleep(0.2);


    //write data to 0x40 address
    reg_addr = 0x40;
    n_bytes = 32;
    i2c_write(SLAVE_ADDR, reg_addr, battery_data, n_bytes);
    sleep(0.2);

    //write checksum value to 0x60
    checksum();

}


//read voltage divider
uint16_t readVDivider()
{
    p_return_value = read_flash_block(0x68, 0x0e);
    return (uint16_t)(*(p_return_value + 14)) << 8 | *(p_return_value + 15);

}

//read pack_configuration
uint16_t read_pack_configuration()
{
    p_return_value = read_flash_block(0x40, 0x00);
    return (uint16_t)(*(p_return_value + 0)) << 8 | *(p_return_value + 1);

}

//read design capacity
uint16_t read_design_capacity()
{
    p_return_value = read_flash_block(0x30, 0x0b);
    return (uint16_t)(*(p_return_value + 11)) << 8 | *(p_return_value + 12);

}

//read design energy
uint16_t read_design_energy()
{
    p_return_value = read_flash_block(0x30, 0x0d);
    return (uint16_t)(*(p_return_value + 13)) << 8 | *(p_return_value + 14);

}

//read flash_update_ok_voltage
uint16_t read_flash_update_ok_voltage()
{
    p_return_value = read_flash_block(0x44, 0x00);
    return (uint16_t)(*(p_return_value + 0)) << 8 | *(p_return_value + 1);

}

//read no. of cell
uint8_t read_number_of_cell()
{
    p_return_value = read_flash_block(0x40, 0x07);
    return  *(p_return_value+7);

}

//read design energy scale
uint8_t read_design_energy_scale()
{
    p_return_value = read_flash_block(0x30, 0x1e);
    return  *(p_return_value+30);

}

//set voltage divider value
void set_vdivider(uint16_t v_divider)
{
    uint8_t msb = v_divider >> 8 ;
    uint8_t lsb = v_divider & 0x00FF;

    p_return_value = read_flash_block(0x68,0x0e);
    
    for(uint8_t i = 0; i < 32; i++)
    {
        battery_data[i] = *(p_return_value+i);
    }

    battery_data[14] = msb; //msb
    battery_data[15] = lsb; //lsb

    write_flash_block(0x68, 0x0e);
}


//set series cell
void set_series_cell(uint8_t series_cell)
{

    p_return_value = read_flash_block(0x40,0x07);
    
    for(uint8_t i = 0; i < 32; i++)
    {
        battery_data[i] = *(p_return_value+i);
    }

    battery_data[7] = series_cell; //number of series cell
    

    write_flash_block(0x40, 0x07);

    
}

//set design capacity
void set_design_capacity(uint16_t design_capacity)
{
    uint8_t msb = design_capacity >> 8 ;
    uint8_t lsb = design_capacity & 0x00FF;

    p_return_value = read_flash_block(0x30,0x0b);
    
    for(uint8_t i = 0; i < 32; i++)
    {
        battery_data[i] = *(p_return_value+i);
    }

    battery_data[11] = msb; //msb
    battery_data[12] = lsb; //lsb

    write_flash_block(0x30, 0x0b);
}

//set design energy scale
void set_design_energy_scale(uint8_t design_energy_scale)
{
   
    p_return_value = read_flash_block(0x30,0x1e);
    
    for(uint8_t i = 0; i < 32; i++)
    {
        battery_data[i] = *(p_return_value+i);
    }

    battery_data[30] = design_energy_scale;//design_energy_scale = 10

    write_flash_block(0x30, 0x1e);
}

//set design energy
void set_design_energy(uint16_t design_energy)
{
    uint8_t msb = design_energy >> 8 ;
    uint8_t lsb = design_energy & 0x00FF;

    p_return_value = read_flash_block(0x30,0x0d);
    
    for(uint8_t i = 0; i < 32; i++)
    {
        battery_data[i] = *(p_return_value+i);
    }

    battery_data[13] = msb; //msb
    battery_data[14] = lsb; //lsb

    write_flash_block(0x30, 0x0d);
}

//set VOLTSEL BIT in pack_configuration register
void set_voltsel()
{   
    uint16_t pack_configuration = read_pack_configuration();
    pack_configuration = pack_configuration | 0x0800;

    uint8_t msb = pack_configuration >> 8 ;
    uint8_t lsb = pack_configuration & 0x00FF;

    p_return_value = read_flash_block(0x40,0x00);
    
    for(uint8_t i = 0; i < 32; i++)
    {
        battery_data[i] = *(p_return_value+i);
    }

    battery_data[0] = msb; //msb
    battery_data[1] = lsb; //lsb

    write_flash_block(0x40, 0x00);
}


//set flash update ok cell volt
void set_flash_update_ok_voltage(uint16_t flash_update_ok_voltage)
{
    uint8_t msb = flash_update_ok_voltage >> 8 ;
    uint8_t lsb = flash_update_ok_voltage & 0x00FF;

    p_return_value = read_flash_block(0x44,0x00);
    
    for(uint8_t i = 0; i < 32; i++)
    {
        battery_data[i] = *(p_return_value+i);
    }

    battery_data[0] = msb; //msb
    battery_data[1] = lsb; //lsb

    write_flash_block(0x44, 0x00);
}


//get soc
uint8_t soc()
{
    return *(i2c_read(SLAVE_ADDR,STATE_OF_CHARGE,0x01));
}

//enable calibration
uint16_t enable_calibration()
{
    return read_control(CAL_ENABLE);
}

//enter calibration
uint16_t enter_calibration()
{
    return read_control(ENTER_CAL);
}

//exit calibration
uint16_t calibration_exit()
{
    return read_control(EXIT_CAL);
}

//enable IT
uint16_t it_enable()
{
    return read_control(IT_ENABLE);
}

//control status
uint16_t control_status()
{
    return read_control(CONTROL_STATUS);
}



//get internal temperature 0.1K
float internal_temperature()
{
    p_return_value = (i2c_read(SLAVE_ADDR,INTERNAL_TEMPERATURE,0x02));
    return (float)((uint16_t)(*(p_return_value+1)) << 8 | *p_return_value) ;
}

//get temperature 0.1K
float temperature()
{
    p_return_value = (i2c_read(SLAVE_ADDR,TEMPERATURE,0x01));
    return (float)((uint16_t)(*(p_return_value+1)) << 8 | *p_return_value );
}

//get voltage mV
uint16_t voltage()
{
    p_return_value = (i2c_read(SLAVE_ADDR,VOLTAGE,0x02));
    uint16_t voltage_value = *(p_return_value+1) << 8 | *p_return_value;
    return voltage_value;
}

//get current in mA
int16_t current()
{
    p_return_value = (i2c_read(SLAVE_ADDR,CURRENT,0x02));
    int16_t current_value = *(p_return_value+1) << 8 | *p_return_value;
    return current_value;

}


//get average current mA
int16_t average_current()
{
    p_return_value = (i2c_read(SLAVE_ADDR,AVERAGE_CURRENT,0x02));
    int16_t current_value = *(p_return_value+1) << 8 | *p_return_value;
    return current_value;
}
