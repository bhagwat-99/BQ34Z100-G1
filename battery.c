#include "battery.h"
#include "I2C.h"

#define wait_time 1000000/4 // unit - usec wait_time - (1sec)

// seal the gauge - don't execute the function if device already in sealed state
void gauge_seal()
{
    read_control(SEALED);
}

//unseal the gauge before read/write
static void gauge_unseal()
{
        uint8_t reg_addr    =   0x00;
        uint8_t n_bytes     =   0x02;

        uint8_t reg_data[2];
        reg_data[0]         =   0x14;   //lsb
        reg_data[1]         =   0x04;   //msb

        uint16_t    return_value   =  i2c_write(SLAVE_ADDR,reg_addr, reg_data,n_bytes);
        usleep(wait_time);

        if (return_value < 0 )
        {
            perror("gauge seal failed");
        }

        reg_addr =0x00;
        reg_data[0]=0x72;//lsb
        reg_data[1]=0x36;//msb

        return_value = i2c_write(SLAVE_ADDR,reg_addr, reg_data,n_bytes);
        usleep(wait_time);
        if (return_value < 0 )
        {
            perror("gauge seal failed");
        }
}

//full access mode of gauge. gauge must be in unsealed mode to full access mode
static void gauge_full_access()
{
    uint8_t n_bytes = 2;
    for(uint8_t i=0;i<3;i++)
    {
        uint8_t reg_addr;
        uint8_t reg_data[2];

        reg_addr =0x00;
        reg_data[0]=0xFF;//lsb
        reg_data[1]=0xFF;//msb

        i2c_write(SLAVE_ADDR,reg_addr, reg_data,n_bytes);
        usleep(wait_time);

        reg_addr =0x00;
        reg_data[0]=0xFF;//lsb
        reg_data[1]=0xFF;//msb
        i2c_write(SLAVE_ADDR,reg_addr, reg_data,n_bytes);
        usleep(wait_time);
    }

}

void gauge_unlock()
{
    gauge_unseal();
    gauge_full_access();
}

//reset the guage
void reset_guage()
{
    read_control(RESET);
    usleep(wait_time);
}



//control status
uint16_t control_status()
{
    return read_control(CONTROL_STATUS);
}

//get device type 0x0100 for bq34z100-g1
uint16_t device_type()
{
    return read_control(DEVICE_TYPE);
}

///////////////////////////// data read write functions ///////////////////////////////////////////////////

//enable accessing block data
static void enable_block_data_control()
{
    uint8_t n_bytes     =   1;
    uint8_t reg_addr    =   0x61;

    uint8_t reg_data[1];
    reg_data[0]=0x00;

    i2c_write(SLAVE_ADDR,reg_addr, reg_data,n_bytes);
    usleep(wait_time);
    
}

//read control command
static uint16_t  read_control(uint16_t control_subcommand)
{
    uint8_t n_bytes = 2;
    uint8_t reg_data[2];

    reg_data[0] = control_subcommand & 0x00FF; //lsb
    reg_data[1] = control_subcommand >> 8; //msb

    i2c_write(SLAVE_ADDR, CONTROL, reg_data, n_bytes);
    usleep(wait_time);

    uint8_t * p_return_value = i2c_read(SLAVE_ADDR, CONTROL, n_bytes);
    usleep(wait_time);

    return ((uint16_t)(*(p_return_value+1)) << 8 | *p_return_value) ;

}



static uint8_t checksum(uint8_t * checksum_data)
{
        uint8_t checksum_value = 0x00;

        for (uint8_t n = 0; n < 32; n++)
        {
            checksum_value += *(checksum_data+n);
        }

        checksum_value = 0xFF - checksum_value;

        return checksum_value;

        // //write checksum to address 0x60
        // uint8_t reg_addr = 0x60;
        // uint8_t n_bytes = 0x01;
        // uint8_t reg_data[1];
        // reg_data[0] = checksum_value;

        // uint8_t return_value = i2c_write(SLAVE_ADDR,reg_addr, reg_data,n_bytes);
        // sleep(wait_time);
        // if(return_value < 0 )
        // {
        //     perror("failed to write checksum");
        //     return -1;
        // }
        // return 0;

}


//read flash block
static uint8_t * read_flash_block(uint8_t sub_class, uint8_t offset)
{
    //enable block data
    enable_block_data_control();
    
    //write subclass id to 0x3e
    uint8_t reg_addr = 0x3e;
    uint8_t n_bytes = 1;
    uint8_t reg_data[1];
    reg_data[0] =  sub_class ;
    i2c_write(SLAVE_ADDR, reg_addr, reg_data, n_bytes);
    usleep(wait_time*2);

    //write offset to 0x3f register
    reg_addr = 0x3f;
    n_bytes = 1;
    reg_data[0] =  offset/32 ; //msb
    i2c_write(SLAVE_ADDR, reg_addr, reg_data, n_bytes);
    usleep(wait_time*2);
    
    //read data from 0x40 address
    reg_addr = 0x40;
    n_bytes = 32;
    uint8_t * p_ret_value = i2c_read(SLAVE_ADDR, reg_addr, n_bytes);
    usleep(wait_time*2);
    return p_ret_value;
}


// write 32 byte flash data block
static uint8_t write_flash_block(uint8_t sub_class, uint8_t offset, uint8_t * data)
{
    //enable block data
    enable_block_data_control();
    
    //write subclass id and offset to 0x3e and 0x3f register
    uint8_t reg_addr = 0x3e;
    uint8_t n_bytes = 1;
    uint8_t reg_data[1];
    reg_data[0] =  sub_class ;
    i2c_write(SLAVE_ADDR, reg_addr, reg_data, n_bytes);
    usleep(wait_time*2);

    //write offset to 0x3f register
    reg_addr = 0x3f;
    n_bytes = 1;
    reg_data[0] =  offset/32 ; //msb
    i2c_write(SLAVE_ADDR, reg_addr, reg_data, n_bytes);
    usleep(wait_time);


    //write data to 0x40 address
    reg_addr = 0x40;
    n_bytes = 32;
    i2c_write(SLAVE_ADDR, reg_addr, data, n_bytes);
    usleep(2*wait_time);

    //write checksum to address 0x60
    reg_addr = 0x60;
    n_bytes = 0x01;
    reg_data[0] = checksum(data);

    uint8_t return_value = i2c_write(SLAVE_ADDR,reg_addr, reg_data,n_bytes);
    usleep(2*wait_time);
    if(return_value < 0 )
    {
        perror("failed to write checksum");
        return -1;
    }

    return 0;

}



/////////////////////////////// autocalibration fuctions ///////////////////////////////////////////////////


//enable calibration
static void enable_calibration()
{
    read_control(CAL_ENABLE);
}


//enter calibration
static void enter_calibration()
{
    read_control(ENTER_CAL);
}

//exit calibration
static void calibration_exit()
{
    read_control(EXIT_CAL);
}


// force the device to measure and store board offset
static void board_offset()
{
    read_control(BOARD_OFFSET);
}

 // force the device to measure the internal cc offset
static void cc_offset()
{
    read_control(CC_OFFSET);
}


//report internal cc offset in calibration mode
static void offset_calibration()
{
    read_control(OFFSET_CAL);
}

//enable IT
static void it_enable()
{
    read_control(IT_ENABLE);
}

void autocalibrate()
{
    board_offset();
    cc_offset();
    offset_calibration();
    enable_calibration();
    enter_calibration();
    it_enable();
    usleep(wait_time);

}




////////////////////////// gauge parameter functions //////////////////////////////////////////


//get internal temperature 0.1K
static uint16_t internal_temperature()
{
    uint8_t * p_return_value = (i2c_read(SLAVE_ADDR,INTERNAL_TEMPERATURE,0x02));
    return ((uint16_t)(*(p_return_value+1)) << 8 | *p_return_value) ;
}

//get temperature 0.1K
static uint16_t temperature()
{
    uint8_t * p_return_value = (i2c_read(SLAVE_ADDR,TEMPERATURE,0x01));
    return ((uint16_t)(*(p_return_value+1)) << 8 | *p_return_value );
}

//get voltage mV
static uint16_t voltage()
{
    uint8_t * p_return_value = (i2c_read(SLAVE_ADDR,VOLTAGE,0x02));
    uint16_t voltage_value = *(p_return_value+1) << 8 | *p_return_value;
    return voltage_value;
}

//get current in mA
static int16_t current()
{
    uint8_t * p_return_value = (i2c_read(SLAVE_ADDR,CURRENT,0x02));
    int16_t current_value = *(p_return_value+1) << 8 | *p_return_value;
    return current_value;

}


//get average current mA
static int16_t average_current()
{
    uint8_t * p_return_value = (i2c_read(SLAVE_ADDR,AVERAGE_CURRENT,0x02));
    int16_t current_value = *(p_return_value+1) << 8 | *p_return_value;
    return current_value;
}

//get soc
uint8_t soc()
{
    return *(i2c_read(SLAVE_ADDR,STATE_OF_CHARGE,0x01));
}

void gauge_parameters()
{
    //unseal and full access the gauge
    gauge_unlock();

    // print average current in mV
    printf("average current %d mA\n",average_current());

    // print voltage in mV
    printf("voltage %d mV\n",voltage());


    // //Celsius = (Kelvin – 273.15)
    // //temperature
    // printf("Temperature %f C\n",(float)(temperature()*10)-273.15);

    //Celsius = (Kelvin – 273.15)
    //internal temperature
    printf("Internal Temperature %f C\n",(float)(internal_temperature()*0.1)-273.15);


}



/////////////////////// gauge calibration function //////////////////////////////


//read voltsel bit in pack configuration register
static uint16_t read_voltsel()
{   
    return (read_pack_configuration() & 0x0800) >> 11;
  
}

//read voltage divider
static uint16_t readVDivider()
{
    uint8_t * p_return_value = read_flash_block(0x68, 0x0e);
    return (uint16_t)(*(p_return_value + 14)) << 8 | *(p_return_value + 15);

}

//read pack_configuration
static uint16_t read_pack_configuration()
{
    uint8_t * p_return_value = read_flash_block(0x40, 0x00);
    return (uint16_t)(*(p_return_value + 0)) << 8 | *(p_return_value + 1);

}

//read design capacity
static uint16_t read_design_capacity()
{
    uint8_t * p_return_value = read_flash_block(0x30, 0x0b);

    return (uint16_t)(*(p_return_value + 11)) << 8 | *(p_return_value + 12);

}

//read design energy
static uint16_t read_design_energy()
{
    uint8_t * p_return_value = read_flash_block(0x30, 0x0d);
    return (uint16_t)(*(p_return_value + 13)) << 8 | *(p_return_value + 14);

}

//read flash_update_ok_voltage
static uint16_t read_flash_update_ok_voltage()
{
    uint8_t * p_return_value = read_flash_block(0x44, 0x00);
    return (uint16_t)(*(p_return_value + 0)) << 8 | *(p_return_value + 1);

}

//read no. of cell
static uint16_t read_series_cell()
{
    uint8_t * p_return_value = read_flash_block(0x40, 0x07);
    return  *(p_return_value+7);

}

//read design energy scale
static uint16_t read_design_energy_scale()
{
    uint8_t * p_return_value = read_flash_block(0x30, 0x1e);
    return  *(p_return_value+30);

}

//set voltage divider value
static void set_vdivider(uint16_t v_divider)
{
    uint8_t data[32];
    uint8_t msb = v_divider >> 8 ;
    uint8_t lsb = v_divider & 0x00FF;

    uint8_t * p_return_value = read_flash_block(0x68,0x0e);
    usleep(wait_time);
    
    for(uint8_t i = 0; i < 32; i++)
    {
        data[i] = *(p_return_value+i);
    }

    data[14] = msb; //msb
    data[15] = lsb; //lsb

    write_flash_block(0x68, 0x0e,data);
    usleep(wait_time);

}


//set series cell
static void set_series_cell(uint16_t series_cell)
{
    uint8_t data[32];
    uint8_t * p_return_value = read_flash_block(0x40,0x07);
    usleep(wait_time);
    
    for(uint8_t i = 0; i < 32; i++)
    {
        data[i] = *(p_return_value+i);
    }

    data[7] = (uint8_t)series_cell; //number of series cell
    

    write_flash_block(0x40, 0x07,data);
    usleep(wait_time);

    
}

//set design capacity
static void set_design_capacity(uint16_t design_capacity)
{
    uint8_t data[32];
    uint8_t msb = design_capacity >> 8 ;
    uint8_t lsb = design_capacity & 0x00FF;

    uint8_t * p_return_value = read_flash_block(0x30,0x0b);
    usleep(wait_time);
    
    for(uint8_t i = 0; i < 32; i++)
    {
        data[i] = *(p_return_value+i);
    }

    data[11] = msb; //msb
    data[12] = lsb; //lsb

    write_flash_block(0x30, 0x0b,data);
    usleep(wait_time);
}

//set design energy scale
static void set_design_energy_scale(uint16_t design_energy_scale)
{
    uint8_t data[32];
   
    uint8_t * p_return_value = read_flash_block(0x30,0x1e);
    usleep(wait_time);
    
    for(uint8_t i = 0; i < 32; i++)
    {
        data[i] = *(p_return_value+i);
    }

    data[30] = (uint8_t)design_energy_scale;//design_energy_scale = 10

    write_flash_block(0x30, 0x1e,data);
    usleep(wait_time);
}

//set design energy
static void set_design_energy(uint16_t design_energy)
{
    uint8_t data[32];
    uint8_t msb = design_energy >> 8 ;
    uint8_t lsb = design_energy & 0x00FF;

    uint8_t * p_return_value = read_flash_block(0x30,0x0d);
    usleep(wait_time);
    
    for(uint8_t i = 0; i < 32; i++)
    {
        data[i] = *(p_return_value+i);
    }

    data[13] = msb; //msb
    data[14] = lsb; //lsb

    write_flash_block(0x30, 0x0d,data);
    usleep(wait_time);
}

//set VOLTSEL BIT in pack_configuration register
static void set_voltsel(uint16_t dummy_value)
{   
    uint8_t data[32];
    uint16_t pack_configuration = read_pack_configuration();
    usleep(wait_time);
    pack_configuration = pack_configuration | 0x0800;

    uint8_t msb = pack_configuration >> 8 ;
    uint8_t lsb = pack_configuration & 0x00FF;

    uint8_t * p_return_value = read_flash_block(0x40,0x00);
    usleep(wait_time);
    
    for(uint8_t i = 0; i < 32; i++)
    {
        data[i] = *(p_return_value+i);
    }

    data[0] = msb; //msb
    data[1] = lsb; //lsb

    write_flash_block(0x40, 0x00,data);
    usleep(wait_time);
}


//set flash update ok cell volt
void set_flash_update_ok_voltage(uint16_t flash_update_ok_voltage)
{
    uint8_t data[32];
    uint8_t msb = flash_update_ok_voltage >> 8 ;
    uint8_t lsb = flash_update_ok_voltage & 0x00FF;

    uint8_t * p_return_value = read_flash_block(0x44,0x00);
    usleep(wait_time);

    
    for(uint8_t i = 0; i < 32; i++)
    {
        data[i] = *(p_return_value+i);
    }

    data[0] = msb; //msb
    data[1] = lsb; //lsb

    write_flash_block(0x44, 0x00,data);
    usleep(wait_time);

 }


void gauge_verify_and_calibrate()
{
    gauge_unlock();

    // setting flash update ok voltage

    verify_calibrate_func(read_flash_update_ok_voltage,set_flash_update_ok_voltage,1500);


    // set voltsel bit
    verify_calibrate_func(read_voltsel,set_voltsel,1);


    // set series cell
    verify_calibrate_func(read_series_cell,set_series_cell,4);

    // set voltage divider
    verify_calibrate_func(readVDivider,set_vdivider,37364);


    // set design energy scale
    verify_calibrate_func(read_design_energy_scale,set_design_energy_scale,10);


    // set design capacity
    verify_calibrate_func(read_design_capacity,set_design_capacity,20000);

    // set design energy
    verify_calibrate_func(read_design_energy,set_design_energy,25200);

    printf("Successfully done calibration\n");

    calibrated_data();

}


static void verify_calibrate_func(uint16_t (*read_func)(), void (*set_func)(uint16_t),uint16_t value)
{

    uint8_t attempt = 0;
    while(read_func() != value && attempt < 3)
    {
        set_func(value);
        if(read_func() != value)
        {
            attempt++;
        }
        
    }
    if(attempt>=3)
    {
        failed_to_calibrate(value);
    }
}

static void failed_to_calibrate(uint16_t value)
{
    if(value == 1)
    {
        printf("Failed to set voltsel");
    }
    else if (value == 4)
    {
        printf("Failed to set series cell");
    }
    else if (value == 10)
    {
        printf("Failed to set design energy scale");
    }
    else if (value == 20000)
    {
        printf("Failed to set design capacity");
    }
    else if (value == 37364)
    {
        printf("Failed to set voltage divider");
    }
    else if (value == 25200)
    {
        printf("Failed to set design energy");
    }
    else if (value == 1500)
    {
        printf("Failed to set flash update ok voltage");
    }
}

static void calibrated_data()
{
    printf("flash update voltage : %d\n",read_flash_update_ok_voltage());

    printf("voltsel : %x\n",read_voltsel());

    printf("series cell : %x\n",read_series_cell());

    printf("vdivider : %d\n",readVDivider());

    printf("design energy scale: %d\n",read_design_energy_scale());

    printf("design capacity : %d\n",read_design_capacity());

    printf("design energy: %d\n",read_design_energy());

}

/////////////////////////////////////////////////////////////////////////////////
