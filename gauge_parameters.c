#include "battery.h"
#include "I2C.h"


// for apalis imx8 
char *i2c_bus = "/dev/apalis-i2c1";

//for colibri imx6uul
//char *i2c_bus = "/dev/i2c-0";


int main()
{
        // start the i2c bus
        i2c_init(i2c_bus);

        //gauge_parameters();
        
        // write battery parameters to file
		int ret_val = write_to_file();
        if(ret_val == -1)
		{
			return -1;
		}

        // close the i2c bus
        i2c_close(i2c_bus);
       
        return 0;
}



