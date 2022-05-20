#include "battery.h"
#include "I2C.h"

// for apalis imx8 
char *i2c_bus = "/dev/apalis-i2c1";

//for colibri imx6ull
//char *i2c_bus = "/dev/i2c-0";

int main()
{
        // open the i2c bus
        i2c_init(i2c_bus);
        

        int ret_val;
        gauge_unlock();

        //verify and set all the parameter. Alert when fail to set
        ret_val = gauge_verify_and_calibrate();
        if(ret_val == -1)
        {
                return -1;
        }

        //autocalibrate();

        //close the i2c bus
        i2c_close(i2c_bus);
        

        return 0;
}


