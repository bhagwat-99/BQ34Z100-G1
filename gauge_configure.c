#include "battery.h"
#include "I2C.h"

// for apalis imx8 
char *i2c_bus = "/dev/apalis-i2c1";

//for colibri imx6ull
//char *i2c_bus = "/dev/i2c-0";

int main()
{
        if (i2c_init(i2c_bus) < 0)
        {
                return -1;
        }

        int ret_val;
        ret_val = gauge_unlock();
        if(ret_val == -1)
        {
                return -1;
        }

        //verify and set all the parameter. Alert when fail to set
        ret_val = gauge_verify_and_calibrate();
        if(ret_val == -1)
        {
                return -1;
        }

        //autocalibrate();
        ret_val = i2c_close(i2c_bus);
        if(ret_val == -1)
        {
                return -1;
        }

        return 0;
}


