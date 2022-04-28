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
        //verify and set all the parameter. Alert when fail to set
        gauge_verify_and_calibrate();

        autocalibrate();
        i2c_close(i2c_bus);
        return 0;
}
