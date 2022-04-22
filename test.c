#include "battery.h"
#include "I2C.h"


// for apalis imx8 
char *i2c_bus = "/dev/apalis-i2c1";

//for colibri imx6uul
//char *i2c_bus = "/dev/i2c-0";

int main()
{
        // start the i2c bus
        if(i2c_init(i2c_bus)<0)
        {
                return -1;
        }

        // unseal the gauge - work when it is sealed
        gauge_unseal();
        printf("unseal done\n");
        printf("%x\n",control_status());

        // full access the gauge - it should be unsealed first
        gauge_full_access();
        printf("full access done\n");
        printf("%x\n",control_status());


        // print current value in mA
        printf("current %d mA\n",current());


        // print average current in mV
        printf("average current %d mA\n",average_current());


        // print voltage in mV
        printf("voltage %d mV\n",voltage());

        // close the i2c bus
        i2c_close(i2c_bus);
        return 0;
}
