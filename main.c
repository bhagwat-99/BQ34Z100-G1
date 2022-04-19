#include "battery.h"

const char *i2c_bus = "/dev/apalis-i2c1";

int main()
{
        if(i2c_init(i2c_bus)<0)
        {
                return -1;
        }

        printf("current : %f\n",current());

        i2c_close(i2c_bus);
        return 0;
}
