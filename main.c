#include "battery.h"


int main()
{
        if(i2c_init()<0)
        {
                return -1;
        }

        printf("current : %f\n",current());

        i2c_close();
        return 0;
}
