#include "battery.h"

int main()
{
        if(i2c_init()<0)
        {
                return -1;
        }

        i2c_close();
        return 0;
}
