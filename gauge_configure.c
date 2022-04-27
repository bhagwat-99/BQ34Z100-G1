#include "battery.h"
#include "I2C.h"


// for apalis imx8 
char *i2c_bus = "/dev/apalis-i2c1";

//for colibri imx6uul
//char *i2c_bus = "/dev/i2c-0";


int main()
{
        if (i2c_init(i2c_bus) < 0)
        {
                return -1;
        }

        gauge_unseal();
        gauge_full_access();

        printf("design capacity: %d\n",read_design_capacity());
        

        // printf("flash update ok voltage before: %d\n",read_flash_update_ok_voltage());
        // set_flash_update_ok_voltage(2000);
        // printf("flash update ok voltage after: %d\n",read_flash_update_ok_voltage());


        // printf("voltsel before: %d\n",read_voltsel());
        // set_voltsel();
        // printf("voltsel after: %d\n",read_voltsel());


        // printf("series cell before: %d\n",read_series_cell());
        // set_series_cell(1);
        // printf("series cell after: %d\n",read_series_cell());


        // //37364
        // printf("vdivider before: %d\n",readVDivider());
        // set_vdivider(37364);
        // printf("vdivider after: %d\n",readVDivider());


        i2c_close(i2c_bus);
        return 0;
}
