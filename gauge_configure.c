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

        // unseal the guage
        gauge_unseal();
        printf("unseal done\n");
        printf("%x\n", control_status());

        // full access of guage
        gauge_full_access();
        printf("full access done\n");
        printf("%x\n", control_status());

        // flash update ok voltage default : 2800 mv  updated :  1500 mv
        printf("flash update ok voltage before :  %d \n", read_flash_update_ok_voltage());
        set_flash_update_ok_voltage(1500);
        sleep(1);
        printf("flash update ok voltage after :  %d \n", read_flash_update_ok_voltage());

        // voltsel bit default : 0 - 1s cell. Updated : 1 - multicell upto 7s
        //printf("voltsel bit before: %x \n", read_pack_configuration());
        printf("voltsel bit before: %x \n", read_voltsel());
        set_voltsel(1);
        sleep(1);
        //printf("voltsel bit before: %x \n", read_pack_configuration());
        printf("voltsel bit after: %d \n", read_voltsel());

        //set series cell
        printf("series cell : %d \n", read_series_cell());
        set_series_cell(4);
        sleep(1);        
        printf("series cell : %d \n", read_series_cell());

        //design energy scale default : 1 updated : 10
        printf("design energy scale : %d\n",read_design_energy_scale());
        set_design_energy_scale(10);
        sleep(1);
        printf("design energy scale : %d\n",read_design_energy_scale());


        // // voltage divider
        // // default : 5000
       
        // // 5v - 5000
        // // 16v - 19182
        // // 32v - 37364
        // // 48v - 55545
        // printf("vdivider before : %d \n", readVDivider());
        // set_vdivider(37364);
        // sleep(1);
        // printf("vdivider after : %d\n", readVDivider());

       
        // // design capacity default : 1000 updated : 20000
        // printf("design capacity : %d\n", read_design_capacity());
        // set_design_capacity(20000);
        // sleep(1);
        // printf("design capacity : %d\n", read_design_capacity());



        // // design energy default :      updated : 25200
        // printf("design energy : %d\n",read_design_energy());
        // set_design_energy(25200);
        // sleep(1);
        // printf("design energy : %d\n",read_design_energy());

        i2c_close(i2c_bus);
        return 0;
}
