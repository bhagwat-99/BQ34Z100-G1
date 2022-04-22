#include "battery.h"
#include "I2C.h"

char *i2c_bus = "/dev/apalis-i2c1";

unsigned char data[32];
unsigned char *p_data;
unsigned char ret_value;

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
        printf("flash update ok voltage :  %d \n", read_flash_update_ok_voltage());
        set_flash_update_ok_voltage(1500);
        sleep(1);
        printf("flash update ok voltage :  %d \n", read_flash_update_ok_voltage());

        // voltsel bit default : 0 - 1s cell. Updated : 1 - multicell upto 7s
        printf("voltsel bit : %d \n", read_voltsel);
        set_voltsel();
        sleep(1);
        printf("voltsel bit : %d \n", read_voltsel);

        // voltage divider
        // default : 5000
        // Updated
        // 32v - 37364
        // 16v -
        // 48v -
        printf("vdivider : %d \n", readVDivider());
        set_vdivider(37364);
        sleep(1);
        printf("vdivider : %d\n", readVDivider());

        // design capacity default : 1000 updated : 20000
        printf("design capacity : %d\n", read_design_capacity());

        // printf("design energy : %d\n",read_design_energy());

        // printf("ok voltage : %d\n",read_flash_update_ok_voltage());

        // printf("series cell : %d\n",read_number_of_cell());

        // printf("design energy scale : %d\n",read_design_energy_scale());
        //  printf("vdivider %d\n",read_design_energy());

        // set_design_energy(25200);

        // sleep(1);

        // printf("current %d mA\n",current());

        // printf("voltage %d mV\n",voltage());

        // printf("average current %d mA\n",average_current());

        i2c_close(i2c_bus);
        return 0;
}
