#include "battery.h"
#include "I2C.h"

#define wait_time 0.2

// for apalis imx8 
char *i2c_bus = "/dev/apalis-i2c1";

//for colibri imx6ull
//char *i2c_bus = "/dev/i2c-0";


uint8_t * p_ret_val;
uint8_t ret_val;
uint8_t data[32]={0};

int main()
{
        if (i2c_init(i2c_bus) < 0)
        {
                return -1;
        }

        printf("control_status : %x\n",control_status());
        gauge_unseal();
        printf("control_status : %x\n",control_status());
        gauge_full_access();
        printf("control_status : %x\n",control_status());

        printf("flash update voltage before : %d\n",read_flash_update_ok_voltage());

        set_flash_update_ok_voltage(1500);

        printf("flash update voltage after : %d\n",read_flash_update_ok_voltage());

        printf("voltsel before : %x\n",read_voltsel());

        set_voltsel();

        printf("voltsel after : %x\n",read_voltsel());

        printf("series cell before : %x\n",read_series_cell());

        set_series_cell(4);

        printf("series cell after : %x\n",read_series_cell());

        printf("vdivider before : %d\n",readVDivider());

        set_vdivider(37364);

        printf("vdivider after : %d\n",readVDivider());

        printf("design energy scale before: %d\n",read_design_energy_scale());

        set_design_energy_scale(10);

        printf("design energy scale after: %d\n",read_design_energy_scale());

        printf("design capacity before: %d\n",read_design_capacity());

        set_design_capacity(20000);

        printf("design capacity after: %d\n",read_design_capacity());

        printf("design energy before : %d\n",read_design_energy());

        set_design_energy(25200);

        printf("design energy after: %d\n",read_design_energy());

        i2c_close(i2c_bus);
        return 0;
}
