#include "battery.h"
#include "I2C.h"

char *i2c_bus = "/dev/apalis-i2c1";

unsigned char data[32];
unsigned char * p_data;
unsigned char ret_value;


int main()
{
        if(i2c_init(i2c_bus)<0)
        {
                return -1;
        }

        gauge_unseal();
        printf("unseal done\n");
        printf("%x\n",control_status());

        gauge_full_access();
        printf("full access done\n");
        printf("%x\n",control_status());

        reset_guage();
        printf("rset done\n");
        printf("%x\n",control_status());

        gauge_unseal();
        printf("unseal done\n");
        printf("%x\n",control_status());

        gauge_full_access();
        printf("full access done\n");
        printf("%x\n",control_status());
        

        //enable_block_data_control();
        // printf("enabled block data");

        // printf("vdivider : %d\n",readVDivider());

        // printf("pack config : %x\n",read_pack_configuration());

        // printf("design capacity : %d\n",read_design_capacity());

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
