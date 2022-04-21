#include "battery.h"
#include "I2C.h"

char *i2c_bus = "/dev/i2c-0";

unsigned char data[32];
unsigned char * p_data;
unsigned char ret_value;


int main()
{
        if(i2c_init(i2c_bus)<0)
        {
                return -1;
        }

        // p_data = read_flash_block(0x30,0x00);
        // for(uint8_t i=0 ; i< 32; i++)
        // {
        //         printf("i= %d : %x\n",i,*(p_data+i));
        // }

        // reset_guage();
        // printf("rset done\n");
        // sleep(1);
        // gauge_seal();
        // printf("seal done\n");
        // sleep(1);
        // gauge_unseal();
        // printf("unseal done\n");
        // sleep(1);
        // gauge_full_access();
        // printf("full access done\n");
        // sleep(1);

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

        printf("current %d mA\n",current());

        printf("voltage %d mV\n",voltage());

        printf("average current %d mA\n",average_current());


        i2c_close(i2c_bus);
        return 0;
}
