#include <stdio.h>

unsigned char reg_data[4];
int main()
{

__uint16_t control_subcommand = 0x5678;
reg_data[0] = control_subcommand & 0x00FF; //lsb
reg_data[1] = control_subcommand >> 8; //msb

printf("reg_data[0]: %x",reg_data[0]);
printf("reg_data[1]: %x",reg_data[1]);


        return 0;
}