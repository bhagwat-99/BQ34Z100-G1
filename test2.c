#include <stdio.h>

int temp_func()
{
        unsigned short temp = 0xffff;
        return temp;
}

int main()
{

int ret_val = temp_func();
unsigned short temp = ret_val;
printf("%x",temp);
return 0;
}